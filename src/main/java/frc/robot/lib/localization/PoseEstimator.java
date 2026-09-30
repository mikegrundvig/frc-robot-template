package frc.robot.lib.localization;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.function.Predicate;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * Where the robot is: odometry, corrected by camera measurements, with the gyro owning heading.
 *
 * <p>The state is the robot's position and a heading offset: the field heading is the gyro's
 * reading plus the offset, so the gyro keeps all short-term authority over heading, and cameras
 * only correct its slow drift. A 3x3 covariance holds the uncertainty of all three and how they
 * relate: an unsure heading makes every meter driven add position uncertainty sideways.
 *
 * <p>Each odometry step predicts: the wheels' motion, along the arc the gyro turned through, moves
 * the position, and the uncertainty grows with the distance driven, the turning, the wheels'
 * disagreement, and time. A camera measurement is applied at the moment its image was taken,
 * however late it arrives: the estimator keeps {@link EstimatorConfig#historySeconds} of steps,
 * rewinds to the measurement, applies it, and replays the steps since, so measurements give the
 * same result in any order of arrival. Each is gated against the estimate first; when several
 * rejected ones agree with each other, odometry was the one that was wrong (a collision, a spin),
 * and the estimate recovers to them.
 *
 * <p>Pure logic: no clock, no hardware, no static state. Every input comes in through a method,
 * stamped with its capture time, so a replay reproduces every decision. Use it from one thread.
 */
public final class PoseEstimator {
  private static final long VELOCITY_WINDOW_NANOS = 100_000_000L;

  private final EstimatorConfig config;
  private final Predicate<Translation2d> onField;
  private final double translationVariancePerMeter;
  private final double translationVariancePerRadian;
  private final double headingVariancePerSecond;
  private final double headingVariancePerRadian;
  private final double wheelHeadingVariancePerRadian;
  private final long historyNanos;
  private final long futureToleranceNanos;
  private final long visionTimeoutNanos;
  private final long recoveryWindowNanos;

  // The history: a ring of odometry steps, each with the state after it.
  private final int capacity;
  private final long[] time;
  private final double[] stepDx;
  private final double[] stepDy;
  private final double[] stepResidual;
  private final boolean[] stepGyro;
  private final double[] yawAfter;
  private final double[] xAfter;
  private final double[] yAfter;
  private final double[] offsetAfter;
  private final double[] covarianceAfter;
  private int oldest;
  private int size;

  // The current state: the newest step's, or, during a replay, the replay's.
  private double x;
  private double y;
  private double offset;
  private final double[] covariance = new double[9];

  // The gyro's reading, unwrapped into one continuous yaw.
  private boolean started;
  private double yaw;
  private double lastGyroYaw;
  private boolean lastGyroConnected;

  private PoseReset initial;
  private boolean placed;
  private long lastResetNanos = Long.MIN_VALUE;

  private final List<VisionMeasurement> pending = new ArrayList<>();
  private final List<Applied> applied = new ArrayList<>();
  private final ArrayDeque<Rejected> rejectedRecently = new ArrayDeque<>();
  private long lastAcceptedNanos = Long.MIN_VALUE;
  private int consecutiveRejections;
  private int acceptedCount;
  private int rejectedCount;
  private int recoveries;
  private double lastInnovationMeters = Double.NaN;

  // Scratch space for a measurement with heading.
  private final double[] innovation3 = new double[3];
  private final double[] s3 = new double[9];
  private final double[] sInverse3 = new double[9];
  private final double[] gain = new double[9];
  private final double[] product = new double[9];

  /** A measurement applied to the history, kept so a rewind can apply it again. */
  private record Applied(long time, VisionMeasurement measurement) {}

  /** A rejected measurement's disagreement with the estimate, for recovery. */
  private record Rejected(long time, double innovationX, double innovationY) {}

  /** A measurement to apply during a replay; fresh ones are gated, applied ones are not. */
  private record Event(long time, VisionMeasurement measurement, boolean fresh) {}

  /**
   * @param config how to weigh the sources
   * @param onField whether a point is on the field: measurements off it are rejected
   * @param initial where the robot is at the start, and how sure that is; for a placeholder (the
   *     robot not placed yet) give a large uncertainty, and the first camera measurement wins
   */
  public PoseEstimator(
      EstimatorConfig config, Predicate<Translation2d> onField, PoseReset initial) {
    this.config = config;
    this.onField = onField;
    this.initial = initial;
    translationVariancePerMeter = square(config.translationStdDevPerMeter());
    translationVariancePerRadian = square(config.translationStdDevPerRadian());
    headingVariancePerSecond = square(Math.toRadians(config.headingDriftDegreesPerMinute())) / 60;
    headingVariancePerRadian =
        square(Math.toRadians(config.headingStdDevDegreesPerTurn())) / (2 * Math.PI);
    wheelHeadingVariancePerRadian =
        square(Math.toRadians(config.wheelHeadingStdDevDegreesPerTurn())) / (2 * Math.PI);
    historyNanos = Timestamps.fromSeconds(config.historySeconds());
    futureToleranceNanos = Timestamps.fromSeconds(config.futureToleranceSeconds());
    visionTimeoutNanos = Timestamps.fromSeconds(config.visionTimeoutSeconds());
    recoveryWindowNanos = Timestamps.fromSeconds(config.recoveryWindowSeconds());
    // Room for the history at up to 1 kHz of odometry.
    capacity = Math.max(16, (int) Math.ceil(config.historySeconds() * 1000) + 1);
    time = new long[capacity];
    stepDx = new double[capacity];
    stepDy = new double[capacity];
    stepResidual = new double[capacity];
    stepGyro = new boolean[capacity];
    yawAfter = new double[capacity];
    xAfter = new double[capacity];
    yAfter = new double[capacity];
    offsetAfter = new double[capacity];
    covarianceAfter = new double[capacity * 9];
    setState(initial);
  }

  /**
   * One odometry step: how the robot moved since the previous step, ending at this time, and what
   * the gyro read then. Steps come in time order, at whatever rate odometry runs (250 Hz on a
   * robot); a step no newer than the last is ignored. The first step only starts the clock and the
   * gyro's reference.
   *
   * @param gyroYawRadians the gyro's yaw, counterclockwise; any zero, wrapped or not
   * @param gyroConnected false while the gyro is lost: the wheels' rotation stands in for it
   */
  public void addOdometry(
      long timestampNanos, BodyMotion motion, double gyroYawRadians, boolean gyroConnected) {
    if (!started) {
      started = true;
      yaw = gyroConnected ? gyroYawRadians : 0;
      lastGyroYaw = gyroYawRadians;
      lastGyroConnected = gyroConnected;
      oldest = 0;
      size = 1;
      time[0] = timestampNanos;
      stepGyro[0] = gyroConnected;
      yawAfter[0] = yaw;
      setState(initial);
      store(0);
      lastResetNanos = timestampNanos;
      return;
    }
    if (timestampNanos <= newestTime()) {
      return;
    }
    // The gyro owns rotation. When it's lost, or has just come back and its reading may have
    // jumped, the wheels' rotation stands in for the step.
    yaw +=
        gyroConnected && lastGyroConnected
            ? MathUtil.angleModulus(gyroYawRadians - lastGyroYaw)
            : motion.dthetaRadians();
    if (gyroConnected) {
      lastGyroYaw = gyroYawRadians;
    }
    lastGyroConnected = gyroConnected;

    if (size == capacity) {
      oldest = index(1);
      size--;
    }
    int slot = index(size);
    size++;
    time[slot] = timestampNanos;
    stepDx[slot] = motion.dxMeters();
    stepDy[slot] = motion.dyMeters();
    stepResidual[slot] = motion.residualMeters();
    stepGyro[slot] = gyroConnected;
    yawAfter[slot] = yaw;
    advance(size - 1, 0, 1);
    store(slot);
    prune();
  }

  /** A camera measurement, to apply at the next {@link #update}. */
  public void addVision(VisionMeasurement measurement) {
    pending.add(measurement);
  }

  /**
   * Applies the measurements added since the last update, each at its capture time, and decides
   * whether odometry needs rescuing. Call it once per loop, after adding the loop's odometry and
   * measurements.
   *
   * @return what happened to each measurement, for the log
   */
  public List<VisionDecision> update() {
    List<VisionDecision> decisions = new ArrayList<>();
    if (!pending.isEmpty()) {
      pending.sort(Comparator.comparingLong(VisionMeasurement::timestampNanos));
      List<VisionMeasurement> usable = new ArrayList<>();
      for (VisionMeasurement measurement : pending) {
        String why = whyNotUsable(measurement);
        if (why == null) {
          usable.add(measurement);
        } else {
          rejectedCount++;
          decisions.add(new VisionDecision(measurement, false, why, Double.NaN, Double.NaN));
        }
      }
      pending.clear();
      if (!usable.isEmpty()) {
        replay(usable, decisions);
      }
    }
    recoverIfTheCamerasAgree();
    return decisions;
  }

  /**
   * Tells the estimator where the robot is, now: placed on its start, or a driver's "against the
   * wall here". History before it is dropped. A reset with no heading (infinite heading variance)
   * keeps the heading as it was.
   */
  public void reset(PoseReset reset) {
    placed = true;
    rejectedRecently.clear();
    consecutiveRejections = 0;
    if (!started) {
      initial = reset;
      setState(reset);
      return;
    }
    setState(reset);
    startHistoryHere();
  }

  /** The current pose: cheap, for driving every loop. */
  public Pose2d pose() {
    return new Pose2d(x, y, new Rotation2d(yaw + offset));
  }

  /** The current estimate, with its uncertainty, velocity, mode, and health. */
  public LocalizationEstimate estimate() {
    long now = started ? newestTime() : 0;
    double moveX = 0;
    double moveY = 0;
    double turned = 0;
    long from = now;
    for (int j = size - 1; j > 0 && time[index(j)] > now - VELOCITY_WINDOW_NANOS; j--) {
      int slot = index(j);
      int previous = index(j - 1);
      double heading = yawAfter[previous] + offset;
      moveX += Math.cos(heading) * stepDx[slot] - Math.sin(heading) * stepDy[slot];
      moveY += Math.sin(heading) * stepDx[slot] + Math.cos(heading) * stepDy[slot];
      turned += yawAfter[slot] - yawAfter[previous];
      from = time[previous];
    }
    double seconds = Timestamps.toSeconds(now - from);
    Translation2d velocity =
        seconds > 0 ? new Translation2d(moveX / seconds, moveY / seconds) : Translation2d.ZERO;
    double yawRate = seconds > 0 ? turned / seconds : 0;

    boolean gyroConnected = !started || stepGyro[index(size - 1)];
    LocalizationEstimate.Mode mode;
    if (!gyroConnected) {
      mode = LocalizationEstimate.Mode.IMU_FAULT;
    } else if (!placed) {
      mode = LocalizationEstimate.Mode.INITIALIZING;
    } else if (consecutiveRejections >= 2) {
      mode = LocalizationEstimate.Mode.LOST;
    } else if (lastAcceptedNanos != Long.MIN_VALUE
        && now - lastAcceptedNanos <= visionTimeoutNanos) {
      mode = LocalizationEstimate.Mode.TRACKING;
    } else {
      mode = LocalizationEstimate.Mode.DEAD_RECKONING;
    }
    double visionAge =
        lastAcceptedNanos == Long.MIN_VALUE
            ? Double.POSITIVE_INFINITY
            : Timestamps.toSeconds(now - lastAcceptedNanos);
    double residual = started ? stepResidual[index(size - 1)] : 0;
    return new LocalizationEstimate(
        now,
        pose(),
        currentCovariance(),
        velocity,
        yawRate,
        mode,
        new LocalizationEstimate.Health(
            visionAge,
            acceptedCount,
            rejectedCount,
            lastInnovationMeters,
            residual,
            recoveries,
            gyroConnected));
  }

  private @Nullable String whyNotUsable(VisionMeasurement measurement) {
    if (!started) {
      return "before odometry started";
    }
    long t = measurement.timestampNanos();
    if (t <= lastResetNanos) {
      return "before the robot was last placed or recovered";
    }
    if (t <= time[oldest]) {
      return "too late: older than the history kept";
    }
    if (t > newestTime() + futureToleranceNanos) {
      return "from the future: after the newest odometry";
    }
    if (!onField.test(measurement.pose().getTranslation())) {
      return "off the field";
    }
    return null;
  }

  /**
   * Rewinds to just before the earliest new measurement and replays the history from there,
   * applying the new measurements and the ones applied before, in time order.
   */
  private void replay(List<VisionMeasurement> usable, List<VisionDecision> decisions) {
    long newest = newestTime();
    // A measurement stamped a little after the newest step is applied at the newest step.
    long earliest = Math.min(usable.get(0).timestampNanos(), newest);
    int start = size - 1;
    while (start > 0 && time[index(start)] >= earliest) {
      start--;
    }
    long startTime = time[index(start)];
    List<Event> events = new ArrayList<>();
    for (Applied done : applied) {
      if (done.time() > startTime) {
        events.add(new Event(done.time(), done.measurement(), false));
      }
    }
    for (VisionMeasurement measurement : usable) {
      events.add(new Event(Math.min(measurement.timestampNanos(), newest), measurement, true));
    }
    events.sort(Comparator.comparingLong(Event::time));
    applied.removeIf(done -> done.time() > startTime);

    restore(index(start));
    int next = 0;
    for (int j = start + 1; j < size; j++) {
      long before = time[index(j - 1)];
      long after = time[index(j)];
      double done = 0;
      while (next < events.size() && events.get(next).time() <= after) {
        Event event = events.get(next++);
        double fraction = (double) (event.time() - before) / (after - before);
        advance(j, done, fraction);
        done = fraction;
        apply(event, yawAt(j, fraction), decisions);
      }
      advance(j, done, 1);
      store(index(j));
    }
    // Only when the history is a single step: measurements at that step itself.
    while (next < events.size()) {
      apply(events.get(next++), yawAfter[index(size - 1)], decisions);
    }
    store(index(size - 1));
  }

  /** Gates a fresh measurement, then applies it as a Kalman update; applies an old one ungated. */
  private void apply(Event event, double yawThen, List<VisionDecision> decisions) {
    VisionMeasurement measurement = event.measurement();
    PoseCovariance noise = measurement.covariance();
    double innovationX = measurement.pose().getX() - x;
    double innovationY = measurement.pose().getY() - y;
    boolean withHeading = noise.hasHeading();
    double chiSquared;
    double gate;
    if (withHeading) {
      innovation3[0] = innovationX;
      innovation3[1] = innovationY;
      innovation3[2] =
          MathUtil.angleModulus(measurement.pose().getRotation().getRadians() - (yawThen + offset));
      System.arraycopy(covariance, 0, s3, 0, 9);
      s3[0] += noise.xx();
      s3[1] += noise.xy();
      s3[3] += noise.xy();
      s3[4] += noise.yy();
      s3[8] += noise.heading();
      if (!Matrices.invert3(s3, sInverse3)) {
        return;
      }
      chiSquared = quadratic3(sInverse3, innovation3);
      gate = config.gateChiSquaredPose();
    } else {
      double s00 = covariance[0] + noise.xx();
      double s01 = covariance[1] + noise.xy();
      double s11 = covariance[4] + noise.yy();
      double determinant = s00 * s11 - s01 * s01;
      if (!(determinant > 0)) {
        return;
      }
      double i00 = s11 / determinant;
      double i01 = -s01 / determinant;
      double i11 = s00 / determinant;
      chiSquared =
          innovationX * (i00 * innovationX + i01 * innovationY)
              + innovationY * (i01 * innovationX + i11 * innovationY);
      gate = config.gateChiSquaredPosition();
      // K = P H^T S^-1, for H picking out the position.
      for (int row = 0; row < 3; row++) {
        gain[row * 3] = covariance[row * 3] * i00 + covariance[row * 3 + 1] * i01;
        gain[row * 3 + 1] = covariance[row * 3] * i01 + covariance[row * 3 + 1] * i11;
      }
    }

    double innovationMeters = Math.hypot(innovationX, innovationY);
    if (event.fresh()) {
      lastInnovationMeters = innovationMeters;
      if (chiSquared > gate) {
        rejectedCount++;
        consecutiveRejections++;
        rejectedRecently.addLast(new Rejected(event.time(), innovationX, innovationY));
        decisions.add(
            new VisionDecision(
                measurement,
                false,
                String.format(Locale.ROOT, "gate: %.1f > %.1f", chiSquared, gate),
                innovationMeters,
                chiSquared));
        return;
      }
      acceptedCount++;
      consecutiveRejections = 0;
      rejectedRecently.clear();
      lastAcceptedNanos = Math.max(lastAcceptedNanos, event.time());
      placed = true;
      decisions.add(
          new VisionDecision(measurement, true, "accepted", innovationMeters, chiSquared));
    }

    if (withHeading) {
      multiply3(covariance, sInverse3, gain);
      x += gain[0] * innovation3[0] + gain[1] * innovation3[1] + gain[2] * innovation3[2];
      y += gain[3] * innovation3[0] + gain[4] * innovation3[1] + gain[5] * innovation3[2];
      offset += gain[6] * innovation3[0] + gain[7] * innovation3[1] + gain[8] * innovation3[2];
      // P = P - K P
      multiply3(gain, covariance, product);
      for (int k = 0; k < 9; k++) {
        covariance[k] -= product[k];
      }
    } else {
      x += gain[0] * innovationX + gain[1] * innovationY;
      y += gain[3] * innovationX + gain[4] * innovationY;
      offset += gain[6] * innovationX + gain[7] * innovationY;
      // P = P - K H P, where H P is P's first two rows.
      for (int row = 0; row < 3; row++) {
        for (int column = 0; column < 3; column++) {
          product[row * 3 + column] =
              gain[row * 3] * covariance[column] + gain[row * 3 + 1] * covariance[3 + column];
        }
      }
      for (int k = 0; k < 9; k++) {
        covariance[k] -= product[k];
      }
    }
    symmetrize();
    applied.add(new Applied(event.time(), measurement));
  }

  /**
   * When enough recent rejected measurements agree with each other, the cameras are right and
   * odometry is wrong: jump to where they put the robot, with a fresh uncertainty.
   */
  private void recoverIfTheCamerasAgree() {
    if (!started) {
      return;
    }
    long newest = newestTime();
    while (!rejectedRecently.isEmpty()
        && rejectedRecently.getFirst().time() < newest - recoveryWindowNanos) {
      rejectedRecently.removeFirst();
    }
    if (rejectedRecently.size() < config.recoveryMeasurements()) {
      return;
    }
    double meanX = 0;
    double meanY = 0;
    for (Rejected rejected : rejectedRecently) {
      meanX += rejected.innovationX();
      meanY += rejected.innovationY();
    }
    meanX /= rejectedRecently.size();
    meanY /= rejectedRecently.size();
    for (Rejected rejected : rejectedRecently) {
      double spread = Math.hypot(rejected.innovationX() - meanX, rejected.innovationY() - meanY);
      if (spread > config.recoveryAgreementMeters()) {
        return;
      }
    }
    x += meanX;
    y += meanY;
    double variance = square(config.recoveryStdDevMeters());
    covariance[0] = variance;
    covariance[4] = variance;
    covariance[1] = 0;
    covariance[3] = 0;
    covariance[2] = 0;
    covariance[6] = 0;
    covariance[5] = 0;
    covariance[7] = 0;
    startHistoryHere();
    recoveries++;
    consecutiveRejections = 0;
    rejectedRecently.clear();
    lastAcceptedNanos = newest;
    placed = true;
  }

  /** Moves the current state through part of step j: from fraction f0 of it to f1. */
  private void advance(int j, double f0, double f1) {
    double fraction = f1 - f0;
    if (fraction <= 0) {
      return;
    }
    int slot = index(j);
    int previous = index(j - 1);
    double stepTurn = yawAfter[slot] - yawAfter[previous];
    double heading = yawAfter[previous] + f0 * stepTurn + offset;
    double turn = fraction * stepTurn;
    double dx = fraction * stepDx[slot];
    double dy = fraction * stepDy[slot];
    // The wheels' travel, along the arc the gyro says the robot turned through.
    double along = dx;
    double across = dy;
    if (Math.abs(turn) > 1e-9) {
      double s = Math.sin(turn) / turn;
      double c = (1 - Math.cos(turn)) / turn;
      along = s * dx - c * dy;
      across = c * dx + s * dy;
    }
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);
    double moveX = cos * along - sin * across;
    double moveY = sin * along + cos * across;
    x += moveX;
    y += moveY;

    // P = F P F^T, where F = I plus a heading column: turning the offset swings the move.
    double a = -moveY;
    double b = moveX;
    double p02 = covariance[2];
    double p12 = covariance[5];
    double p22 = covariance[8];
    covariance[0] += 2 * a * p02 + a * a * p22;
    covariance[1] += a * p12 + b * p02 + a * b * p22;
    covariance[3] = covariance[1];
    covariance[4] += 2 * b * p12 + b * b * p22;
    covariance[2] += a * p22;
    covariance[6] = covariance[2];
    covariance[5] += b * p22;
    covariance[7] = covariance[5];

    // Plus the step's own uncertainty.
    double distance = Math.hypot(stepDx[slot], stepDy[slot]);
    double seconds = Timestamps.toSeconds(time[slot] - time[previous]);
    double residual = fraction * stepResidual[slot] * config.residualScale();
    double positionNoise =
        fraction
                * (translationVariancePerMeter * distance
                    + translationVariancePerRadian * Math.abs(stepTurn))
            + residual * residual;
    double headingNoise =
        fraction
            * (headingVariancePerSecond * seconds
                + (stepGyro[slot] ? headingVariancePerRadian : wheelHeadingVariancePerRadian)
                    * Math.abs(stepTurn));
    covariance[0] += positionNoise;
    covariance[4] += positionNoise;
    covariance[8] += headingNoise;
  }

  private double yawAt(int j, double fraction) {
    double before = yawAfter[index(j - 1)];
    return before + fraction * (yawAfter[index(j)] - before);
  }

  private void setState(PoseReset reset) {
    x = reset.pose().getX();
    y = reset.pose().getY();
    PoseCovariance given = reset.covariance();
    if (given.hasHeading()) {
      offset = reset.pose().getRotation().getRadians() - yaw;
      covariance[8] = given.heading();
    }
    covariance[0] = given.xx();
    covariance[1] = given.xy();
    covariance[3] = given.xy();
    covariance[4] = given.yy();
    covariance[2] = 0;
    covariance[6] = 0;
    covariance[5] = 0;
    covariance[7] = 0;
  }

  /** Drops the history: the newest step, with the current state, becomes its start. */
  private void startHistoryHere() {
    oldest = index(size - 1);
    size = 1;
    store(oldest);
    applied.clear();
    lastResetNanos = time[oldest];
  }

  /** Drops steps older than the history keeps, and the measurements already inside the oldest. */
  private void prune() {
    long cutoff = newestTime() - historyNanos;
    while (size > 1 && time[index(1)] <= cutoff) {
      oldest = index(1);
      size--;
    }
    long oldestTime = time[oldest];
    applied.removeIf(done -> done.time() <= oldestTime);
  }

  private PoseCovariance currentCovariance() {
    double xx = Math.max(0, covariance[0]);
    double yy = Math.max(0, covariance[4]);
    double limit = Math.sqrt(xx * yy);
    double xy = Math.clamp(covariance[1], -limit, limit);
    return new PoseCovariance(xx, xy, yy, Math.max(0, covariance[8]));
  }

  private void symmetrize() {
    covariance[1] = covariance[3] = (covariance[1] + covariance[3]) / 2;
    covariance[2] = covariance[6] = (covariance[2] + covariance[6]) / 2;
    covariance[5] = covariance[7] = (covariance[5] + covariance[7]) / 2;
  }

  private void store(int slot) {
    xAfter[slot] = x;
    yAfter[slot] = y;
    offsetAfter[slot] = offset;
    System.arraycopy(covariance, 0, covarianceAfter, slot * 9, 9);
  }

  private void restore(int slot) {
    x = xAfter[slot];
    y = yAfter[slot];
    offset = offsetAfter[slot];
    System.arraycopy(covarianceAfter, slot * 9, covariance, 0, 9);
  }

  private long newestTime() {
    return time[index(size - 1)];
  }

  private int index(int k) {
    return (oldest + k) % capacity;
  }

  private static double quadratic3(double[] m, double[] v) {
    double sum = 0;
    for (int row = 0; row < 3; row++) {
      for (int column = 0; column < 3; column++) {
        sum += v[row] * m[row * 3 + column] * v[column];
      }
    }
    return sum;
  }

  private static void multiply3(double[] a, double[] b, double[] out) {
    for (int row = 0; row < 3; row++) {
      for (int column = 0; column < 3; column++) {
        out[row * 3 + column] =
            a[row * 3] * b[column]
                + a[row * 3 + 1] * b[3 + column]
                + a[row * 3 + 2] * b[6 + column];
      }
    }
  }

  private static double square(double value) {
    return value * value;
  }
}
