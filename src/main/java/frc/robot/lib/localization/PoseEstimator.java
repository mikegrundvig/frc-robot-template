package frc.robot.lib.localization;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
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
 * the position, and the uncertainty grows with the distance driven, the turning, and time, and with
 * whatever makes the wheels suspect: their disagreement, a tilt that lifts them, a grip at its
 * limit, a collision (see {@link DriveConditions}). A camera measurement is applied at the moment
 * its image was taken, however late it arrives: the estimator keeps {@link
 * EstimatorConfig#historySeconds} of steps, rewinds to the measurement, applies it, and replays the
 * steps since, so measurements give the same result in any order of arrival. Each is gated against
 * the estimate first; when several rejected ones agree with each other, odometry was the one that
 * was wrong (a collision, a spin), and the estimate recovers to them: to their heading first, if
 * that is what they agree is wrong (a bad heading reset, a robot placed backwards), since every
 * position worked out with a wrong heading is wrong too.
 *
 * <p>Its parts: {@link OdometryHistory} keeps the steps, {@link PoseFilter} does the filter's math,
 * and {@link Recovery} judges when the cameras agree against odometry. This class feeds them and
 * keeps the books.
 *
 * <p>Pure logic: no clock, no hardware, no static state. Every input comes in through a method,
 * stamped with its capture time, so a replay reproduces every decision. Use it from one thread.
 */
public final class PoseEstimator implements CameraLocalizer {
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
  private final double tiltStartRadians;
  private final double tiltZeroRadians;
  private final double collisionVariance;
  private final long collisionWindowNanos;
  private final double handlingVariancePerSecond;
  private final long disagreementWindowNanos;

  private final OdometryHistory history;
  private final PoseFilter filter = new PoseFilter();
  private final Recovery recovery;

  // The gyro's reading, unwrapped into one continuous yaw.
  private boolean started;
  private double yaw;
  private double lastGyroYaw;
  private boolean lastGyroConnected;

  // What the wheels are going through, and the doubt it adds up to.
  private DriveConditions conditions = DriveConditions.NORMAL;
  private double suspectTravel;
  private boolean collisionPending;
  private long collisionUntilNanos = Long.MIN_VALUE;

  private PoseReset initial;
  private boolean placed;
  private long lastResetNanos = Long.MIN_VALUE;

  private final List<VisionMeasurement> pending = new ArrayList<>();
  private final List<LandmarkEvent> pendingLandmarks = new ArrayList<>();
  private final List<LandmarkDecision> landmarkDecisions = new ArrayList<>();
  private final List<Correction> applied = new ArrayList<>();
  private final List<Agreement> agreements = new ArrayList<>();
  private long lastAcceptedNanos = Long.MIN_VALUE;
  private int consecutiveRejections;
  private int acceptedCount;
  private int rejectedCount;
  private int recoveries;
  private double lastInnovationMeters = Double.NaN;

  /** A measurement the history applies at its time: a camera's, or a landmark. */
  private sealed interface Correction permits CameraCorrection, LandmarkCorrection {
    long time();
  }

  private record CameraCorrection(long time, VisionMeasurement measurement) implements Correction {}

  private record LandmarkCorrection(long time, LandmarkEvent landmark) implements Correction {}

  /** A correction to apply during a replay: a fresh one is gated, one applied before is not. */
  private record Event(Correction correction, boolean fresh) {
    long time() {
      return correction.time();
    }
  }

  /** An accepted measurement's distance from the estimate, for the health's agreement. */
  private record Agreement(long time, double meters) {}

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
    tiltStartRadians = Math.toRadians(config.tiltStartDegrees());
    tiltZeroRadians = Math.toRadians(config.tiltZeroDegrees());
    collisionVariance = square(config.collisionStdDevMeters());
    collisionWindowNanos = Timestamps.fromSeconds(config.collisionWindowSeconds());
    handlingVariancePerSecond = square(config.handlingStdDevPerSqrtSecond());
    disagreementWindowNanos = Timestamps.fromSeconds(config.disagreementWindowSeconds());
    // Room for the history at up to 1 kHz of odometry.
    history =
        new OdometryHistory(Math.max(16, (int) Math.ceil(config.historySeconds() * 1000) + 1));
    recovery = new Recovery(config);
    filter.set(initial, yaw);
  }

  /**
   * One odometry step: how the robot moved since the previous step, ending at this time, and what
   * the gyro read then. Steps come in time order, at whatever rate odometry runs (250 Hz on a
   * robot); a step no newer than the last is ignored. The first step only starts the clock and the
   * gyro's reference.
   *
   * @param gyroYawRadians the gyro's yaw, counterclockwise; any zero, wrapped or not
   * @param gyroReporting false while the gyro is lost: the wheels' rotation stands in for it. A yaw
   *     that isn't a number counts as lost, so one bad reading can't poison the heading.
   */
  public void addOdometry(
      long timestampNanos, BodyMotion motion, double gyroYawRadians, boolean gyroReporting) {
    boolean gyroConnected = gyroReporting && Double.isFinite(gyroYawRadians);
    if (!started) {
      started = true;
      yaw = gyroConnected ? gyroYawRadians : 0;
      lastGyroYaw = gyroConnected ? gyroYawRadians : 0;
      lastGyroConnected = gyroConnected;
      history.start(timestampNanos, gyroConnected, yaw);
      filter.set(initial, yaw);
      history.store(0, filter);
      lastResetNanos = timestampNanos;
      return;
    }
    long previousTime = history.newestTime();
    if (timestampNanos <= previousTime) {
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

    // A tilt the ground explains still rolls on the ground: the travel counts at the slope's
    // cosine. Tilt beyond it may have lifted the wheels: their travel counts for less.
    double trust = wheelTrust(conditions.unexplainedTiltRadians());
    double across = Math.cos(conditions.groundSlopeRadians());
    double extra =
        extraVariance(
            previousTime,
            timestampNanos,
            Math.hypot(motion.dxMeters(), motion.dyMeters()),
            trust,
            motion.residualMeters());
    int step =
        history.add(
            timestampNanos,
            trust * across * motion.dxMeters(),
            trust * across * motion.dyMeters(),
            motion.residualMeters(),
            extra,
            gyroConnected,
            yaw);
    advance(step, 0, 1);
    history.store(step, filter);
    prune();
  }

  /**
   * What the robot's other sensors say about its wheels, for the odometry steps added after this
   * until the next call. A new collision makes the next step uncertain by {@link
   * EstimatorConfig#collisionStdDevMeters}: the robot may have moved without its wheels seeing it.
   * Jolts within {@link EstimatorConfig#collisionWindowSeconds} of the last are the same collision.
   */
  public void setConditions(DriveConditions conditions) {
    this.conditions = conditions;
    if (conditions.collision() && started) {
      collisionPending |= !(history.newestTime() <= collisionUntilNanos);
      collisionUntilNanos = history.newestTime() + collisionWindowNanos;
    }
  }

  /** A camera measurement, to apply at the next {@link #update}. */
  @Override
  public void addVision(VisionMeasurement measurement) {
    pending.add(measurement);
  }

  /**
   * A landmark, to apply at the next {@link #update} at its time: an update along one axis, gated
   * like a camera's measurement, and kept apart from the cameras' counts, agreement, and recovery.
   */
  public void addLandmark(LandmarkEvent landmark) {
    pendingLandmarks.add(landmark);
  }

  /** What the last {@link #update} did with each landmark, for the log. */
  public List<LandmarkDecision> landmarkDecisions() {
    return List.copyOf(landmarkDecisions);
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
    landmarkDecisions.clear();
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
    List<LandmarkEvent> landmarks = new ArrayList<>();
    for (LandmarkEvent landmark : pendingLandmarks) {
      String why = whyNotInHistory(landmark.timestampNanos());
      if (why == null) {
        landmarks.add(landmark);
      } else {
        landmarkDecisions.add(new LandmarkDecision(landmark, false, why, Double.NaN, Double.NaN));
      }
    }
    pendingLandmarks.clear();
    if (!usable.isEmpty() || !landmarks.isEmpty()) {
      replay(usable, landmarks, decisions);
    }
    recoverIfTheCamerasAgree();
    if (started) {
      long cutoff = history.newestTime() - disagreementWindowNanos;
      agreements.removeIf(agreement -> agreement.time() < cutoff);
    }
    return decisions;
  }

  /**
   * Tells the estimator where the robot is, now: placed on its start, or a driver's "against the
   * wall here". History before it is dropped. A reset with no heading (infinite heading variance)
   * keeps the heading as it was.
   */
  public void reset(PoseReset reset) {
    placed = true;
    recovery.clear();
    consecutiveRejections = 0;
    suspectTravel = 0;
    if (!started) {
      initial = reset;
      filter.set(reset, yaw);
      return;
    }
    filter.set(reset, yaw);
    startHistoryHere();
  }

  /**
   * Tells the estimator which way the robot faces, now, keeping where it is: a driver's "facing
   * straight downfield" when the heading has gone wrong. History before it is dropped.
   *
   * @param stdDevRadians how sure: how straight a driver lines the robot up by eye
   */
  public void resetHeading(Rotation2d heading, double stdDevRadians) {
    if (!(stdDevRadians > 0) || Double.isInfinite(stdDevRadians)) {
      throw new IllegalArgumentException("a heading reset's uncertainty: " + stdDevRadians);
    }
    recovery.clear();
    consecutiveRejections = 0;
    double variance = square(stdDevRadians);
    if (!started) {
      PoseCovariance before = initial.covariance();
      initial =
          new PoseReset(
              initial.timestampNanos(),
              new Pose2d(initial.pose().getTranslation(), heading),
              new PoseCovariance(before.xx(), before.xy(), before.yy(), variance),
              initial.reason());
      filter.set(initial, yaw);
      return;
    }
    filter.resetHeading(heading.getRadians() - yaw, variance);
    startHistoryHere();
  }

  /**
   * Where the robot was at a moment in the history, as the estimate now has it: for tying a camera
   * image to the robot's heading when the image was taken.
   *
   * @return empty outside the history
   */
  @Override
  public Optional<Pose2d> poseAt(long timestampNanos) {
    if (!started || timestampNanos < history.time(0)) {
      return Optional.empty();
    }
    if (timestampNanos >= history.newestTime()) {
      return timestampNanos <= history.newestTime() + futureToleranceNanos
          ? Optional.of(pose())
          : Optional.empty();
    }
    return Optional.of(history.poseAt(timestampNanos, filter.offset()));
  }

  /** The current pose: cheap, for driving every loop. */
  public Pose2d pose() {
    return filter.pose(yaw);
  }

  /** The current estimate, with its uncertainty, velocity, mode, and health. */
  public LocalizationEstimate estimate() {
    long now = started ? history.newestTime() : 0;
    OdometryHistory.Motion motion =
        started
            ? history.motionOver(VELOCITY_WINDOW_NANOS, filter.offset())
            : new OdometryHistory.Motion(Translation2d.ZERO, 0);
    boolean gyroConnected = !started || history.gyro(history.size() - 1);
    double visionAge =
        lastAcceptedNanos == Long.MIN_VALUE
            ? Double.POSITIVE_INFINITY
            : Timestamps.toSeconds(now - lastAcceptedNanos);
    double disagreement = Double.NaN;
    for (Agreement agreement : agreements) {
      if (agreement.time() >= now - disagreementWindowNanos
          && !(agreement.meters() <= disagreement)) {
        disagreement = agreement.meters();
      }
    }
    return new LocalizationEstimate(
        now,
        pose(),
        filter.covariance(),
        motion.velocity(),
        motion.yawRate(),
        mode(
            gyroConnected,
            placed,
            consecutiveRejections >= config.lostAfterRejections(),
            lastAcceptedNanos != Long.MIN_VALUE && now - lastAcceptedNanos <= visionTimeoutNanos),
        new LocalizationEstimate.Health(
            visionAge,
            acceptedCount,
            rejectedCount,
            lastInnovationMeters,
            disagreement,
            started ? history.residual(history.size() - 1) : 0,
            suspectTravel,
            conditions.tiltRadians(),
            started && now <= collisionUntilNanos,
            recoveries,
            gyroConnected));
  }

  /**
   * What the estimate rests on, in order: a lost gyro, then not placed yet, then cameras that keep
   * disagreeing, then recent camera measurements, else odometry alone.
   *
   * @param lost whether the cameras have kept disagreeing, measurement after measurement
   * @param visionRecent whether a camera measurement was accepted recently
   */
  static LocalizationEstimate.Mode mode(
      boolean gyroConnected, boolean placed, boolean lost, boolean visionRecent) {
    if (!gyroConnected) {
      return LocalizationEstimate.Mode.IMU_FAULT;
    } else if (!placed) {
      return LocalizationEstimate.Mode.INITIALIZING;
    } else if (lost) {
      return LocalizationEstimate.Mode.LOST;
    } else if (visionRecent) {
      return LocalizationEstimate.Mode.TRACKING;
    }
    return LocalizationEstimate.Mode.DEAD_RECKONING;
  }

  private @Nullable String whyNotUsable(VisionMeasurement measurement) {
    String why = whyNotInHistory(measurement.timestampNanos());
    if (why != null) {
      return why;
    }
    if (!onField.test(measurement.pose().getTranslation())) {
      return "off the field";
    }
    return null;
  }

  /** Why a measurement at a time can't be applied: before the history, or after it. */
  private @Nullable String whyNotInHistory(long t) {
    if (!started) {
      return "before odometry started";
    }
    if (t <= lastResetNanos) {
      return "before the robot was last placed or recovered";
    }
    if (t <= history.time(0)) {
      return "too late: older than the history kept";
    }
    if (t > history.newestTime() + futureToleranceNanos) {
      return "from the future: after the newest odometry";
    }
    return null;
  }

  /**
   * Rewinds to just before the earliest new measurement and replays the history from there,
   * applying the new measurements and the ones applied before, in time order.
   */
  private void replay(
      List<VisionMeasurement> usable,
      List<LandmarkEvent> landmarks,
      List<VisionDecision> decisions) {
    long newest = history.newestTime();
    // A measurement stamped a little after the newest step is applied at the newest step.
    long earliest = newest;
    for (VisionMeasurement measurement : usable) {
      earliest = Math.min(earliest, measurement.timestampNanos());
    }
    for (LandmarkEvent landmark : landmarks) {
      earliest = Math.min(earliest, landmark.timestampNanos());
    }
    int start = history.lastStepBefore(earliest);
    long startTime = history.time(start);
    List<Event> events = new ArrayList<>();
    for (Correction done : applied) {
      if (done.time() > startTime) {
        events.add(new Event(done, false));
      }
    }
    for (VisionMeasurement measurement : usable) {
      events.add(
          new Event(
              new CameraCorrection(Math.min(measurement.timestampNanos(), newest), measurement),
              true));
    }
    for (LandmarkEvent landmark : landmarks) {
      events.add(
          new Event(
              new LandmarkCorrection(Math.min(landmark.timestampNanos(), newest), landmark), true));
    }
    events.sort(Comparator.comparingLong(Event::time));
    applied.removeIf(done -> done.time() > startTime);

    history.restore(start, filter);
    int next = 0;
    for (int j = start + 1; j < history.size(); j++) {
      long before = history.time(j - 1);
      long after = history.time(j);
      double done = 0;
      while (next < events.size() && events.get(next).time() <= after) {
        Event event = events.get(next++);
        double fraction = (double) (event.time() - before) / (after - before);
        advance(j, done, fraction);
        done = fraction;
        apply(event, history.yawAt(j, fraction), decisions);
      }
      advance(j, done, 1);
      history.store(j, filter);
    }
    // Only when the history is a single step: measurements at that step itself.
    while (next < events.size()) {
      apply(events.get(next++), history.yawAfter(history.size() - 1), decisions);
    }
    history.store(history.size() - 1, filter);
  }

  private void apply(Event event, double yawThen, List<VisionDecision> decisions) {
    switch (event.correction()) {
      case CameraCorrection camera -> applyCamera(camera, event.fresh(), yawThen, decisions);
      case LandmarkCorrection landmark -> applyLandmark(landmark, event.fresh());
    }
  }

  /** Gates a fresh camera measurement, then applies it as a Kalman update; applies an old one. */
  private void applyCamera(
      CameraCorrection camera, boolean fresh, double yawThen, List<VisionDecision> decisions) {
    VisionMeasurement measurement = camera.measurement();
    PoseFilter.Innovation innovation = filter.innovation(measurement, yawThen);
    if (innovation == null) {
      return;
    }
    if (fresh && !gateCamera(camera, innovation, decisions)) {
      return;
    }
    filter.correct(innovation);
    applied.add(camera);
  }

  /**
   * Whether a fresh camera measurement passes the gate, keeping the books either way: the counts,
   * the agreement, and what recovery needs from a rejection.
   */
  private boolean gateCamera(
      CameraCorrection camera, PoseFilter.Innovation innovation, List<VisionDecision> decisions) {
    VisionMeasurement measurement = camera.measurement();
    double gate =
        innovation.withHeading() ? config.gateChiSquaredPose() : config.gateChiSquaredPosition();
    double chiSquared = innovation.chiSquared();
    lastInnovationMeters = innovation.meters();
    // Written so a measurement that isn't a number fails the gate instead of passing it.
    if (!(chiSquared <= gate)) {
      rejectedCount++;
      consecutiveRejections++;
      recovery.rejected(
          new Recovery.Rejected(
              camera.time(),
              innovation.x(),
              innovation.y(),
              innovation.heading(),
              innovation.positionChiSquared() <= config.gateChiSquaredPosition()));
      decisions.add(
          new VisionDecision(
              measurement,
              false,
              String.format(Locale.ROOT, "gate: %.1f > %.1f", chiSquared, gate),
              innovation.meters(),
              chiSquared));
      return false;
    }
    acceptedCount++;
    consecutiveRejections = 0;
    recovery.clear();
    lastAcceptedNanos = Math.max(lastAcceptedNanos, camera.time());
    agreements.add(new Agreement(camera.time(), innovation.meters()));
    placed = true;
    decisions.add(
        new VisionDecision(measurement, true, "accepted", innovation.meters(), chiSquared));
    return true;
  }

  /**
   * A landmark, as a Kalman update along one axis: it moves the estimate across the line, and
   * leaves it alone along it. Gated like a camera's measurement, but none of the cameras' records
   * change: a landmark is no camera, and one axis says nothing about the rest.
   */
  private void applyLandmark(LandmarkCorrection correction, boolean fresh) {
    LandmarkEvent landmark = correction.landmark();
    PoseFilter.AxisInnovation innovation = filter.innovationAlong(landmark);
    if (innovation == null) {
      return;
    }
    if (fresh) {
      double chiSquared = innovation.chiSquared();
      boolean passes = chiSquared <= config.gateChiSquaredAxis();
      landmarkDecisions.add(
          new LandmarkDecision(
              landmark,
              passes,
              passes
                  ? "accepted"
                  : String.format(
                      Locale.ROOT, "gate: %.1f > %.1f", chiSquared, config.gateChiSquaredAxis()),
              Math.abs(innovation.meters()),
              chiSquared));
      if (!passes) {
        return;
      }
    }
    filter.correct(innovation);
    applied.add(correction);
  }

  /**
   * When enough recent rejected measurements agree with each other, the cameras are right and
   * odometry is wrong: turn to the heading they agree on, or else jump to where they put the robot,
   * with a fresh uncertainty.
   */
  private void recoverIfTheCamerasAgree() {
    if (!started) {
      return;
    }
    long newest = history.newestTime();
    if (!recovery.enoughAsOf(newest)) {
      return;
    }
    double turn = recovery.headingCorrection();
    if (!Double.isNaN(turn)) {
      filter.turn(
          turn,
          square(config.lostPositionStdDevMeters()),
          square(Math.toRadians(config.recoveryHeadingStdDevDegrees())));
      recovered();
      return;
    }
    Translation2d move = recovery.positionCorrection();
    if (move == null) {
      return;
    }
    filter.shift(move.getX(), move.getY(), square(config.recoveryStdDevMeters()));
    recovered();
    lastAcceptedNanos = newest;
    placed = true;
  }

  /** After a recovery: the history starts again from it, and so do the doubts. */
  private void recovered() {
    startHistoryHere();
    suspectTravel = 0;
    recoveries++;
    consecutiveRejections = 0;
    recovery.clear();
  }

  /** Moves the current state through part of step j: from fraction f0 of it to f1. */
  private void advance(int j, double f0, double f1) {
    double fraction = f1 - f0;
    if (fraction <= 0) {
      return;
    }
    double stepTurn = history.yawAfter(j) - history.yawAfter(j - 1);
    double stepDx = history.dx(j);
    double stepDy = history.dy(j);
    double distance = Math.hypot(stepDx, stepDy);
    double seconds = Timestamps.toSeconds(history.time(j) - history.time(j - 1));
    filter.predict(
        history.yawAfter(j - 1) + f0 * stepTurn,
        fraction * stepTurn,
        fraction * stepDx,
        fraction * stepDy,
        fraction
            * (translationVariancePerMeter * distance
                + translationVariancePerRadian * Math.abs(stepTurn)
                + history.extra(j)),
        fraction
            * (headingVariancePerSecond * seconds
                + (history.gyro(j) ? headingVariancePerRadian : wheelHeadingVariancePerRadian)
                    * Math.abs(stepTurn)));
  }

  /**
   * How much of the wheels' travel counts at a tilt: all of it when level, less from the start
   * angle on, none from the zero angle.
   */
  private double wheelTrust(double tiltRadians) {
    return 1
        - Math.clamp(
            (tiltRadians - tiltStartRadians) / (tiltZeroRadians - tiltStartRadians), 0.0, 1.0);
  }

  /**
   * A step's position variance beyond the base terms. Its suspect travel (the share of the wheels'
   * travel the tilt left out or the grip put in doubt, and the wheels' disagreement) adds to the
   * recent suspect travel before squaring, since slip keeps its direction for a while; a new
   * collision adds its variance; and a disabled robot may be carried.
   */
  private double extraVariance(long from, long to, double distance, double trust, double residual) {
    double share =
        Math.max(
            1 - trust, conditions.tractionLimited() ? config.tractionLimitedSlipFraction() : 0);
    double seconds = Timestamps.toSeconds(to - from);
    double carried = suspectTravel * Math.exp(-seconds / config.slipCorrelationSeconds());
    suspectTravel = carried + share * distance + config.residualScale() * residual;
    double variance = square(suspectTravel) - square(carried);
    if (conditions.disabled()) {
      variance += handlingVariancePerSecond * seconds;
    }
    if (collisionPending) {
      variance += collisionVariance;
      collisionPending = false;
    }
    return variance;
  }

  /** Drops the history: the newest step, with the current state, becomes its start. */
  private void startHistoryHere() {
    history.keepOnlyNewest();
    history.store(0, filter);
    applied.clear();
    agreements.clear();
    lastResetNanos = history.time(0);
  }

  /** Drops steps older than the history keeps, and the measurements already inside the oldest. */
  private void prune() {
    history.dropThrough(history.newestTime() - historyNanos);
    long oldestTime = history.time(0);
    applied.removeIf(done -> done.time() <= oldestTime);
  }

  private static double square(double value) {
    return value * value;
  }
}
