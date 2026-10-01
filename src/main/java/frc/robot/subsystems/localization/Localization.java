package frc.robot.subsystems.localization;

import frc.robot.lib.field.Terrain;
import frc.robot.lib.localization.BodyMotion;
import frc.robot.lib.localization.CameraLocalizer;
import frc.robot.lib.localization.CrestLandmarks;
import frc.robot.lib.localization.DriveConditions;
import frc.robot.lib.localization.DriveKinematicsModel;
import frc.robot.lib.localization.DrivetrainReadings;
import frc.robot.lib.localization.ImuCrossCheck;
import frc.robot.lib.localization.ImuPlacement;
import frc.robot.lib.localization.ImuReadings;
import frc.robot.lib.localization.LandmarkDecision;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.localization.LocalizationGate;
import frc.robot.lib.localization.OdometrySample;
import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.PoseEstimator;
import frc.robot.lib.localization.PoseReset;
import frc.robot.lib.localization.SlipCheck;
import frc.robot.lib.localization.TerrainCrossing;
import frc.robot.lib.localization.Timestamps;
import frc.robot.lib.localization.VisionDecision;
import frc.robot.lib.localization.YawSource;
import frc.robot.lib.localization.YawTrack;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.RobotController;
import org.wpilib.util.Alert;

/**
 * Where the robot is: the one owner of the pose estimator. Everything that asks where the robot is
 * asks here, and nothing else changes the estimate.
 *
 * <p>Each loop, in order:
 *
 * <ol>
 *   <li>{@link #periodic}, with what the drivetrain read: picks what turns odometry (the gyro, the
 *       backup IMU while the gyro is lost, or the wheels), weighs the wheels by what the IMU says
 *       (a tilt, a jolt, the wheels sliding: {@link SlipCheck}), feeds every wheel reading to the
 *       estimator, follows the robot over raised terrain, and takes a crest it crossed as a
 *       landmark;
 *   <li>the cameras, through {@link #forCameras()}: each image's measurement, at its capture time;
 *   <li>{@link #update}: applies them, and says whether the estimate is good enough to score from.
 * </ol>
 *
 * <p>Independent of the drivetrain: its wheels come in as {@link OdometrySample}s, through its
 * {@link DriveKinematicsModel}, so swerve, tank, or any other localizes the same way. Replacing the
 * drivetrain's code (with AdvantageKit's 2027 swerve template, say) only needs it to report {@link
 * DrivetrainReadings}.
 */
public final class Localization implements AutoCloseable {

  /** Below this speed, in m/s, the robot has no direction of travel to measure a slope along. */
  private static final double MOVING = 0.25;

  /**
   * On a known bump, a tilt up to this much steeper than its ramps is still the ground's: bumpers,
   * suspension, and the IMU's mount.
   */
  private static final double GROUND_SLOPE_MARGIN = Math.toRadians(3);

  /** How far off a bump's footprint the estimate may put the robot and still be on it. */
  private static final double MAX_GROUND_REACH_METERS = 1.0;

  /**
   * A crest places the robot's center within about this far across it: a guess, to measure on a
   * real robot (docs/calibration.md, a terrain crossing).
   */
  private static final double CREST_SPREAD_METERS = 0.05;

  /** A crest is taken for a bump only when the estimate is within this far of it. */
  private static final double CREST_REACH_METERS = 1.0;

  /** A reset to "exactly here": a millimeter and a milliradian. */
  private static final PoseCovariance EXACTLY = PoseCovariance.fromStdDevs(0.001, 0.001, 0.001);

  private final PoseEstimator estimator;
  private final DriveKinematicsModel kinematics;
  private final Terrain ground;
  private final double wheelReach;
  private final CrestLandmarks crestLandmarks;
  private final YawTrack gyroYaws = new YawTrack();
  private final YawTrack backupYaws = new YawTrack();

  /** The gyro and the backup IMU should turn alike. */
  private final ImuCrossCheck crossCheck = new ImuCrossCheck(ImuCrossCheck.Settings.DEFAULTS);

  /** Crossings of raised terrain, from the IMU. */
  private final TerrainCrossing terrain = new TerrainCrossing(TerrainCrossing.Settings.DEFAULTS);

  /** The wheels against the IMU: whether they slide over the floor together. */
  private final SlipCheck slipCheck;

  /** The last wheel reading's time, last loop, and which IMU read its yaw, and what it read. */
  private long lastStepNanos = Long.MIN_VALUE;

  private YawSource lastStepSource = YawSource.WHEELS;
  private double lastStepYaw = Double.NaN;

  /** Where the IMUs sit, and whether the gyro's accelerometer reads along the robot's axes. */
  private final ImuPlacement imus;

  private YawSource yawSource = YawSource.GYRO;

  /** The previous wheel reading, which the next one's motion is measured from. */
  private @Nullable OdometrySample lastReading;

  /** The IMU the robot's attitude comes from this loop: null without one. */
  private @Nullable ImuReadings attitude;

  /** What the IMU and the motors said about trusting the wheels this loop. */
  private DriveConditions conditions = DriveConditions.NORMAL;

  private LocalizationEstimate estimate;

  /** Why the estimate isn't good enough to score from; null when it is. */
  private @Nullable String notReadyToScore = "not started";

  private int landmarksAccepted;

  /** The uncertainty as {@link #describe()} last showed it; NaN before it did. */
  private double shownCentimeters = Double.NaN;

  private double shownDegrees = Double.NaN;

  private final Alert gyroDisconnected =
      new Alert(
          "Localization",
          "gyroDisconnected",
          "Gyro disconnected: heading estimated from wheels, and will drift",
          Alert.Level.HIGH);
  private final Alert usingBackup =
      new Alert(
          "Localization",
          "usingBackupImu",
          "Gyro disconnected: heading from the backup IMU",
          Alert.Level.MEDIUM);
  private final Alert imusDisagree =
      new Alert(
          "Localization",
          "imusDisagree",
          "The gyro and the backup IMU disagree",
          Alert.Level.MEDIUM);
  private final Alert camerasWithoutPlaces =
      new Alert(
          "Localization",
          "camerasWithoutPlaces",
          "More cameras than the estimator keeps apart",
          Alert.Level.MEDIUM);

  /**
   * @param estimator the pose estimator, which this owns from now on
   * @param kinematics how the drivetrain's wheels move the robot
   * @param terrain the field's raised terrain: a tilt it explains keeps the wheels trusted, and its
   *     crests are landmarks
   * @param wheelReachMeters how far the wheels reach from the robot's center
   * @param imus where the IMUs sit, and whether the gyro's accelerometer reads along the robot's
   *     axes
   */
  public Localization(
      PoseEstimator estimator,
      DriveKinematicsModel kinematics,
      Terrain terrain,
      double wheelReachMeters,
      ImuPlacement imus) {
    this.estimator = estimator;
    this.kinematics = kinematics;
    this.ground = terrain;
    this.wheelReach = wheelReachMeters;
    crestLandmarks = new CrestLandmarks(terrain, CREST_SPREAD_METERS, CREST_REACH_METERS);
    this.imus = imus;
    slipCheck =
        new SlipCheck(SlipCheck.Settings.DEFAULTS, wheelReachMeters, imus.accelerometerChecked());
    estimate = estimator.estimate();
  }

  /**
   * One loop of the drivetrain's readings: after it reads its sensors, and before the cameras.
   *
   * @param enabled whether the robot is enabled; while disabled, people may carry it where its
   *     wheels can't see
   */
  public void periodic(DrivetrainReadings readings, boolean enabled) {
    ImuReadings gyro = readings.gyro();
    ImuReadings backup = readings.backupImu();
    Steps steps = steps(readings.wheels(), gyro, backup);
    // One IMU says everything this loop: the one whose yaw turns odometry.
    attitude =
        switch (steps.source()) {
          case GYRO -> gyro;
          case BACKUP -> backup;
          case WHEELS -> null;
        };
    compareWithTheImu(steps, attitude, readings.velocities());
    conditions = conditions(enabled);
    estimator.setConditions(conditions);
    for (Step step : steps.steps()) {
      estimator.addOdometry(step.timestampNanos(), step.motion(), step.yaw(), steps.source());
    }
    watchTerrain(readings.velocities());
    checkImus(gyro, backup);
    gyroDisconnected.set(!gyro.usable() && !backup.usable());
    usingBackup.set(!gyro.usable() && backup.usable());
    Logger.recordOutput("Localization/TiltDegrees", Math.toDegrees(conditions.tiltRadians()));
    Logger.recordOutput(
        "Localization/GroundSlopeDegrees", Math.toDegrees(conditions.groundSlopeRadians()));
    Logger.recordOutput("Localization/CollisionShare", conditions.collisionShare());
    Logger.recordOutput("Localization/SlipMetersPerSecond", conditions.slipMetersPerSecond());
    Logger.recordOutput(
        "Localization/TurnSlipMetersPerSecond", slipCheck.turnSlipMetersPerSecond());
    Logger.recordOutput(
        "Localization/SpeedSlipMetersPerSecond", slipCheck.speedSlipMetersPerSecond());
    Logger.recordOutput(
        "Localization/TurnApartDegrees", Math.toDegrees(slipCheck.turnApartRadians()));
    Logger.recordOutput("Localization/SlideMetersPerSecond", slipCheck.slideMetersPerSecond());
    estimate = estimator.estimate();
  }

  /** Where the cameras send their measurements, and learn where the robot was. */
  public CameraLocalizer forCameras() {
    return estimator;
  }

  /**
   * Applies the loop's measurements, and says whether the estimate is good enough to score from.
   * After the cameras.
   *
   * @return what became of each camera measurement, for the cameras' records
   */
  public List<VisionDecision> update() {
    List<VisionDecision> decisions = estimator.update();
    List<String> landmarks = new ArrayList<>();
    for (LandmarkDecision decision : estimator.landmarkDecisions()) {
      landmarks.add(
          String.format(
              Locale.ROOT,
              "%s: %s, %.2f m across",
              decision.landmark().name(),
              decision.reason(),
              decision.innovationMeters()));
      landmarksAccepted += decision.accepted() ? 1 : 0;
    }
    Logger.recordOutput("Localization/Landmarks", landmarks.toArray(String[]::new));
    Logger.recordOutput("Localization/LandmarksAccepted", landmarksAccepted);
    estimate = estimator.estimate();
    notReadyToScore = LocalizationGate.SCORING.whyNot(estimate);
    List<String> beyond = estimator.camerasWithoutPlaces();
    if (!beyond.isEmpty()) {
      camerasWithoutPlaces.setText(
          "More cameras than the estimator keeps apart: "
              + String.join(", ", beyond)
              + " count each image's mount and tag errors as new, so their images together claim"
              + " more than they know");
    }
    camerasWithoutPlaces.set(!beyond.isEmpty());
    log();
    return decisions;
  }

  /** Where the robot is: cheap, for driving every loop. */
  public Pose2d pose() {
    return estimator.pose();
  }

  /** The estimate, with its uncertainty, mode, and health, as of the last step. */
  public LocalizationEstimate estimate() {
    return estimate;
  }

  /**
   * The drive team put the robot here: on its start before the match, say. Call it after {@link
   * #periodic}, so a turn made in placing it isn't counted twice.
   *
   * @param how sure the placement is
   * @param reason for the log, such as "placed on Center"
   */
  public void place(Pose2d pose, PoseCovariance how, String reason) {
    estimator.reset(new PoseReset(RobotController.getTime(), pose, how, reason));
    estimate = estimator.estimate();
  }

  /** The robot is exactly here, now. */
  public void resetPose(Pose2d pose) {
    place(pose, EXACTLY, "reset");
  }

  /**
   * The robot faces this heading, as sure as the given spread; its position stays. If that's wrong,
   * the cameras' headings will agree against it and the estimate turns back to them.
   */
  public void resetHeading(Rotation2d heading, double stdDevRadians) {
    estimator.resetHeading(heading, stdDevRadians);
    estimate = estimator.estimate();
  }

  /**
   * Whether the estimate is good enough to score from, by {@link LocalizationGate#SCORING}: bind
   * scoring and aiming commands to it.
   */
  public boolean readyToScore() {
    return notReadyToScore == null;
  }

  /** One line for the pit and the drive team: ready to score, or why not. */
  public String scoring() {
    return notReadyToScore == null ? "Ready to score" : "Not ready: " + notReadyToScore;
  }

  /**
   * One line for the pit: what the pose rests on, and how sure it is, to the centimeter and the
   * tenth of a degree. Each number shown moves only once the real one has moved that much from it,
   * so the line doesn't flicker as the uncertainty wanders within a step.
   */
  public String describe() {
    PoseCovariance uncertainty = estimate.covariance();
    shownCentimeters = held(shownCentimeters, 100 * uncertainty.largestPositionStdDev(), 1);
    shownDegrees = held(shownDegrees, Math.toDegrees(uncertainty.headingStdDev()), 0.1);
    String basis =
        switch (estimate.mode()) {
          case INITIALIZING -> "Not placed yet";
          case TRACKING -> "Tracking tags";
          case DEAD_RECKONING -> "Dead reckoning";
          case LOST -> "Lost: recovering";
          case IMU_FAULT -> "Gyro lost";
        };
    return String.format(
        Locale.ROOT, "%s: within %.0f cm, %.1f°", basis, shownCentimeters, shownDegrees);
  }

  /** A number as shown, to a step: it moves only once the value has moved a step from it. */
  private static double held(double shown, double value, double step) {
    return Math.abs(value - shown) < step ? shown : Math.round(value / step) * step;
  }

  /** The robot's pitch, from the IMU: zero without one. Positive is nose down. */
  public double pitchRadians() {
    ImuReadings imu = attitude;
    return imu == null ? 0 : imu.pitchRadians();
  }

  /** The robot's roll, from the IMU: zero without one. */
  public double rollRadians() {
    ImuReadings imu = attitude;
    return imu == null ? 0 : imu.rollRadians();
  }

  /** What turns odometry now: the gyro, the backup IMU, or the wheels. */
  public YawSource yawSource() {
    return yawSource;
  }

  /** What the IMU said about trusting the wheels this loop. */
  public DriveConditions conditions() {
    return conditions;
  }

  /** Where the robot is on raised terrain: level, climbing, or descending. */
  public TerrainCrossing.Phase terrainPhase() {
    return terrain.phase();
  }

  /**
   * How many times the robot has crossed raised terrain: up, over, down, and level again. An auto
   * that drives over a bump waits for this to count up.
   */
  public int terrainCrossings() {
    return terrain.crossings();
  }

  /** How many crests the estimator has taken as landmarks since the robot started. */
  public int landmarksAccepted() {
    return landmarksAccepted;
  }

  /**
   * What the IMU says about trusting the wheels: the robot's tilt, a jolt beyond driving, the
   * wheels sliding over the floor, and whether people might be carrying it. The IMU that turns
   * odometry says (the gyro, or the backup while the gyro is lost); without one, nothing can.
   */
  private DriveConditions conditions(boolean enabled) {
    ImuReadings imu = attitude;
    double tilt =
        imu == null
            ? 0
            // The angle between the robot's up and straight up.
            : Math.acos(Math.min(1, Math.cos(imu.pitchRadians()) * Math.cos(imu.rollRadians())));
    return new DriveConditions(
        tilt,
        imu == null ? 0 : groundSlope(tilt),
        slipCheck.collisionShare(),
        slipCheck.slipMetersPerSecond(),
        !enabled);
  }

  /**
   * Gives the slip check the loop's wheel readings and what the IMU that turns odometry felt
   * ({@link SlipCheck}): whether the wheels slid, and whether the IMU felt a collision. A loop
   * without wheel readings, or with another IMU than the last, starts it over.
   *
   * @param imu the IMU that turns odometry this loop, or null without one
   * @param wheels the robot's velocity as its wheels measure it, robot-relative
   */
  private void compareWithTheImu(Steps steps, @Nullable ImuReadings imu, ChassisVelocities wheels) {
    Felt felt = imu == null ? Felt.NOTHING : driving(imu);
    List<Step> loop = steps.steps();
    if (loop.isEmpty() || imu == null) {
      slipCheck.startOver(felt.forward(), felt.left());
      return;
    }
    Step last = loop.get(loop.size() - 1);
    YawSource source = steps.source();
    if (source == lastStepSource && lastStepNanos < last.timestampNanos()) {
      double wheelTurn = 0;
      for (Step step : loop) {
        wheelTurn += step.motion().dthetaRadians();
      }
      slipCheck.update(
          new SlipCheck.Sample(
              Timestamps.toSeconds(last.timestampNanos() - lastStepNanos),
              wheelTurn,
              MathUtil.angleModulus(last.yaw() - lastStepYaw),
              last.yaw(),
              imu.yawRateRadiansPerSecond(),
              source == YawSource.GYRO ? imus.gyro() : imus.backup(),
              wheels,
              felt.forward(),
              felt.left()));
    } else {
      slipCheck.startOver(felt.forward(), felt.left());
    }
    lastStepNanos = last.timestampNanos();
    lastStepSource = source;
    lastStepYaw = last.yaw();
  }

  /**
   * What an IMU felt of the robot's own motion, along the robot's axes, in m/s²: its reading less
   * gravity's share. Not a number where the IMU can't say.
   */
  private record Felt(double forward, double left) {
    static final Felt NOTHING = new Felt(Double.NaN, Double.NaN);
  }

  /**
   * What the IMU felt of the robot's own motion: its reading less gravity's share, which it feels
   * along the robot's axes as the robot tilts (WPILib's attitude: a positive pitch is nose down, a
   * positive roll the left side up).
   */
  private static Felt driving(ImuReadings imu) {
    double pitch = imu.pitchRadians();
    double roll = imu.rollRadians();
    return new Felt(
        imu.forwardAccelerationMetersPerSecSq() + ImuReadings.STANDARD_GRAVITY * Math.sin(pitch),
        imu.leftAccelerationMetersPerSecSq()
            - ImuReadings.STANDARD_GRAVITY * Math.sin(roll) * Math.cos(pitch));
  }

  /**
   * How much of a tilt the ground explains: on a known bump (where the estimate puts the robot,
   * give or take its wheels' reach and the estimate's doubt), up to its ramps' slope; none on flat
   * ground.
   */
  private double groundSlope(double tilt) {
    if (ground.bumps().isEmpty() || tilt == 0) {
      return 0;
    }
    double doubt = 2 * estimator.estimate().covariance().largestPositionStdDev();
    double reach = Math.min(MAX_GROUND_REACH_METERS, wheelReach + doubt);
    return ground
        .under(pose().getTranslation(), reach)
        .map(bump -> Math.min(tilt, bump.slopeRadians() + GROUND_SLOPE_MARGIN))
        .orElse(0.0);
  }

  /**
   * One wheel reading, as the estimator takes it: when, how the robot moved since, the yaw then.
   */
  private record Step(long timestampNanos, BodyMotion motion, double yaw) {}

  /** A loop's wheel readings as steps, and what turns them. */
  private record Steps(List<Step> steps, YawSource source) {}

  /**
   * Every wheel reading since the last loop, as the robot's motion between readings, with the yaw
   * at each: the gyro's, or while it's lost the backup IMU's (interpolated between its readings),
   * or with neither, none, and the wheels' turning stands in, which the estimator says drifts. The
   * estimator is told which, so it keeps each IMU's scale error apart, and turns the step where the
   * source changes by the wheels, rather than take the two IMUs' different zeros as a turn.
   */
  private Steps steps(List<OdometrySample> wheels, ImuReadings gyro, ImuReadings backup) {
    track(gyroYaws, gyro);
    track(backupYaws, backup);
    YawSource source =
        gyroYaws.hasReadings()
            ? YawSource.GYRO
            : backupYaws.hasReadings() ? YawSource.BACKUP : YawSource.WHEELS;
    YawTrack yaws = source == YawSource.GYRO ? gyroYaws : backupYaws;
    if (!wheels.isEmpty()) {
      yawSource = source;
    }
    List<Step> steps = new ArrayList<>(wheels.size());
    for (OdometrySample reading : wheels) {
      OdometrySample previous = lastReading;
      BodyMotion motion =
          previous == null ? new BodyMotion(0, 0, 0, 0) : kinematics.motion(previous, reading);
      double yaw = source == YawSource.WHEELS ? Double.NaN : yaws.at(reading.timestampNanos());
      steps.add(new Step(reading.timestampNanos(), motion, yaw));
      lastReading = reading;
    }
    return new Steps(steps, source);
  }

  /** Takes an IMU's yaw readings this loop, or forgets them all while it's lost. */
  private static void track(YawTrack yaws, ImuReadings imu) {
    int count = imu.yawTimesNanos().size();
    if (!imu.usable() || count == 0) {
      yaws.clear();
      return;
    }
    long[] times = new long[count];
    double[] values = new double[count];
    for (int i = 0; i < count; i++) {
      times[i] = imu.yawTimesNanos().get(i);
      values[i] = imu.yawsRadians().get(i);
    }
    yaws.update(times, values);
  }

  /**
   * Follows the robot over raised terrain (the slope along the way it moves, and its tilt), and
   * takes a crest it passed as a landmark: the robot was on the bump's crest line then.
   */
  private void watchTerrain(ChassisVelocities moving) {
    boolean still = Math.hypot(moving.vx, moving.vy) < MOVING;
    double way = Math.atan2(moving.vy, moving.vx);
    double climb = still ? 0 : TerrainCrossing.climb(pitchRadians(), rollRadians(), way);
    double fieldWay = still ? Double.NaN : way + pose().getRotation().getRadians();
    terrain.update(RobotController.getTime(), climb, conditions.tiltRadians(), fieldWay);
    terrain
        .crest()
        .ifPresent(
            crest -> {
              Logger.recordOutput(
                  "Localization/CrestSeconds", Timestamps.toSeconds(crest.timestampNanos()));
              estimator
                  .poseAt(crest.timestampNanos())
                  .flatMap(then -> crestLandmarks.landmark(crest, then.getTranslation()))
                  .ifPresent(estimator::addLandmark);
            });
    Logger.recordOutput("Localization/ClimbDegrees", Math.toDegrees(climb));
    Logger.recordOutput("Localization/Terrain", terrain.phase().name());
    Logger.recordOutput("Localization/TerrainCrossings", terrain.crossings());
  }

  /**
   * While both report, compares the gyro with the backup IMU: a sustained disagreement means one is
   * failing, loose, or needs its scale calibrated. It raises the question rather than silently
   * picking one.
   */
  private void checkImus(ImuReadings gyro, ImuReadings backup) {
    if (gyro.usable() && backup.usable()) {
      crossCheck.update(RobotController.getTime(), gyro.yawRadians(), backup.yawRadians());
    } else {
      crossCheck.reset();
    }
    double disagreement = Math.toDegrees(crossCheck.disagreementRadians());
    if (crossCheck.disagrees()) {
      imusDisagree.setText(
          String.format(
              Locale.ROOT,
              "The gyro and the backup IMU disagree by %.0f° over the last second: one is failing,"
                  + " loose, or needs its scale calibrated",
              Math.abs(disagreement)));
    }
    imusDisagree.set(crossCheck.disagrees());
    Logger.recordOutput("Localization/ImuDisagreementDegrees", disagreement);
    Logger.recordOutput("Localization/YawSource", yawSource.name());
  }

  /** Logs the estimate and what it rests on. */
  private void log() {
    PoseCovariance uncertainty = estimate.covariance();
    LocalizationEstimate.Health health = estimate.health();
    Logger.recordOutput("Localization/Pose", estimate.pose());
    Logger.recordOutput("Localization/Mode", estimate.mode().name());
    Logger.recordOutput("Localization/PositionStdDevMeters", uncertainty.largestPositionStdDev());
    Logger.recordOutput(
        "Localization/HeadingStdDevDegrees", Math.toDegrees(uncertainty.headingStdDev()));
    Logger.recordOutput("Localization/ResidualMeters", health.residualMeters());
    Logger.recordOutput("Localization/SuspectTravelMeters", health.suspectTravelMeters());
    Logger.recordOutput("Localization/RecentDisagreementMeters", health.recentDisagreementMeters());
    Logger.recordOutput("Localization/Recoveries", health.recoveries());
    Logger.recordOutput("Localization/ReadyToScore", notReadyToScore == null);
    Logger.recordOutput(
        "Localization/NotReadyBecause", notReadyToScore == null ? "" : notReadyToScore);
  }

  boolean isGyroDisconnectedAlertActive() {
    return gyroDisconnected.get();
  }

  boolean isUsingBackupAlertActive() {
    return usingBackup.get();
  }

  boolean isCamerasWithoutPlacesAlertActive() {
    return camerasWithoutPlaces.get();
  }

  boolean isImuDisagreementAlertActive() {
    return imusDisagree.get();
  }

  @Override
  public void close() {
    gyroDisconnected.close();
    usingBackup.close();
    imusDisagree.close();
    camerasWithoutPlaces.close();
  }
}
