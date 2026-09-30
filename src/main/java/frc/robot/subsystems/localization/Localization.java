package frc.robot.subsystems.localization;

import frc.robot.lib.field.Terrain;
import frc.robot.lib.localization.BodyMotion;
import frc.robot.lib.localization.CameraLocalizer;
import frc.robot.lib.localization.CrestLandmarks;
import frc.robot.lib.localization.DriveConditions;
import frc.robot.lib.localization.DriveKinematicsModel;
import frc.robot.lib.localization.DrivetrainReadings;
import frc.robot.lib.localization.ImuCrossCheck;
import frc.robot.lib.localization.ImuReadings;
import frc.robot.lib.localization.LandmarkDecision;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.localization.LocalizationGate;
import frc.robot.lib.localization.OdometrySample;
import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.PoseEstimator;
import frc.robot.lib.localization.PoseReset;
import frc.robot.lib.localization.TerrainCrossing;
import frc.robot.lib.localization.Timestamps;
import frc.robot.lib.localization.VisionDecision;
import frc.robot.lib.localization.YawTrack;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
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
 *       backup IMU while the gyro is lost, or the wheels), weighs the wheels by what the IMU and
 *       the motors say (a tilt, a jolt, a grip at its limit), feeds every wheel reading to the
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
  /** What turns odometry: the gyro, the backup IMU while the gyro is lost, or the wheels. */
  public enum YawSource {
    GYRO,
    BACKUP,
    WHEELS
  }

  /**
   * A jolt beyond this, in m/s², is a collision (6800's threshold): hard driving stays near the
   * wheels' grip, about 1 g.
   */
  private static final double COLLISION_ACCELERATION = 20.0;

  /** Below this speed, in m/s, the robot has no direction of travel to measure a slope along. */
  private static final double MOVING = 0.25;

  /**
   * On a known bump, a tilt up to this much steeper than its ramps is still the ground's: bumpers,
   * suspension, and the IMU's mount.
   */
  private static final double GROUND_SLOPE_MARGIN = Math.toRadians(3);

  /** How far off a bump's footprint the estimate may put the robot and still be on it. */
  private static final double MAX_GROUND_REACH_METERS = 1.0;

  /** A crest places the robot's center within about this far across it (to measure: M7). */
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

  /**
   * @param estimator the pose estimator, which this owns from now on
   * @param kinematics how the drivetrain's wheels move the robot
   * @param terrain the field's raised terrain: a tilt it explains keeps the wheels trusted, and its
   *     crests are landmarks
   * @param wheelReachMeters how far the wheels reach from the robot's center
   */
  public Localization(
      PoseEstimator estimator,
      DriveKinematicsModel kinematics,
      Terrain terrain,
      double wheelReachMeters) {
    this.estimator = estimator;
    this.kinematics = kinematics;
    this.ground = terrain;
    this.wheelReach = wheelReachMeters;
    crestLandmarks = new CrestLandmarks(terrain, CREST_SPREAD_METERS, CREST_REACH_METERS);
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
    attitude = gyro.usable() ? gyro : backup.usable() ? backup : null;
    conditions = conditions(readings, enabled);
    estimator.setConditions(conditions);
    addOdometry(readings.wheels(), gyro, backup);
    watchTerrain(readings.velocities());
    checkImus(gyro, backup);
    gyroDisconnected.set(!gyro.usable() && !backup.usable());
    usingBackup.set(!gyro.usable() && backup.usable());
    Logger.recordOutput("Localization/TiltDegrees", Math.toDegrees(conditions.tiltRadians()));
    Logger.recordOutput(
        "Localization/GroundSlopeDegrees", Math.toDegrees(conditions.groundSlopeRadians()));
    Logger.recordOutput("Localization/Collision", conditions.collision());
    Logger.recordOutput("Localization/TractionLimited", conditions.tractionLimited());
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

  /** One line for the pit: what the pose rests on, and how sure it is. */
  public String describe() {
    PoseCovariance uncertainty = estimate.covariance();
    String basis =
        switch (estimate.mode()) {
          case INITIALIZING -> "Not placed yet";
          case TRACKING -> "Tracking tags";
          case DEAD_RECKONING -> "Dead reckoning";
          case LOST -> "Lost: recovering";
          case IMU_FAULT -> "Gyro lost";
        };
    return String.format(
        Locale.ROOT,
        "%s: within %.0f cm, %.1f°",
        basis,
        100 * uncertainty.largestPositionStdDev(),
        Math.toDegrees(uncertainty.headingStdDev()));
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

  /** What the IMU and the motors said about trusting the wheels this loop. */
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
   * What the IMU and the motors say about trusting the wheels: the robot's tilt, a jolt beyond
   * driving, wheels at the edge of their grip, and whether people might be carrying it. The gyro's
   * IMU says, or the backup's while the gyro is lost; without either, only the motors can say.
   */
  private DriveConditions conditions(DrivetrainReadings readings, boolean enabled) {
    ImuReadings imu = attitude;
    if (imu == null) {
      return new DriveConditions(0, 0, false, readings.tractionLimited(), !enabled);
    }
    // The angle between the robot's up and straight up.
    double tilt =
        Math.acos(Math.min(1, Math.cos(imu.pitchRadians()) * Math.cos(imu.rollRadians())));
    boolean collision =
        Math.hypot(imu.forwardAccelerationMetersPerSecSq(), imu.leftAccelerationMetersPerSecSq())
            > COLLISION_ACCELERATION;
    return new DriveConditions(
        tilt, groundSlope(tilt), collision, readings.tractionLimited(), !enabled);
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
   * Feeds every wheel reading since the last loop to the estimator, as the robot's motion between
   * readings, with the yaw at each: the gyro's, or while it's lost the backup IMU's (interpolated
   * between its readings), or with neither, the wheels' turning, which the estimator says drifts.
   * The step where the source changes turns by the wheels too, so the estimator starts over from
   * the new source's reading instead of taking the two IMUs' different zeros as a turn.
   */
  private void addOdometry(List<OdometrySample> wheels, ImuReadings gyro, ImuReadings backup) {
    track(gyroYaws, gyro);
    track(backupYaws, backup);
    YawSource source =
        gyroYaws.hasReadings()
            ? YawSource.GYRO
            : backupYaws.hasReadings() ? YawSource.BACKUP : YawSource.WHEELS;
    YawTrack yaws = source == YawSource.GYRO ? gyroYaws : backupYaws;
    boolean switching = !wheels.isEmpty() && source != yawSource;
    if (!wheels.isEmpty()) {
      yawSource = source;
    }
    for (int k = 0; k < wheels.size(); k++) {
      OdometrySample reading = wheels.get(k);
      OdometrySample previous = lastReading;
      BodyMotion motion =
          previous == null ? new BodyMotion(0, 0, 0, 0) : kinematics.motion(previous, reading);
      boolean turnedByImu = source != YawSource.WHEELS && !(switching && k == 0);
      double yaw = source == YawSource.WHEELS ? 0 : yaws.at(reading.timestampNanos());
      estimator.addOdometry(reading.timestampNanos(), motion, yaw, turnedByImu);
      lastReading = reading;
    }
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

  boolean isImuDisagreementAlertActive() {
    return imusDisagree.get();
  }

  @Override
  public void close() {
    gyroDisconnected.close();
    usingBackup.close();
    imusDisagree.close();
  }
}
