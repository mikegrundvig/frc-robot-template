package com.michaelgrundvig.frc.localization;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;

/**
 * Where the robot is: localization as one service, told what the robot's sensors read and asked
 * what it makes of them. It owns the {@link PoseEstimator} and everything around it: which IMU
 * turns odometry, how far the wheels are trusted, the robot's crossings of raised terrain and the
 * crests it takes as landmarks, and solving each camera image as the robot was when it was taken.
 *
 * <p>Each loop, on the robot loop's thread, in this order:
 *
 * <ol>
 *   <li>{@link #drivetrain}, with every wheel reading since the last loop and both IMUs;
 *   <li>{@link #place}, if the drive team placed the robot this loop;
 *   <li>{@link #mountDoubt} for each camera, and {@link #camera} for each image that arrived,
 *       stamped when it was taken;
 *   <li>{@link #update}: the one call that applies the loop's measurements, and says what became of
 *       each.
 * </ol>
 *
 * <p>Pure, like the rest of this package: it reads no clock (each loop's time comes in a {@link
 * LoopContext}), keeps no static state, and knows nothing of the robot program, the simulator, or
 * Pitwall. Its outputs are a function of its inputs and its configuration, so a replay of a logged
 * match reproduces every one. Use it from one thread.
 *
 * <p>Independent of the drivetrain: its wheels come in as {@link OdometrySample}s, through its
 * {@link DriveKinematicsModel}, so swerve, tank, or any other localizes the same way.
 *
 * <p>Each image is solved against the field's tags through one {@link VisionPipeline}, and with its
 * camera's mount from the robot's definition, looked up by name in {@link #solve}: the two places a
 * venue's surveyed tags and a camera that rides a joint would change what an image is solved with.
 */
public final class LocalizationService {
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

  /** How far back "lately" reaches, for how fast the robot has turned: 254 used 0.3 s. */
  private static final double RECENT_TURNING_SECONDS = 0.3;

  /** How much of the robot's attitude is kept, for each image's capture time. */
  private static final double ATTITUDE_SECONDS = 0.5;

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

  /**
   * Where the IMUs are mounted, and whether each one's accelerometer reads along the robot's axes.
   */
  private final Imus imus;

  /** The last wheel reading's time, last loop, and which IMU read its yaw, and what it read. */
  private long lastStepNanos = Long.MIN_VALUE;

  private YawSource lastStepSource = YawSource.WHEELS;
  private double lastStepYaw = Double.NaN;
  private YawSource yawSource = YawSource.GYRO;

  /** The previous wheel reading, which the next one's motion is measured from. */
  private @Nullable OdometrySample lastReading;

  /** The IMU the robot's attitude comes from this loop: null without one. */
  private @Nullable ImuReadings attitude;

  /** What the IMU and the motors said about trusting the wheels this loop. */
  private DriveConditions conditions = DriveConditions.NORMAL;

  /** How far the robot climbed along its way this loop, in radians. */
  private double climb;

  /** The crest the robot crossed this loop, if it crossed one. */
  private Optional<TerrainCrossing.Crest> crest = Optional.empty();

  private LocalizationEstimate estimate;
  private int landmarksAccepted;

  // The cameras: each one's mount and the field's tags to solve its images with, and how far the
  // pit has tuned them to be trusted.
  private final VisionPipeline pipeline;
  private final Map<String, CameraConfig> cameras = new LinkedHashMap<>();
  private VisionConfig vision = VisionConfig.DEFAULTS;

  /** What the robot last said of each camera's mount: its doubt, and the distance judged at. */
  private final Map<String, double[]> mountDoubts = new HashMap<>();

  /** Half a second of the robot's attitude, for each image's capture time. */
  private final AttitudeHistory attitudes;

  /** How fast the robot turned in each of the latest loops, for how fast it has turned lately. */
  private final double[] recentYawRates;

  private int yawRateLoop;

  /**
   * How the robot moves this loop, and how sure its heading is: what this loop's images are solved
   * with. Taken once a loop, by its first image or its update, so it includes a placement.
   */
  private @Nullable Motion motion;

  /**
   * @param speedMetersPerSecond how fast the robot moves over the field
   * @param recentYawRateRadiansPerSecond the fastest it turned lately, either way
   * @param headingStdDevRadians how sure the estimate is of its heading
   */
  private record Motion(
      double speedMetersPerSecond,
      double recentYawRateRadiansPerSecond,
      double headingStdDevRadians) {}

  /**
   * Localization for a robot on a field: its estimator starts with the robot not placed yet, so
   * anywhere on the field, and the first camera measurement (or placing it) wins.
   *
   * @param field the field: its tags, which images are solved against; its terrain, which a tilt
   *     the ground explains and a crest crossed come from; and its edges, past which no measurement
   *     can put the robot
   * @param robot what the robot is: its wheels, its IMUs, and its cameras' mounts
   * @param config how the estimator weighs its sources
   * @param loopSeconds how long one robot loop is
   */
  public LocalizationService(
      FieldModel field, RobotDefinition robot, EstimatorConfig config, double loopSeconds) {
    this(
        new PoseEstimator(config, field::contains, notPlacedYet(field)),
        field,
        field.terrain(),
        robot,
        loopSeconds);
  }

  /**
   * Localization from its parts: an estimator already made (in a test, one that starts where the
   * robot stands), and a terrain other than the field's own.
   *
   * @param estimator the pose estimator, which this owns from now on
   * @param field the field whose tags images are solved against
   * @param terrain the ground the robot drives on: a tilt it explains keeps the wheels trusted, and
   *     its crests are landmarks
   * @param robot what the robot is: its wheels, its IMUs, and its cameras' mounts
   * @param loopSeconds how long one robot loop is
   */
  public LocalizationService(
      PoseEstimator estimator,
      FieldModel field,
      Terrain terrain,
      RobotDefinition robot,
      double loopSeconds) {
    if (!(loopSeconds > 0) || Double.isInfinite(loopSeconds)) {
      throw new IllegalArgumentException("a loop takes some time: " + loopSeconds);
    }
    this.estimator = estimator;
    Translation2d[] wheels = robot.moduleTranslations();
    kinematics = new SwerveKinematicsModel(wheels);
    double reach = 0;
    for (Translation2d wheel : wheels) {
      reach = Math.max(reach, wheel.getNorm());
    }
    wheelReach = reach;
    ground = terrain;
    crestLandmarks = new CrestLandmarks(terrain, CREST_SPREAD_METERS, CREST_REACH_METERS);
    imus = robot.imus();
    slipCheck =
        new SlipCheck(
            SlipCheck.Settings.DEFAULTS,
            reach,
            robot.drivetrain().modules().wheelCoefficientOfFriction()
                * ImuReadings.STANDARD_GRAVITY);
    pipeline = new VisionPipeline(field);
    for (CameraConfig camera : robot.cameras()) {
      cameras.put(camera.name(), camera);
    }
    attitudes = new AttitudeHistory(loops(ATTITUDE_SECONDS, loopSeconds));
    recentYawRates = new double[loops(RECENT_TURNING_SECONDS, loopSeconds)];
    estimate = estimator.estimate();
  }

  /**
   * Until the robot is placed it could be anywhere: the middle of the field stands in, with an
   * uncertainty as big as the field.
   */
  private static PoseReset notPlacedYet(FieldModel field) {
    return new PoseReset(
        0,
        new Pose2d(field.length() / 2, field.width() / 2, Rotation2d.ZERO),
        PoseCovariance.fromStdDevs(field.length(), field.width(), Math.PI),
        "not placed yet");
  }

  /** How many loops a stretch of time takes, to the nearest loop, and at least two. */
  private static int loops(double seconds, double loopSeconds) {
    return Math.max(2, (int) Math.round(seconds / loopSeconds));
  }

  /**
   * One loop of the drivetrain's readings: after it reads its sensors, and before the cameras.
   * Picks what turns odometry (the gyro, the backup IMU while the gyro is lost, or the wheels),
   * weighs the wheels by what the IMU says (a tilt, a jolt, the wheels sliding: {@link SlipCheck}),
   * feeds every wheel reading to the estimator, follows the robot over raised terrain, and takes a
   * crest it crossed as a landmark.
   */
  public void drivetrain(DrivetrainReadings readings, LoopContext loop) {
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
    conditions = conditions(loop.enabled());
    estimator.setConditions(conditions);
    for (Step step : steps.steps()) {
      estimator.addOdometry(step.timestampNanos(), step.motion(), step.yaw(), steps.source());
    }
    watchTerrain(readings.velocities(), loop.nowNanos());
    checkImus(gyro, backup, loop.nowNanos());
    estimate = estimator.estimate();
    motion = null;
  }

  /**
   * The drive team put the robot here (on its start before the match, say), or a driver's "against
   * the wall here". After {@link #drivetrain}, so a turn made in placing it isn't counted twice.
   */
  public void place(PoseReset reset) {
    estimator.reset(reset);
    estimate = estimator.estimate();
  }

  /**
   * The robot faces this heading, as sure as the given spread; its position stays. If that's wrong,
   * the cameras' headings will agree against it and the estimate turns back to them.
   */
  public void resetHeading(Rotation2d heading, double stdDevRadians) {
    estimator.resetHeading(heading, stdDevRadians);
    estimate = estimator.estimate();
  }

  /** A landmark the robot crossed, applied at its time on the next {@link #update}. */
  public void landmark(LandmarkEvent event) {
    estimator.addLandmark(event);
  }

  /**
   * How far the cameras' images are trusted, from the next image on: as committed, or as the pit
   * tuned it. Until told, {@link VisionConfig#DEFAULTS}.
   */
  public void vision(VisionConfig settings) {
    vision = settings;
  }

  /**
   * How far a camera's mount may have moved since it was calibrated ({@link MountCheck}), as of
   * now: its images from now on are widened by it, and with {@link
   * EstimatorConfig.HeadingCorrection#HEADING_MEASUREMENTS_AND_MOUNT_DOUBT} the estimator takes it
   * as part of the camera's repeated errors. Before the cameras' images, each loop.
   *
   * @param doubtMeters how far its measurements may be off from where its mount moved
   * @param distanceMeters how far its tags were, on average, when that was judged; NaN until then
   */
  public void mountDoubt(String camera, double doubtMeters, double distanceMeters) {
    estimator.mountDoubt(camera, doubtMeters, distanceMeters);
    mountDoubts.put(camera, new double[] {doubtMeters, distanceMeters});
  }

  /** One camera image, from every tag it saw: see {@link #camera(TagDetections, TagAllowlist)}. */
  public VisionResult camera(TagDetections frame) {
    return camera(frame, null);
  }

  /**
   * One camera image, stamped when it was taken: solved as the robot was then ({@link #capture}),
   * widened by its camera's mount's doubt ({@link #mountDoubt}), and its measurement, if it makes
   * one, applied at that moment on the next {@link #update}.
   *
   * @param onlyTags the tags the current task trusts ({@link TagAllowlist}); null for every tag
   * @return what the image made: a measurement, or why not
   * @throws IllegalArgumentException if the robot's definition has no camera by the frame's name
   */
  public VisionResult camera(TagDetections frame, @Nullable TagAllowlist onlyTags) {
    VisionResult result = solve(frame, onlyTags);
    VisionMeasurement measurement = result.measurement();
    if (measurement != null) {
      camera(measurement);
    }
    return result;
  }

  /**
   * What {@link #camera(TagDetections, TagAllowlist)} would make of an image, without applying it:
   * for an image the robot holds back (from a camera being calibrated, or while autonomous starts),
   * whose outcome is still logged.
   */
  public VisionResult solve(TagDetections frame, @Nullable TagAllowlist onlyTags) {
    CameraConfig camera = cameras.get(frame.camera());
    if (camera == null) {
      throw new IllegalArgumentException(
          "the robot's definition has no camera named " + frame.camera());
    }
    double[] doubt = mountDoubts.get(frame.camera());
    return pipeline.process(
        frame,
        camera,
        capture(frame.timestampNanos()),
        vision,
        new VisionPipeline.Trust(doubt == null ? 0 : doubt[0], onlyTags));
  }

  /**
   * A camera measurement solved elsewhere (a Limelight's own pose, say, or a test's), applied at
   * its capture time on the next {@link #update}.
   */
  public void camera(VisionMeasurement measurement) {
    estimator.addVision(measurement);
  }

  /**
   * The robot when an image was taken, as the vision pipeline solves it: its heading then, if the
   * estimate reaches back that far, its tilt then, and how it moves this loop.
   */
  public CaptureContext capture(long timestampNanos) {
    Motion now = motion();
    Rotation2d heading = estimator.poseAt(timestampNanos).map(Pose2d::getRotation).orElse(null);
    AttitudeHistory.Attitude tilted = attitudes.at(timestampNanos);
    return new CaptureContext(
        heading,
        now.headingStdDevRadians(),
        tilted.pitchRadians(),
        tilted.rollRadians(),
        now.speedMetersPerSecond(),
        now.recentYawRateRadiansPerSecond());
  }

  /**
   * Applies the loop's measurements and landmarks, each at its capture time, and says what became
   * of each. After the cameras; once a loop.
   *
   * @param loop the loop: the one {@link #drivetrain} was given
   */
  public LocalizationOutput update(LoopContext loop) {
    // This loop's images are solved as of now, whether any came or not: the record of the robot's
    // turning and tilt takes one entry a loop.
    motion();
    List<VisionDecision> decisions = estimator.update();
    List<LandmarkDecision> landmarks = estimator.landmarkDecisions();
    for (LandmarkDecision decision : landmarks) {
      landmarksAccepted += decision.accepted() ? 1 : 0;
    }
    estimate = estimator.estimate();
    List<FrameVerdict> frames = new ArrayList<>(decisions.size());
    for (VisionDecision decision : decisions) {
      frames.add(FrameVerdict.of(decision));
    }
    return new LocalizationOutput(estimate, integrity(), frames, landmarks);
  }

  /** Where the robot is: cheap, for driving every loop. */
  public Pose2d pose() {
    return estimator.pose();
  }

  /** Where the robot was at a moment, if the estimate reaches back that far. */
  public Optional<Pose2d> poseAt(long timestampNanos) {
    return estimator.poseAt(timestampNanos);
  }

  /** The estimate, with its uncertainty, mode, and health, as of the last step. */
  public LocalizationEstimate estimate() {
    return estimate;
  }

  /** A pose on the field in 3D: as high as the ground under it, tilted as the IMU says now. */
  public Pose3d pose3d(Pose2d pose) {
    return new Pose3d(
        pose.getX(),
        pose.getY(),
        ground.heightAt(pose.getTranslation()),
        new Rotation3d(rollRadians(), pitchRadians(), pose.getRotation().getRadians()));
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

  /** How far the wheels are trusted this loop, and why. */
  public WheelTrust wheels() {
    return new WheelTrust(
        conditions,
        slipCheck.turnSlipMetersPerSecond(),
        slipCheck.speedSlipMetersPerSecond(),
        slipCheck.turnApartRadians(),
        slipCheck.slideMetersPerSecond(),
        slipCheck.unexplainedMetersPerSecSq(),
        slipCheck.biasMetersPerSecSq(),
        slipCheck.leaksGravity());
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

  /** How far the robot climbed along the way it moved this loop: zero standing still. */
  public double climbRadians() {
    return climb;
  }

  /** The crest of raised terrain the robot crossed this loop, if it crossed one. */
  public Optional<TerrainCrossing.Crest> crest() {
    return crest;
  }

  /** How far the gyro and the backup IMU have turned apart lately, while both report. */
  public double imuDisagreementRadians() {
    return crossCheck.disagreementRadians();
  }

  /**
   * Whether the gyro and the backup IMU disagree for long: one is failing, loose, or needs its
   * scale calibrated.
   */
  public boolean imusDisagree() {
    return crossCheck.disagrees();
  }

  /** How many crests the estimator has taken as landmarks since the robot started. */
  public int landmarksAccepted() {
    return landmarksAccepted;
  }

  /**
   * This loop's images' motion: taken by the first image, or the update, so it includes the loop's
   * placement, and the robot's attitude and turning are recorded once a loop.
   */
  private Motion motion() {
    Motion now = motion;
    if (now != null) {
      return now;
    }
    attitudes.add(estimate.timestampNanos(), pitchRadians(), rollRadians());
    recentYawRates[yawRateLoop] = Math.abs(estimate.yawRateRadiansPerSecond());
    yawRateLoop = (yawRateLoop + 1) % recentYawRates.length;
    double recentYawRate = 0;
    for (double rate : recentYawRates) {
      recentYawRate = Math.max(recentYawRate, rate);
    }
    now =
        new Motion(
            estimate.fieldVelocity().getNorm(),
            recentYawRate,
            estimate.covariance().headingStdDev());
    motion = now;
    return now;
  }

  /**
   * How far each source is trusted: each camera the estimator has heard from, in the order it first
   * did, then the robot's others; and the wheels.
   */
  private Integrity integrity() {
    Map<String, CameraTrust> trust = new LinkedHashMap<>();
    List<String> beyond = estimator.camerasWithoutPlaces();
    for (String camera : estimator.cameras()) {
      trust.put(camera, trust(camera, !beyond.contains(camera)));
    }
    for (String camera : cameras.keySet()) {
      trust.putIfAbsent(camera, trust(camera, true));
    }
    for (String camera : mountDoubts.keySet()) {
      trust.putIfAbsent(camera, trust(camera, true));
    }
    return new Integrity(trust, wheels(), Optional.empty(), Set.of());
  }

  private CameraTrust trust(String camera, boolean keptApart) {
    double[] doubt = mountDoubts.get(camera);
    return doubt == null
        ? new CameraTrust(0, Double.NaN, keptApart)
        : new CameraTrust(doubt[0], doubt[1], keptApart);
  }

  /**
   * What the IMU says about trusting the wheels: the robot's tilt, a jolt beyond driving, the
   * wheels sliding over the floor, and whether people might be carrying it. The IMU that turns
   * odometry says (the gyro, or the backup while the gyro is lost); without one, nothing can.
   */
  private DriveConditions conditions(boolean enabled) {
    ImuReadings imu = attitude;
    double tilt = imu == null ? 0 : tilt(imu);
    return new DriveConditions(
        tilt,
        imu == null ? 0 : groundSlope(tilt),
        slipCheck.collisionShare(),
        slipCheck.slipMetersPerSecond(),
        !enabled);
  }

  /** How far the robot is tilted from level, by an IMU's attitude: its up against straight up. */
  private static double tilt(ImuReadings imu) {
    return Math.acos(Math.min(1, Math.cos(imu.pitchRadians()) * Math.cos(imu.rollRadians())));
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
      ImuMount mount = source == YawSource.GYRO ? imus.gyro() : imus.backup();
      slipCheck.update(
          new SlipCheck.Sample(
              Timestamps.toSeconds(last.timestampNanos() - lastStepNanos),
              wheelTurn,
              MathUtil.angleModulus(last.yaw() - lastStepYaw),
              last.yaw(),
              imu.yawRateRadiansPerSecond(),
              mount.position(),
              mount.accelerometerChecked(),
              wheels,
              felt.forward(),
              felt.left(),
              tilt(imu)));
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
  private void watchTerrain(ChassisVelocities moving, long nowNanos) {
    boolean still = Math.hypot(moving.vx, moving.vy) < MOVING;
    double way = Math.atan2(moving.vy, moving.vx);
    climb = still ? 0 : TerrainCrossing.climb(pitchRadians(), rollRadians(), way);
    double fieldWay = still ? Double.NaN : way + pose().getRotation().getRadians();
    terrain.update(nowNanos, climb, conditions.tiltRadians(), fieldWay);
    crest = terrain.crest();
    crest.ifPresent(
        crossed ->
            estimator
                .poseAt(crossed.timestampNanos())
                .flatMap(then -> crestLandmarks.landmark(crossed, then.getTranslation()))
                .ifPresent(estimator::addLandmark));
  }

  /**
   * While both report, compares the gyro with the backup IMU: a sustained disagreement means one is
   * failing, loose, or needs its scale calibrated. It raises the question rather than silently
   * picking one.
   */
  private void checkImus(ImuReadings gyro, ImuReadings backup, long nowNanos) {
    if (gyro.usable() && backup.usable()) {
      crossCheck.update(nowNanos, gyro.yawRadians(), backup.yawRadians());
    } else {
      crossCheck.reset();
    }
  }
}
