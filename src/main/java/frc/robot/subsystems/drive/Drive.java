package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import frc.robot.lib.config.Body;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.field.Terrain;
import frc.robot.lib.localization.BodyMotion;
import frc.robot.lib.localization.DriveConditions;
import frc.robot.lib.localization.ImuCrossCheck;
import frc.robot.lib.localization.OdometrySample;
import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.PoseEstimator;
import frc.robot.lib.localization.PoseReset;
import frc.robot.lib.localization.SwerveKinematicsModel;
import frc.robot.lib.localization.TerrainCrossing;
import frc.robot.lib.localization.Timestamps;
import frc.robot.lib.localization.YawTrack;
import java.util.Arrays;
import java.util.Locale;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.RobotController;
import org.wpilib.util.Alert;

/**
 * The swerve drivetrain: four {@link Module}s and a gyro, whose high-rate readings it turns into
 * odometry for the pose estimator.
 *
 * <p>Call {@link #periodic(boolean)} once per loop, before the command scheduler runs. Driving
 * commands call {@link #runVelocity(ChassisVelocities)} every loop they are active.
 *
 * <p>The drivetrain's configuration is read every loop, so values tuned while the robot runs take
 * effect at once: new gains and current limits go to the motor controllers the loop they change.
 *
 * <p>Hardware comes in through the IO layer: {@link PhoenixDrivetrain} on a CTRE robot, {@link
 * DriveSimulation} in the simulator, the log in a replay. A stand-in until AdvantageKit's 2027
 * swerve template ships: see docs/stand-ins.md.
 */
public class Drive implements Mechanism, AutoCloseable {
  private static final double LOOP_PERIOD_S = 0.02;
  private static final double STICK_DEADBAND = 0.1;

  /**
   * A jolt beyond this, in m/s², is a collision (6800's threshold): hard driving stays near the
   * wheels' grip, about 1 g.
   */
  private static final double COLLISION_ACCELERATION = 20.0;

  /** A wheel this far short of its commanded speed, in m/s, is at the edge of its grip. */
  private static final double SPEED_DEFICIT = 0.5;

  /** Below this speed, in m/s, the robot has no direction of travel to measure a slope along. */
  private static final double MOVING = 0.25;

  /**
   * On a known bump, a tilt up to this much steeper than its ramps is still the ground's: bumpers,
   * suspension, and the IMU's mount.
   */
  private static final double GROUND_SLOPE_MARGIN = Math.toRadians(3);

  /** How far off a bump's footprint the estimate may put the robot and still be on it. */
  private static final double MAX_GROUND_REACH_METERS = 1.0;

  /** A reset to "exactly here": a millimeter and a milliradian. */
  private static final PoseCovariance EXACTLY = PoseCovariance.fromStdDevs(0.001, 0.001, 0.001);

  private final Supplier<SwerveConfig> config;
  private final Scheduler scheduler;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final GyroIO backupGyroIO;
  private final GyroIOInputsAutoLogged backupInputs = new GyroIOInputsAutoLogged();
  private final YawTrack gyroYaws = new YawTrack();
  private final YawTrack backupYaws = new YawTrack();

  /**
   * The gyro and the backup IMU should turn alike: over a second, within 3° and 5% of the turn
   * (their scales differ a little).
   */
  private final ImuCrossCheck crossCheck = new ImuCrossCheck(1.0, Math.toRadians(3), 0.05);

  /** What turns odometry: the gyro, the backup IMU while the gyro is lost, or the wheels. */
  public enum YawSource {
    GYRO,
    BACKUP,
    WHEELS
  }

  private YawSource yawSource = YawSource.GYRO;
  private final Module[] modules = new Module[4];
  private final SwerveDriveKinematics kinematics;
  private final SwerveKinematicsModel odometry;
  private final PoseEstimator estimator;
  private final Lock odometryLock;

  /** The configuration the motor controllers have, and the limits taken from it. */
  private SwerveConfig applied;

  private double maxLinearSpeedMps;
  private double maxAngularSpeedRadPerSec;
  private double skewLookaheadSec;

  /** The previous odometry reading, which the next one's motion is measured from. */
  private @Nullable OdometrySample lastReading;

  /** What the IMU and the motors said about trusting the wheels this loop. */
  private DriveConditions conditions = DriveConditions.NORMAL;

  /** The field's bumps, for telling a tilt the ground explains from one it doesn't. */
  private Terrain ground = Terrain.FLAT;

  /** How far the wheels reach from the robot's center. */
  private final double wheelReach;

  /**
   * Crossings of raised terrain, from the IMU: 1678's thresholds (10° steep, 5° level), and a
   * second and a half to cross a flat top.
   */
  private final TerrainCrossing terrain =
      new TerrainCrossing(Math.toRadians(10), Math.toRadians(5), 0.1, 1.5);

  private final Alert gyroDisconnectedAlert;
  private final Alert usingBackupAlert;
  private final Alert imusDisagreeAlert;
  private final Alert[] moduleDisconnectedAlerts = new Alert[4];

  /**
   * @param name alert group name: "Drive" on the robot, another name for a second instance
   * @param body the robot's frame, which places the wheels
   * @param config the drivetrain's configuration, read every loop so tuned values take effect
   * @param estimator where odometry goes, and where the robot's pose comes from
   */
  public Drive(
      String name,
      GyroIO gyroIO,
      ModuleIO[] moduleIOs,
      Body body,
      Supplier<SwerveConfig> config,
      Scheduler scheduler,
      PoseEstimator estimator) {
    this(
        name,
        gyroIO,
        new GyroIO() {},
        moduleIOs,
        body,
        config,
        scheduler,
        estimator,
        new ReentrantLock());
  }

  /**
   * @param backupGyroIO a second IMU (Systemcore's own): checked against the gyro, and turning
   *     odometry while the gyro is lost
   * @param odometryLock held while the drivetrain reads its IO, when a background thread records
   *     the devices' high-rate readings, so every device's readings stay in step
   */
  public Drive(
      String name,
      GyroIO gyroIO,
      GyroIO backupGyroIO,
      ModuleIO[] moduleIOs,
      Body body,
      Supplier<SwerveConfig> config,
      Scheduler scheduler,
      PoseEstimator estimator,
      Lock odometryLock) {
    if (moduleIOs.length != 4) {
      throw new IllegalArgumentException("swerve needs exactly 4 modules");
    }
    this.config = config;
    this.scheduler = scheduler;
    this.gyroIO = gyroIO;
    this.backupGyroIO = backupGyroIO;
    SwerveConfig initial = config.get();
    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(moduleIOs[i], i, initial.modules().wheelRadius());
    }
    applied = initial;
    apply(initial);
    kinematics = new SwerveDriveKinematics(initial.moduleTranslations(body));
    double reach = 0;
    for (Translation2d wheel : initial.moduleTranslations(body)) {
      reach = Math.max(reach, wheel.getNorm());
    }
    wheelReach = reach;
    odometry = new SwerveKinematicsModel(initial.moduleTranslations(body));
    this.estimator = estimator;
    this.odometryLock = odometryLock;

    gyroDisconnectedAlert =
        new Alert(
            name,
            "gyroDisconnected",
            "Gyro disconnected: heading estimated from wheels, and will drift",
            Alert.Level.HIGH);
    usingBackupAlert =
        new Alert(
            name,
            "usingBackupImu",
            "Gyro disconnected: heading from the backup IMU",
            Alert.Level.MEDIUM);
    imusDisagreeAlert =
        new Alert(name, "imusDisagree", "The gyro and the backup IMU disagree", Alert.Level.MEDIUM);
    for (int i = 0; i < 4; i++) {
      moduleDisconnectedAlerts[i] =
          new Alert(
              name,
              "module" + Module.NAMES[i] + "Disconnected",
              Module.NAMES[i] + " swerve module disconnected: driving on three",
              Alert.Level.HIGH);
    }
  }

  @Override
  public Scheduler getRegisteredScheduler() {
    return scheduler;
  }

  /**
   * Read sensors and update odometry. Once per loop, before commands.
   *
   * @param enabled whether the robot is enabled; outputs are stopped while disabled
   */
  public void periodic(boolean enabled) {
    SwerveConfig now = config.get();
    if (!now.equals(applied)) {
      apply(now);
    }
    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      backupGyroIO.updateInputs(backupInputs);
      for (Module module : modules) {
        module.readInputs();
      }
    } finally {
      odometryLock.unlock();
    }
    Logger.processInputs("Drive/Gyro", gyroInputs);
    Logger.processInputs("Drive/BackupGyro", backupInputs);
    for (Module module : modules) {
      module.logInputs();
    }
    if (!enabled) {
      for (Module module : modules) {
        module.stop();
      }
      Logger.recordOutput("Drive/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    conditions = readConditions(enabled);
    estimator.setConditions(conditions);
    updateOdometry();
    watchTerrain();
    checkImus();

    gyroDisconnectedAlert.set(!usable(gyroInputs) && !usable(backupInputs));
    usingBackupAlert.set(!usable(gyroInputs) && usable(backupInputs));
    for (int i = 0; i < 4; i++) {
      moduleDisconnectedAlerts[i].set(!modules[i].isConnected());
    }
    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/ModuleVelocities", moduleVelocities());
    Logger.recordOutput("Drive/TiltDegrees", Math.toDegrees(conditions.tiltRadians()));
    Logger.recordOutput(
        "Drive/GroundSlopeDegrees", Math.toDegrees(conditions.groundSlopeRadians()));
    Logger.recordOutput("Drive/Collision", conditions.collision());
    Logger.recordOutput("Drive/TractionLimited", conditions.tractionLimited());
  }

  /**
   * While both report, compares the gyro with the backup IMU: a sustained disagreement means one is
   * failing, loose, or needs its scale calibrated. It raises the question rather than silently
   * picking one.
   */
  private void checkImus() {
    if (usable(gyroInputs) && usable(backupInputs)) {
      crossCheck.update(
          RobotController.getTime(), gyroInputs.yaw.getRadians(), backupInputs.yaw.getRadians());
    } else {
      crossCheck.reset();
    }
    double disagreement = Math.toDegrees(crossCheck.disagreementRadians());
    imusDisagreeAlert.setText(
        String.format(
            Locale.ROOT,
            "The gyro and the backup IMU disagree by %.0f° over the last second: one is failing,"
                + " loose, or needs its scale calibrated",
            Math.abs(disagreement)));
    imusDisagreeAlert.set(crossCheck.disagrees());
    Logger.recordOutput("Drive/ImuDisagreementDegrees", disagreement);
    Logger.recordOutput("Drive/YawSource", yawSource.name());
  }

  /** What turns odometry now: the gyro, the backup IMU, or the wheels. */
  public YawSource yawSource() {
    return yawSource;
  }

  /** Follows the robot over raised terrain: the slope along the way it moves, and its tilt. */
  private void watchTerrain() {
    ChassisVelocities moving = getRobotRelativeVelocities();
    boolean still = Math.hypot(moving.vx, moving.vy) < MOVING;
    double way = Math.atan2(moving.vy, moving.vx);
    double climb = still ? 0 : TerrainCrossing.climb(pitchRadians(), rollRadians(), way);
    double fieldWay = still ? Double.NaN : way + getPose().getRotation().getRadians();
    terrain.update(RobotController.getTime(), climb, conditions.tiltRadians(), fieldWay);
    terrain
        .crest()
        .ifPresent(
            crest ->
                Logger.recordOutput(
                    "Drive/CrestSeconds", Timestamps.toSeconds(crest.timestampNanos())));
    Logger.recordOutput("Drive/ClimbDegrees", Math.toDegrees(climb));
    Logger.recordOutput("Drive/Terrain", terrain.phase().name());
    Logger.recordOutput("Drive/TerrainCrossings", terrain.crossings());
  }

  /** Where the robot is on raised terrain: level, climbing, or descending. */
  public TerrainCrossing.Phase terrainPhase() {
    return terrain.phase();
  }

  /** The crest of raised terrain the robot passed this loop, if it passed one: a landmark. */
  public Optional<TerrainCrossing.Crest> crest() {
    return terrain.crest();
  }

  /**
   * How many times the robot has crossed raised terrain: up, over, down, and level again. An auto
   * that drives over a bump waits for this to count up.
   */
  public int terrainCrossings() {
    return terrain.crossings();
  }

  /**
   * What the IMU and the motors say about trusting the wheels: the robot's tilt, a jolt beyond
   * driving, and wheels at the edge of their grip (a motor at its current limit, or a wheel well
   * short of its commanded speed), and whether people might be carrying it. The gyro's IMU says, or
   * the backup's while the gyro is lost; without either, only the motors can say.
   */
  private DriveConditions readConditions(boolean enabled) {
    boolean tractionLimited = false;
    for (Module module : modules) {
      tractionLimited |=
          module.isTorqueLimited() || module.speedDeficitMetersPerSecond() > SPEED_DEFICIT;
    }
    GyroIOInputsAutoLogged imu = imu();
    if (imu == null) {
      return new DriveConditions(0, 0, false, tractionLimited, !enabled);
    }
    // The angle between the robot's up and straight up.
    double tilt = Math.acos(Math.min(1, Math.cos(imu.pitchRad) * Math.cos(imu.rollRad)));
    boolean collision =
        Math.hypot(imu.accelerationXMetersPerSecSq, imu.accelerationYMetersPerSecSq)
            > COLLISION_ACCELERATION;
    return new DriveConditions(tilt, groundSlope(tilt), collision, tractionLimited, !enabled);
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
        .under(getPose().getTranslation(), reach)
        .map(bump -> Math.min(tilt, bump.slopeRadians() + GROUND_SLOPE_MARGIN))
        .orElse(0.0);
  }

  /** The field's bumps: a tilt one of them explains keeps the wheels trusted. Flat until given. */
  public void setTerrain(Terrain terrain) {
    ground = terrain;
  }

  /** The IMU to read the robot's attitude from: the gyro's, or the backup's while it's lost. */
  private @Nullable GyroIOInputsAutoLogged imu() {
    if (usable(gyroInputs)) {
      return gyroInputs;
    }
    return usable(backupInputs) ? backupInputs : null;
  }

  /**
   * Whether an IMU can be read from: connected, and reading numbers. One that sends a value that
   * isn't a number counts as lost, so a bad reading costs its readings, never the robot program.
   */
  private static boolean usable(GyroIOInputsAutoLogged imu) {
    return imu.connected
        && Double.isFinite(imu.yaw.getRadians())
        && Double.isFinite(imu.pitchRad)
        && Double.isFinite(imu.rollRad);
  }

  /** Sends new gains and limits to the modules, and keeps the limits driving uses. */
  private void apply(SwerveConfig drivetrain) {
    for (Module module : modules) {
      module.configure(drivetrain.gains(), drivetrain.limits());
    }
    maxLinearSpeedMps = drivetrain.limits().maxLinearSpeed().in(MetersPerSecond);
    maxAngularSpeedRadPerSec = drivetrain.limits().maxAngularSpeed().in(RadiansPerSecond);
    skewLookaheadSec = drivetrain.gains().skewLookahead().in(Seconds);
    applied = drivetrain;
  }

  /** Drive at robot-relative velocities (m/s, m/s, rad/s). Call every loop while driving. */
  public void runVelocity(ChassisVelocities robotRelative) {
    // Correct for the robot rotating during the 20 ms step, which otherwise makes it drift
    // sideways while spinning and driving at the same time.
    ChassisVelocities discrete = robotRelative.discretize(LOOP_PERIOD_S);
    SwerveModuleVelocity[] setpoints =
        SwerveDriveKinematics.desaturateWheelVelocities(
            kinematics.toSwerveModuleVelocities(discrete), maxLinearSpeedMps);
    SwerveModuleVelocity[] sent = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      sent[i] = modules[i].runSetpoint(setpoints[i]);
    }
    Logger.recordOutput("Drive/Setpoints", setpoints);
    Logger.recordOutput("Drive/SetpointsOptimized", sent);
  }

  /**
   * Feeds every high-rate reading since the last loop to the estimator, as the robot's motion
   * between readings, with the yaw at each: the gyro's, or while it's lost the backup IMU's
   * (interpolated between its once-a-loop readings), or with neither, the wheels' turning, which
   * the estimator says drifts. The step where the source changes turns by the wheels too, so the
   * estimator starts over from the new source's reading instead of taking the two IMUs' different
   * zeros as a turn.
   */
  private void updateOdometry() {
    int readings = Integer.MAX_VALUE;
    for (Module module : modules) {
      readings = Math.min(readings, module.odometryReadings());
    }
    track(gyroYaws, gyroInputs);
    track(backupYaws, backupInputs);
    YawSource source =
        gyroYaws.hasReadings()
            ? YawSource.GYRO
            : backupYaws.hasReadings() ? YawSource.BACKUP : YawSource.WHEELS;
    YawTrack yaws = source == YawSource.GYRO ? gyroYaws : backupYaws;
    boolean switching = readings > 0 && source != yawSource;
    if (readings > 0) {
      yawSource = source;
    }
    for (int k = 0; k < readings; k++) {
      double[] distances = new double[4];
      double[] angles = new double[4];
      boolean numbers = true;
      for (int i = 0; i < 4; i++) {
        distances[i] = modules[i].odometryDistanceMeters(k);
        angles[i] = modules[i].odometryAngleRadians(k);
        numbers &= Double.isFinite(distances[i]) && Double.isFinite(angles[i]);
      }
      long stamp = modules[0].odometryTimestampNanos(k);
      if (!numbers || stamp < 0) {
        continue; // a reading that isn't one is left out: the next good one covers the motion
      }
      OdometrySample reading = new OdometrySample(stamp, distances, angles);
      BodyMotion motion =
          lastReading == null ? new BodyMotion(0, 0, 0, 0) : odometry.motion(lastReading, reading);
      boolean turnedByImu = source != YawSource.WHEELS && !(switching && k == 0);
      double yaw = source == YawSource.WHEELS ? 0 : yaws.at(reading.timestampNanos());
      estimator.addOdometry(reading.timestampNanos(), motion, yaw, turnedByImu);
      lastReading = reading;
    }
  }

  /** Takes an IMU's readings this loop, or forgets them all while it's lost. */
  private static void track(YawTrack yaws, GyroIOInputsAutoLogged imu) {
    int readings = Math.min(imu.odometryTimestampsNanos.length, imu.odometryYawsRad.length);
    boolean numbers = true;
    for (int i = 0; i < readings; i++) {
      numbers &= Double.isFinite(imu.odometryYawsRad[i]);
    }
    if (!usable(imu) || !numbers || readings == 0) {
      yaws.clear();
      return;
    }
    yaws.update(
        Arrays.copyOf(imu.odometryTimestampsNanos, readings),
        Arrays.copyOf(imu.odometryYawsRad, readings));
  }

  /** The robot's pitch, from the IMU: zero without one. See {@link GyroIO} for the convention. */
  public double pitchRadians() {
    GyroIOInputsAutoLogged imu = imu();
    return imu == null ? 0 : imu.pitchRad;
  }

  /** The robot's roll, from the IMU: zero without one. */
  public double rollRadians() {
    GyroIOInputsAutoLogged imu = imu();
    return imu == null ? 0 : imu.rollRad;
  }

  /** Where the robot is, as the pose estimator has it. */
  public Pose2d getPose() {
    return estimator.pose();
  }

  /** Tells the pose estimator the robot is exactly here, now. */
  public void resetPose(Pose2d pose) {
    estimator.reset(new PoseReset(RobotController.getTime(), pose, EXACTLY, "reset"));
  }

  /** Measured robot-relative velocities. */
  public ChassisVelocities getRobotRelativeVelocities() {
    return kinematics.toChassisVelocities(moduleVelocities());
  }

  /** The modules' positions, as WPILib's odometry takes them. */
  SwerveModulePosition[] modulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  private SwerveModuleVelocity[] moduleVelocities() {
    SwerveModuleVelocity[] velocities = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      velocities[i] = modules[i].getVelocity();
    }
    return velocities;
  }

  /**
   * Field-relative driving from sticks, for one loop. Inputs are -1..1 along the field's axes (turn
   * the driver's stick to the field first: a red driver's forward is -x), and +omega
   * counterclockwise. Deadband, then squared for fine control near center.
   */
  public void driveWithSticks(double x, double y, double omega) {
    double linear = MathUtil.applyDeadband(Math.hypot(x, y), STICK_DEADBAND);
    linear = Math.min(1.0, linear * linear);
    Rotation2d direction = new Rotation2d(Math.atan2(y, x));
    double turn = MathUtil.applyDeadband(omega, STICK_DEADBAND);
    turn = Math.copySign(turn * turn, turn);

    double speed = linear * maxLinearSpeedMps;
    ChassisVelocities fieldRelative =
        new ChassisVelocities(
            speed * direction.getCos(),
            speed * direction.getSin(),
            turn * maxAngularSpeedRadPerSec);
    // Skew compensation: by the time the wheels reach their new angles the robot has already
    // rotated, so convert using the heading it is about to have.
    Rotation2d futureHeading =
        getPose().getRotation().plus(new Rotation2d(fieldRelative.omega * skewLookaheadSec));
    runVelocity(fieldRelative.toRobotRelative(futureHeading));
  }

  /**
   * Drives straight ahead, robot-relative, at a steady speed for a while, then stops: a pit check
   * that every wheel turns the right way.
   */
  public Command driveForward(double metersPerSecond, double seconds) {
    int loops = (int) Math.round(seconds / LOOP_PERIOD_S);
    return run(coroutine -> {
          for (int i = 0; i < loops; i++) {
            runVelocity(new ChassisVelocities(metersPerSecond, 0, 0));
            coroutine.yield();
          }
          runVelocity(new ChassisVelocities(0, 0, 0));
        })
        .named("Drive forward");
  }

  /** Drive with sticks until interrupted. See {@link #driveWithSticks}. */
  public Command joystickDrive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return run(coroutine -> {
          while (true) {
            driveWithSticks(x.getAsDouble(), y.getAsDouble(), omega.getAsDouble());
            coroutine.yield();
          }
        })
        .named("Drive with joysticks");
  }

  /** The gyro's yaw, as of this loop: for calibrating against, and WPILib's odometry. */
  public Rotation2d gyroYaw() {
    return gyroInputs.yaw;
  }

  /** How far each wheel has turned, in radians: front left, front right, back left, back right. */
  public double[] wheelRotationsRadians() {
    double[] rotations = new double[4];
    for (int i = 0; i < 4; i++) {
      rotations[i] = modules[i].wheelRotationRadians();
    }
    return rotations;
  }

  /** Where each wheel touches the floor, from the robot's center. */
  public Translation2d[] moduleLocations() {
    return kinematics.getModules();
  }

  boolean isGyroDisconnectedAlertActive() {
    return gyroDisconnectedAlert.get();
  }

  boolean isUsingBackupAlertActive() {
    return usingBackupAlert.get();
  }

  boolean isImuDisagreementAlertActive() {
    return imusDisagreeAlert.get();
  }

  /** What the IMU and the motors said about trusting the wheels this loop. */
  DriveConditions conditions() {
    return conditions;
  }

  @Override
  public void close() {
    gyroDisconnectedAlert.close();
    usingBackupAlert.close();
    imusDisagreeAlert.close();
    for (Alert alert : moduleDisconnectedAlerts) {
      alert.close();
    }
  }
}
