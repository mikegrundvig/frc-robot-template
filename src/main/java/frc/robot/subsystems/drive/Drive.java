package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import frc.robot.lib.config.Body;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.localization.BodyMotion;
import frc.robot.lib.localization.DriveConditions;
import frc.robot.lib.localization.OdometrySample;
import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.PoseEstimator;
import frc.robot.lib.localization.PoseReset;
import frc.robot.lib.localization.SwerveKinematicsModel;
import frc.robot.lib.localization.TerrainCrossing;
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

  /** A reset to "exactly here": a millimeter and a milliradian. */
  private static final PoseCovariance EXACTLY = PoseCovariance.fromStdDevs(0.001, 0.001, 0.001);

  private final Supplier<SwerveConfig> config;
  private final Scheduler scheduler;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
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

  /** Crossings of raised terrain, from the IMU: 1678's thresholds (10° steep, 5° level). */
  private final TerrainCrossing terrain =
      new TerrainCrossing(Math.toRadians(10), Math.toRadians(5), 0.1);

  private final Alert gyroDisconnectedAlert;
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
    this(name, gyroIO, moduleIOs, body, config, scheduler, estimator, new ReentrantLock());
  }

  /**
   * @param odometryLock held while the drivetrain reads its IO, when a background thread records
   *     the devices' high-rate readings, so every device's readings stay in step
   */
  public Drive(
      String name,
      GyroIO gyroIO,
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
    SwerveConfig initial = config.get();
    for (int i = 0; i < 4; i++) {
      modules[i] = new Module(moduleIOs[i], i, initial.modules().wheelRadius());
    }
    applied = initial;
    apply(initial);
    kinematics = new SwerveDriveKinematics(initial.moduleTranslations(body));
    odometry = new SwerveKinematicsModel(initial.moduleTranslations(body));
    this.estimator = estimator;
    this.odometryLock = odometryLock;

    gyroDisconnectedAlert =
        new Alert(
            name,
            "gyroDisconnected",
            "Gyro disconnected: heading estimated from wheels, and will drift",
            Alert.Level.HIGH);
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
      for (Module module : modules) {
        module.readInputs();
      }
    } finally {
      odometryLock.unlock();
    }
    Logger.processInputs("Drive/Gyro", gyroInputs);
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

    gyroDisconnectedAlert.set(!gyroInputs.connected);
    for (int i = 0; i < 4; i++) {
      moduleDisconnectedAlerts[i].set(!modules[i].isConnected());
    }
    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/ModuleVelocities", moduleVelocities());
    Logger.recordOutput("Drive/TiltDegrees", Math.toDegrees(conditions.tiltRadians()));
    Logger.recordOutput("Drive/Collision", conditions.collision());
    Logger.recordOutput("Drive/TractionLimited", conditions.tractionLimited());
  }

  /** Follows the robot over raised terrain: the slope along the way it moves, and its tilt. */
  private void watchTerrain() {
    ChassisVelocities moving = getRobotRelativeVelocities();
    double climb =
        Math.hypot(moving.vx, moving.vy) < MOVING
            ? 0
            : TerrainCrossing.climb(
                pitchRadians(), rollRadians(), Math.atan2(moving.vy, moving.vx));
    terrain.update(RobotController.getTime(), climb, conditions.tiltRadians());
    Logger.recordOutput("Drive/ClimbDegrees", Math.toDegrees(climb));
    Logger.recordOutput("Drive/Terrain", terrain.phase().name());
    Logger.recordOutput("Drive/TerrainCrossings", terrain.crossings());
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

  /**
   * What the IMU and the motors say about trusting the wheels: the robot's tilt, a jolt beyond
   * driving, and wheels at the edge of their grip (a motor at its current limit, or a wheel well
   * short of its commanded speed), and whether people might be carrying it. Without the gyro, only
   * the motors can say.
   */
  private DriveConditions readConditions(boolean enabled) {
    boolean tractionLimited = false;
    for (Module module : modules) {
      tractionLimited |=
          module.isTorqueLimited() || module.speedDeficitMetersPerSecond() > SPEED_DEFICIT;
    }
    if (!gyroInputs.connected) {
      return new DriveConditions(0, false, tractionLimited, !enabled);
    }
    // The angle between the robot's up and straight up.
    double tilt =
        Math.acos(Math.min(1, Math.cos(gyroInputs.pitchRad) * Math.cos(gyroInputs.rollRad)));
    boolean collision =
        Math.hypot(gyroInputs.accelerationXMetersPerSecSq, gyroInputs.accelerationYMetersPerSecSq)
            > COLLISION_ACCELERATION;
    return new DriveConditions(tilt, collision, tractionLimited, !enabled);
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
   * between readings, with the gyro's yaw at each. When the gyro is lost, the estimator turns the
   * robot by the wheels instead, and says so.
   */
  private void updateOdometry() {
    int readings = Integer.MAX_VALUE;
    for (Module module : modules) {
      readings = Math.min(readings, module.odometryReadings());
    }
    boolean gyroHasReadings =
        gyroInputs.connected
            && gyroInputs.odometryYawsRad.length >= readings
            && gyroInputs.odometryTimestampsNanos.length >= readings;
    for (int k = 0; k < readings; k++) {
      double[] distances = new double[4];
      double[] angles = new double[4];
      for (int i = 0; i < 4; i++) {
        distances[i] = modules[i].odometryDistanceMeters(k);
        angles[i] = modules[i].odometryAngleRadians(k);
      }
      OdometrySample reading =
          new OdometrySample(modules[0].odometryTimestampNanos(k), distances, angles);
      BodyMotion motion =
          lastReading == null ? new BodyMotion(0, 0, 0, 0) : odometry.motion(lastReading, reading);
      double yaw = gyroHasReadings ? gyroInputs.odometryYawsRad[k] : 0;
      estimator.addOdometry(reading.timestampNanos(), motion, yaw, gyroHasReadings);
      lastReading = reading;
    }
  }

  /** The robot's pitch, from the IMU: zero without it. See {@link GyroIO} for the convention. */
  public double pitchRadians() {
    return gyroInputs.connected ? gyroInputs.pitchRad : 0;
  }

  /** The robot's roll, from the IMU: zero without it. */
  public double rollRadians() {
    return gyroInputs.connected ? gyroInputs.rollRad : 0;
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

  /** The gyro's yaw, as of this loop. */
  Rotation2d gyroYaw() {
    return gyroInputs.yaw;
  }

  /** What the IMU and the motors said about trusting the wheels this loop. */
  DriveConditions conditions() {
    return conditions;
  }

  @Override
  public void close() {
    gyroDisconnectedAlert.close();
    for (Alert alert : moduleDisconnectedAlerts) {
      alert.close();
    }
  }
}
