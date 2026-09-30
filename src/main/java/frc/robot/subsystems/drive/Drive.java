package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import frc.robot.Constants;
import frc.robot.lib.config.Body;
import frc.robot.lib.config.SwerveConfig;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveDriveOdometry;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.MathUtil;
import org.wpilib.util.Alert;

/**
 * The swerve drivetrain: four {@link Module}s, a gyro, and odometry.
 *
 * <p>Call {@link #periodic(boolean)} once per loop, before the command scheduler runs. Driving
 * commands call {@link #runVelocity(ChassisVelocities)} every loop they are active.
 *
 * <p>The drivetrain's configuration is read every loop, so values tuned while the robot runs take
 * effect at once: new gains and current limits go to the motor controllers the loop they change.
 *
 * <p>A stand-in until AdvantageKit's 2027 swerve template ships: see docs/stand-ins.md.
 */
public class Drive implements Mechanism, AutoCloseable {
  private static final double LOOP_PERIOD_S = 0.02;
  private static final double STICK_DEADBAND = 0.1;

  private final Supplier<SwerveConfig> config;
  private final Scheduler scheduler;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  /** The configuration the motor controllers have, and the limits taken from it. */
  private SwerveConfig applied;

  private double maxLinearSpeedMps;
  private double maxAngularSpeedRadPerSec;
  private double skewLookaheadSec;

  private SwerveModulePosition[] lastPositions;
  private Rotation2d heading = Rotation2d.ZERO;

  private final Alert gyroDisconnectedAlert;
  private final Alert[] moduleDisconnectedAlerts = new Alert[4];

  /**
   * @param name alert group name: "Drive" on the robot, another name for a second instance
   * @param body the robot's frame, which places the wheels
   * @param config the drivetrain's configuration, read every loop so tuned values take effect
   */
  public Drive(
      String name,
      GyroIO gyroIO,
      ModuleIO[] moduleIOs,
      Body body,
      Supplier<SwerveConfig> config,
      Scheduler scheduler) {
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
    lastPositions = modulePositions();
    odometry = new SwerveDriveOdometry(kinematics, heading, lastPositions);

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
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (Module module : modules) {
      module.periodic();
    }
    if (!enabled) {
      for (Module module : modules) {
        module.stop();
      }
      Logger.recordOutput("Drive/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    SwerveModulePosition[] positions = modulePositions();
    if (gyroInputs.connected) {
      heading = gyroInputs.yaw;
    } else {
      // Fall back to the heading change the wheels imply. Worse, but better than stopping.
      Twist2d twist = kinematics.toTwist2d(lastPositions, positions);
      heading = heading.plus(new Rotation2d(twist.dtheta));
    }
    lastPositions = positions;
    odometry.update(heading, positions);

    // Simulation has no gyro: its wheels never slip, so they give the heading exactly.
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Constants.Mode.SIM);
    for (int i = 0; i < 4; i++) {
      moduleDisconnectedAlerts[i].set(!modules[i].isConnected());
    }
    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/ModuleVelocities", moduleVelocities());
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

  public Pose2d getPose() {
    return odometry.getPose();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(heading, modulePositions(), pose);
  }

  /** Measured robot-relative velocities. */
  public ChassisVelocities getRobotRelativeVelocities() {
    return kinematics.toChassisVelocities(moduleVelocities());
  }

  private SwerveModulePosition[] modulePositions() {
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

  @Override
  public void close() {
    gyroDisconnectedAlert.close();
    for (Alert alert : moduleDisconnectedAlerts) {
      alert.close();
    }
  }
}
