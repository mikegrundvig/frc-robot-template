package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.localization.OdometrySample;
import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.experiment.Rig;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import frc.robot.lib.timing.RobotLoop;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.MathUtil;
import org.wpilib.util.Alert;

/**
 * The swerve drivetrain: four {@link Module}s and a gyro. It drives, and each loop it reports what
 * its sensors read ({@link #readings()}) for localization, which owns where the robot is.
 *
 * <p>Call {@link #periodic(boolean)} once per loop, before localization and the command scheduler.
 * Driving commands call {@link #runVelocity(ChassisVelocities)} every loop they are active.
 *
 * <p>The drivetrain's configuration is read every loop, so values tuned while the robot runs take
 * effect at once: new gains and current limits go to the motor controllers the loop they change.
 *
 * <p>Hardware comes in through the IO layer ({@link DriveHardware}): {@link PhoenixDrivetrain} on a
 * CTRE robot, the simulator's in the simulator ({@code frc.robot.sim}), the log in a replay. A
 * stand-in until AdvantageKit's 2027 swerve template ships: see docs/stand-ins.md.
 */
public class Drive implements Mechanism, AutoCloseable {
  private static final double STICK_DEADBAND = 0.1;

  /**
   * Wheels that report connected but send no readings this long have lost what reads them: on a
   * robot, the thread that records their high-rate readings.
   */
  private static final double STOPPED_SECONDS = 0.1;

  /** The wheels by name, as a steering test records them. */
  private static final List<String> WHEELS =
      List.of("Front left", "Front right", "Back left", "Back right");

  private final Supplier<DrivetrainControl> config;
  private final Scheduler scheduler;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final GyroIO backupImuIO;
  private final GyroIOInputsAutoLogged backupInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final SwerveDriveKinematics kinematics;

  /** Where each wheel touches the floor, from the robot's center. */
  private final Translation2d[] moduleTranslations;

  /** The wheels' radius, as built: their centers are this far above the floor. */
  private final double wheelRadiusMeters;

  /** The wheels' poses as last logged, reused loop after loop. */
  private final Pose3d[] modulePoses = new Pose3d[4];

  /** The steer angles the logged poses were made for: NaN before the first. */
  private final double[] posedAngles = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

  private final Supplier<Pose2d> pose;
  private final Lock odometryLock;

  /** The configuration the motor controllers have, and the limits taken from it. */
  private DrivetrainControl applied;

  private double maxLinearSpeedMps;
  private double maxAngularSpeedRadPerSec;
  private double skewLookaheadSec;

  /** What the sensors read this loop, for localization. */
  private DrivetrainReadings readings;

  private final Alert[] moduleDisconnectedAlerts = new Alert[4];
  private final Alert[] moduleUnconfiguredAlerts = new Alert[4];
  private final Alert odometryStopped;

  /** Loops in a row that the wheels reported connected but sent no readings. */
  private int loopsWithoutReadings;

  /** The steer angle last held for a steering test, in radians. */
  private double heldSteerRadians;

  /**
   * The drivetrain's configuration as the pit tunes it: its gains and limits, each within the range
   * that's safe to dial in on a running robot (a range narrower than what's valid). Publish it, and
   * give the drivetrain its {@link LiveConfig#get}.
   */
  public static LiveConfig<DrivetrainControl> tuning(DrivetrainControl committed) {
    return LiveConfig.of("drivetrain", committed)
        .number("gains.driveKs", Limits.between(0, 1)) // volts
        .number("gains.driveKv", Limits.between(0, 5)) // volts per meter per second
        .number("gains.driveKp", Limits.between(0, 20))
        .number("gains.steerKp", Limits.between(0, 100))
        .number("gains.steerKd", Limits.between(0, 5))
        .measure("gains.skewLookahead", Seconds.of(0), Seconds.of(0.1))
        .measure("limits.driveStatorCurrent", Amps.of(10), Amps.of(120))
        .measure("limits.driveSupplyCurrent", Amps.of(10), Amps.of(80))
        // Never above a 40 A breaker's rating, and stepping down well before the fastest one can
        // trip at twice its rating (1.5 s at 80 A, the supply limit's top): docs/values.md has the
        // times.
        .measure("limits.driveSupplyLowerCurrent", Amps.of(10), Amps.of(40))
        .measure("limits.driveSupplyLowerTime", Seconds.of(0.1), Seconds.of(1))
        .measure("limits.steerStatorCurrent", Amps.of(5), Amps.of(80))
        .measure("limits.maxLinearSpeed", MetersPerSecond.of(0.5), MetersPerSecond.of(6))
        .measure("limits.maxAngularSpeed", RotationsPerSecond.of(0.1), RotationsPerSecond.of(3));
  }

  /**
   * @param name alert group name: "Drive" on the robot, another name for a second instance
   * @param hardware the modules, the gyro, and the backup IMU
   * @param robot the robot's definition: its frame and its drivetrain's geometry place the wheels
   * @param config how the drivetrain is driven, read every loop so tuned values take effect
   * @param pose where the robot is, from localization: field-relative driving turns by its heading
   */
  public Drive(
      String name,
      DriveHardware hardware,
      RobotDefinition robot,
      Supplier<DrivetrainControl> config,
      Scheduler scheduler,
      Supplier<Pose2d> pose) {
    this.config = config;
    this.scheduler = scheduler;
    this.gyroIO = hardware.gyro();
    this.backupImuIO = hardware.backupImu();
    this.pose = pose;
    this.odometryLock = hardware.odometryLock();
    DrivetrainControl initial = config.get();
    for (int i = 0; i < 4; i++) {
      modules[i] =
          new Module(hardware.modules().get(i), i, robot.drivetrain().modules().wheelRadius());
    }
    applied = initial;
    apply(initial);
    moduleTranslations = robot.moduleTranslations();
    wheelRadiusMeters = robot.drivetrain().modules().wheelRadius().in(Meters);
    kinematics = new SwerveDriveKinematics(moduleTranslations);
    readings =
        DrivetrainReadings.builder()
            .wheels(List.of())
            .gyro(ImuReadings.NONE)
            .backupImu(ImuReadings.NONE)
            .velocities(new ChassisVelocities())
            .build();

    odometryStopped =
        new Alert(
            name,
            "odometryStopped",
            "Odometry stopped: the wheels report connected, but no readings have arrived for "
                + STOPPED_SECONDS
                + " s. Restart the robot code.",
            Alert.Level.HIGH);
    for (int i = 0; i < 4; i++) {
      String module = Module.NAMES.get(i);
      moduleDisconnectedAlerts[i] =
          new Alert(
              name,
              "module" + module + "Disconnected",
              module + " swerve module disconnected: driving on three",
              Alert.Level.HIGH);
      moduleUnconfiguredAlerts[i] =
          new Alert(
              name,
              "module" + module + "Unconfigured",
              module + " swerve module not configured",
              Alert.Level.HIGH);
    }
  }

  @Override
  public Scheduler getRegisteredScheduler() {
    return scheduler;
  }

  /**
   * Reads the sensors, and stops the outputs while disabled. Once per loop, before localization and
   * the commands.
   *
   * @param enabled whether the robot is enabled
   */
  public void periodic(boolean enabled) {
    DrivetrainControl now = config.get();
    if (!now.equals(applied)) {
      apply(now);
    }
    odometryLock.lock();
    try {
      gyroIO.updateInputs(gyroInputs);
      backupImuIO.updateInputs(backupInputs);
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

    List<OdometrySample> wheels = wheelReadings();
    boolean connected = true;
    for (Module module : modules) {
      connected &= module.isConnected();
    }
    loopsWithoutReadings = connected && wheels.isEmpty() ? loopsWithoutReadings + 1 : 0;
    odometryStopped.set(loopsWithoutReadings > RobotLoop.loops(STOPPED_SECONDS));
    readings =
        DrivetrainReadings.builder()
            .wheels(wheels)
            .gyro(imuReadings(gyroInputs))
            .backupImu(imuReadings(backupInputs))
            .velocities(getRobotRelativeVelocities())
            .build();

    for (int i = 0; i < 4; i++) {
      moduleDisconnectedAlerts[i].set(!modules[i].isConnected());
      String unconfigured = modules[i].unconfigured();
      if (!unconfigured.isEmpty() && !moduleUnconfiguredAlerts[i].get()) {
        moduleUnconfiguredAlerts[i].setText(
            Module.NAMES.get(i)
                + " swerve module: "
                + unconfigured
                + " didn't take its configuration and runs on defaults (no current limits):"
                + " restart the robot code once it's connected");
      }
      moduleUnconfiguredAlerts[i].set(!unconfigured.isEmpty());
    }
    Logger.recordOutput("Drive/Pose", getPose());
    Logger.recordOutput("Drive/ModuleVelocities", moduleVelocities());
    Logger.recordOutput("Drive/ModulePoses", modulePoses());
  }

  /**
   * Each wheel's center in the robot's frame, turned to the angle its module measures: front left,
   * front right, back left, back right. AdvantageScope turns the schematic robot's wheels by these
   * (its components 0 to 3, generated into ascope_assets/ from the robot's configuration).
   */
  Pose3d[] modulePoses() {
    for (int i = 0; i < 4; i++) {
      double angle = modules[i].steerAngle().getRadians();
      // A wheel that hasn't turned keeps its pose: nothing new to make, most loops.
      if (angle != posedAngles[i]) {
        Translation2d at = moduleTranslations[i];
        modulePoses[i] =
            new Pose3d(at.getX(), at.getY(), wheelRadiusMeters, new Rotation3d(0, 0, angle));
        posedAngles[i] = angle;
      }
    }
    return modulePoses;
  }

  /** What the sensors read this loop: the wheels at every high-rate reading, and the IMUs. */
  public DrivetrainReadings readings() {
    return readings;
  }

  /**
   * Every high-rate reading of the wheels since the last loop, oldest first. A reading that isn't
   * one (a value that isn't a number, a time before the clock started) is left out: the next good
   * one covers the motion.
   */
  private List<OdometrySample> wheelReadings() {
    int count = Integer.MAX_VALUE;
    for (Module module : modules) {
      count = Math.min(count, module.odometryReadings());
    }
    List<OdometrySample> wheels = new ArrayList<>(count);
    for (int k = 0; k < count; k++) {
      double[] distances = new double[4];
      double[] angles = new double[4];
      boolean numbers = true;
      for (int i = 0; i < 4; i++) {
        distances[i] = modules[i].odometryDistanceMeters(k);
        angles[i] = modules[i].odometryAngleRadians(k);
        numbers &= Double.isFinite(distances[i]) && Double.isFinite(angles[i]);
      }
      long stamp = modules[0].odometryTimestampNanos(k);
      if (numbers && stamp >= 0) {
        wheels.add(new OdometrySample(stamp, distances, angles));
      }
    }
    return wheels;
  }

  /** One IMU's readings this loop, as localization takes them. */
  private static ImuReadings imuReadings(GyroIOInputsAutoLogged imu) {
    int count = Math.min(imu.odometryTimestampsNanos.length, imu.odometryYawsRad.length);
    List<Long> times = new ArrayList<>(count);
    List<Double> yaws = new ArrayList<>(count);
    for (int i = 0; i < count; i++) {
      times.add(imu.odometryTimestampsNanos[i]);
      yaws.add(imu.odometryYawsRad[i]);
    }
    return ImuReadings.builder()
        .reporting(imu.connected)
        .yawRadians(imu.yaw.getRadians())
        .yawRateRadiansPerSecond(imu.yawVelocityRadPerSec)
        .pitchRadians(imu.pitchRad)
        .rollRadians(imu.rollRad)
        .forwardAccelerationMetersPerSecSq(imu.accelerationXMetersPerSecSq)
        .leftAccelerationMetersPerSecSq(imu.accelerationYMetersPerSecSq)
        .yawTimesNanos(times)
        .yawsRadians(yaws)
        .build();
  }

  /** Sends new gains and limits to the modules, and keeps the limits driving uses. */
  private void apply(DrivetrainControl drivetrain) {
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
    // Correct for the robot rotating during the loop's step, which otherwise makes it drift
    // sideways while spinning and driving at the same time.
    ChassisVelocities discrete = robotRelative.discretize(RobotLoop.PERIOD_SECONDS);
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

  /** Where the robot is, as localization has it. */
  public Pose2d getPose() {
    return pose.get();
  }

  /** Measured robot-relative velocities. */
  public ChassisVelocities getRobotRelativeVelocities() {
    return kinematics.toChassisVelocities(moduleVelocities());
  }

  /** The modules' positions, as WPILib's odometry takes them. */
  public SwerveModulePosition[] modulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** Each module's measured wheel speed and angle, in the order of {@code moduleTranslations}. */
  public SwerveModuleVelocity[] moduleVelocities() {
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
    int loops = RobotLoop.loops(seconds);
    return run(coroutine -> {
          for (int i = 0; i < loops; i++) {
            runVelocity(new ChassisVelocities(metersPerSecond, 0, 0));
            coroutine.yield();
          }
          runVelocity(new ChassisVelocities(0, 0, 0));
        })
        .named("Drive forward");
  }

  /**
   * Points every wheel at one angle, relative to the robot, with the drive wheels held still: the
   * steering's test motion. Call it every loop the test runs.
   *
   * @param radians the angle, counterclockwise from the robot's forward
   */
  public void holdSteerAngle(double radians) {
    heldSteerRadians = radians;
    Rotation2d angle = new Rotation2d(radians);
    for (Module module : modules) {
      module.holdSteer(angle);
    }
  }

  /**
   * The steering as a Pitwall rig, so the page can tune it by experiment: its gains ({@code
   * steerKp} and {@code steerKd}, tuned where they always are), every wheel pointed at one angle as
   * the setpoint, and for each wheel its angle at every high-rate reading (what's scored), and once
   * a loop its steering motor's voltage and the most current it drew since the last. Everything it
   * records is a logged input, so a replay records the same.
   *
   * @param tuning the drivetrain's configuration as the pit tunes it
   */
  public Rig steeringRig(LiveConfig<DrivetrainControl> tuning) {
    Rig.Builder rig =
        Rig.builder()
            .name("Steering")
            .gains(tuning, "gains.steerKp", "gains.steerKd")
            .setpoint(Radians, this::holdSteerAngle)
            .limits(Degrees.of(-180), Degrees.of(180));
    for (int i = 0; i < 4; i++) {
      Module module = modules[i];
      rig.measurement(
          WHEELS.get(i),
          Degrees,
          (now, series) -> {
            for (int k = 0; k < module.odometryReadings(); k++) {
              // The angle nearest the one held, so a wheel near ±180° doesn't jump a turn.
              double angle =
                  heldSteerRadians
                      + MathUtil.angleModulus(module.odometryAngleRadians(k) - heldSteerRadians);
              series.add(module.odometryTimestampNanos(k), Math.toDegrees(angle));
            }
          });
    }
    for (int i = 0; i < 4; i++) {
      Module module = modules[i];
      rig.signal(WHEELS.get(i) + " voltage", Volts, module::steerAppliedVolts);
      rig.signal(WHEELS.get(i) + " current", Amps, module::steerCurrentAmps);
    }
    return rig.requires(this).build();
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

  boolean isModuleUnconfiguredAlertActive(int module) {
    return moduleUnconfiguredAlerts[module].get();
  }

  boolean isOdometryStoppedAlertActive() {
    return odometryStopped.get();
  }

  @Override
  public void close() {
    for (Alert alert : moduleDisconnectedAlerts) {
      alert.close();
    }
    for (Alert alert : moduleUnconfiguredAlerts) {
      alert.close();
    }
    odometryStopped.close();
  }
}
