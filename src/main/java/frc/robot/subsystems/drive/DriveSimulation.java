package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.localization.BodyMotion;
import frc.robot.lib.localization.OdometrySample;
import frc.robot.lib.localization.SwerveKinematicsModel;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;

/**
 * The simulated drivetrain as it really moves: four modules' motors, and the robot's true pose on
 * the field. Its sensors report that truth with {@link SensorErrors}, as real sensors would, so the
 * pose the robot code works out can drift from where the robot really is, and localization is
 * tested against that gap instead of a perfect world.
 *
 * <p>Call {@link #step} once per loop, before the robot code reads its sensors. It runs the physics
 * at 250 Hz and takes the modules' and gyro's readings at each of those steps, as a robot's
 * high-rate odometry does. The robot moves as its wheels roll, at their true radius and true angle;
 * it doesn't slip on its own, since traction isn't modeled yet (see docs/stand-ins.md), but {@link
 * #spin} makes a wheel slip and {@link #shove} moves the robot the way a collision does.
 */
public final class DriveSimulation {
  /** Physics steps, and odometry readings, per robot loop: 250 Hz at a 50 Hz loop. */
  static final int STEPS_PER_LOOP = 5;

  private final ModuleIOSim[] modules = new ModuleIOSim[4];
  private final GyroIOSim gyro;
  private final double[] trueRadiusMeters = new double[4];
  private final SwerveKinematicsModel kinematics;
  private OdometrySample lastTrue;
  private Pose2d truePose = Pose2d.ZERO;
  private double yawTurnedRadians;
  private double yawRateRadiansPerSecond;
  private double elapsedSeconds;

  /**
   * @param config the modules as configured: the code's belief about them
   * @param moduleLocations where the modules sit, from the robot's center
   * @param errors how the real wheels and sensors differ from the configuration
   */
  public DriveSimulation(
      SwerveConfig.Modules config, Translation2d[] moduleLocations, SensorErrors errors) {
    if (moduleLocations.length != 4) {
      throw new IllegalArgumentException("swerve needs exactly 4 modules");
    }
    kinematics = new SwerveKinematicsModel(moduleLocations);
    double radius = config.wheelRadius().in(Meters);
    for (int i = 0; i < 4; i++) {
      modules[i] = new ModuleIOSim(config, Math.toRadians(errors.steerOffsetDegrees().get(i)));
      trueRadiusMeters[i] = radius * errors.wheelRadiusScale().get(i);
    }
    gyro = new GyroIOSim(this, errors);
    lastTrue = trueWheels();
  }

  /** The modules' motor controllers and encoders, for the drivetrain. */
  public ModuleIO[] modules() {
    return new ModuleIO[] {modules[0], modules[1], modules[2], modules[3]};
  }

  /** The robot's gyro. */
  public GyroIO gyro() {
    return gyro;
  }

  /**
   * Runs the physics through one loop, and takes the sensors' readings along the way.
   *
   * @param seconds the loop's length
   * @param nowNanos the robot clock at the loop's end: the last reading's time
   */
  public void step(double seconds, long nowNanos) {
    if (!(seconds > 0)) {
      throw new IllegalArgumentException("a step takes time: " + seconds);
    }
    double dt = seconds / STEPS_PER_LOOP;
    long dtNanos = Math.round(dt * 1e9);
    for (int remaining = STEPS_PER_LOOP - 1; remaining >= 0; remaining--) {
      for (ModuleIOSim module : modules) {
        module.step(dt);
      }
      OdometrySample wheels = trueWheels();
      BodyMotion motion = kinematics.motion(lastTrue, wheels);
      truePose =
          truePose.plus(
              new Twist2d(motion.dxMeters(), motion.dyMeters(), motion.dthetaRadians()).exp());
      yawTurnedRadians += motion.dthetaRadians();
      yawRateRadiansPerSecond = motion.dthetaRadians() / dt;
      lastTrue = wheels;
      elapsedSeconds += dt;
      long readingTime = nowNanos - remaining * dtNanos;
      for (ModuleIOSim module : modules) {
        module.sample(readingTime);
      }
      gyro.sample(readingTime);
    }
  }

  /** Where the robot really is. */
  public Pose2d truePose() {
    return truePose;
  }

  /**
   * Puts the robot somewhere, as the drive team does: its wheels don't turn, but its gyro feels any
   * turn, as a real one would.
   */
  public void place(Pose2d pose) {
    yawTurnedRadians += pose.getRotation().minus(truePose.getRotation()).getRadians();
    truePose = pose;
  }

  /**
   * Moves the robot without its wheels turning, as another robot's push or a collision does:
   * odometry can't see it, and only the gyro sees any turn. A shove is in the robot's own frame.
   */
  public void shove(Transform2d shove) {
    place(truePose.plus(shove));
  }

  /**
   * Spins one wheel without the robot moving: a slip, which its encoder counts and the floor never
   * sees. Odometry catches it as the wheels disagreeing.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  public void spin(int module, double radians) {
    modules[module].spin(radians);
  }

  /** Every true turn since the start, driving or placed: what a gyro integrates. */
  double yawTurnedRadians() {
    return yawTurnedRadians;
  }

  double yawRateRadiansPerSecond() {
    return yawRateRadiansPerSecond;
  }

  double elapsedSeconds() {
    return elapsedSeconds;
  }

  /** The wheels as they truly rolled: the rotation that moved the robot, at the true radius. */
  private OdometrySample trueWheels() {
    double[] distances = new double[4];
    double[] angles = new double[4];
    for (int i = 0; i < 4; i++) {
      distances[i] = modules[i].wheelRotationRadians() * trueRadiusMeters[i];
      Rotation2d angle = modules[i].trueSteerAngle();
      angles[i] = angle.getRadians();
    }
    return new OdometrySample(0, distances, angles);
  }
}
