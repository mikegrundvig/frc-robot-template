package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.SwerveConfig;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;

/**
 * The simulated drivetrain as it really moves: four modules' motors, and the robot's true pose on
 * the field. Its sensors report that truth with {@link SensorErrors}, as real sensors would, so the
 * pose the robot code works out can drift from where the robot really is, and localization is
 * tested against that gap instead of a perfect world.
 *
 * <p>Call {@link #step} once per loop, before the robot code reads its sensors. The robot moves as
 * its wheels roll, at their true radius and true angle; it never slips, since traction isn't
 * modeled yet (see docs/stand-ins.md). {@link #shove} moves it the way a collision would.
 */
public final class DriveSimulation {
  private final ModuleIOSim[] modules = new ModuleIOSim[4];
  private final double[] trueRadiusMeters = new double[4];
  private final SwerveDriveKinematics kinematics;
  private final SensorErrors errors;
  private SwerveModulePosition[] lastTruePositions;
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
    this.errors = errors;
    kinematics = new SwerveDriveKinematics(moduleLocations);
    double radius = config.wheelRadius().in(Meters);
    for (int i = 0; i < 4; i++) {
      modules[i] = new ModuleIOSim(config, Math.toRadians(errors.steerOffsetDegrees().get(i)));
      trueRadiusMeters[i] = radius * errors.wheelRadiusScale().get(i);
    }
    lastTruePositions = truePositions();
  }

  /** The modules' motor controllers and encoders, for the drivetrain. */
  public ModuleIO[] modules() {
    return new ModuleIO[] {modules[0], modules[1], modules[2], modules[3]};
  }

  /** A gyro on this robot. */
  public GyroIO gyro() {
    return new GyroIOSim(this, errors);
  }

  /** Runs the motors for a step, and moves the robot as its wheels truly rolled. */
  public void step(double seconds) {
    if (!(seconds > 0)) {
      throw new IllegalArgumentException("a step takes time: " + seconds);
    }
    for (ModuleIOSim module : modules) {
      module.step(seconds);
    }
    SwerveModulePosition[] now = truePositions();
    Twist2d twist = kinematics.toTwist2d(lastTruePositions, now);
    truePose = truePose.plus(twist.exp());
    yawTurnedRadians += twist.dtheta;
    yawRateRadiansPerSecond = twist.dtheta / seconds;
    lastTruePositions = now;
    elapsedSeconds += seconds;
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

  private SwerveModulePosition[] truePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] =
          new SwerveModulePosition(
              modules[i].wheelRotationRadians() * trueRadiusMeters[i], modules[i].trueSteerAngle());
    }
    return positions;
  }
}
