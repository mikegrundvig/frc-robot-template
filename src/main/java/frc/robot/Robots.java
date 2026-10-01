package frc.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import frc.robot.lib.config.Body;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.Motor;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.localization.EstimatorConfig;
import frc.robot.lib.localization.VisionConfig;
import java.util.List;
import java.util.Optional;

/**
 * The team's robots, one {@link RobotConfig} each. A test builds every one, so an invalid value
 * fails the build rather than the robot.
 *
 * <p>Every value is set by name through a builder, which asks for each in turn: none can be left
 * out or land in the wrong place.
 *
 * <p>The template's robot is a typical competition build, at the 2026 rule maximums where the rules
 * set one (the next season's rules replace them at kickoff). Replace each value with your robot's
 * as you design and measure it: docs/values.md says how to measure each, and where today's number
 * came from. Note a measured value where it's set, as the calibration procedures do: {@code
 * .mass(Pounds.of(131.2)) // measured YYYY-MM-DD: two scales, bumpers and battery on}.
 */
public final class Robots {
  private Robots() {}

  /**
   * Frame 27.5 x 27.5 in, the largest square within 2026's 110 in frame perimeter (R104), with
   * bumpers 4 in beyond it (R403), and 30 in tall, 2026's starting height limit (R104). Mass 149.5
   * lb: 135 lb of robot and bumpers (R408) plus a 14.5 lb battery (R601). The 6.7 kg m^2 yaw
   * inertia is an estimate, not a measurement. The center of gravity is centered and low: the best
   * case.
   */
  private static final Body BODY =
      Body.builder()
          .frameLength(Inches.of(27.5))
          .frameWidth(Inches.of(27.5))
          .height(Inches.of(30))
          .bumperThickness(Inches.of(4))
          .mass(Pounds.of(149.5))
          .yawMomentOfInertia(KilogramSquareMeters.of(6.7))
          .cgHeight(Meters.of(0.15))
          .build();

  /**
   * SDS MK4i modules with L2 gearing (6.75:1 drive, 150/7:1 steer) and 4 in wheels, driven by
   * Kraken X60s without FOC: it needs a Phoenix Pro license, and under this drivetrain's 30 A limit
   * it adds nothing. Free speed: 4.73 m/s.
   */
  private static final SwerveConfig.Modules MODULES =
      SwerveConfig.Modules.builder()
          .driveMotor(Motor.KRAKEN_X60)
          .steerMotor(Motor.KRAKEN_X60)
          .wheelRadius(Inches.of(2))
          .driveGearing(6.75)
          .steerGearing(150.0 / 7.0)
          .wheelCoefficientOfFriction(1.2)
          .build();

  /**
   * The modules sit on the corners, where MK4i wheels are 2.625 in in from the frame's edges.
   *
   * <p>Drive stator limit 30 A. Under hard acceleration, weight shifts off the front wheels, and by
   * a simple weight-transfer estimate they start to slip near 47 A with this tread (friction 1.2;
   * about 50 A at 1.3, new) and this low center of gravity, or near 31 A with worn tread (friction
   * 1.0) and a center of gravity at 0.3 m. More current than that only spins the wheels and drains
   * the battery. Supply limit 60 A per drive motor. Steering needs little torque: 40 A, the value
   * Phoenix Tuner X's swerve generator writes, as the 2026 code kept it.
   *
   * <p>Top speed 4.4 m/s leaves the velocity control headroom under the free speed; spinning is
   * capped at a driver-friendly 1 rotation per second. kV is the ideal value for these motors and
   * gearing; kS and the drive's kP are starting values from simulation, and the steering's kP and
   * kD are the 2026 code's (100 V per rotation and 0.5, converted). Measure and tune them all on
   * the real robot.
   */
  private static final SwerveConfig DRIVETRAIN =
      SwerveConfig.builder()
          .modules(MODULES)
          .wheelInset(Inches.of(2.625))
          .limits(
              SwerveConfig.Limits.builder()
                  .driveStatorCurrent(Amps.of(30))
                  .driveSupplyCurrent(Amps.of(60))
                  .steerStatorCurrent(Amps.of(40))
                  .maxLinearSpeed(MetersPerSecond.of(4.4))
                  .maxAngularSpeed(RotationsPerSecond.of(1))
                  .build())
          .gains(
              SwerveConfig.Gains.builder()
                  .driveKs(0.0)
                  .driveKv(MODULES.idealDriveKv())
                  .driveKp(2.0)
                  .steerKp(16.0)
                  .steerKd(0.08)
                  .skewLookahead(Seconds.of(0.03))
                  .build())
          .build();

  /**
   * A 1280 x 800 global-shutter camera with an 80 degree diagonal field of view, typical for
   * AprilTags. The frame rate and latency are guesses: measure yours (the Vision log's {@code
   * LatencySeconds} for each camera). The latency matters on the robot, not only in the simulator,
   * which runs its cameras with it: an image arriving more than {@code
   * VisionConfig.latencyMarginSeconds} later than it is left out, so a guess too low drops images.
   */
  private static final CameraConfig.Sensor APRILTAG_CAMERA =
      CameraConfig.Sensor.builder()
          .widthPixels(1280)
          .heightPixels(800)
          .diagonalFieldOfView(Degrees.of(80))
          .frameRate(Hertz.of(30))
          .latency(Milliseconds.of(30))
          .build();

  /**
   * Four cameras as a 2026 competition robot mounted them: two facing forward and out at 23
   * degrees, two facing backward, all near the frame's sides about 21 in up and tilted up 8 to 12.5
   * degrees. Positions as that team's 2026 code gives them, from the robot's origin on the floor.
   */
  private static final List<CameraConfig> CAMERAS =
      List.of(
          CameraConfig.builder()
              .name("front-left")
              .forward(Inches.of(-7.75))
              .left(Inches.of(12.25))
              .up(Inches.of(21))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(10))
              .yaw(Degrees.of(23.162))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .build(),
          CameraConfig.builder()
              .name("front-right")
              .forward(Inches.of(-8.625))
              .left(Inches.of(-12.25))
              .up(Inches.of(20.875))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(8.2))
              .yaw(Degrees.of(-23.162))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .build(),
          CameraConfig.builder()
              .name("back-left")
              .forward(Inches.of(-10.5))
              .left(Inches.of(12))
              .up(Inches.of(21))
              .roll(Degrees.of(-2))
              .tiltUp(Degrees.of(10))
              .yaw(Degrees.of(180.985))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .build(),
          CameraConfig.builder()
              .name("back-right")
              .forward(Inches.of(-11.6875))
              .left(Inches.of(-12))
              .up(Inches.of(20.705))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(12.5))
              .yaw(Degrees.of(180.985))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .build());

  /**
   * The template's robot. It has no wiring: on real hardware its drivetrain stays still and says
   * so. Give a real robot its own entry, with its wiring from Phoenix Tuner X's swerve generator.
   */
  public static final RobotConfig SIM =
      RobotConfig.builder()
          .name("sim")
          .body(BODY)
          .drivetrain(DRIVETRAIN)
          .drivetrainWiring(Optional.empty())
          .cameras(CAMERAS)
          .vision(VisionConfig.DEFAULTS)
          .estimator(EstimatorConfig.DEFAULTS)
          .build();

  /** Every robot the team has. Add each real robot as it is built and measured. */
  public static List<RobotConfig> all() {
    return List.of(SIM);
  }

  /**
   * The robot this program is running on: the one place that decides, so a new robot is one line
   * here. With several robots, choose by something the robot itself knows (a device's serial
   * number, a file on its storage) rather than by editing this before each deploy.
   */
  public static RobotConfig current() {
    return SIM;
  }
}
