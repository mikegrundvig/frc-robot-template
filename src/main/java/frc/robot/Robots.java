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
import java.util.List;

/**
 * The team's robots, one {@link RobotConfig} each. A test builds every one, so an invalid value
 * fails the build rather than the robot.
 *
 * <p>Every value is set by name through a builder, which asks for each in turn: none can be left
 * out or land in the wrong place.
 *
 * <p>The template's robot is a typical competition build, at the 2026 rule maximums where the rules
 * set one (the next season's rules replace them at kickoff). Replace each value with your robot's
 * as you design and measure it.
 */
public final class Robots {
  private Robots() {}

  /**
   * Frame 27.5 x 27.5 in, the largest square within 2026's 110 in frame perimeter (R104), with
   * bumpers 4 in beyond it (R403). Mass 149.5 lb: 135 lb of robot and bumpers (R408) plus a 14.5 lb
   * battery (R601). The 30 in height and 6.7 kg m^2 yaw inertia are typical values, not
   * measurements. The center of gravity is centered and low: the best case.
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
   * a simple weight-transfer estimate they start to slip near 47 A with new tread and this low
   * center of gravity, or near 31 A with worn tread (friction 1.0) and a center of gravity at 0.3
   * m. More current than that only spins the wheels and drains the battery. Supply limit 60 A per
   * drive motor; steering needs little torque, so 40 A.
   *
   * <p>Top speed 4.4 m/s leaves the velocity control headroom under the free speed; spinning is
   * capped at a driver-friendly 1 rotation per second. kV is the ideal value for these motors and
   * gearing; the other gains are starting values from simulation. Measure and tune them all on the
   * real robot.
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
   * AprilTags. The frame rate and latency are conservative guesses; measure yours.
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
   * Four cameras, one near each corner, 9 in up and tilted 20 degrees up, facing out diagonally so
   * that together they see all the way around the robot.
   */
  private static final List<CameraConfig> CAMERAS =
      List.of(
          CameraConfig.builder()
              .name("front-left")
              .forward(Inches.of(11.5))
              .left(Inches.of(11.5))
              .up(Inches.of(9))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(20))
              .yaw(Degrees.of(45))
              .sensor(APRILTAG_CAMERA)
              .build(),
          CameraConfig.builder()
              .name("front-right")
              .forward(Inches.of(11.5))
              .left(Inches.of(-11.5))
              .up(Inches.of(9))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(20))
              .yaw(Degrees.of(-45))
              .sensor(APRILTAG_CAMERA)
              .build(),
          CameraConfig.builder()
              .name("back-left")
              .forward(Inches.of(-11.5))
              .left(Inches.of(11.5))
              .up(Inches.of(9))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(20))
              .yaw(Degrees.of(135))
              .sensor(APRILTAG_CAMERA)
              .build(),
          CameraConfig.builder()
              .name("back-right")
              .forward(Inches.of(-11.5))
              .left(Inches.of(-11.5))
              .up(Inches.of(9))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(20))
              .yaw(Degrees.of(-135))
              .sensor(APRILTAG_CAMERA)
              .build());

  /** The template's robot. */
  public static final RobotConfig SIM =
      RobotConfig.builder().name("sim").body(BODY).drivetrain(DRIVETRAIN).cameras(CAMERAS).build();

  /** Every robot the team has. Add each real robot as it is built and measured. */
  public static List<RobotConfig> all() {
    return List.of(SIM);
  }
}
