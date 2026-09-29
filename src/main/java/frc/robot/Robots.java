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
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * The team's robots, one {@link RobotConfig} each. A test builds every one, so an invalid value
 * fails the build rather than the robot.
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
      new Body(
          Inches.of(27.5),
          Inches.of(27.5),
          Inches.of(30),
          Inches.of(4),
          Pounds.of(149.5),
          KilogramSquareMeters.of(6.7),
          Meters.of(0.15));

  /**
   * SDS MK4i modules with L2 gearing (6.75:1 drive, 150/7:1 steer) and 4 in wheels, driven by
   * Kraken X60s without FOC: it needs a Phoenix Pro license, and under this drivetrain's 30 A limit
   * it adds nothing. Free speed: 4.73 m/s.
   */
  private static final SwerveConfig.Modules MODULES =
      new SwerveConfig.Modules(
          Motor.KRAKEN_X60, Motor.KRAKEN_X60, Inches.of(2), 6.75, 150.0 / 7.0, 1.2);

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
      new SwerveConfig(
          MODULES,
          Inches.of(2.625),
          new SwerveConfig.Limits(
              Amps.of(30),
              Amps.of(60),
              Amps.of(40),
              MetersPerSecond.of(4.4),
              RotationsPerSecond.of(1)),
          new SwerveConfig.Gains(0.0, MODULES.idealDriveKv(), 2.0, 16.0, 0.08, Seconds.of(0.03)));

  /**
   * Four AprilTag cameras, one near each corner, facing out diagonally so that together they see
   * all the way around the robot.
   */
  private static final List<CameraConfig> CAMERAS =
      List.of(
          cornerCamera("front-left", Inches.of(11.5), Inches.of(11.5), Degrees.of(45)),
          cornerCamera("front-right", Inches.of(11.5), Inches.of(-11.5), Degrees.of(-45)),
          cornerCamera("back-left", Inches.of(-11.5), Inches.of(11.5), Degrees.of(135)),
          cornerCamera("back-right", Inches.of(-11.5), Inches.of(-11.5), Degrees.of(-135)));

  /** The template's robot. */
  public static final RobotConfig SIM = new RobotConfig("sim", BODY, DRIVETRAIN, CAMERAS);

  /** Every robot the team has. Add each real robot as it is built and measured. */
  public static List<RobotConfig> all() {
    return List.of(SIM);
  }

  /**
   * A camera 9 in up, tilted 20 degrees up. A 1280 x 800 global-shutter camera with an 80 degree
   * diagonal field of view is typical for AprilTags; the frame rate and latency are conservative
   * guesses.
   */
  private static CameraConfig cornerCamera(String name, Distance x, Distance y, Angle yaw) {
    return new CameraConfig(
        name,
        new Transform3d(x, y, Inches.of(9), new Rotation3d(Degrees.of(0), Degrees.of(-20), yaw)),
        1280,
        800,
        Degrees.of(80),
        Hertz.of(30),
        Milliseconds.of(30));
  }
}
