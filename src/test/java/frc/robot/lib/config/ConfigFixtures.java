package frc.robot.lib.config;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/** Valid configuration pieces for tests to start from, independent of the team's robots. */
public final class ConfigFixtures {
  private ConfigFixtures() {}

  /** A 28 x 24 in frame with 3.5 in bumpers (a 35 x 31 in outline), 30 in tall. */
  public static Body body() {
    return new Body(
        Inches.of(28),
        Inches.of(24),
        Inches.of(30),
        Inches.of(3.5),
        Kilograms.of(60),
        KilogramSquareMeters.of(6),
        Meters.of(0.2));
  }

  /** Kraken X60 drive at 6.75:1 on 2 in radius wheels: 4.73 m/s free speed. */
  public static SwerveConfig.Modules modules() {
    return new SwerveConfig.Modules(
        Motor.KRAKEN_X60, Motor.KRAKEN_X44, Inches.of(2), 6.75, 12.8, 1.2);
  }

  public static SwerveConfig.Limits limits() {
    return new SwerveConfig.Limits(
        Amps.of(40), Amps.of(60), Amps.of(30), MetersPerSecond.of(4), RotationsPerSecond.of(1));
  }

  public static SwerveConfig.Gains gains() {
    return new SwerveConfig.Gains(0.1, 2.5, 2.0, 16.0, 0.08, Seconds.of(0.03));
  }

  /** Wheels 2.625 in in from the frame's edges. */
  public static SwerveConfig swerve() {
    return new SwerveConfig(modules(), Inches.of(2.625), limits(), gains());
  }

  /** A forward-facing 1280 x 800 camera at the given position. */
  public static CameraConfig camera(String name, Translation3d position) {
    return new CameraConfig(
        name,
        new Transform3d(position, new Rotation3d()),
        1280,
        800,
        Degrees.of(80),
        Hertz.of(30),
        Milliseconds.of(30));
  }
}
