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

import org.wpilib.math.geometry.Translation3d;

/**
 * Valid configuration pieces for tests to start from, independent of the team's robots. To change a
 * value, copy one: {@code BodyBuilder.builder(body()).frameLength(...).build()}.
 */
public final class ConfigFixtures {
  private ConfigFixtures() {}

  /** A 28 x 24 in frame with 3.5 in bumpers (a 35 x 31 in outline), 30 in tall. */
  public static Body body() {
    return Body.builder()
        .frameLength(Inches.of(28))
        .frameWidth(Inches.of(24))
        .height(Inches.of(30))
        .bumperThickness(Inches.of(3.5))
        .mass(Kilograms.of(60))
        .yawMomentOfInertia(KilogramSquareMeters.of(6))
        .cgHeight(Meters.of(0.2))
        .build();
  }

  /** Kraken X60 drive at 6.75:1 on 2 in radius wheels: 4.73 m/s free speed. */
  public static SwerveConfig.Modules modules() {
    return SwerveConfig.Modules.builder()
        .driveMotor(Motor.KRAKEN_X60)
        .steerMotor(Motor.KRAKEN_X44)
        .wheelRadius(Inches.of(2))
        .driveGearing(6.75)
        .steerGearing(12.8)
        .wheelCoefficientOfFriction(1.2)
        .build();
  }

  public static SwerveConfig.Limits limits() {
    return SwerveConfig.Limits.builder()
        .driveStatorCurrent(Amps.of(40))
        .driveSupplyCurrent(Amps.of(60))
        .driveSupplyLowerCurrent(Amps.of(35))
        .driveSupplyLowerTime(Seconds.of(0.5))
        .steerStatorCurrent(Amps.of(30))
        .maxLinearSpeed(MetersPerSecond.of(4))
        .maxAngularSpeed(RotationsPerSecond.of(1))
        .build();
  }

  public static SwerveConfig.Gains gains() {
    return SwerveConfig.Gains.builder()
        .driveKs(0.1)
        .driveKv(2.5)
        .driveKp(2.0)
        .steerKp(16.0)
        .steerKd(0.08)
        .skewLookahead(Seconds.of(0.03))
        .build();
  }

  /** Wheels 2.625 in in from the frame's edges. */
  public static SwerveConfig swerve() {
    return SwerveConfig.builder()
        .modules(modules())
        .wheelInset(Inches.of(2.625))
        .limits(limits())
        .gains(gains())
        .build();
  }

  /** A 1280 x 800 camera with an 80 degree field of view. */
  public static CameraConfig.Sensor sensor() {
    return CameraConfig.Sensor.builder()
        .widthPixels(1280)
        .heightPixels(800)
        .diagonalFieldOfView(Degrees.of(80))
        .frameRate(Hertz.of(30))
        .latency(Milliseconds.of(30))
        .sightRange(Meters.of(7))
        .build();
  }

  /** A level, forward-facing camera at the given position. */
  public static CameraConfig camera(String name, Translation3d position) {
    return CameraConfig.builder()
        .name(name)
        .forward(position.getMeasureX())
        .left(position.getMeasureY())
        .up(position.getMeasureZ())
        .roll(Degrees.of(0))
        .tiltUp(Degrees.of(0))
        .yaw(Degrees.of(0))
        .sensor(sensor())
        .noiseScale(1.0)
        .build();
  }
}
