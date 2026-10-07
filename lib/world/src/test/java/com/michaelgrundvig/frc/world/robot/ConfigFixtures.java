package com.michaelgrundvig.frc.world.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.physical.Inertia;
import java.util.List;
import org.wpilib.math.geometry.Translation3d;

/**
 * Valid pieces of a robot's definition for tests to start from, independent of the team's robots.
 * To change a value, copy one: {@code BodyBuilder.builder(body()).frameLength(...).build()}.
 */
public final class ConfigFixtures {
  private ConfigFixtures() {}

  /** A 28 x 24 in frame with 3.5 in bumpers (a 35 x 31 in outline), 30 in tall. */
  public static Body body() {
    return Body.builder()
        .frameLength(Inches.of(28))
        .frameWidth(Inches.of(24))
        .height(Inches.of(30))
        .bumperEnds(Inches.of(3.5))
        .bumperSides(Inches.of(3.5))
        .bumperBottom(Inches.of(2.5))
        .bumperTop(Inches.of(5.75))
        .weight(Body.Weight.weighed(Kilograms.of(60)))
        .inertia(
            Body.Turning.tensor(
                Inertia.builder().ixx(6).iyy(6).izz(6).ixy(0).ixz(0).iyz(0).build()))
        .cg(new Translation3d(0, 0, 0.2))
        .build();
  }

  /** A reduction that loses nothing. */
  public static Gearing gearing(double ratio) {
    return Gearing.builder().ratio(ratio).efficiency(1.0).build();
  }

  /** Kraken X60 drive at 6.75:1 on 2 in radius wheels: 4.73 m/s free speed. */
  public static DrivetrainGeometry.ModuleType modules() {
    return DrivetrainGeometry.ModuleType.builder()
        .driveMotor(Motor.KRAKEN_X60)
        .steerMotor(Motor.KRAKEN_X44)
        .wheelRadius(Inches.of(2))
        .driveGearing(gearing(6.75))
        .steerGearing(gearing(12.8))
        .couplingRatio(0)
        .wheelCoefficientOfFriction(1.2)
        .driveInertia(KilogramSquareMeters.of(0.001))
        .steerInertia(KilogramSquareMeters.of(0.004))
        .build();
  }

  /** Wheels 2.625 in in from the frame's edges. */
  public static DrivetrainGeometry drivetrain() {
    return DrivetrainGeometry.inset(modules(), body(), Inches.of(2.625));
  }

  /** A 1280 x 800 camera with an 80 degree field of view. */
  public static CameraConfig.Sensor sensor() {
    return CameraConfig.Sensor.builder()
        .widthPixels(1280)
        .heightPixels(800)
        .fieldOfView(Degrees.of(80))
        .fieldOfViewAcross(CameraConfig.Across.DIAGONAL)
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

  /** The body and drivetrain above, with these cameras, and IMUs not yet measured. */
  public static RobotDefinition definition(List<CameraConfig> cameras) {
    return RobotDefinition.builder()
        .name("test")
        .body(body())
        .drivetrain(drivetrain())
        .imus(Imus.UNCHECKED)
        .cameras(cameras)
        .build();
  }
}
