package com.michaelgrundvig.frc.simulation;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Motor;
import java.util.List;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;

/**
 * A robot for the simulation's own tests, independent of any team's: a 27.5 in square frame of 68
 * kg on four Kraken X60 swerve modules at 6.75:1 on 2 in wheels, and motor controllers that drive
 * it the simplest way ({@link Controllers}).
 */
final class TestRobot {
  static final Body BODY =
      Body.builder()
          .frameLength(Inches.of(27.5))
          .frameWidth(Inches.of(27.5))
          .height(Inches.of(30))
          .bumperThickness(Inches.of(3.5))
          .mass(Kilograms.of(68))
          .yawMomentOfInertia(KilogramSquareMeters.of(6.7))
          .cgHeight(Meters.of(0.15))
          .build();

  static final DrivetrainGeometry DRIVETRAIN =
      DrivetrainGeometry.builder()
          .modules(
              DrivetrainGeometry.Modules.builder()
                  .driveMotor(Motor.KRAKEN_X60)
                  .steerMotor(Motor.KRAKEN_X60)
                  .wheelRadius(Inches.of(2))
                  .driveGearing(Gearing.builder().ratio(6.75).efficiency(1.0).build())
                  .steerGearing(Gearing.builder().ratio(150.0 / 7).efficiency(1.0).build())
                  .wheelCoefficientOfFriction(1.2)
                  .driveInertia(KilogramSquareMeters.of(0.001))
                  .steerInertia(KilogramSquareMeters.of(0.004))
                  .build())
          .wheelInset(Inches.of(2.625))
          .build();

  static final double RADIUS = DRIVETRAIN.modules().wheelRadius().in(Meters);
  static final DCMotor MOTOR = DRIVETRAIN.modules().driveMotor().model();
  static final double GEARING = DRIVETRAIN.modules().driveGearing().ratio();

  private TestRobot() {}

  /** Today's physics for the robot, its treads as configured. */
  static KinematicPhysics physics() {
    return new KinematicPhysics(BODY, DRIVETRAIN, List.of(1.0, 1.0, 1.0, 1.0));
  }

  /** Where each wheel sits, from the robot's center. */
  static Translation2d[] wheels() {
    return DRIVETRAIN.moduleTranslations(BODY);
  }

  /**
   * Motor controllers for the tests: every drive motor asked for one voltage, within the battery's
   * (12 V unless sagged), under one stator current limit, which the physics holds; each wheel
   * steered where told, at once.
   */
  static final class Controllers implements Actuators {
    double volts;
    double limitAmps = 30;
    double batteryVolts = 12;
    final double[] steer = new double[4];

    /** Every wheel steered this way. */
    Controllers steer(double radians) {
      java.util.Arrays.fill(steer, radians);
      return this;
    }

    @Override
    public void actuate(double seconds, Physics physics, WheelDrive[] drives) {
      double asked = Math.clamp(volts, -batteryVolts, batteryVolts);
      for (int i = 0; i < drives.length; i++) {
        drives[i].drive(asked, 0, limitAmps, batteryVolts);
        drives[i].steer(steer[i]);
      }
    }
  }
}
