package frc.robot.lib.robot;

import static frc.robot.lib.robot.ConfigFixtures.body;
import static frc.robot.lib.robot.ConfigFixtures.drivetrain;
import static frc.robot.lib.robot.ConfigFixtures.gearing;
import static frc.robot.lib.robot.ConfigFixtures.modules;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

class DrivetrainGeometryTest {
  @Test
  void keepsItsSettings() {
    DrivetrainGeometry drivetrain = drivetrain();

    assertThat(drivetrain.modules()).isEqualTo(modules());
    assertThat(drivetrain.wheelInset().in(Inches)).isCloseTo(2.625, within(1e-9));

    DrivetrainGeometry.Modules modules = drivetrain.modules();
    assertThat(modules.driveMotor()).isEqualTo(Motor.KRAKEN_X60);
    assertThat(modules.steerMotor()).isEqualTo(Motor.KRAKEN_X44);
    assertThat(modules.wheelRadius().in(Inches)).isCloseTo(2, within(1e-9));
    assertThat(modules.driveGearing()).isEqualTo(gearing(6.75));
    assertThat(modules.steerGearing()).isEqualTo(gearing(12.8));
    assertThat(modules.wheelCoefficientOfFriction()).isEqualTo(1.2);
    assertThat(modules.driveInertia().in(KilogramSquareMeters)).isEqualTo(0.001);
    assertThat(modules.steerInertia().in(KilogramSquareMeters)).isEqualTo(0.004);
  }

  @Test
  void freeSpeedComesFromTheMotorGearingAndWheel() {
    // A Kraken X60 turns 6000 rpm free: 100 rotations per second, divided by the gearing, times
    // the wheel's circumference.
    double expected = 6000.0 / 60 / 6.75 * 2 * Math.PI * Inches.of(2).in(Meters);

    assertThat(modules().freeSpeed().in(MetersPerSecond)).isCloseTo(expected, within(1e-9));
    assertThat(modules().idealDriveKv()).isCloseTo(12 / expected, within(1e-9));
  }

  /** The drive and the steering are each one motor through their own gearing. */
  @Test
  void eachWheelIsDrivenAndTurnedByOneMotor() {
    assertThat(modules().drive())
        .isEqualTo(
            Actuator.Motors.builder()
                .motor(Motor.KRAKEN_X60)
                .count(1)
                .gearing(gearing(6.75))
                .build());
    assertThat(modules().steer().motor()).isEqualTo(Motor.KRAKEN_X44);
    assertThat(modules().steer().count()).isEqualTo(1);
    assertThat(modules().steer().gearing()).isEqualTo(gearing(12.8));
  }

  @Test
  void wheelsSitInFromTheFrameCorners() {
    // A 28 x 24 in frame with wheels 2.625 in in from its edges.
    double x = Inches.of(14 - 2.625).in(Meters);
    double y = Inches.of(12 - 2.625).in(Meters);

    assertThat(drivetrain().moduleTranslations(body()))
        .as("front left, front right, back left, back right")
        .containsExactly(
            new Translation2d(x, y),
            new Translation2d(x, -y),
            new Translation2d(-x, y),
            new Translation2d(-x, -y));
  }

  @Test
  void wheelsMustFitInsideTheFrame() {
    // The frame is 28 in long and 24 in wide: a 12 in inset meets in the middle side to side,
    // and a 14 in inset meets in the middle front to back.
    var wide = DrivetrainGeometryBuilder.builder(drivetrain()).wheelInset(Inches.of(12)).build();
    var deep = DrivetrainGeometryBuilder.builder(drivetrain()).wheelInset(Inches.of(14)).build();

    assertThat(wide.whyWheelsDontFit(body())).contains("center of the frame");
    assertRejected("wheelInset", () -> wide.moduleTranslations(body()));
    assertRejected("wheelInset", () -> deep.moduleTranslations(body()));
  }

  @Test
  void rejectsImpossibleModuleValues() {
    assertRejected("Modules.wheelRadius", () -> changeModules().wheelRadius(Inches.of(0)).build());
    assertRejected(
        "Modules.wheelCoefficientOfFriction",
        () -> changeModules().wheelCoefficientOfFriction(Double.NaN).build());
    assertRejected(
        "Modules.driveInertia",
        () -> changeModules().driveInertia(KilogramSquareMeters.of(0)).build());
    assertRejected(
        "Modules.steerInertia",
        () -> changeModules().steerInertia(KilogramSquareMeters.of(-0.004)).build());
    assertRejected(
        "DrivetrainGeometry.wheelInset",
        () -> DrivetrainGeometryBuilder.builder(drivetrain()).wheelInset(Inches.of(-1)).build());
  }

  private static DrivetrainGeometryModulesBuilder changeModules() {
    return DrivetrainGeometryModulesBuilder.builder(modules());
  }

  private static void assertRejected(String field, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageStartingWith(field);
  }
}
