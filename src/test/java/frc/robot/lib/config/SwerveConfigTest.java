package frc.robot.lib.config;

import static frc.robot.lib.config.ConfigFixtures.body;
import static frc.robot.lib.config.ConfigFixtures.gains;
import static frc.robot.lib.config.ConfigFixtures.limits;
import static frc.robot.lib.config.ConfigFixtures.modules;
import static frc.robot.lib.config.ConfigFixtures.swerve;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatNoException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

class SwerveConfigTest {
  @Test
  void keepsItsSettings() {
    SwerveConfig swerve = swerve();

    assertThat(swerve.modules()).isEqualTo(modules());
    assertThat(swerve.wheelInset().in(Inches)).isCloseTo(2.625, within(1e-9));
    assertThat(swerve.limits()).isEqualTo(limits());
    assertThat(swerve.gains()).isEqualTo(gains());

    SwerveConfig.Modules modules = swerve.modules();
    assertThat(modules.driveMotor()).isEqualTo(Motor.KRAKEN_X60);
    assertThat(modules.steerMotor()).isEqualTo(Motor.KRAKEN_X44);
    assertThat(modules.wheelRadius().in(Inches)).isCloseTo(2, within(1e-9));
    assertThat(modules.driveGearing()).isEqualTo(6.75);
    assertThat(modules.steerGearing()).isEqualTo(12.8);
    assertThat(modules.wheelCoefficientOfFriction()).isEqualTo(1.2);

    SwerveConfig.Limits limits = swerve.limits();
    assertThat(limits.driveStatorCurrent().in(Amps)).isCloseTo(40, within(1e-9));
    assertThat(limits.driveSupplyCurrent().in(Amps)).isCloseTo(60, within(1e-9));
    assertThat(limits.steerStatorCurrent().in(Amps)).isCloseTo(30, within(1e-9));
    assertThat(limits.maxLinearSpeed().in(MetersPerSecond)).isCloseTo(4, within(1e-9));
    assertThat(limits.maxAngularSpeed().in(RotationsPerSecond)).isCloseTo(1, within(1e-9));

    SwerveConfig.Gains gains = swerve.gains();
    assertThat(gains.driveKs()).isEqualTo(0.1);
    assertThat(gains.driveKv()).isEqualTo(2.5);
    assertThat(gains.driveKp()).isEqualTo(2.0);
    assertThat(gains.steerKp()).isEqualTo(16.0);
    assertThat(gains.steerKd()).isEqualTo(0.08);
    assertThat(gains.skewLookahead().in(Seconds)).isCloseTo(0.03, within(1e-9));
  }

  @Test
  void freeSpeedComesFromTheMotorGearingAndWheel() {
    // A Kraken X60 turns 6000 rpm free: 100 rotations per second, divided by the gearing, times
    // the wheel's circumference.
    double expected = 6000.0 / 60 / 6.75 * 2 * Math.PI * Inches.of(2).in(Meters);

    assertThat(modules().freeSpeed().in(MetersPerSecond)).isCloseTo(expected, within(1e-9));
    assertThat(modules().idealDriveKv()).isCloseTo(12 / expected, within(1e-9));
  }

  @Test
  void wheelsSitInFromTheFrameCorners() {
    // A 28 x 24 in frame with wheels 2.625 in in from its edges.
    double x = Inches.of(14 - 2.625).in(Meters);
    double y = Inches.of(12 - 2.625).in(Meters);

    assertThat(swerve().moduleTranslations(body()))
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
    var wide = new SwerveConfig(modules(), Inches.of(12), limits(), gains());
    var deep = new SwerveConfig(modules(), Inches.of(14), limits(), gains());

    assertRejected("wheelInset", () -> wide.moduleTranslations(body()));
    assertRejected("wheelInset", () -> deep.moduleTranslations(body()));
  }

  @Test
  void topSpeedCannotExceedTheFreeSpeed() {
    var tooFast =
        new SwerveConfig.Limits(
            Amps.of(40), Amps.of(60), Amps.of(30), MetersPerSecond.of(5), RotationsPerSecond.of(1));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> new SwerveConfig(modules(), Inches.of(2.625), tooFast, gains()))
        .withMessageContaining("free speed");
  }

  @Test
  void rejectsImpossibleModuleValues() {
    var drive = Motor.KRAKEN_X60;
    var steer = Motor.KRAKEN_X44;

    assertRejected(
        "wheelRadius", () -> new SwerveConfig.Modules(drive, steer, Inches.of(0), 6.75, 12.8, 1.2));
    assertRejected(
        "driveGearing",
        () -> new SwerveConfig.Modules(drive, steer, Inches.of(2), -6.75, 12.8, 1.2));
    assertRejected(
        "steerGearing", () -> new SwerveConfig.Modules(drive, steer, Inches.of(2), 6.75, 0, 1.2));
    assertRejected(
        "wheelCoefficientOfFriction",
        () -> new SwerveConfig.Modules(drive, steer, Inches.of(2), 6.75, 12.8, Double.NaN));
    assertRejected(
        "wheelInset", () -> new SwerveConfig(modules(), Inches.of(-1), limits(), gains()));
  }

  @Test
  void limitsMustBePositive() {
    var amps = Amps.of(40);
    var speed = MetersPerSecond.of(4);
    var spin = RotationsPerSecond.of(1);

    assertRejected(
        "driveStatorCurrent", () -> new SwerveConfig.Limits(Amps.of(0), amps, amps, speed, spin));
    assertRejected(
        "driveSupplyCurrent", () -> new SwerveConfig.Limits(amps, Amps.of(-60), amps, speed, spin));
    assertRejected(
        "steerStatorCurrent",
        () -> new SwerveConfig.Limits(amps, amps, Amps.of(Double.NaN), speed, spin));
    assertRejected(
        "maxLinearSpeed",
        () -> new SwerveConfig.Limits(amps, amps, amps, MetersPerSecond.of(0), spin));
    assertRejected(
        "maxAngularSpeed",
        () ->
            new SwerveConfig.Limits(
                amps, amps, amps, speed, RotationsPerSecond.of(Double.POSITIVE_INFINITY)));
  }

  @Test
  void gainsMayBeZeroButNotNegative() {
    var none = Seconds.of(0);

    assertThatNoException().isThrownBy(() -> new SwerveConfig.Gains(0, 0, 0, 0, 0, none));
    assertRejected("driveKs", () -> new SwerveConfig.Gains(-0.1, 0, 0, 0, 0, none));
    assertRejected("driveKv", () -> new SwerveConfig.Gains(0, Double.NaN, 0, 0, 0, none));
    assertRejected("driveKp", () -> new SwerveConfig.Gains(0, 0, -1, 0, 0, none));
    assertRejected("steerKp", () -> new SwerveConfig.Gains(0, 0, 0, -1, 0, none));
    assertRejected("steerKd", () -> new SwerveConfig.Gains(0, 0, 0, 0, -1, none));
    assertRejected("skewLookahead", () -> new SwerveConfig.Gains(0, 0, 0, 0, 0, Seconds.of(-0.01)));
    assertRejected(
        "skewLookahead", () -> new SwerveConfig.Gains(0, 0, 0, 0, 0, Seconds.of(Double.NaN)));
  }

  private static void assertRejected(String field, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageStartingWith(field);
  }
}
