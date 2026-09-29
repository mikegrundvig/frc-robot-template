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
    var wide = SwerveConfigBuilder.builder(swerve()).wheelInset(Inches.of(12)).build();
    var deep = SwerveConfigBuilder.builder(swerve()).wheelInset(Inches.of(14)).build();

    assertRejected("wheelInset", () -> wide.moduleTranslations(body()));
    assertRejected("wheelInset", () -> deep.moduleTranslations(body()));
  }

  @Test
  void topSpeedCannotExceedTheFreeSpeed() {
    var tooFast =
        SwerveConfigLimitsBuilder.builder(limits()).maxLinearSpeed(MetersPerSecond.of(5)).build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> SwerveConfigBuilder.builder(swerve()).limits(tooFast).build())
        .withMessageContaining("free speed");
  }

  @Test
  void rejectsImpossibleModuleValues() {
    assertRejected("wheelRadius", () -> changeModules().wheelRadius(Inches.of(0)).build());
    assertRejected("driveGearing", () -> changeModules().driveGearing(-6.75).build());
    assertRejected("steerGearing", () -> changeModules().steerGearing(0).build());
    assertRejected(
        "wheelCoefficientOfFriction",
        () -> changeModules().wheelCoefficientOfFriction(Double.NaN).build());
    assertRejected(
        "wheelInset",
        () -> SwerveConfigBuilder.builder(swerve()).wheelInset(Inches.of(-1)).build());
  }

  @Test
  void limitsMustBePositive() {
    assertRejected(
        "driveStatorCurrent", () -> changeLimits().driveStatorCurrent(Amps.of(0)).build());
    assertRejected(
        "driveSupplyCurrent", () -> changeLimits().driveSupplyCurrent(Amps.of(-60)).build());
    assertRejected(
        "steerStatorCurrent", () -> changeLimits().steerStatorCurrent(Amps.of(Double.NaN)).build());
    assertRejected(
        "maxLinearSpeed", () -> changeLimits().maxLinearSpeed(MetersPerSecond.of(0)).build());
    assertRejected(
        "maxAngularSpeed",
        () ->
            changeLimits()
                .maxAngularSpeed(RotationsPerSecond.of(Double.POSITIVE_INFINITY))
                .build());
  }

  @Test
  void gainsMayBeZeroButNotNegative() {
    assertThatNoException()
        .isThrownBy(
            () ->
                SwerveConfig.Gains.builder()
                    .driveKs(0)
                    .driveKv(0)
                    .driveKp(0)
                    .steerKp(0)
                    .steerKd(0)
                    .skewLookahead(Seconds.of(0))
                    .build());
    assertRejected("driveKs", () -> changeGains().driveKs(-0.1).build());
    assertRejected("driveKv", () -> changeGains().driveKv(Double.NaN).build());
    assertRejected("driveKp", () -> changeGains().driveKp(-1).build());
    assertRejected("steerKp", () -> changeGains().steerKp(-1).build());
    assertRejected("steerKd", () -> changeGains().steerKd(-1).build());
    assertRejected("skewLookahead", () -> changeGains().skewLookahead(Seconds.of(-0.01)).build());
    assertRejected(
        "skewLookahead", () -> changeGains().skewLookahead(Seconds.of(Double.NaN)).build());
  }

  private static SwerveConfigModulesBuilder changeModules() {
    return SwerveConfigModulesBuilder.builder(modules());
  }

  private static SwerveConfigLimitsBuilder changeLimits() {
    return SwerveConfigLimitsBuilder.builder(limits());
  }

  private static SwerveConfigGainsBuilder changeGains() {
    return SwerveConfigGainsBuilder.builder(gains());
  }

  private static void assertRejected(String field, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageStartingWith(field);
  }
}
