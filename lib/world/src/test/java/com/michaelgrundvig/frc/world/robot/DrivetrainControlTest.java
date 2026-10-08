package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatNoException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;

class DrivetrainControlTest {
  /** The template robot's drivetrain: SDS MK4i, L2, on Kraken X60s, 4 in wheels. */
  private static final DrivetrainGeometry TEMPLATES =
      DrivetrainGeometry.uniform(
          DrivetrainGeometry.ModuleType.builder()
              .driveMotor(Motor.KRAKEN_X60)
              .steerMotor(Motor.KRAKEN_X60)
              .wheelRadius(Inches.of(2))
              .driveGearing(Gearing.builder().ratio(6.75).efficiency(1.0).build())
              .steerGearing(Gearing.builder().ratio(150.0 / 7.0).efficiency(1.0).build())
              .couplingRatio(50.0 / 14.0)
              .wheelCoefficientOfFriction(1.2)
              .driveInertia(KilogramSquareMeters.of(0.001))
              .steerInertia(KilogramSquareMeters.of(0.004))
              .build(),
          Inches.of(10.375),
          Inches.of(10.375));

  @Test
  void keepsItsSettings() {
    DrivetrainControl control = control();

    assertThat(control.limits()).isEqualTo(limits());
    assertThat(control.gains()).isEqualTo(gains());

    DrivetrainControl.Limits limits = control.limits();
    assertThat(limits.driveStatorCurrent().in(Amps)).isCloseTo(40, within(1e-9));
    assertThat(limits.driveSupplyCurrent().in(Amps)).isCloseTo(60, within(1e-9));
    assertThat(limits.driveSupplyLowerCurrent().in(Amps)).isCloseTo(35, within(1e-9));
    assertThat(limits.driveSupplyLowerTime().in(Seconds)).isCloseTo(0.5, within(1e-9));
    assertThat(limits.steerStatorCurrent().in(Amps)).isCloseTo(30, within(1e-9));
    assertThat(limits.maxLinearSpeed().in(MetersPerSecond)).isCloseTo(4, within(1e-9));
    assertThat(limits.maxAngularSpeed().in(RotationsPerSecond)).isCloseTo(1, within(1e-9));

    DrivetrainControl.Gains gains = control.gains();
    assertThat(gains.driveKs()).isEqualTo(0.1);
    assertThat(gains.driveKv()).isEqualTo(2.5);
    assertThat(gains.driveKp()).isEqualTo(2.0);
    assertThat(gains.steerKp()).isEqualTo(16.0);
    assertThat(gains.steerKd()).isEqualTo(0.08);
    assertThat(gains.skewLookahead().in(Seconds)).isCloseTo(0.03, within(1e-9));
  }

  /** The template's drivetrain reaches 4.73 m/s free: 4 m/s leaves room, 5 m/s doesn't. */
  @Test
  void topSpeedCannotExceedTheFreeSpeed() {
    var drivetrain = TEMPLATES;
    var tooFast =
        DrivetrainControlBuilder.builder(control())
            .limits(changeLimits().maxLinearSpeed(MetersPerSecond.of(5)).build())
            .build();

    assertThat(control().whyTooFastFor(drivetrain)).isNull();
    assertThat(tooFast.whyTooFastFor(drivetrain))
        .isEqualTo(
            "5 m/s is faster than the drivetrain's free speed of 4.729 m/s: give maxLinearSpeed at"
                + " most that");
  }

  @Test
  void limitsMustBePositive() {
    assertRejected(
        "Limits.driveStatorCurrent", () -> changeLimits().driveStatorCurrent(Amps.of(0)).build());
    assertRejected(
        "Limits.driveSupplyCurrent", () -> changeLimits().driveSupplyCurrent(Amps.of(-60)).build());
    assertRejected(
        "Limits.driveSupplyLowerCurrent",
        () -> changeLimits().driveSupplyLowerCurrent(Amps.of(0)).build());
    assertRejected(
        "Limits.steerStatorCurrent",
        () -> changeLimits().steerStatorCurrent(Amps.of(Double.NaN)).build());
    assertRejected(
        "Limits.maxLinearSpeed",
        () -> changeLimits().maxLinearSpeed(MetersPerSecond.of(0)).build());
    assertRejected(
        "Limits.maxAngularSpeed",
        () ->
            changeLimits()
                .maxAngularSpeed(RotationsPerSecond.of(Double.POSITIVE_INFINITY))
                .build());
  }

  /**
   * Phoenix steps the drive's supply limit down after a time from just over zero to 5 s. At zero it
   * never steps down, which the template says by setting the lower current to the supply limit
   * instead, where it shows.
   */
  @Test
  void theStepDownTakesFromJustOverZeroToFiveSeconds() {
    assertRejected(
        "Limits.driveSupplyLowerTime",
        () -> changeLimits().driveSupplyLowerTime(Seconds.of(0)).build());
    assertRejected(
        "Limits.driveSupplyLowerTime",
        () -> changeLimits().driveSupplyLowerTime(Seconds.of(5.1)).build());
    assertThatNoException()
        .isThrownBy(() -> changeLimits().driveSupplyLowerTime(Seconds.of(5)).build());
    assertThatNoException()
        .as("no step-down, said where it shows")
        .isThrownBy(() -> changeLimits().driveSupplyLowerCurrent(Amps.of(60)).build());
  }

  @Test
  void gainsMayBeZeroButNotNegative() {
    assertThatNoException()
        .isThrownBy(
            () ->
                DrivetrainControl.Gains.builder()
                    .driveKs(0)
                    .driveKv(0)
                    .driveKp(0)
                    .steerKp(0)
                    .steerKd(0)
                    .skewLookahead(Seconds.of(0))
                    .headingKp(0)
                    .translationKp(0)
                    .build());
    assertRejected("Gains.driveKs", () -> changeGains().driveKs(-0.1).build());
    assertRejected("Gains.driveKv", () -> changeGains().driveKv(Double.NaN).build());
    assertRejected("Gains.driveKp", () -> changeGains().driveKp(-1).build());
    assertRejected("Gains.steerKp", () -> changeGains().steerKp(-1).build());
    assertRejected("Gains.headingKp", () -> changeGains().headingKp(-1).build());
    assertRejected("Gains.translationKp", () -> changeGains().translationKp(-1).build());
    assertRejected("Gains.steerKd", () -> changeGains().steerKd(-1).build());
    assertRejected(
        "Gains.skewLookahead", () -> changeGains().skewLookahead(Seconds.of(-0.01)).build());
    assertRejected(
        "Gains.skewLookahead", () -> changeGains().skewLookahead(Seconds.of(Double.NaN)).build());
  }

  private static DrivetrainControl.Limits limits() {
    return DrivetrainControl.Limits.builder()
        .driveStatorCurrent(Amps.of(40))
        .driveSupplyCurrent(Amps.of(60))
        .driveSupplyLowerCurrent(Amps.of(35))
        .driveSupplyLowerTime(Seconds.of(0.5))
        .steerStatorCurrent(Amps.of(30))
        .maxLinearSpeed(MetersPerSecond.of(4))
        .maxAngularSpeed(RotationsPerSecond.of(1))
        .build();
  }

  private static DrivetrainControl.Gains gains() {
    return DrivetrainControl.Gains.builder()
        .driveKs(0.1)
        .driveKv(2.5)
        .driveKp(2.0)
        .steerKp(16.0)
        .steerKd(0.08)
        .skewLookahead(Seconds.of(0.03))
        .headingKp(5.0)
        .translationKp(3.0)
        .build();
  }

  private static DrivetrainControl control() {
    return DrivetrainControl.builder().limits(limits()).gains(gains()).build();
  }

  private static DrivetrainControlLimitsBuilder changeLimits() {
    return DrivetrainControlLimitsBuilder.builder(limits());
  }

  private static DrivetrainControlGainsBuilder changeGains() {
    return DrivetrainControlGainsBuilder.builder(gains());
  }

  private static void assertRejected(String field, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageStartingWith(field);
  }
}
