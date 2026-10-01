package com.michaelgrundvig.frc.pitwall.experiment;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.Test;

class ProfilesTest {
  private static final double QUARTER_TURN = Math.PI / 2;

  @Test
  void aStepAndBackLeadsInGoesThereHoldsAndComesBack() {
    Profile profile =
        Profiles.stepAndBack().from(Degrees.of(0)).to(Degrees.of(90)).hold(Seconds.of(0.8));

    assertThat(profile.baseUnit()).isEqualTo(Radians);
    assertThat(profile.setpoint(0)).isZero();
    assertThat(profile.setpoint(0.49)).as("the lead-in").isZero();
    assertThat(profile.setpoint(0.5)).isCloseTo(QUARTER_TURN, within(1e-12));
    assertThat(profile.setpoint(1.29)).isCloseTo(QUARTER_TURN, within(1e-12));
    assertThat(profile.setpoint(1.3)).isZero();
    assertThat(profile.setpoint(5)).as("after the end, the last setpoint").isZero();
    assertThat(profile.durationSeconds()).isCloseTo(2.1, within(1e-12));
    assertThat(profile.edges())
        .containsExactly(
            new Profile.Edge(0.5, 0, QUARTER_TURN), new Profile.Edge(1.3, QUARTER_TURN, 0));
  }

  @Test
  void aStepStaysThere() {
    Profile profile =
        Profiles.step()
            .from(Meters.of(0.2))
            .to(Meters.of(0.6))
            .hold(Seconds.of(1))
            .leadIn(Seconds.of(0.25));

    assertThat(profile.setpoint(0.1)).isEqualTo(0.2);
    assertThat(profile.setpoint(0.3)).isEqualTo(0.6);
    assertThat(profile.setpoint(9)).isEqualTo(0.6);
    assertThat(profile.durationSeconds()).isEqualTo(1.25);
    assertThat(profile.edges()).containsExactly(new Profile.Edge(0.25, 0.2, 0.6));
  }

  @Test
  void refusesAStepThatMixesQuantitiesGoesNowhereOrNeverHolds() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Profiles.step().from(Degrees.of(0)).to(Meters.of(1)).hold(Seconds.of(1)))
        .withMessageContaining("same kind of quantity");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> Profiles.step().from(Degrees.of(90)).to(Degrees.of(90)).hold(Seconds.of(1)))
        .withMessageContaining("goes somewhere");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> Profiles.step().from(Degrees.of(0)).to(Degrees.of(90)).hold(Seconds.of(0)))
        .withMessageContaining("holds for some time");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Profiles.step()
                    .from(Degrees.of(0))
                    .to(Degrees.of(90))
                    .hold(Seconds.of(1))
                    .leadIn(Seconds.of(-1)))
        .withMessageContaining("can't be negative");
  }

  @Test
  void saysWhatItIs() {
    assertThat(
            Profiles.stepAndBack()
                .from(Degrees.of(0))
                .to(Degrees.of(90))
                .hold(Seconds.of(0.8))
                .toString())
        .isEqualTo("step and back, 0 ° to 90 °, held 0.8 s");
    assertThat(
            Profiles.step()
                .from(Radians.of(0.25))
                .to(Radians.of(1.5708))
                .hold(Seconds.of(1))
                .toString())
        .isEqualTo("step, 0.25 rad to 1.5708 rad, held 1 s");
  }
}
