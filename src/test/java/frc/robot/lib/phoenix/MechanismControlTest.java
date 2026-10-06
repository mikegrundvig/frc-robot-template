package frc.robot.lib.phoenix;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;

import java.util.Optional;
import org.junit.jupiter.api.Test;

/** How a mechanism is driven is checked when it's written down. */
class MechanismControlTest {
  @Test
  void seededGainsLeaveTheFeedforwardToTheDefinition() {
    MechanismControl.Gains gains = MechanismControl.Gains.seeded();

    assertThat(gains.kV()).isEmpty();
    assertThat(gains.kA()).isEmpty();
    assertThat(gains.kG()).isEmpty();
    assertThat(gains.kP()).isZero();
  }

  @Test
  void aGainIsNeverNegativeButKgMayBe() {
    assertThatThrownBy(
            () ->
                MechanismControl.Gains.builder()
                    .kP(-1)
                    .kI(0)
                    .kD(0)
                    .kS(0)
                    .kV(Optional.of(-0.1))
                    .kA(Optional.empty())
                    .kG(Optional.of(Double.NaN))
                    .build())
        .hasMessage(
            "Gains.kP: -1 must be zero or more\n"
                + "Gains.kV: -0.1 must be zero or more\n"
                + "Gains.kG: NaN must be a finite number");
    assertThat(
            MechanismControl.Gains.builder()
                .kP(0)
                .kI(0)
                .kD(0)
                .kS(0)
                .kV(Optional.empty())
                .kA(Optional.empty())
                .kG(Optional.of(-0.4)) // a counterbalance stronger than the arm
                .build()
                .kG())
        .contains(-0.4);
  }

  @Test
  void softLimitsGoUp() {
    assertThatThrownBy(() -> MechanismControl.SoftLimits.builder().lower(1).upper(1).build())
        .hasMessage("SoftLimits.upper: 1 must be above the lower limit (1)");
  }

  @Test
  void aProfileMoves() {
    assertThatThrownBy(
            () ->
                MechanismControl.Profile.builder()
                    .cruiseVelocity(0)
                    .acceleration(1)
                    .jerk(-1)
                    .build())
        .hasMessage(
            "Profile.cruiseVelocity: 0 must be positive\nProfile.jerk: -1 must be zero or more");
  }

  @Test
  void theSupplyLimitStepsDownWithinPhoenixsFiveSeconds() {
    assertThatThrownBy(
            () ->
                MechanismControl.CurrentLimits.builder()
                    .stator(Amps.of(40))
                    .supply(Amps.of(40))
                    .supplyLower(Amps.of(30))
                    .supplyLowerTime(Seconds.of(6))
                    .build())
        .hasMessage("CurrentLimits.supplyLowerTime: 6 s must be at most 5 s");
  }

  /** In a line: the feedforward seeded, the default limits, braking; a flywheel coasts. */
  @Test
  void aJointsControlStartsFromTheDefaults() {
    MechanismControl arm = MechanismControl.position(30);
    MechanismControl roller = MechanismControl.velocity(0.02);

    assertThat(arm.gains().kP()).isEqualTo(30);
    assertThat(arm.gains().kV()).as("seeded from the definition").isEmpty();
    assertThat(arm.gains().kG()).isEmpty();
    assertThat(arm.profile()).isEmpty();
    assertThat(arm.currentLimits()).isEqualTo(MechanismControl.CurrentLimits.DEFAULTS);
    assertThat(arm.neutralMode()).isEqualTo(MechanismControl.NeutralMode.BRAKE);
    assertThat(arm.homing()).isEmpty();
    assertThat(roller.neutralMode()).isEqualTo(MechanismControl.NeutralMode.BRAKE);
    assertThat(roller.coasting().neutralMode()).isEqualTo(MechanismControl.NeutralMode.COAST);
    assertThat(MechanismControl.CurrentLimits.DEFAULTS.stator()).isEqualTo(Amps.of(40));
  }

  /** Each change sets its one value and leaves the rest. */
  @Test
  void eachChangeSetsOneValue() {
    MechanismControl arm =
        MechanismControl.position(30)
            .withKd(1)
            .withProfile(8, 30)
            .withSoftLimits(0, 2)
            .withStatorLimit(Amps.of(60))
            .withHoming(MechanismControl.Homing.assumeAtReference());

    assertThat(arm.gains().kP()).isEqualTo(30);
    assertThat(arm.gains().kD()).isEqualTo(1);
    assertThat(arm.profile().orElseThrow().cruiseVelocity()).isEqualTo(8);
    assertThat(arm.profile().orElseThrow().acceleration()).isEqualTo(30);
    assertThat(arm.softLimits().orElseThrow().upper()).isEqualTo(2);
    assertThat(arm.currentLimits().stator()).isEqualTo(Amps.of(60));
    assertThat(arm.currentLimits().supply()).isEqualTo(Amps.of(60));
    assertThat(arm.homing()).contains(MechanismControl.Homing.assumeAtReference());
  }
}
