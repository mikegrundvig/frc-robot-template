package com.michaelgrundvig.frc.tuning;

import static com.michaelgrundvig.frc.tuning.Fixtures.machine;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.Test;

class ConfigTunerTest {
  @Test
  void findsEveryTunableValueByPath() {
    var tuner = new ConfigTuner<>(machine());

    assertThat(tuner.knobs())
        .extracting(ConfigTuner.Knob::path)
        .containsExactly(
            "drive.limits.current",
            "drive.limits.maxSpeed",
            "drive.gains.kP",
            "drive.gains.lookahead");
    assertThat(tuner.knobs())
        .extracting(ConfigTuner.Knob::unit)
        .as("measures in their committed unit; plain numbers without one")
        .containsExactly(Amps.symbol(), MetersPerSecond.symbol(), "", Seconds.symbol());
  }

  @Test
  void startsAtTheCommittedValues() {
    var tuner = new ConfigTuner<>(machine());

    assertThat(tuner.current()).isEqualTo(tuner.committed()).isEqualTo(machine());
    assertThat(tuner.value("drive.gains.kP")).isEqualTo(16);
    assertThat(tuner.value("drive.limits.current")).isCloseTo(40, within(1e-9));
    assertThat(tuner.changes()).isEmpty();
  }

  @Test
  void aTunedValueRebuildsTheConfiguration() {
    var tuner = new ConfigTuner<>(machine());

    tuner.set("drive.gains.kP", 18.5);
    tuner.set("drive.limits.current", 35);

    assertThat(tuner.current().drive().gains().kP()).isEqualTo(18.5);
    assertThat(tuner.current().drive().limits().current().in(Amps)).isCloseTo(35, within(1e-9));
    assertThat(tuner.current().drive().motor())
        .as("the rest is untouched")
        .isEqualTo(machine().drive().motor());
    assertThat(tuner.committed()).isEqualTo(machine());
    assertThat(tuner.value("drive.gains.kP")).isEqualTo(18.5);
    assertThat(tuner.committedValue("drive.gains.kP")).isEqualTo(16);
  }

  @Test
  void aRefusedValueLeavesTheConfigurationAsItWas() {
    var tuner = new ConfigTuner<>(machine());
    tuner.set("drive.gains.kP", 18.5);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> tuner.set("drive.gains.kP", -1))
        .withMessage("kP must be zero or more, got -1.0");
    // Checked across records too: the drive can't be told to outrun its motor.
    assertThatIllegalArgumentException()
        .isThrownBy(() -> tuner.set("drive.limits.maxSpeed", 10))
        .withMessage("maxSpeed is faster than the motor's free speed");

    assertThat(tuner.current().drive().gains().kP()).isEqualTo(18.5);
    assertThat(tuner.value("drive.limits.maxSpeed")).isCloseTo(4, within(1e-9));
  }

  @Test
  void listsWhatDiffersWithTheBuilderCallThatCommitsIt() {
    var tuner = new ConfigTuner<>(machine());

    tuner.set("drive.gains.kP", 18.5);
    tuner.set("drive.limits.current", 35);
    tuner.set("drive.limits.maxSpeed", 4.2);
    tuner.set("drive.gains.lookahead", 0.05);

    String amps = " " + Amps.symbol();
    String metersPerSecond = " " + MetersPerSecond.symbol();
    String seconds = " " + Seconds.symbol();
    assertThat(tuner.changes())
        .containsExactly(
            new ConfigTuner.Change(
                "drive.limits.current", "40.0" + amps, "35.0" + amps, ".current(Amps.of(35.0))"),
            new ConfigTuner.Change(
                "drive.limits.maxSpeed",
                "4.0" + metersPerSecond,
                "4.2" + metersPerSecond,
                ".maxSpeed(MetersPerSecond.of(4.2))"),
            new ConfigTuner.Change("drive.gains.kP", "16.0", "18.5", ".kP(18.5)"),
            new ConfigTuner.Change(
                "drive.gains.lookahead",
                "0.03" + seconds,
                "0.05" + seconds,
                ".lookahead(Seconds.of(0.05))"));
  }

  @Test
  void settingTheCommittedValueAgainIsNoChange() {
    var tuner = new ConfigTuner<>(machine());

    tuner.set("drive.gains.kP", 18.5);
    tuner.set("drive.gains.kP", 16);

    assertThat(tuner.changes()).isEmpty();
  }

  @Test
  void resetReturnsToTheCommittedConfiguration() {
    var tuner = new ConfigTuner<>(machine());
    tuner.set("drive.gains.kP", 18.5);

    tuner.reset();

    assertThat(tuner.current()).isEqualTo(machine());
    assertThat(tuner.value("drive.gains.kP")).isEqualTo(16);
    assertThat(tuner.changes()).isEmpty();
  }

  @Test
  void onlyTunableValuesCanBeTuned() {
    var tuner = new ConfigTuner<>(machine());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> tuner.set("drive.gains.kI", 1))
        .withMessage("no tunable value at drive.gains.kI");
    assertThatIllegalArgumentException()
        .as("a fact about the hardware")
        .isThrownBy(() -> tuner.value("drive.motor.freeSpeedMetersPerSecond"))
        .withMessage("no tunable value at drive.motor.freeSpeedMetersPerSecond");
  }

  @Test
  void aConfigurationWithNothingTunableHasNoKnobs() {
    assertThat(new ConfigTuner<>(new Fixtures.Motor(4.5)).knobs()).isEmpty();
  }

  /** Only numbers and measures can be dialed in. */
  record Named(@Tunable String name) {}

  @Test
  void onlyNumbersAndMeasuresCanBeMarkedTunable() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new ConfigTuner<>(new Named("front")))
        .withMessageStartingWith("name is marked @Tunable");
  }
}
