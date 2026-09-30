package com.michaelgrundvig.frc.pitwall;

import static com.michaelgrundvig.frc.pitwall.Fixtures.machine;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Volts;

import java.util.LinkedHashMap;
import java.util.Map;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

class LiveConfigTest {
  private static final String KP = "drive.gains.kP";

  @AfterEach
  void reset() {
    TunableRegistry.reset();
  }

  /** The test machine, its drive's gains and limits and its travel tunable. */
  private static LiveConfig<Fixtures.Machine> live() {
    return LiveConfig.of("machine", machine())
        .number(KP, Limits.between(0, 100))
        .number("drive.gains.lookahead", Limits.between(0, 0.1))
        .number("drive.limits.current", Limits.between(1, 80))
        .number("drive.limits.maxSpeed", Limits.atMost(10))
        .number("travel.min", Limits.NONE)
        .number("travel.max", Limits.NONE);
  }

  @Test
  void theChosenValuesAreItsKnobsWithTheirUnitsAndLimits() {
    assertThat(live().knobs())
        .containsExactly(
            new LiveConfig.Knob(KP, "", Limits.between(0, 100)),
            new LiveConfig.Knob("drive.gains.lookahead", "s", Limits.between(0, 0.1)),
            new LiveConfig.Knob("drive.limits.current", "A", Limits.between(1, 80)),
            new LiveConfig.Knob("drive.limits.maxSpeed", "m/s", Limits.atMost(10)),
            new LiveConfig.Knob("travel.min", "", Limits.NONE),
            new LiveConfig.Knob("travel.max", "", Limits.NONE));
  }

  @Test
  void aChangeRebuildsTheConfigurationThroughItsRecords() {
    var live = live();

    assertThat(live.set(Map.of(KP, 18.5, "drive.limits.current", 35.0))).isTrue();

    assertThat(live.get().drive().gains().kP()).isEqualTo(18.5);
    assertThat(live.value("drive.limits.current")).isEqualTo(35);
    assertThat(live.committed()).isEqualTo(machine());
  }

  /** The tee asks before it applies anything: asking changes nothing. */
  @Test
  void whyNotSaysWhyWithoutChangingAnything() {
    var live = live();

    assertThat(live.whyNot(Map.of(KP, 18.5))).isNull();
    assertThat(live.whyNot(Map.of(KP, -1.0))).isEqualTo(KP + ": -1 is below its limit of 0");
    assertThat(live.whyNot(Map.of("travel.min", 12.0)))
        .as("the record's own check")
        .isEqualTo("min must be below max, got 12.0 and 10.0");
    assertThat(live.whyNot(Map.of("drive.motor.freeSpeedMetersPerSecond", 9.0)))
        .isEqualTo("machine: drive.motor.freeSpeedMetersPerSecond isn't tunable");
    assertThat(live.get()).isEqualTo(machine());
  }

  /** Valid is not the same as safe: a gain of 500 passes the record's check, not its limit. */
  @Test
  void aValueTheRecordsWouldTakeIsStillRefusedOutsideItsLimits() {
    var live = live();

    assertThat(live.set(Map.of(KP, 500.0))).isFalse();

    assertThat(live.get()).isEqualTo(machine());
  }

  @Test
  void aCheckAcrossRecordsRefusesTheChange() {
    var live = live();

    assertThat(live.whyNot(Map.of("drive.limits.maxSpeed", 5.0)))
        .isEqualTo("maxSpeed is faster than the motor's free speed");
  }

  @Test
  void valuesValidOnlyTogetherChangeTogether() {
    var live = live();
    Map<String, Double> both = new LinkedHashMap<>();
    both.put("travel.min", 12.0);
    both.put("travel.max", 20.0);

    assertThat(live.set(both)).isTrue();

    assertThat(live.get().travel()).isEqualTo(new Fixtures.Range(12, 20));
  }

  @Test
  void severalValuesChangeAllOrNone() {
    var live = live();

    assertThat(live.set(Map.of(KP, 18.5, "drive.limits.current", -1.0))).isFalse();

    assertThat(live.get()).isEqualTo(machine());
  }

  @Test
  void listsWhatDiffersWithTheBuilderCallThatCommitsIt() {
    var live = live();
    live.set(Map.of(KP, 18.5, "drive.limits.current", 35.0));

    assertThat(live.changes())
        .containsExactly(
            new LiveConfig.Change(KP, "16.0", "18.5", ".kP(18.5)"),
            new LiveConfig.Change(
                "drive.limits.current", "40.0 A", "35.0 A", ".current(Amps.of(35.0))"));
    assertThat(live.report())
        .first()
        .asString()
        .contains("committed machine configuration")
        .contains("builder calls");
  }

  @Test
  void revertReturnsToTheCommittedConfiguration() {
    var live = live();
    live.set(Map.of(KP, 18.5));

    live.revert();

    assertThat(live.get()).isEqualTo(machine());
    assertThat(live.changes()).isEmpty();
  }

  @Test
  void onlyNumbersAndMeasuresThatExistCanBeChosenOnce() {
    var config = LiveConfig.of("machine", machine());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> config.number("drive.gains.kI", Limits.NONE))
        .withMessage("machine: nothing is at drive.gains.kI");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> config.number("name", Limits.NONE))
        .withMessageContaining("isn't a number or a measure");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> config.number("drive.gains", Limits.NONE))
        .withMessageContaining("isn't a number or a measure");
    config.number(KP, Limits.NONE);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> config.number(KP, Limits.NONE))
        .withMessageContaining("tunable already");
  }

  @Test
  void aCommittedValueOutsideItsOwnLimitsIsAMistake() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> LiveConfig.of("machine", machine()).number(KP, Limits.atMost(10)))
        .withMessageContaining("outside its tuning limits");
  }

  /** Published like any WPILib complex tunable: each value a tunable, its limits on it. */
  @Test
  void itsValuesAreWpilibTunablesCarryingTheirLimitsAndUnits() {
    var dashboard = new MockTunableBackend();
    TunableRegistry.registerBackend("", dashboard);

    TunableRegistry.publish("Machine", live());

    assertThat(dashboard.getDouble("Machine/drive/gains/kP")).isEqualTo(16);
    assertThat(Limits.of(dashboard.getTunable("Machine/drive/gains/kP").getConfig()))
        .isEqualTo(Limits.between(0, 100));
    assertThat(dashboard.getTunable("Machine/drive/limits/current").getConfig().getProperties())
        .contains("\"unit\":\"A\"");
  }

  /** WPILib's contract: a setter never throws. A refused value is declined, and nothing breaks. */
  @Test
  void withoutPitwallARefusedValueIsQuietlyDeclined() {
    var dashboard = new MockTunableBackend();
    TunableRegistry.registerBackend("", dashboard);
    var live = live();
    TunableRegistry.publish("Machine", live);

    dashboard.setDouble("Machine/drive/gains/kP", 500);
    TunableRegistry.update();
    assertThat(live.get().drive().gains().kP()).as("declined").isEqualTo(16);

    dashboard.setDouble("Machine/drive/gains/kP", 18);
    TunableRegistry.update();
    assertThat(live.get().drive().gains().kP()).isEqualTo(18);
  }

  @Test
  void noValuesChangeNothingAndTheCommittedValueAgainIsNoChange() {
    var live = live();

    assertThat(live.set(Map.of())).isTrue();
    assertThat(live.set(Map.of(KP, 16.0))).isTrue();

    assertThat(live.changes()).isEmpty();
  }

  /** A measure's limits are measures: they mean the same however its value was written. */
  @Test
  void aMeasuresLimitsAreMeasuresInAnyUnit() {
    var live =
        LiveConfig.of("machine", machine())
            .measure("drive.limits.current", Amps.of(1), Amps.of(80))
            .measure("drive.gains.lookahead", Milliseconds.of(0), Milliseconds.of(100));

    assertThat(live.knobs())
        .containsExactly(
            new LiveConfig.Knob("drive.limits.current", "A", Limits.between(1, 80)),
            new LiveConfig.Knob("drive.gains.lookahead", "s", Limits.between(0, 0.1)));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> LiveConfig.of("machine", machine()).measure(KP, Amps.of(0), Amps.of(1)))
        .withMessageContaining("isn't a measure of the same kind");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                LiveConfig.of("machine", machine())
                    .measure("drive.limits.current", Volts.of(0), Volts.of(1)))
        .withMessageContaining("isn't a measure of the same kind");
  }
}
