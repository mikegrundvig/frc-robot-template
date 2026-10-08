package com.michaelgrundvig.frc.pitwall;

import static com.michaelgrundvig.frc.pitwall.Fixtures.machine;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.tuple;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Volts;

import io.avaje.jsonb.Jsonb;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
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

  /** A property of a published tunable, from the JSON its configuration carries them in. */
  private static String property(MockTunableBackend dashboard, String path, String name) {
    Object properties =
        Jsonb.builder()
            .build()
            .type(Object.class)
            .fromJson(dashboard.getTunable(path).getConfig().getProperties());
    return String.valueOf(((Map<?, ?>) properties).get(name));
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
    assertThat(live.whyNot(Map.of(KP, -1.0)))
        .isEqualTo("machine: " + KP + ": -1 is below its limit of 0");
    assertThat(live.whyNot(Map.of("travel.min", 12.0)))
        .as("the record's own check")
        .isEqualTo("machine: min must be below max, got 12.0 and 10.0");
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
        .isEqualTo("machine: maxSpeed is faster than the motor's free speed");
  }

  @Test
  void aCandidateIsTheConfigurationAChangeWouldMakeWithoutMakingIt() {
    var live = live();

    Fixtures.Machine candidate = live.candidate(Map.of(KP, 18.5));

    assertThat(candidate.drive().gains().kP()).isEqualTo(18.5);
    assertThat(live.get()).isEqualTo(machine());
  }

  @Test
  void aRefusalGivesEveryReasonUnderTheConfigurationsName() {
    var live = live();
    Map<String, Double> values = new LinkedHashMap<>();
    values.put(KP, -1.0);
    values.put("drive.limits.current", 500.0);
    values.put("travel.middle", 3.0);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> live.candidate(values))
        .withMessage(
            "machine: drive.gains.kP: -1 is below its limit of 0\n"
                + "machine: drive.limits.current: 500 A is above its limit of 80 A\n"
                + "machine: travel.middle isn't tunable");
  }

  /** A record that reports each value it rejects on a line, as the robot's records do. */
  record Pair(double a, double b) {
    Pair {
      if (a < 0 && b < 0) {
        throw new IllegalArgumentException("Pair.a: must be zero or more\nPair.b: must be too");
      }
    }
  }

  @Test
  void eachLineOfARecordsRefusalIsUnderTheConfigurationsName() {
    var live =
        LiveConfig.of("pair", new Pair(1, 1)).number("a", Limits.NONE).number("b", Limits.NONE);

    assertThat(live.whyNot(Map.of("a", -1.0, "b", -1.0)))
        .isEqualTo("pair: Pair.a: must be zero or more\npair: Pair.b: must be too");
  }

  /** The page shows each reason beside the value it names: the reasons say which. */
  @Test
  void aRecordsRefusalIsTiedToEachValueItNames() {
    var live =
        LiveConfig.of("pair", new Pair(1, 1)).number("a", Limits.NONE).number("b", Limits.NONE);
    List<LiveConfig.Problem> problems = new ArrayList<>();

    assertThat(live.candidate(Map.of("a", -1.0, "b", -1.0), problems)).isNull();

    assertThat(problems)
        .containsExactly(
            new LiveConfig.Problem(
                Outcome.Layer.RECORD,
                List.of("a"),
                "LiveConfigTest.Pair",
                "",
                "must be zero or more",
                "pair: Pair.a: must be zero or more"),
            new LiveConfig.Problem(
                Outcome.Layer.RECORD,
                List.of("b"),
                "LiveConfigTest.Pair",
                "",
                "must be too",
                "pair: Pair.b: must be too"));
  }

  /** A pair with a name of its own, as a camera's configuration names itself. */
  record Named(double a, double b) {
    Named {
      if (a > 10) {
        throw new IllegalArgumentException("Named[left].a: " + a + " is over 10");
      }
    }
  }

  /** Two pairs, and a check across them from the root: one naming a value, one naming none. */
  record Both(Named left, Pair right) {
    Both {
      if (left.a() + right.a() > 15) {
        throw new IllegalArgumentException(
            "Both.right.a: too much with left.a\nthe pairs don't fit");
      }
    }
  }

  @Test
  void aValueIsFoundWhereverTheRecordThatChecksItIs() {
    var live =
        LiveConfig.of("both", new Both(new Named(1, 1), new Pair(1, 1)))
            .number("left.a", Limits.NONE)
            .number("right.a", Limits.NONE);
    List<LiveConfig.Problem> named = new ArrayList<>();
    List<LiveConfig.Problem> across = new ArrayList<>();

    live.candidate(Map.of("left.a", 11.0), named);
    live.candidate(Map.of("right.a", 15.0), across);

    assertThat(named)
        .singleElement()
        .isEqualTo(
            new LiveConfig.Problem(
                Outcome.Layer.RECORD,
                List.of("left.a"),
                "LiveConfigTest.Named",
                "",
                "11.0 is over 10",
                "both: Named[left].a: 11.0 is over 10"));
    assertThat(across)
        .as("a check naming no value concerns the change's values in its record, and the record")
        .containsExactly(
            new LiveConfig.Problem(
                Outcome.Layer.RECORD,
                List.of("right.a"),
                "LiveConfigTest.Both",
                "",
                "too much with left.a",
                "both: Both.right.a: too much with left.a"),
            new LiveConfig.Problem(
                Outcome.Layer.RECORD,
                List.of("right.a"),
                "LiveConfigTest.Both",
                "",
                "the pairs don't fit",
                "both: the pairs don't fit"));
  }

  @Test
  void aValuesOwnLimitsAndTypeAreTiedToIt() {
    var live = live();
    List<LiveConfig.Problem> problems = new ArrayList<>();
    Map<String, Double> values = new LinkedHashMap<>();
    values.put(KP, -1.0);
    values.put("travel.middle", 3.0);

    live.candidate(values, problems);

    assertThat(problems)
        .extracting(LiveConfig.Problem::layer, LiveConfig.Problem::fields, LiveConfig.Problem::text)
        .containsExactly(
            tuple(Outcome.Layer.LIMIT, List.of(KP), "-1 is below its limit of 0"),
            tuple(Outcome.Layer.VALUE, List.of("travel.middle"), "travel.middle isn't tunable"));
  }

  @Test
  void theTeeAcceptsOnlyItsOwnKindOfCandidate() {
    var live = live();
    Fixtures.Machine candidate = live.candidate(Map.of(KP, 18.5));

    live.accept(candidate);

    assertThat(live.get()).isEqualTo(candidate);
    assertThatIllegalArgumentException().isThrownBy(() -> live.accept(new Pair(1, 1)));
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

  /**
   * A configuration written by more than its builder names, for a value, the call that commits it,
   * the value in its place as written: a measure with its unit.
   */
  @Test
  void aValueMayNameTheCallThatCommitsIt() {
    var live =
        LiveConfig.of("machine", machine())
            .number(KP, Limits.between(0, 100), "Machine.tuned(%s)")
            .measure("drive.limits.current", Amps.of(1), Amps.of(80), ".withCurrent(%s)");
    live.set(Map.of(KP, 18.5, "drive.limits.current", 35.0));

    assertThat(live.changes())
        .extracting(LiveConfig.Change::builderCall)
        .containsExactly("Machine.tuned(18.5)", ".withCurrent(Amps.of(35.0))");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> LiveConfig.of("machine", machine()).number(KP, Limits.NONE, ".withKp(kP)"))
        .withMessage("machine: drive.gains.kP's call, .withKp(kP), has no %s for its value");
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
    assertThat(dashboard.getTunable("Machine/drive/gains/kP").getConfig().getProperties())
        .as("what it means, from its record's Javadoc")
        .contains("\"description\":\"volts per meter per second of error\"");
    assertThat(dashboard.getTunable("Machine/travel/min").getConfig().getProperties())
        .as("undocumented")
        .doesNotContain("description");
  }

  /** The description property is JSON: what JSON escapes comes back exactly. */
  @Test
  void theDescriptionPropertyIsTheMeaningAsJson() {
    var dashboard = new MockTunableBackend();
    TunableRegistry.registerBackend("", dashboard);

    TunableRegistry.publish(
        "Symbols",
        LiveConfig.of("symbols", new Fixtures.Symbols(0, 0))
            .number("heading", Limits.NONE)
            .number("resistance", Limits.NONE));

    assertThat(property(dashboard, "Symbols/heading", "description"))
        .isEqualTo("the \"zero\" heading, in ° from C:\\field's corner");
    assertThat(property(dashboard, "Symbols/resistance", "description")).isEqualTo("in Ω, or Ω");
  }

  /**
   * The page says what each value means: its {@code @param} in the record that declares it, found
   * as the value is, through the records that hold it.
   */
  @Test
  void eachValueMeansWhatItsRecordsJavadocSays() {
    var live = live();
    LiveConfig.Documentation docs = live.documentation();

    assertThat(live.recordOf(KP)).isEqualTo(Fixtures.Gains.class);
    assertThat(live.meaning(KP)).isEqualTo("volts per meter per second of error");
    assertThat(docs.value(KP))
        .isEqualTo(
            new LiveConfig.Documentation.Value(
                "Fixtures.Gains", "drive.gains", "volts per meter per second of error"));
    assertThat(live.meaning("drive.limits.current"))
        .isEqualTo("the most current through the motor");
    assertThat(live.meaning("drive.limits.maxSpeed")).as("no @param").isEmpty();
    assertThat(live.meaning("travel.min")).as("no Javadoc").isEmpty();
    assertThatIllegalArgumentException().isThrownBy(() -> live.meaning("drive.motor.kP"));
    assertThat(docs.record()).isEqualTo("Fixtures.Machine");
    assertThat(docs.doc()).isEqualTo(ParameterDocs.NONE);
    assertThat(docs.groups())
        .extracting(LiveConfig.Documentation.Group::path, LiveConfig.Documentation.Group::record)
        .containsExactly(
            tuple("drive", "Fixtures.Drive"),
            tuple("drive.gains", "Fixtures.Gains"),
            tuple("drive.limits", "Fixtures.DriveLimits"),
            tuple("travel", "Fixtures.Range"));
    assertThat(docs.groups().get(1).doc().summary()).isEqualTo("How the drive is controlled.");
    assertThat(live.documentation()).as("read once").isSameAs(docs);
  }

  /** A value behind a component declared by its interface is documented by what's really there. */
  @Test
  void aComponentDeclaredByItsInterfaceIsDocumentedByWhatItHolds() {
    var live =
        LiveConfig.of("carriage", new Fixtures.Carriage(new Fixtures.Lift(3)))
            .number("stage.kP", Limits.between(0, 10));

    assertThat(live.recordOf("stage.kP")).isEqualTo(Fixtures.Lift.class);
    assertThat(live.documentation().groups())
        .containsExactly(
            new LiveConfig.Documentation.Group(
                "stage",
                "Fixtures.Lift",
                ParameterDocs.of(Fixtures.Lift.class),
                "the stage it rides"));
    assertThat(live.documentation().value("stage.kP"))
        .isEqualTo(
            new LiveConfig.Documentation.Value(
                "Fixtures.Lift", "stage", "volts per meter of error"));
    assertThat(live.set(Map.of("stage.kP", 4.0))).isTrue();
    assertThat(live.get()).isEqualTo(new Fixtures.Carriage(new Fixtures.Lift(4)));
  }

  /** Choosing another value reads the documentation again, to take it in. */
  @Test
  void aValueChosenLaterIsDocumentedToo() {
    var live = LiveConfig.of("machine", machine()).number(KP, Limits.between(0, 100));
    live.documentation();

    live.number("drive.limits.current", Limits.between(1, 80));

    assertThat(live.meaning("drive.limits.current"))
        .isEqualTo("the most current through the motor");
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
