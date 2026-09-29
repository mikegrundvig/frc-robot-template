package com.michaelgrundvig.frc.tuning;

import static com.michaelgrundvig.frc.tuning.Fixtures.machine;
import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class TuningTest {
  private static final String KP = "drive.gains.kP";

  /** What each loop logged: the entries' kinds, paths, and request numbers. */
  private final List<String> logged = new ArrayList<>();

  /** Each state the robot was told about. */
  private final List<Tuning.View> shown = new ArrayList<>();

  @Test
  void startsAtTheCommittedConfiguration() {
    var tuning = tuning(true);

    assertThat(tuning.config()).isEqualTo(machine());
    assertThat(tuning.view().enabled()).isTrue();
    assertThat(tuning.view().hasChanges()).isFalse();
    assertThat(tuning.view().values())
        .extracting(Tuning.Value::path)
        .contains(KP, "drive.limits.current", "travel.min");
    assertThat(tuning.report()).isEmpty();
  }

  @Test
  void changesWaitForTheRobotLoop() {
    var tuning = tuning(true);

    var outcome = tuning.requestChanges(Map.of(KP, 18.5));
    assertThat(outcome).as("before the loop").isNotDone();
    assertThat(tuning.config().drive().gains().kP()).isEqualTo(16);

    tuning.periodic(false);

    assertThat(outcome.getNow(null))
        .isEqualTo(new Tuning.Outcome(Tuning.Outcome.Result.APPLIED, ""));
    assertThat(tuning.config().drive().gains().kP()).isEqualTo(18.5);
    assertThat(tuning.view().changes())
        .extracting(ConfigTuner.Change::builderCall)
        .containsExactly(".kP(18.5)");
  }

  @Test
  void aRefusedChangeSaysWhyAndChangesNothing() {
    var tuning = tuning(true);

    var outcome = tuning.requestChanges(Map.of(KP, -1.0));
    tuning.periodic(false);

    assertThat(outcome.getNow(null))
        .isEqualTo(
            new Tuning.Outcome(Tuning.Outcome.Result.REFUSED, "kP must be zero or more, got -1.0"));
    assertThat(tuning.config()).isEqualTo(machine());
  }

  @Test
  void severalValuesChangeTogetherOrNotAtAll() {
    var tuning = tuning(true);

    var together = tuning.requestChanges(ordered("travel.min", 12.0, "travel.max", 20.0));
    var neither = tuning.requestChanges(ordered(KP, 18.5, "drive.limits.current", -1.0));
    tuning.periodic(false);

    assertThat(together.getNow(null).result()).isEqualTo(Tuning.Outcome.Result.APPLIED);
    assertThat(tuning.config().travel()).isEqualTo(new Fixtures.Range(12, 20));
    assertThat(neither.getNow(null).result()).isEqualTo(Tuning.Outcome.Result.REFUSED);
    assertThat(tuning.config().drive().gains().kP()).as("not applied alone").isEqualTo(16);
  }

  @Test
  void requestsApplyInTheOrderTheyArrived() {
    var tuning = tuning(true);

    tuning.requestChanges(Map.of(KP, 18.5));
    tuning.requestChanges(Map.of(KP, 20.0));
    tuning.periodic(false);

    assertThat(tuning.config().drive().gains().kP()).isEqualTo(20);
  }

  @Test
  void nothingChangesWhileTuningIsOff() {
    var tuning = tuning(false);

    var off = tuning.requestChanges(Map.of(KP, 18.5));
    tuning.periodic(false);
    assertThat(off.getNow(null))
        .isEqualTo(
            new Tuning.Outcome(Tuning.Outcome.Result.LOCKED, "Tuning is locked: tuning is off"));

    var on = tuning.requestEnabled(true);
    var change = tuning.requestChanges(Map.of(KP, 18.5));
    tuning.periodic(false);
    assertThat(on.getNow(null).result()).isEqualTo(Tuning.Outcome.Result.APPLIED);
    assertThat(change.getNow(null).result()).isEqualTo(Tuning.Outcome.Result.APPLIED);

    tuning.requestEnabled(false);
    tuning.periodic(false);
    assertThat(tuning.view().enabled()).isFalse();
  }

  @Test
  void theFieldLocksTuningOff() {
    var tuning = tuning(true);

    tuning.periodic(true);
    assertThat(tuning.view().enabled()).isFalse();
    assertThat(tuning.view().fieldConnected()).isTrue();

    var on = tuning.requestEnabled(true);
    var change = tuning.requestChanges(Map.of(KP, 18.5));
    tuning.periodic(true);
    String locked = "Tuning is locked: the field system is connected";
    assertThat(on.getNow(null)).isEqualTo(new Tuning.Outcome(Tuning.Outcome.Result.LOCKED, locked));
    assertThat(change.getNow(null))
        .isEqualTo(new Tuning.Outcome(Tuning.Outcome.Result.LOCKED, locked));

    // Once the field disconnects, tuning stays off until someone switches it on.
    tuning.periodic(false);
    assertThat(tuning.view().enabled()).isFalse();
  }

  @Test
  void resetReturnsToTheCommittedConfiguration() {
    var tuning = tuning(true);
    tuning.requestChanges(Map.of(KP, 18.5));
    tuning.periodic(false);

    var reset = tuning.requestReset();
    tuning.periodic(false);

    assertThat(reset.getNow(null).result()).isEqualTo(Tuning.Outcome.Result.APPLIED);
    assertThat(tuning.config()).isEqualTo(machine());
    assertThat(tuning.view().hasChanges()).isFalse();
  }

  @Test
  void resetNeedsTuningOn() {
    var tuning = tuning(true);
    tuning.requestChanges(Map.of(KP, 18.5));
    tuning.requestEnabled(false);
    var reset = tuning.requestReset();

    tuning.periodic(false);

    assertThat(reset.getNow(null).result()).isEqualTo(Tuning.Outcome.Result.LOCKED);
    assertThat(tuning.config().drive().gains().kP()).isEqualTo(18.5);
  }

  @Test
  void whoeverIsWaitingSeesTheStateTheirRequestLeft() {
    var tuning = tuning(true);
    List<Tuning.View> seen = new ArrayList<>();

    tuning.requestChanges(Map.of(KP, 18.5)).thenRun(() -> seen.add(tuning.view()));
    tuning.periodic(false);

    assertThat(seen).singleElement().extracting(Tuning.View::hasChanges).isEqualTo(true);
  }

  @Test
  void everyLoopLogsTheRequestsItApplies() {
    var tuning = tuning(true);

    tuning.periodic(false);
    tuning.requestChanges(ordered("travel.min", 12.0, "travel.max", 20.0));
    tuning.requestReset();
    tuning.periodic(false);

    assertThat(logged).containsExactly("||[]", "SET,SET,RESET|travel.min,travel.max,|[0, 0, 1]");
  }

  @Test
  void replayAppliesTheLoggedRequestsTogether() {
    // In replay nothing is queued: the log supplies the requests, and whether tuning was on.
    var tuning =
        new Tuning<>(
            "test",
            machine(),
            false,
            inputs -> {
              inputs.enabled = true;
              inputs.kinds = new String[] {"SET", "SET"};
              inputs.paths = new String[] {"travel.min", "travel.max"};
              inputs.values = new double[] {12, 20};
              inputs.requests = new int[] {0, 0};
            },
            view -> {});

    tuning.periodic(false);

    assertThat(tuning.config().travel()).isEqualTo(new Fixtures.Range(12, 20));
  }

  @Test
  void saysWhenTheStateChanges() {
    var tuning = tuning(true);

    tuning.periodic(false);
    assertThat(shown).as("nothing happened").isEmpty();

    tuning.requestChanges(Map.of(KP, 18.5));
    tuning.periodic(false);
    tuning.periodic(true); // the field connects
    assertThat(shown).hasSize(2);
    assertThat(shown.get(0).changes()).hasSize(1);
    assertThat(shown.get(1).fieldConnected()).isTrue();
  }

  @Test
  void theViewSaysWhichValuesDiffer() {
    var tuning = tuning(true);

    tuning.requestChanges(ordered(KP, 18.5, "drive.limits.current", 35.0));
    tuning.periodic(false);

    assertThat(tuning.view().hasChanges()).isTrue();
    assertThat(tuning.view().changedPaths()).isEqualTo("drive.limits.current, " + KP);
  }

  @Test
  void theReportGivesTheBuilderCallForEachChange() {
    var tuning = tuning(true);
    tuning.requestChanges(Map.of(KP, 18.5));
    tuning.periodic(false);

    assertThat(tuning.report())
        .containsExactly(
            "Tuned values differ from the committed configuration of test. To keep them, write"
                + " these builder calls into it:",
            "  drive.gains.kP: 16.0 -> 18.5   .kP(18.5)");
  }

  private Tuning<Fixtures.Machine> tuning(boolean enabled) {
    return new Tuning<>(
        "test",
        machine(),
        enabled,
        inputs ->
            logged.add(
                String.join(",", inputs.kinds)
                    + "|"
                    + String.join(",", inputs.paths)
                    + "|"
                    + Arrays.toString(inputs.requests)),
        shown::add);
  }

  /** Two values in the order given, as a page would send them. */
  private static Map<String, Double> ordered(String a, double first, String b, double second) {
    Map<String, Double> values = new LinkedHashMap<>();
    values.put(a, first);
    values.put(b, second);
    return values;
  }
}
