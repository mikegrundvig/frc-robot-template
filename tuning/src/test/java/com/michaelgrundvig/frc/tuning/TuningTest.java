package com.michaelgrundvig.frc.tuning;

import static com.michaelgrundvig.frc.tuning.Fixtures.machine;
import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class TuningTest {
  private static final String KP = "drive.gains.kP";

  /** What each loop logged: the request kinds, then their paths. */
  private final List<String> logged = new ArrayList<>();

  /** Each view the robot was told about. */
  private final List<Tuning.View> shown = new ArrayList<>();

  @Test
  void startsAtTheCommittedConfiguration() {
    Tuning<Fixtures.Machine> tuning = tuning(true);

    assertThat(tuning.config()).isEqualTo(machine());
    assertThat(tuning.view().enabled()).isTrue();
    assertThat(tuning.view().changes()).isEmpty();
    assertThat(tuning.view().values())
        .extracting(Tuning.Value::path)
        .contains(KP, "drive.limits.current");
    assertThat(tuning.report()).isEmpty();
  }

  @Test
  void changesWaitForTheRobotLoop() {
    Tuning<Fixtures.Machine> tuning = tuning(true);

    tuning.requestSet(KP, 18.5);
    assertThat(tuning.config().drive().gains().kP()).as("before the loop").isEqualTo(16);

    tuning.periodic(false);
    assertThat(tuning.config().drive().gains().kP()).isEqualTo(18.5);
    assertThat(tuning.view().changes())
        .extracting(ConfigTuner.Change::builderCall)
        .containsExactly(".kP(18.5)");
  }

  @Test
  void aRefusedValueSaysWhyAndChangesNothing() {
    Tuning<Fixtures.Machine> tuning = tuning(true);

    tuning.requestSet(KP, -1);
    tuning.periodic(false);
    assertThat(value(tuning, KP).refused()).startsWith("kP must be zero or more");
    assertThat(tuning.config()).isEqualTo(machine());

    tuning.requestSet(KP, 18.5);
    tuning.periodic(false);
    assertThat(value(tuning, KP).refused()).isNull();
  }

  @Test
  void nothingChangesWhileTuningIsOff() {
    Tuning<Fixtures.Machine> tuning = tuning(false);

    tuning.requestSet(KP, 18.5);
    tuning.periodic(false);
    assertThat(value(tuning, KP).refused()).isEqualTo("tuning is off");

    tuning.requestEnabled(true);
    tuning.requestSet(KP, 18.5);
    tuning.periodic(false);
    assertThat(tuning.config().drive().gains().kP()).isEqualTo(18.5);

    tuning.requestEnabled(false);
    tuning.periodic(false);
    assertThat(tuning.view().enabled()).isFalse();
  }

  @Test
  void theFieldLocksTuningOff() {
    Tuning<Fixtures.Machine> tuning = tuning(true);

    tuning.periodic(true);
    assertThat(tuning.view().enabled()).isFalse();
    assertThat(tuning.view().fieldConnected()).isTrue();

    tuning.requestEnabled(true);
    tuning.requestSet(KP, 18.5);
    tuning.periodic(true);
    assertThat(tuning.view().enabled()).isFalse();
    assertThat(value(tuning, KP).refused()).isEqualTo("locked while the field is connected");

    // Once the field disconnects, tuning stays off until someone switches it on.
    tuning.periodic(false);
    assertThat(tuning.view().enabled()).isFalse();
  }

  @Test
  void resetReturnsToTheCommittedConfiguration() {
    Tuning<Fixtures.Machine> tuning = tuning(true);
    tuning.requestSet(KP, 18.5);
    tuning.requestSet("drive.limits.current", -1); // refused
    tuning.periodic(false);

    tuning.requestReset();
    tuning.periodic(false);

    assertThat(tuning.config()).isEqualTo(machine());
    assertThat(tuning.view().changes()).isEmpty();
    assertThat(value(tuning, "drive.limits.current").refused()).isNull();
  }

  @Test
  void resetNeedsTuningOn() {
    Tuning<Fixtures.Machine> tuning = tuning(true);
    tuning.requestSet(KP, 18.5);
    tuning.requestEnabled(false);
    tuning.requestReset();

    tuning.periodic(false);

    assertThat(tuning.config().drive().gains().kP()).isEqualTo(18.5);
  }

  @Test
  void everyLoopLogsTheRequestsItApplies() {
    Tuning<Fixtures.Machine> tuning = tuning(true);

    tuning.periodic(false);
    tuning.requestSet(KP, 18.5);
    tuning.requestReset();
    tuning.periodic(false);

    assertThat(logged).containsExactly("|", "SET,RESET|" + KP + ",");
  }

  @Test
  void replayAppliesTheLoggedRequests() {
    // In replay nothing is queued: the log supplies the requests, and whether tuning was on.
    Tuning<Fixtures.Machine> tuning =
        new Tuning<>(
            "test",
            machine(),
            false,
            inputs -> {
              inputs.enabled = true;
              inputs.kinds = new String[] {"SET"};
              inputs.paths = new String[] {KP};
              inputs.values = new double[] {18.5};
            },
            view -> {});

    tuning.periodic(false);

    assertThat(tuning.config().drive().gains().kP()).isEqualTo(18.5);
  }

  @Test
  void saysWhenWhatThePageShowsChanges() {
    Tuning<Fixtures.Machine> tuning = tuning(true);

    tuning.periodic(false);
    assertThat(shown).as("nothing happened").isEmpty();

    tuning.requestSet(KP, 18.5);
    tuning.periodic(false);
    tuning.periodic(true); // the field connects
    assertThat(shown).hasSize(2);
    assertThat(shown.get(0).changes()).hasSize(1);
    assertThat(shown.get(1).fieldConnected()).isTrue();
  }

  @Test
  void theViewSaysWhichValuesDiffer() {
    Tuning<Fixtures.Machine> tuning = tuning(true);
    assertThat(tuning.view().hasChanges()).isFalse();

    tuning.requestSet(KP, 18.5);
    tuning.requestSet("drive.limits.current", 35);
    tuning.periodic(false);

    assertThat(tuning.view().hasChanges()).isTrue();
    assertThat(tuning.view().changedPaths()).isEqualTo("drive.limits.current, " + KP);
  }

  @Test
  void theReportGivesTheBuilderCallForEachChange() {
    Tuning<Fixtures.Machine> tuning = tuning(true);
    tuning.requestSet(KP, 18.5);
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
        inputs -> logged.add(String.join(",", inputs.kinds) + "|" + String.join(",", inputs.paths)),
        shown::add);
  }

  private static Tuning.Value value(Tuning<?> tuning, String path) {
    return tuning.view().values().stream()
        .filter(value -> value.path().equals(path))
        .findFirst()
        .orElseThrow();
  }
}
