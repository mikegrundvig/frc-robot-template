package com.michaelgrundvig.frc.pitwall;

import static com.michaelgrundvig.frc.pitwall.Fixtures.machine;
import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/** Tuning's switch and its effects, with the machine's configuration published through a tee. */
class TuningTest {
  private static final String KP = "/Machine/drive/gains/kP";

  /** What each loop logged: the requests' kinds. */
  private final List<String> logged = new ArrayList<>();

  /** Each state the robot was told about. */
  private final List<Tuning.View> shown = new ArrayList<>();

  private final TunableTee tee =
      new TunableTee(new MockTunableBackend(), this::whyLocked, inputs -> {}, false);
  private final LiveConfig<Fixtures.Machine> machine =
      LiveConfig.of("machine", machine())
          .number("drive.gains.kP", Limits.between(0, 100))
          .number("travel.min", Limits.NONE)
          .number("travel.max", Limits.NONE);
  private Tuning tuning = tuning(true);

  private @Nullable String whyLocked() {
    return tuning.whyLocked();
  }

  @BeforeEach
  void publish() {
    TunableRegistry.registerBackend("", tee);
    TunableRegistry.publish("Machine", machine);
    tuning = tuning(true);
  }

  @AfterEach
  void reset() {
    TunableRegistry.reset();
  }

  private Tuning tuning(boolean enabled) {
    return new Tuning(
        "test",
        tee::configs,
        tee::pathOf,
        enabled,
        inputs -> logged.add(String.join(",", inputs.kinds)),
        shown::add);
  }

  /** A change through the tee, as the page makes it, applied at the loop's end. */
  private Outcome change(String path, double value) {
    var outcome = tee.requestChanges(Map.of(path, value));
    tee.update();
    return outcome.getNow(null);
  }

  @Test
  void theViewShowsEachConfigurationByTheTunablesItsValuesArePublishedAs() {
    Tuning.View view = tuning.view();

    assertThat(view.configs())
        .singleElement()
        .extracting(Tuning.ConfigView::name)
        .isEqualTo("machine");
    assertThat(view.configs().get(0).values())
        .first()
        .isEqualTo(
            new Tuning.Value(
                KP,
                "drive.gains.kP",
                "",
                Limits.between(0, 100),
                16,
                16,
                "Fixtures.Gains",
                "drive.gains",
                "volts per meter per second of error"));
    assertThat(tuning.report()).isEmpty();
  }

  @Test
  void aChangeShowsInTheViewAndTheReport() {
    assertThat(change(KP, 18.5).result()).isEqualTo(Outcome.Result.APPLIED);

    assertThat(tuning.view().hasChanges()).isTrue();
    assertThat(tuning.view().changedPaths()).isEqualTo("machine drive.gains.kP");
    assertThat(tuning.report())
        .containsExactly(
            "Tuned values differ from the committed machine configuration. To keep them, write"
                + " these builder calls into it:",
            "  drive.gains.kP: 16.0 -> 18.5   .kP(18.5)");
  }

  /** A match runs the committed configuration: what was tuned in the pit goes back. */
  @Test
  void theFieldConnectingReturnsEveryConfigurationToCommitted() {
    change(KP, 18.5);

    tuning.periodic(true);

    assertThat(machine.get()).isEqualTo(machine());
    assertThat(shown).last().satisfies(view -> assertThat(view.hasChanges()).isFalse());
  }

  @Test
  void nothingChangesWhileTuningIsOff() {
    tuning = tuning(false);

    assertThat(change(KP, 18.5))
        .isEqualTo(new Outcome(Outcome.Result.LOCKED, "Tuning is locked: tuning is off"));

    var on = tuning.requestEnabled(true);
    tuning.periodic(false);
    assertThat(on.getNow(null).result()).isEqualTo(Outcome.Result.APPLIED);
    assertThat(change(KP, 18.5).result()).isEqualTo(Outcome.Result.APPLIED);
  }

  @Test
  void theFieldLocksTuningOffUntilSomeoneSwitchesItOn() {
    tuning.periodic(true);
    assertThat(tuning.view().enabled()).isFalse();
    assertThat(tuning.view().fieldConnected()).isTrue();

    var on = tuning.requestEnabled(true);
    tuning.periodic(true);
    String locked = "Tuning is locked: the field system is connected";
    assertThat(on.getNow(null)).isEqualTo(new Outcome(Outcome.Result.LOCKED, locked));
    assertThat(change(KP, 18.5)).isEqualTo(new Outcome(Outcome.Result.LOCKED, locked));

    tuning.periodic(false);
    assertThat(tuning.view().enabled()).isFalse();
  }

  @Test
  void resetReturnsEveryConfigurationToCommitted() {
    change(KP, 18.5);

    var reset = tuning.requestReset();
    tuning.periodic(false);

    assertThat(reset.getNow(null).result()).isEqualTo(Outcome.Result.APPLIED);
    assertThat(machine.get()).isEqualTo(machine());
  }

  @Test
  void resetNeedsTuningOn() {
    change(KP, 18.5);
    tuning.requestEnabled(false);
    var reset = tuning.requestReset();

    tuning.periodic(false);

    assertThat(reset.getNow(null).result()).isEqualTo(Outcome.Result.LOCKED);
    assertThat(machine.get().drive().gains().kP()).isEqualTo(18.5);
  }

  @Test
  void everyLoopLogsTheRequestsItApplies() {
    tuning.periodic(false);
    tuning.requestEnabled(false);
    tuning.requestEnabled(true);
    tuning.requestReset();
    tuning.periodic(false);

    assertThat(logged).containsExactly("", "DISABLE,ENABLE,RESET");
  }

  @Test
  void replayAppliesTheLoggedRequests() {
    change(KP, 18.5);
    var replayed =
        new Tuning(
            "test",
            tee::configs,
            tee::pathOf,
            false,
            inputs -> {
              inputs.enabled = true;
              inputs.kinds = new String[] {"RESET"};
            },
            view -> {});

    replayed.periodic(false);

    assertThat(machine.get()).isEqualTo(machine());
  }

  @Test
  void saysWhenTheStateChangesAndOnlyThen() {
    tuning.periodic(false);
    assertThat(shown).as("nothing happened").isEmpty();

    change(KP, 18.5);
    tuning.periodic(false);
    tuning.periodic(false);
    tuning.periodic(true); // the field connects

    assertThat(shown).hasSize(2);
    assertThat(shown.get(0).changedPaths()).isEqualTo("machine drive.gains.kP");
    assertThat(shown.get(1).fieldConnected()).isTrue();
  }
}
