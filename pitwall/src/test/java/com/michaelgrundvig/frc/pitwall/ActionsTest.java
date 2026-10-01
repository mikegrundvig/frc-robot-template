package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;

class ActionsTest {
  private static final Actions.Gate READY = new Actions.Gate(true, true, false);
  private static final String NOT_UTILITY =
      "actions run only with the robot enabled in utility mode";

  /** A routine that runs until it's canceled or the test finishes it. */
  private static class FakeRoutine implements Actions.Routine {
    boolean running;
    int starts;
    int cancels;
    @Nullable String refusal;

    @Override
    public @Nullable String start() {
      if (refusal != null) {
        return refusal;
      }
      running = true;
      starts++;
      return null;
    }

    @Override
    public boolean isRunning() {
      return running;
    }

    @Override
    public void cancel() {
      running = false;
      cancels++;
    }
  }

  private final List<String> logged = new ArrayList<>();
  private final FakeRoutine home = new FakeRoutine();
  private final Actions actions =
      new Actions(
              inputs ->
                  logged.add(String.join(",", inputs.kinds) + "|" + String.join(",", inputs.names)))
          .add("Home arm", home);

  private Actions.State state() {
    return actions.view().get(0);
  }

  @Test
  void everyActionIsListedReadyBeforeAnyRuns() {
    assertThat(actions.view())
        .containsExactly(
            new Actions.State(
                "Home arm", Actions.Phase.READY, "", NOT_UTILITY, Actions.When.ENABLED_IN_UTILITY));
    assertThat(actions.has("Home arm")).isTrue();
    assertThat(actions.has("Fly")).isFalse();
  }

  @Test
  void anActionStartsInTheRobotLoopAndShowsRunningUntilItEnds() {
    var outcome = actions.requestStart("Home arm");
    assertThat(outcome).as("before the loop").isNotDone();
    assertThat(home.starts).isZero();

    actions.periodic(READY);

    assertThat(outcome.getNow(null)).isEqualTo(Outcome.applied());
    assertThat(state())
        .isEqualTo(
            new Actions.State(
                "Home arm", Actions.Phase.RUNNING, "Running", "", Actions.When.ENABLED_IN_UTILITY));

    home.running = false; // it finishes on its own
    actions.periodic(READY);

    assertThat(state())
        .isEqualTo(
            new Actions.State(
                "Home arm", Actions.Phase.ENDED, "Ended", "", Actions.When.ENABLED_IN_UTILITY));
  }

  @Test
  void actionsRunOnlyWhenEnabledInUtilityModeAndNeverOnTheField() {
    for (var gate :
        List.of(
            new Actions.Gate(false, true, false),
            new Actions.Gate(true, false, false),
            new Actions.Gate(true, true, true))) {
      var outcome = actions.requestStart("Home arm");
      actions.periodic(gate);

      assertThat(outcome.getNow(null).result())
          .as(gate.toString())
          .isEqualTo(Outcome.Result.LOCKED);
    }
    assertThat(home.starts).isZero();
    assertThat(actions.requestStart("Home arm").isDone()).isFalse();
  }

  @Test
  void theLockSaysWhy() {
    var outcome = actions.requestStart("Home arm");
    actions.periodic(new Actions.Gate(true, true, true));
    assertThat(outcome.getNow(null).detail())
        .isEqualTo("Actions are locked: the field system is connected");

    outcome = actions.requestStart("Home arm");
    actions.periodic(new Actions.Gate(true, false, false));
    assertThat(outcome.getNow(null).detail()).isEqualTo("Actions are locked: " + NOT_UTILITY);
  }

  /** The page shows why before anyone presses Run: the same reason a start would be refused. */
  @Test
  void theLockIsKnownBeforeAnythingIsAsked() {
    assertThat(actions.whyLocked()).as("before the first loop").isEqualTo(NOT_UTILITY);

    actions.periodic(READY);
    assertThat(actions.whyLocked()).isNull();

    actions.periodic(new Actions.Gate(true, true, true));
    assertThat(actions.whyLocked()).isEqualTo("the field system is connected");
  }

  @Test
  void disablingOrLeavingUtilityModeCancelsWhatIsRunning() {
    actions.requestStart("Home arm");
    actions.periodic(READY);

    actions.periodic(new Actions.Gate(false, true, false));

    assertThat(home.cancels).isEqualTo(1);
    assertThat(state())
        .isEqualTo(
            new Actions.State(
                "Home arm",
                Actions.Phase.CANCELED,
                "Canceled: " + NOT_UTILITY,
                NOT_UTILITY,
                Actions.When.ENABLED_IN_UTILITY));
  }

  @Test
  void aRunningActionCanBeCanceledFromThePage() {
    actions.requestStart("Home arm");
    actions.periodic(READY);

    var outcome = actions.requestCancel("Home arm");
    actions.periodic(READY);

    assertThat(outcome.getNow(null)).isEqualTo(Outcome.applied());
    assertThat(home.running).isFalse();
    assertThat(state())
        .isEqualTo(
            new Actions.State(
                "Home arm",
                Actions.Phase.CANCELED,
                "Canceled from the page",
                "",
                Actions.When.ENABLED_IN_UTILITY));

    var again = actions.requestCancel("Home arm");
    actions.periodic(READY);
    assertThat(again.getNow(null)).as("nothing to cancel").isEqualTo(Outcome.applied());
    assertThat(home.cancels).isEqualTo(1);
  }

  @Test
  void aRunningActionIsNotStartedTwice() {
    actions.requestStart("Home arm");
    actions.periodic(READY);

    var again = actions.requestStart("Home arm");
    actions.periodic(READY);

    assertThat(again.getNow(null))
        .isEqualTo(new Outcome(Outcome.Result.REFUSED, "Home arm is already running"));
    assertThat(home.starts).isEqualTo(1);
  }

  @Test
  void aRoutineThatCannotStartSaysWhy() {
    home.refusal = "a higher-priority command is using what it needs";

    var outcome = actions.requestStart("Home arm");
    actions.periodic(READY);

    assertThat(outcome.getNow(null)).isEqualTo(new Outcome(Outcome.Result.REFUSED, home.refusal));
    assertThat(state())
        .isEqualTo(
            new Actions.State(
                "Home arm",
                Actions.Phase.REFUSED,
                "Refused: " + home.refusal,
                "",
                Actions.When.ENABLED_IN_UTILITY));
  }

  @Test
  void aRoutineThatThrowsIsRefusedRatherThanStoppingTheRobot() {
    var broken =
        new Actions(inputs -> {})
            .add(
                "Broken",
                new FakeRoutine() {
                  @Override
                  public @Nullable String start() {
                    throw new IllegalStateException("no arm");
                  }
                })
            .add(
                "Silent",
                new FakeRoutine() {
                  @Override
                  public @Nullable String start() {
                    throw new UnsupportedOperationException();
                  }
                });

    var withMessage = broken.requestStart("Broken");
    var without = broken.requestStart("Silent");
    broken.periodic(READY);

    assertThat(withMessage.getNow(null)).isEqualTo(new Outcome(Outcome.Result.REFUSED, "no arm"));
    assertThat(without.getNow(null).detail()).isEqualTo("java.lang.UnsupportedOperationException");
  }

  @Test
  void anUnknownActionIsRefused() {
    var outcome = actions.requestStart("Fly");
    actions.periodic(READY);

    assertThat(outcome.getNow(null))
        .isEqualTo(new Outcome(Outcome.Result.REFUSED, "There is no action named Fly"));
  }

  @Test
  void everyLoopLogsTheRequestsItApplies() {
    actions.periodic(READY);
    actions.requestStart("Home arm");
    actions.requestCancel("Home arm");
    actions.periodic(READY);

    assertThat(logged).containsExactly("|", "START,CANCEL|Home arm,Home arm");
  }

  @Test
  void replayMakesTheLoggedRequests() {
    // In replay nothing is queued: the log supplies the requests.
    var replayed = new FakeRoutine();
    var replay =
        new Actions(
                inputs -> {
                  inputs.kinds = new String[] {"START"};
                  inputs.names = new String[] {"Home arm"};
                })
            .add("Home arm", replayed);

    replay.periodic(READY);

    assertThat(replayed.starts).isEqualTo(1);
  }

  @Test
  void everyActionHasItsOwnName() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> actions.add("Home arm", new FakeRoutine()))
        .withMessage("two actions are named Home arm");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> actions.add(" ", new FakeRoutine()))
        .withMessage("an action needs a name");
  }

  /** A team's routine that throws ends or cancels with the reason; the robot loop runs on. */
  @Test
  void aRoutineThatThrowsNeverStopsTheLoop() {
    FakeRoutine broken =
        new FakeRoutine() {
          @Override
          public boolean isRunning() {
            throw new IllegalStateException("lost track");
          }

          @Override
          public void cancel() {
            throw new IllegalStateException("can't stop");
          }
        };
    actions.add("Spin", broken);
    actions.requestStart("Spin");
    actions.periodic(READY);

    actions.periodic(READY);

    assertThat(actions.view().get(1))
        .isEqualTo(
            new Actions.State(
                "Spin",
                Actions.Phase.ENDED,
                "Ended: checking whether it runs failed: lost track",
                "",
                Actions.When.ENABLED_IN_UTILITY));

    FakeRoutine stubborn =
        new FakeRoutine() {
          @Override
          public void cancel() {
            throw new IllegalStateException("can't stop");
          }
        };
    actions.add("Stubborn", stubborn);
    actions.requestStart("Stubborn");
    actions.periodic(READY);
    actions.requestCancel("Stubborn");

    actions.periodic(READY);

    assertThat(actions.view().get(2))
        .isEqualTo(
            new Actions.State(
                "Stubborn",
                Actions.Phase.CANCELED,
                "Canceled from the page (canceling it failed: can't stop)",
                "",
                Actions.When.ENABLED_IN_UTILITY));
  }

  /** One that powers something down runs only while disabled, and never on the field. */
  @Test
  void anActionForADisabledRobotRunsOnlyWhileDisabled() {
    FakeRoutine powerDown = new FakeRoutine();
    actions.add("Power down", powerDown, Actions.When.DISABLED);
    Actions.Gate disabled = new Actions.Gate(false, false, false);

    var whileEnabled = actions.requestStart("Power down");
    actions.periodic(READY);
    var onTheField = actions.requestStart("Power down");
    actions.periodic(new Actions.Gate(false, false, true));
    var whileDisabled = actions.requestStart("Power down");
    actions.periodic(disabled);

    assertThat(whileEnabled.getNow(null).detail())
        .isEqualTo("Power down is locked: Power down runs only with the robot disabled");
    assertThat(onTheField.getNow(null).detail())
        .isEqualTo("Power down is locked: the field system is connected");
    assertThat(whileDisabled.getNow(null)).isEqualTo(Outcome.applied());
    assertThat(actions.view().get(1))
        .isEqualTo(
            new Actions.State(
                "Power down", Actions.Phase.RUNNING, "Running", "", Actions.When.DISABLED));
    // The others' lock is their own.
    assertThat(actions.whyLocked()).isEqualTo(NOT_UTILITY);
    assertThat(state().locked()).isEqualTo(NOT_UTILITY);

    actions.periodic(READY);

    assertThat(powerDown.cancels).as("enabling cancels it").isEqualTo(1);
    assertThat(actions.view().get(1).detail())
        .isEqualTo("Canceled: Power down runs only with the robot disabled");
  }

  /**
   * Once the field system has connected in this run, an action for a disabled robot is refused for
   * good: a match's dropout clears "field connected" for a moment, and a press then mustn't power
   * anything down.
   */
  @Test
  void anActionForADisabledRobotIsRefusedOnceTheFieldHasConnected() {
    FakeRoutine powerDown = new FakeRoutine();
    actions.add("Power down", powerDown, Actions.When.DISABLED);

    var afterTheField = actions.requestStart("Power down");
    actions.periodic(new Actions.Gate(false, false, false, true));

    assertThat(afterTheField.getNow(null).detail())
        .isEqualTo(
            "Power down is locked: Power down runs only until the field system first connects,"
                + " and it has since the robot program started");
    assertThat(powerDown.starts).isZero();
    assertThat(actions.view().get(1).locked()).contains("first connects");
    assertThat(actions.view().get(1).when()).isEqualTo(Actions.When.DISABLED);
    assertThat(state().when()).isEqualTo(Actions.When.ENABLED_IN_UTILITY);
  }
}
