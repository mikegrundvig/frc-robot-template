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
        .containsExactly(new Actions.State("Home arm", Actions.Phase.READY, ""));
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
    assertThat(state()).isEqualTo(new Actions.State("Home arm", Actions.Phase.RUNNING, "Running"));

    home.running = false; // it finishes on its own
    actions.periodic(READY);

    assertThat(state()).isEqualTo(new Actions.State("Home arm", Actions.Phase.ENDED, "Ended"));
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
            new Actions.State("Home arm", Actions.Phase.CANCELED, "Canceled: " + NOT_UTILITY));
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
        .isEqualTo(new Actions.State("Home arm", Actions.Phase.CANCELED, "Canceled from the page"));

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
            new Actions.State("Home arm", Actions.Phase.REFUSED, "Refused: " + home.refusal));
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
                "Spin", Actions.Phase.ENDED, "Ended: checking whether it runs failed: lost track"));

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
                "Canceled from the page (canceling it failed: can't stop)"));
  }
}
