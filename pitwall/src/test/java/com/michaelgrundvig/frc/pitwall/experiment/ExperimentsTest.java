package com.michaelgrundvig.frc.pitwall.experiment;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Outcome;
import com.michaelgrundvig.frc.pitwall.TunableTee;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicLong;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * Experiments as the page runs them: started and canceled through the robot loop, locked out except
 * in utility mode, watched by the deadman. Each run is driven by hand, loop by loop, as its command
 * would be, on a mechanism whose answer is known ({@link Lag}).
 */
class ExperimentsTest {
  private static final Actions.Gate READY = new Actions.Gate(true, true, false);
  private static final String STEP = "Step to 90° and back";
  private static final String PAGE = "7";

  private final Lag lag = new Lag();
  private final Rig rig = lag.rig();
  private final AtomicLong wallClock = new AtomicLong();
  private final List<String> logged = new ArrayList<>();
  private final List<String> reported = new ArrayList<>();

  /** What a replay's log supplies, loop by loop. */
  private final ExperimentInputs replayed = new ExperimentInputs();

  /** Drives each run by hand, as the scheduler would drive its command. */
  private @Nullable Run driven;

  private final Experiments.Launcher byHand =
      run ->
          new Actions.Routine() {
            @Override
            public @Nullable String start() {
              driven = run;
              return null;
            }

            @Override
            public boolean isRunning() {
              return run.state().phase() == Run.Phase.RUNNING;
            }

            @Override
            public void cancel() {
              run.cancel("Canceled: another command needed Arm"); // as its command's cleanup does
            }
          };

  private final Experiments experiments = experiments(false);

  private Experiments experiments(boolean replaying) {
    return new Experiments(
            new Experiments.Robot("test", true, "abc12345 on main"),
            inputs -> {
              if (replaying) {
                inputs.kinds = replayed.kinds;
                inputs.names = replayed.names;
                inputs.runs = replayed.runs;
              }
              logged.add(
                  String.join(",", inputs.kinds)
                      + "|"
                      + String.join(",", inputs.names)
                      + "|"
                      + Arrays.toString(inputs.runs));
            },
            wallClock::get,
            replaying,
            byHand,
            reported::add)
        .add(new Experiment(STEP, rig, Lag.STEP_AND_BACK));
  }

  @AfterEach
  void reset() {
    TunableRegistry.reset();
  }

  /** One robot loop: Pitwall's part, then the scheduler's, 20 ms of wall time later. */
  private void loop(Actions.Gate gate) {
    experiments.periodic(gate);
    long now = lag.nextLoop();
    Run run = driven;
    if (run != null) {
      run.step(now);
    }
    wallClock.addAndGet(Lag.LOOP_NANOS);
  }

  /** The page asks for a start, and the next loop applies it. */
  private Outcome start(Experiments experiments) {
    CompletableFuture<Outcome> outcome = experiments.requestStart(STEP, PAGE);
    loop(READY);
    return outcome.getNow(new Outcome(Outcome.Result.REFUSED, "not applied"));
  }

  private Run run() {
    return Objects.requireNonNull(experiments.run(), "no run");
  }

  /** How the run's angle answered its first jump. */
  private static StepResponse firstJump(Run run) {
    var steps = Objects.requireNonNull(run.state().scores(), "not scored").steps();
    return Objects.requireNonNull(steps.get("Angle")).get(0).orElseThrow();
  }

  /** Keeps the page in touch, as its heartbeat does, for a number of loops. */
  private void loopsInTouch(int count) {
    for (int i = 0; i < count; i++) {
      experiments.heard(PAGE);
      loop(READY);
    }
  }

  @Test
  void anExperimentRunsInTheRobotLoopToItsEndAndIsScored() {
    assertThat(experiments.run()).as("none yet").isNull();

    assertThat(start(experiments)).isEqualTo(new Outcome(Outcome.Result.APPLIED, ""));
    assertThat(run().state().phase()).isEqualTo(Run.Phase.RUNNING);
    assertThat(run().id()).isEqualTo(1);
    assertThat(run().gains()).isEqualTo(Map.of("gains.kP", 10.0, "gains.kD", 0.0));

    loopsInTouch(110);

    assertThat(run().state().phase()).isEqualTo(Run.Phase.ENDED);
    StepResponse up = firstJump(run());
    assertThat(up.riseSeconds().getAsDouble()).isCloseTo(Math.log(9) / 10, within(0.001));
  }

  @Test
  void itKnowsItsExperimentsAndRigs() {
    assertThat(experiments.experiments()).extracting(Experiment::name).containsExactly(STEP);
    assertThat(experiments.rigs()).containsExactly(rig);
    assertThat(experiments.has(STEP)).isTrue();
    assertThat(experiments.has("Fly")).isFalse();
    assertThat(experiments.robot())
        .isEqualTo(new Experiments.Robot("test", true, "abc12345 on main"));
  }

  @Test
  void experimentsRunOnlyEnabledInUtilityModeAndNeverOnTheField() {
    for (var gate :
        List.of(
            new Actions.Gate(false, true, false),
            new Actions.Gate(true, false, false),
            new Actions.Gate(true, true, true))) {
      var outcome = experiments.requestStart(STEP, PAGE);
      loop(gate);

      assertThat(outcome.getNow(null).result())
          .as(gate.toString())
          .isEqualTo(Outcome.Result.LOCKED);
    }
    assertThat(experiments.run()).isNull();
  }

  @Test
  void leavingUtilityModeCancelsTheRunAndSaysWhy() {
    start(experiments);
    loopsInTouch(10);

    loop(new Actions.Gate(true, false, false));

    assertThat(run().state())
        .isEqualTo(
            new Run.State(
                Run.Phase.CANCELED,
                "Canceled: experiments run only with the robot enabled in utility mode",
                null));
  }

  @Test
  void thePageCancelsARun() {
    start(experiments);
    loopsInTouch(10);

    var outcome = experiments.requestCancel(STEP, run().id(), "");
    loop(READY);

    assertThat(outcome.getNow(null)).isEqualTo(new Outcome(Outcome.Result.APPLIED, ""));
    assertThat(run().state().detail()).isEqualTo("Canceled from the page");
  }

  @Test
  void oneExperimentRunsAtATime() {
    experiments.add(
        new Experiment(
            "Small step",
            rig,
            Profiles.step().from(Degrees.of(0)).to(Degrees.of(10)).hold(Seconds.of(0.5))));
    start(experiments);

    var second = experiments.requestStart("Small step", PAGE);
    loop(READY);

    assertThat(second.getNow(null))
        .isEqualTo(
            new Outcome(
                Outcome.Result.REFUSED, STEP + " is running; one experiment runs at a time"));
    assertThat(run().experiment().name()).isEqualTo(STEP);
  }

  @Test
  void anotherRunStartsOnceTheLastHasEnded() {
    start(experiments);
    loopsInTouch(110);

    assertThat(start(experiments).result()).isEqualTo(Outcome.Result.APPLIED);
    assertThat(run().id()).isEqualTo(2);
  }

  /** The deadman: the run goes on only while the page that started it keeps in touch. */
  @Test
  void aRunWhosePageGoesQuietIsCanceled() {
    start(experiments);

    for (int i = 0; i < 20; i++) {
      wallClock.addAndGet(Duration.ofMillis(100).toNanos()); // 120 ms a loop, heard every loop
      loopsInTouch(1);
    }
    assertThat(run().state().phase()).as("heard from").isEqualTo(Run.Phase.RUNNING);

    experiments.heard("another page"); // someone else's requests don't keep it going
    wallClock.addAndGet(Duration.ofMillis(490).toNanos());
    loop(READY);
    assertThat(run().state().phase()).as("490 ms, then a loop").isEqualTo(Run.Phase.CANCELED);
    assertThat(run().state().detail())
        .isEqualTo("Canceled: the page that started it went quiet for over 500 ms");
  }

  @Test
  void theDeadmansCancelIsLoggedSoAReplayCancelsToo() {
    start(experiments);
    logged.clear();

    wallClock.addAndGet(Duration.ofSeconds(1).toNanos());
    loop(READY);

    assertThat(logged).containsExactly("CANCEL|" + STEP + "|[1]");
    assertThat(reported)
        .containsExactly(
            "run 1 of " + STEP + ": Canceled: the page that started it went quiet for over 500 ms");
  }

  @Test
  void aRunWhosePageClosesIsCanceled() {
    start(experiments);
    loopsInTouch(5);

    experiments.lost("another page");
    loopsInTouch(1);
    assertThat(run().state().phase()).isEqualTo(Run.Phase.RUNNING);

    experiments.lost(PAGE);
    loopsInTouch(1);

    assertThat(run().state().detail()).isEqualTo("Canceled: the page that started it closed");
  }

  /** Curl, or a test, has no event stream: its own requests keep its run going. */
  @Test
  void aRunStartedWithoutAStreamIsKeptGoingByRequestsWithoutOne() {
    experiments.requestStart(STEP, null);
    loop(READY);

    for (int i = 0; i < 40; i++) {
      experiments.heard(null);
      loop(READY);
    }

    assertThat(run().state().phase()).isEqualTo(Run.Phase.RUNNING);
  }

  /** In a replay no page is there: the log's cancels are what stop a run. */
  @Test
  void aReplayRunsWithoutTheDeadman() {
    Experiments replay = experiments(true);
    replayed.kinds = new String[] {"START"};
    replayed.names = new String[] {STEP};
    replayed.runs = new long[] {0};
    replay.periodic(READY);
    replayed.clearRequests();
    Run run = Objects.requireNonNull(replay.run());

    wallClock.addAndGet(Duration.ofSeconds(10).toNanos());
    replay.periodic(READY);

    assertThat(run.state().phase()).isEqualTo(Run.Phase.RUNNING);
    assertThat(run.controller()).as("a replay has no page").isEmpty();
  }

  @Test
  void aRunStoppedBeforeItsCommandRanIsCanceled() {
    Experiments stalled =
        new Experiments(
                new Experiments.Robot("test", true, ""),
                inputs -> {},
                wallClock::get,
                false,
                run ->
                    new Actions.Routine() {
                      @Override
                      public @Nullable String start() {
                        return null;
                      }

                      @Override
                      public boolean isRunning() {
                        return false; // taken off the scheduler before it ran
                      }

                      @Override
                      public void cancel() {}
                    },
                ended -> {})
            .add(new Experiment(STEP, rig, Lag.STEP_AND_BACK));
    stalled.requestStart(STEP, PAGE);
    stalled.periodic(READY);

    stalled.periodic(READY);

    assertThat(Objects.requireNonNull(stalled.run()).state())
        .isEqualTo(
            new Run.State(
                Run.Phase.CANCELED, "Canceled: its command stopped before the run ended", null));
  }

  @Test
  void aStartTheLauncherRefusesStartsNoRun() {
    Experiments busy =
        new Experiments(
                new Experiments.Robot("test", true, ""),
                inputs -> {},
                wallClock::get,
                false,
                run ->
                    new Actions.Routine() {
                      @Override
                      public @Nullable String start() {
                        return "a higher-priority command is using what it needs";
                      }

                      @Override
                      public boolean isRunning() {
                        return false;
                      }

                      @Override
                      public void cancel() {}
                    },
                ended -> {})
            .add(new Experiment(STEP, rig, Lag.STEP_AND_BACK));

    var outcome = busy.requestStart(STEP, PAGE);
    busy.periodic(READY);

    assertThat(outcome.getNow(null).detail())
        .isEqualTo("a higher-priority command is using what it needs");
    assertThat(busy.run()).isNull();
  }

  @Test
  void anExperimentIsCheckedWhenItIsAdded() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> experiments.add(new Experiment(STEP, rig, Lag.STEP_AND_BACK)))
        .withMessage("two experiments are named " + STEP);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                experiments.add(
                    new Experiment(
                        "Too far",
                        rig,
                        Profiles.step()
                            .from(Degrees.of(0))
                            .to(Degrees.of(180))
                            .hold(Seconds.of(1)))))
        .withMessage("Too far can't run on Arm: 180 ° is outside Arm's limits, -100 ° to 100 °");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                experiments.add(
                    new Experiment(
                        "Too long",
                        rig,
                        Profiles.stepAndBack()
                            .from(Degrees.of(0))
                            .to(Degrees.of(10))
                            .hold(Seconds.of(5)))))
        .withMessage("Too long runs 10.5 s; an experiment runs at most 10.0 s");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> experiments.add(new Experiment("Same name", lag.rig(), Lag.STEP_AND_BACK)))
        .withMessage("two rigs are named Arm");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new Experiment(" ", rig, Lag.STEP_AND_BACK))
        .withMessage("an experiment needs a name");
    assertThat(new Experiment("x", rig, Lag.STEP_AND_BACK).settings())
        .isEqualTo(Lag.STEP_AND_BACK.toString());
  }

  /**
   * A page cancels the run it sees just as that run ends and another page starts the next, all in
   * one loop: the cancel names the run it meant, so the new run goes on.
   */
  @Test
  void aCancelForARunThatHasEndedNeverStopsTheNext() {
    start(experiments);
    loopsInTouch(110);
    assertThat(run().state().phase()).isEqualTo(Run.Phase.ENDED);

    var started = experiments.requestStart(STEP, PAGE);
    var late = experiments.requestCancel(STEP, 1, "");
    loop(READY);

    assertThat(started.getNow(null)).isEqualTo(Outcome.applied());
    assertThat(late.getNow(null))
        .isEqualTo(new Outcome(Outcome.Result.LOCKED, "Run 1 has ended already"));
    assertThat(run().id()).isEqualTo(2);
    assertThat(run().state().phase()).isEqualTo(Run.Phase.RUNNING);
    assertThat(logged.get(logged.size() - 1))
        .isEqualTo("START,CANCEL|" + STEP + "," + STEP + "|[0, 1]");
  }

  @Test
  void aCancelSaysWhyAndCancelingWhatIsntRunningChangesNothing() {
    var idle = experiments.requestCancel(STEP, 0, "");
    loop(READY);
    start(experiments);

    var hidden = experiments.requestCancel(STEP, run().id(), "this tab was hidden");
    loop(READY);

    assertThat(idle.getNow(null))
        .isEqualTo(new Outcome(Outcome.Result.LOCKED, STEP + " isn't running"));
    assertThat(hidden.getNow(null)).isEqualTo(Outcome.applied());
    assertThat(run().state().detail())
        .as("its command's own cleanup doesn't change it")
        .isEqualTo("Canceled: this tab was hidden");
    assertThat(run().controller()).isEqualTo(PAGE);
    assertThat(reported).containsExactly("run 1 of " + STEP + ": Canceled: this tab was hidden");
  }

  /**
   * An experiment never changes a gain: the page does, through the tee, which checks it as it
   * checks any change (its limits, then the configuration's own checks). A run records the gains it
   * ran with, so what the checks refused never runs.
   */
  @Test
  void gainsChangeOnlyThroughTheTeesChecksAndARunRecordsThem() {
    TunableTee tee = new TunableTee(new MockTunableBackend(), () -> null, inputs -> {}, false);
    TunableRegistry.registerBackend("", tee);
    TunableRegistry.publish("Arm", lag.config);
    tee.update();

    var beyondLimits = tee.requestChanges(Map.of("/Arm/gains/kP", 60.0));
    var fine = tee.requestChanges(Map.of("/Arm/gains/kP", 20.0));
    tee.update();
    start(experiments);

    assertThat(beyondLimits.getNow(null).detail())
        .isEqualTo("/Arm/gains/kP: 60 is above its limit of 50");
    assertThat(fine.getNow(null)).isEqualTo(new Outcome(Outcome.Result.APPLIED, ""));
    assertThat(run().gains()).containsEntry("gains.kP", 20.0);

    loopsInTouch(110);

    StepResponse up = firstJump(run());
    assertThat(up.riseSeconds().getAsDouble())
        .as("twice the gain, half the time")
        .isCloseTo(Math.log(9) / 20, within(0.001));
  }
}
