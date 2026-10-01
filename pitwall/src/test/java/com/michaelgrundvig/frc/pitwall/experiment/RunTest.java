package com.michaelgrundvig.frc.pitwall.experiment;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * A run, loop by loop, on a mechanism whose answer is known exactly ({@link Lag}): the scores come
 * out as the textbook says, whichever unit the measurement is in.
 */
class RunTest {
  private final Lag lag = new Lag();

  private Run run(Rig rig, Profile profile) {
    return new Run(1, new Experiment("Step and back", rig, profile), "7");
  }

  private static Run.Scores scores(Run run) {
    return Objects.requireNonNull(run.state().scores(), "not scored");
  }

  /** Runs the loop until the run is over, as its command would; returns how many loops it took. */
  private int runToTheEnd(Run run) {
    int loops = 0;
    while (!run.step(lag.nextLoop())) {
      loops++;
    }
    return loops + 1;
  }

  @Test
  void aStepAndBackOnAFirstOrderLagScoresAsTheTextbookSays() {
    Run run = run(lag.rig(), Lag.STEP_AND_BACK);

    runToTheEnd(run);

    assertThat(run.state().phase()).isEqualTo(Run.Phase.ENDED);
    assertThat(run.state().detail()).isEqualTo("Ended");
    Run.Scores scores = scores(run);
    double tau = 1 / 10.0;
    List<Optional<StepResponse>> steps = Objects.requireNonNull(scores.steps().get("Angle"));
    assertThat(steps).hasSize(2);
    for (Optional<StepResponse> step : steps) {
      StepResponse response = step.orElseThrow();
      assertThat(response.riseSeconds().getAsDouble()).isCloseTo(tau * Math.log(9), within(0.001));
      assertThat(response.settlingSeconds().getAsDouble())
          .isCloseTo(tau * Math.log(50), within(0.002));
      assertThat(response.overshoot()).isZero();
      assertThat(response.steadyStateError()).as("in degrees").isCloseTo(0, within(0.1));
    }
  }

  @Test
  void theHighestVoltageIsThePeak() {
    Run run = run(lag.rig(), Lag.STEP_AND_BACK);

    runToTheEnd(run);

    // Read once a loop: the first loop after each jump, 20 ms in, is the highest.
    double first = 10 * Math.PI / 2 * Math.exp(-10 * 0.02);
    assertThat(Objects.requireNonNull(scores(run).peaks().get("Voltage")).getAsDouble())
        .isCloseTo(first, within(0.01));
  }

  @Test
  void itRunsForItsProfilesLengthAndRecordsTheSetpointBesideTheMeasurement() {
    Run run = run(lag.rig(), Lag.STEP_AND_BACK);

    int loops = runToTheEnd(run);

    assertThat(loops).as("2.1 s at 50 loops a second, and the loop that ends it").isEqualTo(106);
    Series setpoint = run.trace().series(Rig.SETPOINT);
    assertThat(setpoint.count()).isEqualTo(105);
    assertThat(setpoint.value(0)).as("in the measurement's unit").isEqualTo(0);
    assertThat(setpoint.value(25)).as("0.5 s in").isCloseTo(90, within(1e-9));
    assertThat(setpoint.value(65)).as("1.3 s in").isEqualTo(0);
    Series angle = run.trace().series("Angle");
    assertThat(angle.count()).as("five readings a loop, from the second").isEqualTo(105 * 5);
    assertThat(angle.time(0)).isGreaterThan(run.startNanos());
  }

  @Test
  void theGainsAreThoseItStartedWith() {
    Run run = run(lag.rig(), Lag.STEP_AND_BACK);
    lag.config.set(Map.of("gains.kP", 20.0));

    assertThat(run.gains()).containsEntry("gains.kP", 10.0).containsEntry("gains.kD", 0.0);
  }

  /**
   * A gain changed while a run runs (from the tuning table, or a dashboard) leaves its scores
   * answering for two sets of gains: it ends saying so, even when the gain goes back.
   */
  @Test
  void aRunWhoseGainsChangedWhileItRanSaysSo() {
    Run run = run(lag.rig(), Lag.STEP_AND_BACK);
    for (int i = 0; i < 30; i++) {
      run.step(lag.nextLoop());
    }
    lag.config.set(Map.of("gains.kP", 20.0));
    run.step(lag.nextLoop());
    lag.config.set(Map.of("gains.kP", 10.0));

    runToTheEnd(run);

    assertThat(run.gainsChanged()).isTrue();
    assertThat(run.state().phase()).as("still scored").isEqualTo(Run.Phase.ENDED);
    assertThat(run.state().detail())
        .isEqualTo(
            "Ended, but its gains changed while it ran (gains.kP 10.0 to 20.0), so it's no fair"
                + " comparison");
  }

  @Test
  void aChangeToAnotherValueOfItsConfigurationIsNoChangeToItsGains() {
    Run run = run(lag.rig(), Lag.STEP_AND_BACK);
    run.step(lag.nextLoop());
    lag.config.set(Map.of("gains.kD", 0.0)); // the same value: a new record, the same gains

    runToTheEnd(run);

    assertThat(run.gainsChanged()).isFalse();
    assertThat(run.state().detail()).isEqualTo("Ended");
  }

  @Test
  void aSetpointOutsideTheRigsLimitsIsHeldAtTheLimit() {
    List<Double> sent = new ArrayList<>();
    Rig narrow =
        Rig.builder()
            .name("Narrow")
            .gains(lag.config, "gains.kP")
            .setpoint(Degrees, sent::add)
            .limits(Degrees.of(0), Degrees.of(45))
            .measurement("Angle", Degrees, () -> 0)
            .build();
    // Past the Experiments' own check, which would refuse this profile outright.
    Run run = run(narrow, Lag.STEP_AND_BACK);

    runToTheEnd(run);

    assertThat(sent).hasSize(105).contains(0.0).doesNotContain(90.0);
    assertThat(sent.get(30)).isCloseTo(45, within(1e-9));
  }

  @Test
  void aSetpointThatCantBeCommandedCancelsTheRunAndSaysWhy() {
    Rig broken =
        Rig.builder()
            .name("Broken")
            .gains(lag.config, "gains.kP")
            .setpoint(
                Radians,
                radians -> {
                  if (radians > 0) {
                    throw new IllegalStateException("the motor controller is gone");
                  }
                })
            .limits(Degrees.of(-100), Degrees.of(100))
            .measurement("Angle", Degrees, () -> 0)
            .build();
    Run run = run(broken, Lag.STEP_AND_BACK);

    runToTheEnd(run);

    assertThat(run.state())
        .isEqualTo(
            new Run.State(
                Run.Phase.CANCELED,
                "Canceled: commanding the setpoint failed: the motor controller is gone",
                null));
  }

  @Test
  void aSignalThatCantBeReadCostsOnlyItsSamples() {
    Rig flaky =
        Rig.builder()
            .name("Flaky")
            .gains(lag.config, "gains.kP")
            .setpoint(Radians, radians -> lag.setpoint = radians)
            .limits(Degrees.of(-100), Degrees.of(100))
            .measurement("Angle", Degrees, () -> Math.toDegrees(lag.angle))
            .signal(
                "Current",
                Amps,
                () -> {
                  throw new IllegalStateException("no reply");
                })
            .build();
    Run run = run(flaky, Lag.STEP_AND_BACK);

    runToTheEnd(run);

    assertThat(run.state().phase()).isEqualTo(Run.Phase.ENDED);
    assertThat(run.state().detail()).isEqualTo("Ended, but reading Current failed: no reply");
    assertThat(run.trace().series("Angle").count()).isEqualTo(105);
    assertThat(scores(run).peaks().get("Current")).isEmpty();
  }

  @Test
  void aCanceledRunStaysCanceledAndIsNotScored() {
    Run run = run(lag.rig(), Lag.STEP_AND_BACK);
    run.step(lag.nextLoop());

    run.cancel("Canceled from the page");
    run.cancel("Canceled: another command needed Arm");

    assertThat(run.step(lag.nextLoop())).as("over").isTrue();
    assertThat(run.state())
        .as("the first reason stands")
        .isEqualTo(new Run.State(Run.Phase.CANCELED, "Canceled from the page", null));
    assertThat(run.controller()).isEqualTo("7");
    assertThat(run.trace().series(Rig.SETPOINT).count()).isEqualTo(1);
  }

  @Test
  void anEndedRunIsNotCanceledAfterward() {
    Run run =
        run(
            lag.rig(),
            Profiles.step().from(Degrees.of(0)).to(Degrees.of(10)).hold(Seconds.of(0.1)));
    runToTheEnd(run);

    run.cancel("Canceled from the page");

    assertThat(run.state().phase()).isEqualTo(Run.Phase.ENDED);
    assertThat(run).hasToString("run 1 of Step and back: Ended");
  }
}
