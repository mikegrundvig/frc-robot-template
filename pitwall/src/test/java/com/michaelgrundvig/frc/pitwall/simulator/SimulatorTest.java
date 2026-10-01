package com.michaelgrundvig.frc.pitwall.simulator;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.Outcome;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;

/**
 * The simulator's controls, on a stand-in for WPILib's simulation: what the page asks waits for the
 * robot loop, which applies it first thing, all of it seen at once; the page's joysticks with their
 * deadman; and the clock, which the loop pauses and the caller's thread resumes and steps.
 */
class SimulatorTest {
  private static final long LOOP_NANOS = 20_000_000;

  private final FakeHal hal = new FakeHal();
  private final AtomicLong clock = new AtomicLong();
  private final AtomicInteger stops = new AtomicInteger();
  private final List<List<String>> logged = new ArrayList<>();
  private boolean lampOut;
  private final Components components =
      new Components(said -> {})
          .add(
              Component.of(
                  "Lamp",
                  "light",
                  Fault.lasting("Out")
                      .does("Dark.")
                      .isOn(() -> lampOut)
                      .apply(values -> lampOut = true)
                      .undo("Lit.", () -> lampOut = false)
                      .build(),
                  Fault.lasting("Dim")
                      .does("Dimmer.")
                      .parameter("Level", "", Limits.between(0, 1), 0.5)
                      .isOn(() -> false)
                      .apply(values -> {})
                      .undo("Bright.", () -> {})
                      .build()));
  private final Simulator simulator =
      new Simulator(hal, clock::get, components, stops::incrementAndGet, logged::add);

  /**
   * One robot loop, if one comes: the clock moves a loop's time, then the simulator's part of it. A
   * paused clock runs none, until a step lets one through.
   */
  private void loop() {
    if (!hal.runsLoop()) {
      return;
    }
    clock.addAndGet(LOOP_NANOS);
    simulator.periodic();
  }

  private void loops(int count) {
    for (int i = 0; i < count; i++) {
      loop();
    }
  }

  @Test
  void aChangeWaitsForTheLoopWhichAppliesItAllAtOnceAndAnnouncesIt() {
    CompletableFuture<Outcome> enabled =
        simulator.requestDriverStation(
            state ->
                state
                    .withMode(RobotMode.UTILITY)
                    .withAllianceStation(AllianceStationID.RED_3)
                    .withEnabled(true));

    assertThat(enabled).isNotDone();
    assertThat(hal.driverStation().enabled()).isFalse();
    loop();

    assertThat(enabled).isCompletedWithValue(Outcome.applied());
    DriverStationState state = hal.driverStation();
    assertThat(state.enabled()).isTrue();
    assertThat(state.dsAttached()).as("enabling attaches the Driver Station").isTrue();
    assertThat(state.mode()).isEqualTo(RobotMode.UTILITY);
    assertThat(state.allianceStation()).isEqualTo(AllianceStationID.RED_3);
    assertThat(hal.writes()).as("one write").isEqualTo(1);
    assertThat(hal.announced()).as("announced, so the loop sees it now").isEqualTo(1);
  }

  @Test
  void aChangeThatChangesNothingWritesNothing() {
    simulator.requestDriverStation(state -> state.withEnabled(false));
    loop();

    assertThat(hal.writes()).isZero();
    assertThat(hal.announced()).isZero();
  }

  @Test
  void withoutADriverStationTheRobotIsDisabled() {
    simulator.requestDriverStation(state -> state.withEnabled(true));
    loop();
    simulator.requestDriverStation(state -> state.withDsAttached(false));
    loop();

    assertThat(hal.driverStation().enabled()).isFalse();
    assertThat(hal.driverStation().dsAttached()).isFalse();
  }

  /** The match time counts down while enabled, as the GUI's does, once the page sets it. */
  @Test
  void theMatchTimeCountsDownWhileEnabled() {
    simulator.requestDriverStation(state -> state.withMatchTime(15));
    loops(50); // disabled: it holds
    assertThat(hal.driverStation().matchTime()).isEqualTo(15);

    simulator.requestDriverStation(state -> state.withEnabled(true));
    loop();
    loops(50);
    assertThat(hal.driverStation().matchTime())
        .isCloseTo(14, org.assertj.core.api.Assertions.within(0.03));

    loops(1000);
    assertThat(hal.driverStation().matchTime()).as("never below zero").isZero();
    assertThat(hal.writes()).as("counting writes the match time alone").isEqualTo(2);
    assertThat(hal.matchTimes()).isPositive();
  }

  @Test
  void aJoystickThePageNeverDroveIsNeverTouched() {
    simulator.joystick(0, input(0.5), "1");
    loops(3);

    assertThat(hal.joystick(0)).isEqualTo(input(0.5));
    assertThat(hal.joystick(1)).isNull();
  }

  /** The latest input wins: a loop takes it once, however many came since the last. */
  @Test
  void aLoopTakesTheLatestInput() {
    simulator.joystick(0, input(0.2), "1");
    simulator.joystick(0, input(0.7), "1");
    loop();

    assertThat(hal.joystick(0)).isEqualTo(input(0.7));
    assertThat(hal.announced()).isEqualTo(1);
  }

  /** The page repeats its input while anything is pressed; silent for 500 ms, it's centered. */
  @Test
  void theDeadmanCentersAJoystickWhenThePageGoesQuiet() {
    simulator.joystick(0, input(0.5), "1");
    loops(20); // 400 ms
    simulator.joystick(0, input(0.5), "1"); // the page's repeat
    loops(24); // 480 ms since
    assertThat(hal.joystick(0)).isEqualTo(input(0.5));

    loops(2); // 520 ms since
    assertThat(hal.joystick(0)).isEqualTo(JoystickInput.CENTERED);
    assertThat(Objects.requireNonNull(simulator.view().get("simulator")).get("joysticks"))
        .isEqualTo(
            Map.of(
                "0", Map.of("fromPage", true, "centeredByDeadman", true, "stream", "1"),
                "1", Map.of("fromPage", false, "centeredByDeadman", false, "stream", "")));

    simulator.joystick(0, input(0.3), "1"); // back in touch
    loop();
    assertThat(hal.joystick(0)).isEqualTo(input(0.3));
  }

  /** Centered, there's nothing for the deadman to stop: the page needn't repeat it. */
  @Test
  void aCenteredJoystickNeedsNoRepeat() {
    simulator.joystick(0, input(0.5), "1");
    loop();
    simulator.joystick(0, JoystickInput.CENTERED, "1");
    loop();
    int announced = hal.announced();

    loops(100);

    assertThat(hal.announced()).isEqualTo(announced);
  }

  /** When the page's event stream closes, its joysticks are let go: centered, and not its own. */
  @Test
  void aJoystickIsLetGoWhenItsPagesStreamCloses() {
    simulator.joystick(1, input(-0.4), "7");
    loop();

    simulator.lost("8"); // another page's
    loop();
    assertThat(hal.joystick(1)).isEqualTo(input(-0.4));

    simulator.lost("7");
    loop();
    assertThat(hal.joystick(1)).isEqualTo(JoystickInput.CENTERED);
    assertThat(Objects.requireNonNull(simulator.view().get("simulator")).get("joysticks"))
        .asString()
        .contains("1={fromPage=false");
  }

  /**
   * Two pages drive one joystick: the latest input wins, and only the page driving it now can let
   * it go. The other page switching its driving off changes nothing.
   */
  @Test
  void onlyThePageDrivingAJoystickNowLetsItGo() {
    simulator.joystick(0, input(0.5), "1");
    loop();
    simulator.joystick(0, input(-0.5), "2"); // another page drives it too, and is now the latest
    loop();
    assertThat(hal.joystick(0)).isEqualTo(input(-0.5));

    simulator.joystick(0, null, "1"); // the first page switches its driving off
    loop();
    assertThat(hal.joystick(0)).as("still the second page's").isEqualTo(input(-0.5));
    assertThat(joysticks()).contains("0={fromPage=true, centeredByDeadman=false, stream=2}");

    simulator.lost("1"); // and closes: nothing of its own to let go
    loop();
    assertThat(hal.joystick(0)).isEqualTo(input(-0.5));

    simulator.joystick(0, null, "2");
    loop();
    assertThat(hal.joystick(0)).isEqualTo(JoystickInput.CENTERED);
    assertThat(joysticks()).contains("0={fromPage=false, centeredByDeadman=false, stream=}");

    simulator.joystick(0, null, "3"); // a let go of a joystick nobody drives changes nothing
    loop();
    assertThat(hal.joystick(0)).isEqualTo(JoystickInput.CENTERED);
  }

  /** The loop pauses the clock, so its own last view says it's paused. */
  @Test
  void theLoopPausesTheClockOnceItHasAppliedTheRest() {
    CompletableFuture<Outcome> enabled =
        simulator.requestDriverStation(state -> state.withEnabled(true));
    CompletableFuture<Outcome> paused = simulator.requestPause();
    loop();

    assertThat(enabled).isCompleted();
    assertThat(paused).isCompletedWithValue(Outcome.applied());
    assertThat(hal.isPaused()).isTrue();
    assertThat(simulator.lastView().get("simulator"))
        .containsEntry("paused", true)
        .containsEntry("enabled", true);
  }

  /** A paused clock runs no loops: resuming and stepping happen on the caller's thread. */
  @Test
  void resumingAndSteppingHappenAtOnce() {
    assertThat(simulator.step())
        .as("not paused")
        .isCompletedWithValue(
            new Outcome(
                Outcome.Result.LOCKED, "Step runs one loop of a paused simulator: pause it"));
    simulator.requestPause();
    loop();
    CompletableFuture<Outcome> enabled =
        simulator.requestDriverStation(state -> state.withEnabled(true));
    loops(10);
    assertThat(enabled).as("paused: no loop comes").isNotDone();

    CompletableFuture<Outcome> stepped = simulator.step();
    assertThat(hal.steps()).isEqualTo(1);
    assertThat(stepped).as("done once the loop has run").isNotDone();
    loops(10);
    assertThat(stepped).isCompletedWithValue(Outcome.applied());
    assertThat(enabled).as("asked while paused, applied by the step").isCompleted();
    assertThat(hal.isPaused()).as("one loop, then paused still").isTrue();

    CompletableFuture<Outcome> resumed = simulator.resume();
    assertThat(hal.isPaused()).isFalse();
    loop();
    assertThat(resumed).isCompleted();
  }

  /**
   * A pause asked while paused is answered at once: it would wait for a loop that never comes, and
   * then pause again right after a resume.
   */
  @Test
  void aPauseAskedWhilePausedIsAnsweredAtOnceAndAResumeHolds() {
    simulator.requestPause();
    loop();
    assertThat(hal.isPaused()).isTrue();

    CompletableFuture<Outcome> again = simulator.requestPause(); // a second page, a double click
    CompletableFuture<Outcome> resumed = simulator.resume();
    loops(3);

    assertThat(again).isCompletedWithValue(Outcome.applied());
    assertThat(resumed).isCompletedWithValue(Outcome.applied());
    assertThat(hal.isPaused()).as("the resume holds").isFalse();
  }

  /** A resume drops a pause asked before it that the loop hasn't got to yet. */
  @Test
  void aResumeDropsAPauseAskedBeforeIt() {
    CompletableFuture<Outcome> paused = simulator.requestPause();
    simulator.resume();
    loops(3);

    assertThat(hal.isPaused()).isFalse();
    assertThat(paused)
        .isCompletedWithValue(
            new Outcome(Outcome.Result.LOCKED, "Resumed since it was asked: not paused"));
  }

  /**
   * Stop asked while the loop is between its check for a Stop and its pause (the server's thread
   * can land there): the loop doesn't pause, so the next loop runs, and stops the program.
   */
  @Test
  void aStopAskedMidLoopIsntLostToAPause() {
    components.add(
        Component.of(
            "Server",
            "a stand-in",
            Fault.event("Stops")
                .does("Asks to stop partway through the loop, as the server's thread could.")
                .apply(values -> simulator.stop())
                .undoneBy("Nothing.")
                .build()));
    simulator.requestFault(Setting.of("Server", "Stops"));
    simulator.requestPause();
    loop();

    assertThat(hal.isPaused()).as("a Stop is due: no pause").isFalse();
    assertThat(stops).hasValue(0);
    loop();
    assertThat(stops).hasValue(1);
  }

  /** Something in the loop that throws leaves no request waiting for an answer. */
  @Test
  void aLoopThatThrowsStillAnswersEveryRequest() {
    components.add(
        Component.of(
            "Broken",
            "a stand-in",
            Fault.lasting("Throws")
                .does("Can't say whether it's on.")
                .isOn(
                    () -> {
                      throw new IllegalStateException("broken");
                    })
                .apply(values -> {})
                .undo("Nothing.", () -> {})
                .build()));
    CompletableFuture<Outcome> enabled =
        simulator.requestDriverStation(state -> state.withEnabled(true));
    CompletableFuture<Map<String, Map<String, Object>>> view = simulator.requestView();

    assertThatThrownBy(this::loop).hasMessage("broken");

    assertThat(enabled).isCompletedWithValue(Outcome.applied());
    assertThat(view).isCompletedExceptionally();
  }

  /** Stop asks the loop to end the program, resuming a paused clock so a loop runs to do it. */
  @Test
  void stopEndsTheProgramFromTheLoop() {
    hal.pause();

    simulator.stop();
    assertThat(hal.isPaused()).isFalse();
    assertThat(stops).hasValue(0);
    loop();

    assertThat(stops).hasValue(1);
  }

  @Test
  void faultsAreAppliedByTheLoopAndLoggedWhenTheyChange() {
    CompletableFuture<Outcome> out = simulator.requestFault(Setting.of("Lamp", "Out"));
    CompletableFuture<Outcome> wrong =
        simulator.requestFault(Setting.of("Lamp", "Dim", Map.of("Level", 3.0)));
    loop();

    assertThat(out).isCompletedWithValue(Outcome.applied());
    assertThat(wrong)
        .isCompletedWithValue(
            new Outcome(Outcome.Result.REFUSED, "Level: 3 is above its limit of 1"));
    assertThat(lampOut).isTrue();
    assertThat(logged).containsExactly(List.of("Lamp: Out"));

    simulator.requestUndoAll();
    loops(3);
    assertThat(lampOut).isFalse();
    assertThat(logged).containsExactly(List.of("Lamp: Out"), List.of());
  }

  @Test
  void theViewSaysWhatTheRobotProgramSees() {
    simulator.requestDriverStation(
        state -> state.withMode(RobotMode.AUTONOMOUS).withFmsAttached(true).withMatchTime(14.2));
    loop();

    Map<String, Object> state = simulator.view().get("simulator");

    assertThat(state)
        .containsEntry("enabled", false)
        .containsEntry("mode", "AUTONOMOUS")
        .containsEntry("allianceStation", "UNKNOWN")
        .containsEntry("fmsAttached", true)
        .containsEntry("dsAttached", false)
        .containsEntry("matchTime", 15.0)
        .containsEntry("paused", false);
    assertThat(simulator.view()).containsOnlyKeys("simulator", "components", "scenarios");
  }

  @Test
  void aViewAskedForIsBuiltByTheNextLoop() {
    CompletableFuture<Map<String, Map<String, Object>>> asked = simulator.requestView();

    assertThat(simulator.requestView()).as("shared until built").isSameAs(asked);
    loop();

    assertThat(asked).isDone();
    assertThat(asked.join()).containsKey("components");
  }

  /** The joysticks as the view shows them, as text. */
  private String joysticks() {
    return String.valueOf(
        Objects.requireNonNull(simulator.view().get("simulator")).get("joysticks"));
  }

  private static JoystickInput input(double leftX) {
    return new JoystickInput(List.of(leftX, 0.0), List.of(false, true), List.of(-1));
  }
}
