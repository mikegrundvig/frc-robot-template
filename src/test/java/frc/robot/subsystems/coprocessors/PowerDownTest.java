package frc.robot.subsystems.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.coprocessor.table.Board;
import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import com.michaelgrundvig.frc.coprocessor.table.Table;
import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Devices.Level;
import com.michaelgrundvig.frc.pitwall.Notices.Notice;
import com.michaelgrundvig.frc.pitwall.Outcome;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonVersion;

/**
 * Powering the coprocessors down before the robot is switched off, the whole sequence through the
 * simulated agents and Pitwall's action: asked of every one, waited on until each is gone on both
 * its ports, then "Safe to switch off", and, if the robot stays on, that they're halted.
 */
class PowerDownTest {
  private static final long LOOP = 20_000_000L;
  private static final Computer FRONT =
      new Computer("vision-front", 11, Board.ORANGEPI_5, List.of("front-left"), 5808);
  private static final Computer BACK =
      new Computer("vision-back", 12, Board.ORANGEPI_5, List.of("back-left"), 5808);
  private static final CompiledTable TABLE =
      new CompiledTable(
          new Table(24680, 5808, List.of(FRONT, BACK)),
          PhotonVersion.versionString,
          "recipe-1",
          Map.of());
  private static final Actions.Gate DISABLED = new Actions.Gate(false, false, false);
  private static final Actions.Gate ENABLED = new Actions.Gate(true, false, false);

  /** Disabled, after the field system connected once in this run (a match, then the queue). */
  private static final Actions.Gate AFTER_THE_FIELD = new Actions.Gate(false, false, false, true);

  private final CoprocessorIOSim front =
      new CoprocessorIOSim(TABLE, FRONT, Coprocessors.Settings.DEFAULTS);
  private final CoprocessorIOSim back =
      new CoprocessorIOSim(TABLE, BACK, Coprocessors.Settings.DEFAULTS);
  private final FakeButton button = new FakeButton();
  private final Coprocessors coprocessors =
      new Coprocessors(
          TABLE,
          new CoprocessorIO[] {front, back},
          new PhotonVisionIOSim(List.of(front, back)),
          button,
          Coprocessors.Settings.DEFAULTS);
  private final Actions actions = new Actions(inputs -> {});
  private long now;

  /** A button whose state the test sets. */
  private static final class FakeButton implements PowerButtonIO {
    boolean pressed;

    @Override
    public void updateInputs(PowerButtonIOInputs inputs) {
      inputs.pressed = pressed;
    }
  }

  PowerDownTest() {
    coprocessors.addTo(actions);
  }

  @AfterEach
  void close() {
    coprocessors.close();
  }

  /** Runs the robot's loop this long: Pitwall's actions first, as in the robot's own loop. */
  private void run(double seconds, Actions.Gate gate) {
    for (long end = now + Math.round(seconds * 1e9); now < end; now += LOOP) {
      actions.periodic(gate);
      front.step(now);
      back.step(now);
      coprocessors.periodic(now, gate.enabled());
    }
  }

  private void run(double seconds) {
    run(seconds, DISABLED);
  }

  /** Runs this long, and says whether the page ever said it was safe to switch off meanwhile. */
  private boolean saidSafe(double seconds) {
    boolean safe = false;
    for (long end = now + Math.round(seconds * 1e9); now < end; ) {
      run(0.02);
      safe |= notice().equals("OK: Safe to switch off");
    }
    return safe;
  }

  private Actions.Phase phase() {
    return actions.view().get(0).phase();
  }

  private String notice() {
    Notice notice = coprocessors.notice();
    return notice == null ? "" : notice.level() + ": " + notice.text();
  }

  @Test
  void poweredDownTheyreSafeToSwitchOffThenHalted() {
    run(2);
    assertThat(coprocessors.notice()).isNull();

    CompletableFuture<Outcome> asked = actions.requestStart(Coprocessors.POWER_DOWN);
    run(1);

    assertThat(asked.getNow(null)).isEqualTo(Outcome.applied());
    assertThat(phase()).isEqualTo(Actions.Phase.RUNNING);
    assertThat(notice())
        .isEqualTo("NOTE: Powering down coprocessors: waiting on vision-front, vision-back");
    assertThat(coprocessors.card("vision-front").state()).isEqualTo("Powering down");

    // They halt three seconds after asking; then neither port answers.
    run(3);

    assertThat(front.isHalted()).isTrue();
    assertThat(phase()).as("the action ends once they're gone").isEqualTo(Actions.Phase.ENDED);
    assertThat(notice()).isEqualTo("OK: Safe to switch off");
    assertThat(coprocessors.card("vision-front").state()).isEqualTo("Powered down");
    assertThat(coprocessors.card("vision-front").level()).isEqualTo(Level.NOTE);
    assertThat(coprocessors.card("vision-front").lines().get(0).value())
        .startsWith("powered down from Pitwall");

    // Missing on purpose: the alert says so, rather than that it failed.
    run(3);
    assertThat(coprocessors.alerts("vision-front"))
        .containsExactly(
            "MEDIUM: Coprocessor vision-front (10.246.80.11) was powered down from Pitwall:"
                + " cycle the robot's power to restart it");

    // The robot left on: halted boards stay off until its power is cycled.
    run(175);
    assertThat(notice()).isEqualTo("OK: Safe to switch off");
    run(5);
    assertThat(notice())
        .isEqualTo("WARNING: Coprocessors halted: cycle the robot's power to restart them");
  }

  /** A board that never got the request is never safe to switch off, however quiet it is. */
  @Test
  void aBoardThatCouldntBeReachedIsNeverSafe() {
    run(1);
    front.unreachable(true); // its Ethernet unplugged: quiet on both ports, and still running

    actions.requestStart(Coprocessors.POWER_DOWN);
    boolean safe = saidSafe(40);

    assertThat(safe).isFalse();
    assertThat(front.isHalted()).as("it was never asked").isFalse();
    assertThat(notice())
        .isEqualTo(
            "PROBLEM: Don't switch off: vision-front couldn't be reached: check it before switching"
                + " off");
    assertThat(coprocessors.card("vision-front").lines().get(0).value())
        .isEqualTo("couldn't be asked to power down: connect timed out");
    assertThat(coprocessors.card("vision-front").lines().get(0).level()).isEqualTo(Level.PROBLEM);
    assertThat(coprocessors.alerts("vision-front"))
        .contains(
            "MEDIUM: Coprocessor vision-front (10.246.80.11) stopped answering: connect timed out")
        .anySatisfy(
            alert ->
                assertThat(alert)
                    .isEqualTo(
                        "HIGH: Not powered down 30 s after being asked: vision-front (couldn't be"
                            + " reached: connect timed out). Don't switch the robot off yet:"
                            + " check each first"));
  }

  /** A request that found no one is asked again each second, until it's taken. */
  @Test
  void aRequestThatFoundNoOneIsAskedAgainUntilItsTaken() {
    run(1);
    front.unreachable(true);
    actions.requestStart(Coprocessors.POWER_DOWN);
    run(5);

    front.unreachable(false); // plugged back in
    run(5);

    assertThat(front.isHalted()).isTrue();
    assertThat(notice()).isEqualTo("OK: Safe to switch off");
  }

  /** A board that refuses (an image without the request) is said so, and not asked again. */
  @Test
  void aBoardThatRefusesIsNamedAndNotAskedAgain() {
    back.ignoresPowerDown(true);
    run(1);

    actions.requestStart(Coprocessors.POWER_DOWN);
    run(29);
    List<String> early = coprocessors.alerts("vision-back");
    int asked = back.powerDownRequests();
    run(2);

    assertThat(early).isEmpty();
    assertThat(asked).as("a refusal isn't asked again").isEqualTo(1);
    assertThat(coprocessors.alerts("vision-back"))
        .containsExactly(
            "HIGH: Not powered down 30 s after being asked: vision-back (refused: answered 404)."
                + " Don't switch the robot off yet: check each first");
    assertThat(coprocessors.card("vision-back").state()).isEqualTo("Not powered down");
    assertThat(coprocessors.card("vision-back").lines().get(0).value())
        .isEqualTo("refused to power down: answered 404");
    assertThat(coprocessors.card("vision-back").lines().get(0).level()).isEqualTo(Level.PROBLEM);
    assertThat(notice())
        .isEqualTo(
            "PROBLEM: Don't switch off: vision-back refused to power down: check it before"
                + " switching off");
    assertThat(phase()).as("still waiting").isEqualTo(Actions.Phase.RUNNING);
  }

  /** One that took the request but is still up past the timeout is named as still answering. */
  @Test
  void oneStillAnsweringAfterThirtySecondsIsNamed() {
    back.haltAfter(60);
    run(1);

    actions.requestStart(Coprocessors.POWER_DOWN);
    run(31);

    assertThat(coprocessors.alerts("vision-back"))
        .containsExactly(
            "HIGH: Not powered down 30 s after being asked: vision-back (still answering). Don't"
                + " switch the robot off yet: check each first");
    assertThat(coprocessors.card("vision-back").state()).isEqualTo("Powering down");
    assertThat(coprocessors.card("vision-back").lines().get(0).value())
        .isEqualTo("asked to power down, accepted; its agent and PhotonVision still answering");
    assertThat(notice())
        .isEqualTo(
            "PROBLEM: Don't switch off: vision-back still answering: check it before switching"
                + " off");
  }

  /** A button reading pressed from the start is wiring, not a press: it does nothing. */
  @Test
  void aButtonPressedFromTheStartDoesNothing() {
    button.pressed = true;

    run(3);

    assertThat(phase()).isEqualTo(Actions.Phase.READY);
    assertThat(front.isHalted()).isFalse();
    assertThat(coprocessors.buttonAlert())
        .isEqualTo(
            "MEDIUM: The coprocessors' power-down button has read pressed since the robot program"
                + " started: check its wiring. It works once it has read released");

    button.pressed = false;
    run(0.1);
    button.pressed = true;
    run(1.2);

    assertThat(coprocessors.buttonAlert()).isEmpty();
    assertThat(phase()).as("a press after a release works").isEqualTo(Actions.Phase.RUNNING);
  }

  /** Once the field system has connected in this run, the button powers nothing down. */
  @Test
  void theButtonDoesNothingOnceTheFieldHasConnected() {
    run(1, AFTER_THE_FIELD);

    button.pressed = true;
    run(1.5, AFTER_THE_FIELD);
    run(4, AFTER_THE_FIELD);

    assertThat(phase()).isEqualTo(Actions.Phase.READY);
    assertThat(front.powerDownRequests()).isZero();
    assertThat(front.isHalted()).isFalse();
  }

  @Test
  void itRunsOnlyWhileDisabledAndEnablingStopsTheWait() {
    run(1);

    CompletableFuture<Outcome> whileEnabled = actions.requestStart(Coprocessors.POWER_DOWN);
    run(1, ENABLED);
    assertThat(whileEnabled.getNow(null).result()).isEqualTo(Outcome.Result.LOCKED);
    assertThat(front.isHalted()).isFalse();

    back.ignoresPowerDown(true);
    actions.requestStart(Coprocessors.POWER_DOWN);
    run(1);
    run(1, ENABLED);

    assertThat(phase()).isEqualTo(Actions.Phase.CANCELED);
    assertThat(coprocessors.notice()).as("no longer waiting").isNull();
  }

  @Test
  void theButtonHeldASecondPowersThemDown() {
    run(1);

    button.pressed = true;
    run(0.5);
    assertThat(phase()).as("a brush against it does nothing").isEqualTo(Actions.Phase.READY);
    run(0.6);
    button.pressed = false;
    run(0.1);

    assertThat(phase()).isEqualTo(Actions.Phase.RUNNING);
    run(4);
    assertThat(notice()).isEqualTo("OK: Safe to switch off");
  }
}
