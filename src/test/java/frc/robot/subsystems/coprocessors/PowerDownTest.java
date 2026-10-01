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
          new Table(1234, 5808, List.of(FRONT, BACK)),
          PhotonVersion.versionString,
          "recipe-1",
          Map.of());
  private static final Actions.Gate DISABLED = new Actions.Gate(false, false, false);
  private static final Actions.Gate ENABLED = new Actions.Gate(true, false, false);

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
            "MEDIUM: Coprocessor vision-front (10.12.34.11) was powered down from Pitwall: cycle"
                + " the robot's power to restart it");

    // The robot left on: halted boards stay off until its power is cycled.
    run(175);
    assertThat(notice()).isEqualTo("OK: Safe to switch off");
    run(5);
    assertThat(notice())
        .isEqualTo("WARNING: Coprocessors halted: cycle the robot's power to restart them");
  }

  @Test
  void oneStillAnsweringAfterThirtySecondsIsNamed() {
    back.ignoresPowerDown(true);
    run(1);

    actions.requestStart(Coprocessors.POWER_DOWN);
    run(29);
    List<String> early = coprocessors.alerts("vision-back");
    run(2);

    assertThat(early).isEmpty();
    assertThat(coprocessors.alerts("vision-back"))
        .containsExactly(
            "HIGH: Still answering 30 s after being asked to power down: vision-back (answered"
                + " 404). Don't switch the robot off yet: power it down from its own page, or"
                + " unplug it once it has been checked");
    assertThat(coprocessors.card("vision-back").state()).isEqualTo("Powering down");
    assertThat(coprocessors.card("vision-back").lines().get(0).value())
        .isEqualTo("asked to power down, answered 404; its agent and PhotonVision still answering");
    assertThat(coprocessors.card("vision-back").lines().get(0).level()).isEqualTo(Level.WARNING);
    assertThat(notice())
        .isEqualTo(
            "PROBLEM: Don't switch off: vision-back still answering after being asked to power"
                + " down");
    assertThat(phase()).as("still waiting").isEqualTo(Actions.Phase.RUNNING);
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
