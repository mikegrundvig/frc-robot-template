package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.coprocessor.table.Board;
import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import com.michaelgrundvig.frc.coprocessor.table.Table;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.subsystems.coprocessors.CoprocessorIO;
import frc.robot.subsystems.coprocessors.CoprocessorIO.CoprocessorIOInputs;
import frc.robot.subsystems.coprocessors.CoprocessorIOSim;
import frc.robot.subsystems.coprocessors.Coprocessors;
import frc.robot.subsystems.coprocessors.PhotonVisionIO.PhotonVisionIOInputs;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.photonvision.PhotonVersion;

/**
 * A simulated coprocessor's live faults, as the page applies them: each changes what the robot's
 * code reads from its health agent, and undoing it puts that back.
 */
@ExtendWith(RobotSim.class)
class CoprocessorFaultsTest {
  private static final String PART = Faults.coprocessor("vision-front");
  private static final long SECOND = 1_000_000_000L;
  private static final Computer FRONT =
      new Computer(
          "vision-front", 11, Board.ORANGEPI_5, List.of("front-left", "front-right"), 5808);
  private static final CompiledTable TABLE =
      new CompiledTable(
          new Table(0, 5808, List.of(FRONT)),
          PhotonVersion.versionString,
          "recipe-1",
          Map.of("vision-front", "hash-1"));

  private final SimulatedWorld world =
      new SimulatedWorld(
          RobotSim.FIELD, Robots.SIM.drivetrain(), Robots.SIM.body(), List.of(), SensorErrors.NONE);
  private final Components components = new Components(said -> {});
  private final CoprocessorIO coprocessor =
      world.coprocessors(TABLE, Coprocessors.Settings.DEFAULTS)[0];
  private long now;

  @BeforeEach
  void declare() {
    world.addTo(components);
  }

  @AfterEach
  void close() {
    world.close();
  }

  /** The world moves on this long; then the robot reads its coprocessor, as its loop would. */
  private CoprocessorIOInputs after(double seconds) {
    now += Math.round(seconds * SECOND);
    world.step(now);
    CoprocessorIOInputs inputs = new CoprocessorIOInputs();
    coprocessor.updateInputs(inputs);
    return inputs;
  }

  private void apply(String fault, Map<String, Double> values) {
    assertThat(components.apply(Setting.of(PART, fault, values))).isNull();
  }

  private void undo(String fault) {
    assertThat(components.undo(PART, fault)).isNull();
  }

  @Test
  void aCoprocessorIsDeclaredWithItsFaults() {
    assertThat(components.components()).extracting(c -> c.name()).contains(PART);
    assertThat(components.on()).isEmpty();
    assertThat(after(0).everAnswered).isTrue();
  }

  @Test
  void unreachableItStopsAnsweringUntilUndone() {
    after(1);

    apply(Faults.UNREACHABLE, Map.of());
    CoprocessorIOInputs silent = after(2);
    undo(Faults.UNREACHABLE);
    CoprocessorIOInputs back = after(1);

    assertThat(silent.answered).isFalse();
    assertThat(silent.error).isEqualTo("connect timed out");
    assertThat(silent.ageSeconds).isGreaterThan(1.5);
    assertThat(back.answered).isTrue();
    assertThat(back.ageSeconds).isZero();
    assertThat(components.on()).isEmpty();
  }

  @Test
  void theWrongVersionIsInItsStampAndWhatItPublishes() {
    apply(Faults.WRONG_VERSION, Map.of());
    CoprocessorIOInputs wrong = after(1);
    PhotonVisionIOInputs published = new PhotonVisionIOInputs();
    world.photonVision().updateInputs(published);
    undo(Faults.WRONG_VERSION);

    assertThat(wrong.photonVisionVersion).isEqualTo(CoprocessorIOSim.OTHER_VERSION);
    assertThat(published.reported).isEqualTo(CoprocessorIOSim.OTHER_VERSION);
    assertThat(after(1).photonVisionVersion).isEqualTo(PhotonVersion.versionString);
  }

  @Test
  void hotItsZonesNearTheirTripAndPastItCapped() {
    apply(Faults.HOT, Map.of(Faults.BELOW_TRIP, 3.0));
    CoprocessorIOInputs warm = after(1);
    apply(Faults.HOT, Map.of(Faults.BELOW_TRIP, -2.0));
    CoprocessorIOInputs throttled = after(1);
    undo(Faults.HOT);
    CoprocessorIOInputs cool = after(1);

    assertThat(warm.zoneCelsius).contains(82.0);
    assertThat(warm.bigCoreLimitMhz).isEqualTo(warm.bigCoreMaxMhz);
    assertThat(throttled.bigCoreLimitMhz).isLessThan(throttled.bigCoreMaxMhz);
    assertThat(cool.bigCoreLimitMhz).isEqualTo(cool.bigCoreMaxMhz);
    assertThat(cool.zoneCelsius[1]).isLessThan(70);
  }

  @Test
  void aCameraUnpluggedIsntPresentUntilPluggedBackIn() {
    apply(Faults.CAMERA_UNPLUGGED, Map.of(Faults.CAMERA, 2.0));
    CoprocessorIOInputs unplugged = after(1);
    undo(Faults.CAMERA_UNPLUGGED);

    assertThat(unplugged.camerasPresent).containsExactly("front-left");
    assertThat(unplugged.usbCameras).hasSize(1);
    assertThat(after(1).camerasPresent).containsExactly("front-left", "front-right");
    assertThat(
            components.apply(Setting.of(PART, Faults.CAMERA_UNPLUGGED, Map.of(Faults.CAMERA, 1.5))))
        .contains("whole number");
  }

  @Test
  void photonVisionRestartingCountsAndComesBack() {
    after(1);

    apply(Faults.PHOTONVISION_RESTARTS, Map.of());
    CoprocessorIOInputs restarting = after(1);
    CoprocessorIOInputs back = after(5);

    assertThat(restarting.photonVisionRestarts).isEqualTo(1);
    assertThat(restarting.photonVisionState).isEqualTo("activating/auto-restart");
    assertThat(back.photonVisionState).isEqualTo("active/running");
    assertThat(back.journalErrors).isEqualTo(1);
  }

  @Test
  void twoBoardsTakeTurnsAnsweringUntilUndone() {
    apply(Faults.DUPLICATED, Map.of());
    after(1);
    after(1);
    CoprocessorIOInputs twice = after(1);
    undo(Faults.DUPLICATED);

    assertThat(twice.macsTakingTurns).hasSize(2);
  }

  @Test
  void settingsChangedDifferFromTheStampedOnes() {
    apply(Faults.SETTINGS_CHANGED, Map.of());
    CoprocessorIOInputs changed = after(1);
    undo(Faults.SETTINGS_CHANGED);

    assertThat(changed.settingsHash).isNotEqualTo("hash-1");
    assertThat(changed.stampedSettingsHash).isEqualTo("hash-1");
    assertThat(after(1).settingsHash).isEqualTo("hash-1");
  }

  /** Asked to power down, it halts a few seconds later, and stays halted: the fault keeps it up. */
  @Test
  void askedToPowerDownItHaltsUnlessItIgnoresIt() {
    after(1);
    apply(Faults.IGNORES_POWER_DOWN, Map.of());
    coprocessor.powerDown();
    CoprocessorIOInputs ignoring = after(5);
    undo(Faults.IGNORES_POWER_DOWN);

    coprocessor.powerDown();
    CoprocessorIOInputs halting = after(1);
    CoprocessorIOInputs halted = after(4);
    apply(Faults.UNREACHABLE, Map.of());
    undo(Faults.UNREACHABLE);
    CoprocessorIOInputs still = after(2);

    assertThat(ignoring.powerDownAnswer).isEqualTo("answered 404");
    assertThat(ignoring.agentPortOpen).isTrue();
    assertThat(ignoring.answered).isTrue();
    assertThat(halting.powerDownAnswer).isEqualTo("accepted");
    assertThat(halting.photonVisionPortOpen).isTrue();
    assertThat(halted.portsTried).isTrue();
    assertThat(halted.agentPortOpen).isFalse();
    assertThat(halted.photonVisionPortOpen).isFalse();
    assertThat(halted.answered).isFalse();
    assertThat(still.answered).as("halted until the simulator starts again").isFalse();
  }
}
