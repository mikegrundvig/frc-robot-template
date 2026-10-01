package frc.robot.subsystems.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import com.michaelgrundvig.frc.coprocessor.table.Board;
import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import com.michaelgrundvig.frc.coprocessor.table.Table;
import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.pitwall.Devices.Card;
import com.michaelgrundvig.frc.pitwall.Devices.Level;
import com.michaelgrundvig.frc.pitwall.Devices.Line;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.zip.ZipInputStream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonVersion;

/**
 * The coprocessors as the robot judges them, each against this build: a simulated agent answering
 * as a healthy board would, then as a troubled one, and the alerts and card that follow.
 */
class CoprocessorsTest {
  private static final long LOOP = 20_000_000L;
  private static final Computer FRONT =
      new Computer(
          "vision-front", 11, Board.ORANGEPI_5, List.of("front-left", "front-right"), 5808);

  private final List<AutoCloseable> built = new ArrayList<>();

  @SuppressWarnings("NullAway.Init") // each test builds them first
  private CoprocessorIOSim agent;

  @SuppressWarnings("NullAway.Init")
  private Coprocessors coprocessors;

  private long now;

  /** The table as this build would compile it, with or without settings committed. */
  private static CompiledTable table(String settingsHash) {
    return new CompiledTable(
        new Table(24680, 5808, List.of(FRONT)),
        PhotonVersion.versionString,
        "recipe-1",
        settingsHash.isEmpty() ? Map.of() : Map.of("vision-front", settingsHash));
  }

  private void build(CompiledTable table) {
    agent = new CoprocessorIOSim(table, FRONT, Coprocessors.Settings.DEFAULTS);
    coprocessors =
        new Coprocessors(
            table,
            new CoprocessorIO[] {agent},
            new PhotonVisionIOSim(List.of(agent)),
            Coprocessors.Settings.DEFAULTS);
    built.add(coprocessors);
  }

  @AfterEach
  void close() throws Exception {
    for (AutoCloseable each : built) {
      each.close();
    }
  }

  /** Runs the robot's loop this long, the agent answering once a second on its clock. */
  private void run(double seconds, boolean enabled) {
    for (long end = now + Math.round(seconds * 1e9); now < end; now += LOOP) {
      agent.step(now);
      coprocessors.periodic(now, enabled);
    }
  }

  private void run(double seconds) {
    run(seconds, false);
  }

  private List<String> alerts() {
    return coprocessors.alerts("vision-front");
  }

  private Card card() {
    return coprocessors.card("vision-front");
  }

  private Line line(String label) {
    return card().lines().stream()
        .filter(line -> line.label().equals(label))
        .findFirst()
        .orElseThrow(() -> new AssertionError("no line " + label + " in " + card()));
  }

  @Test
  void aHealthyCoprocessorRaisesNothing() {
    build(table(""));

    run(5);

    assertThat(alerts()).isEmpty();
    assertThat(coprocessors.isMissing("vision-front")).isFalse();
    assertThat(card().state()).isEqualTo("Present");
    assertThat(line("Version").value()).contains(PhotonVersion.versionString, "as this build's");
    assertThat(line("Stamp").level()).isEqualTo(Level.OK);
    assertThat(line("Cameras").value()).isEqualTo("2 of 2: front-left, front-right");
    assertThat(line("Heat").value()).isEqualTo("bigcore0-thermal about 55 °C, trips at 85 °C");
    assertThat(line("Load").value()).startsWith("big cores about 40% busy");
    assertThat(line("Load").value()).contains("free to run at 2400 MHz");
    assertThat(line("Since boot").value()).contains("restarted never", "last shutdown clean");
    // No settings in Git yet: worth saying, not wrong.
    assertThat(line("Settings").level()).isEqualTo(Level.NOTE);
    assertThat(card().level()).isEqualTo(Level.NOTE);
    assertThat(coprocessors.names()).containsExactly("vision-front");
  }

  @Test
  void aCoprocessorThatNeverAnsweredIsMissingHighOnlyWhileEnabled() {
    build(table(""));
    agent.unreachable(true);

    run(1, false);
    List<String> disabled = alerts();
    run(1, true);

    assertThat(disabled)
        .containsExactly(
            "MEDIUM: Coprocessor vision-front (10.246.80.11) hasn't answered since the robot"
                + " program started: connect timed out");
    assertThat(alerts()).hasSize(1).first().asString().startsWith("HIGH: ");
    assertThat(card().state()).isEqualTo("Missing");
    assertThat(card().level()).isEqualTo(Level.PROBLEM);
    assertThat(card().lines()).hasSize(1);
  }

  @Test
  void aCoprocessorThatStopsAnsweringIsMissingAfterThreeSeconds() {
    build(table(""));
    run(2);

    agent.unreachable(true);
    run(1.9);
    boolean missingSoon = coprocessors.isMissing("vision-front");
    run(1.5);

    assertThat(missingSoon).as("its last answer is under 3 s old").isFalse();
    assertThat(alerts())
        .containsExactly(
            "MEDIUM: Coprocessor vision-front (10.246.80.11) stopped answering: connect timed out");
    assertThat(line("Answer").value()).startsWith("none for 4 s: connect timed out");
    // What it said last stays on its card.
    assertThat(line("Version").value()).contains(PhotonVersion.versionString);

    agent.unreachable(false);
    run(1.5);
    assertThat(alerts()).isEmpty();
  }

  @Test
  void anotherPhotonVisionIsTheWrongImageNamingBoth() {
    build(table(""));
    agent.wrongVersion(true);

    run(2);

    assertThat(alerts())
        .anySatisfy(
            alert ->
                assertThat(alert)
                    .startsWith("HIGH: Coprocessor vision-front runs the wrong image: ")
                    .contains(CoprocessorIOSim.OTHER_VERSION, PhotonVersion.versionString))
        .anySatisfy(
            alert ->
                assertThat(alert)
                    .startsWith(
                        "HIGH: PhotonVision reports "
                            + CoprocessorIOSim.OTHER_VERSION
                            + " (vision-front), and this robot program's PhotonLib is "
                            + PhotonVersion.versionString));
    assertThat(line("Version").level()).isEqualTo(Level.PROBLEM);
    assertThat(line("Stamp").level()).as("the rest of its stamp is right").isEqualTo(Level.OK);
  }

  @Test
  void anotherRecipeIsTheWrongImageToo() {
    // An image built by recipe-1, judged by a build whose recipe is recipe-2.
    agent = new CoprocessorIOSim(table(""), FRONT, Coprocessors.Settings.DEFAULTS);
    coprocessors =
        new Coprocessors(
            new CompiledTable(
                new Table(24680, 5808, List.of(FRONT)),
                PhotonVersion.versionString,
                "recipe-2",
                Map.of()),
            new CoprocessorIO[] {agent},
            new PhotonVisionIOSim(List.of(agent)),
            Coprocessors.Settings.DEFAULTS);
    built.add(coprocessors);

    run(2);

    assertThat(alerts()).singleElement().asString().contains("recipe-1", "recipe-2");
    assertThat(line("Stamp").level()).isEqualTo(Level.PROBLEM);
    assertThat(line("Stamp").value()).contains("recipe-2");
  }

  @Test
  void aCoprocessorNearItsTripPointIsHotAndPastItThrottled() {
    build(table(""));
    agent.heat(2);
    run(2);

    List<String> warm = alerts();
    Line heat = line("Heat");
    agent.heat(-1);
    run(2);

    assertThat(warm)
        .containsExactly(
            "MEDIUM: Coprocessor vision-front is hot: bigcore0-thermal 83 °C, trips at 85 °C."
                + " PhotonVision slows as it throttles: cool it");
    assertThat(heat.level()).isEqualTo(Level.WARNING);
    assertThat(alerts())
        .anySatisfy(alert -> assertThat(alert).contains("big cores held to 1800 of 2400 MHz"));
    assertThat(line("Load").level()).isEqualTo(Level.WARNING);
  }

  @Test
  void aCameraUnpluggedIsMissingAgainstTheTable() {
    build(table(""));
    agent.unplugCamera(1);

    run(2);

    assertThat(alerts())
        .contains(
            "MEDIUM: Coprocessor vision-front is missing camera front-right: is it plugged in,"
                + " into the port its settings expect?");
    assertThat(line("Cameras").value()).isEqualTo("1 of 2, front-right missing");
    assertThat(line("Cameras").level()).isEqualTo(Level.PROBLEM);
    // The kernel logged the disconnect, which the agent counts.
    assertThat(alerts()).anySatisfy(alert -> assertThat(alert).contains("USB disconnect"));
    assertThatIllegalArgumentException().isThrownBy(() -> agent.unplugCamera(2));
  }

  @Test
  void photonVisionRestartingIsSaidSinceBoot() {
    build(table(""));
    run(1);

    agent.restartPhotonVision();
    run(2);

    assertThat(alerts())
        .contains(
            "MEDIUM: PhotonVision on vision-front restarted once since the robot first heard this"
                + " boot: read its journal (Pit, its card's Download logs)")
        .anySatisfy(alert -> assertThat(alert).contains("1 error to read", "status=1/FAILURE"));
    assertThat(line("Since boot").value()).contains("restarted once", "1 error in its journal");
    assertThat(line("Since boot").level()).isEqualTo(Level.WARNING);
  }

  @Test
  void twoBoardsTakingTurnsAtOneAddressAreSaid() {
    build(table(""));
    agent.twoBoards(true);

    run(4);

    assertThat(alerts())
        .singleElement()
        .asString()
        .startsWith("HIGH: Two boards answer at 10.246.80.11 (vision-front), MACs ");
    assertThat(card().state()).isEqualTo("Two answer");
    assertThat(line("Address").level()).isEqualTo(Level.PROBLEM);
  }

  @Test
  void settingsThatDifferFromGitAreAMediumAlert() {
    build(table("hash-1"));
    run(2);
    assertThat(line("Settings").value()).startsWith("as in Git (hash-1)");

    agent.settingsChanged(true);
    run(2);

    assertThat(alerts())
        .containsExactly(
            "MEDIUM: Coprocessor vision-front's PhotonVision settings differ from Git: back them"
                + " up and commit them, or reflash it");
    assertThat(line("Settings").level()).isEqualTo(Level.WARNING);
  }

  @Test
  void aProgramWithoutItsTableSaysSo() {
    Coprocessors none =
        new Coprocessors(
            Coprocessors.NO_TABLE,
            new CoprocessorIO[] {},
            new PhotonVisionIO() {},
            Coprocessors.Settings.DEFAULTS);
    built.add(none);

    none.periodic(0, false);
    Actions actions = new Actions(inputs -> {});
    none.addTo(actions);
    var powerDown = actions.requestStart(Coprocessors.POWER_DOWN);
    actions.periodic(new Actions.Gate(false, false, false));

    assertThat(none.names()).isEmpty();
    assertThat(powerDown.getNow(null).detail()).isEqualTo("the table has no coprocessors");
    assertThatIllegalArgumentException().isThrownBy(() -> none.card("vision-front"));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                new Coprocessors(
                    table(""),
                    new CoprocessorIO[] {},
                    new PhotonVisionIO() {},
                    Coprocessors.Settings.DEFAULTS));
  }

  @Test
  void itsCardOffersPhotonVisionItsBackupAndItsJournal() throws Exception {
    build(table(""));
    agent.restartPhotonVision();
    run(2);
    Devices devices = new Devices(() -> now);

    coprocessors.addTo(devices);

    Devices.Device device = devices.named("vision-front").orElseThrow();
    assertThat(device.links())
        .containsExactly(new Devices.Link("Open PhotonVision", "http://10.246.80.11:5800"));
    Devices.File backup = device.download("settings").orElseThrow().fetch().fetch();
    assertThat(backup.name()).isEqualTo("vision-front-settings.zip");
    try (ZipInputStream zip = new ZipInputStream(backup.body())) {
      assertThat(zip.getNextEntry().getName())
          .isEqualTo("coprocessors/vision-front/settings/README.txt");
    }
    Devices.LogPage errors =
        device.log().orElseThrow().read(new Devices.LogQuery("", 3, "photonvision.service"));
    assertThat(errors.entries())
        .singleElement()
        .satisfies(
            entry -> {
              assertThat(entry.message()).contains("status=1/FAILURE");
              // Logged at the simulator's start, which the robot placed on its own clock.
              assertThat(entry.time()).isEqualTo("robot 0.000 s");
              assertThat(entry.unit()).isEqualTo("photonvision.service");
            });
    Devices.LogPage all = device.log().orElseThrow().read(new Devices.LogQuery("", 7, ""));
    assertThat(all.entries()).hasSize(5);
    assertThat(all.next()).as("nothing more waiting").isEmpty();
  }

  @Test
  void settingsAreChecked() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                CoprocessorsSettingsBuilder.builder(Coprocessors.Settings.DEFAULTS)
                    .answerTimeoutSeconds(0.9)
                    .build())
        .withMessageContaining("fit in pollPeriodSeconds");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                CoprocessorsSettingsBuilder.builder(Coprocessors.Settings.DEFAULTS)
                    .staleAfterSeconds(1)
                    .build())
        .withMessageContaining("longer than pollPeriodSeconds");
  }

  // ---- What the review asked of the alerts ----

  /** The table with a second computer, after this one in the table's order. */
  private static final Computer BACK =
      new Computer("vision-back", 12, Board.ORANGEPI_5, List.of("back-left"), 5808);

  /** A PhotonVision version as the topic hands it over, loop by loop. */
  private static final class FakeVersions implements PhotonVisionIO {
    String reported = "";
    String[] published = {};

    @Override
    public void updateInputs(PhotonVisionIOInputs inputs) {
      inputs.library = PhotonVersion.versionString;
      inputs.reported = reported;
      inputs.published = published;
      published = new String[] {};
    }
  }

  /**
   * One board publishes another version, then another board the right one, which the topic then
   * holds: the wrong one was still heard, and stays said for the rest of the run.
   */
  @Test
  void aWrongVersionPublishedBeforeTheRightOneStaysSaid() {
    agent = new CoprocessorIOSim(table(""), FRONT, Coprocessors.Settings.DEFAULTS);
    FakeVersions versions = new FakeVersions();
    coprocessors =
        new Coprocessors(
            table(""), new CoprocessorIO[] {agent}, versions, Coprocessors.Settings.DEFAULTS);
    built.add(coprocessors);
    versions.published = new String[] {CoprocessorIOSim.OTHER_VERSION, PhotonVersion.versionString};
    versions.reported = PhotonVersion.versionString;

    run(5);

    assertThat(alerts())
        .singleElement()
        .asString()
        .startsWith("HIGH: PhotonVision reports " + CoprocessorIOSim.OTHER_VERSION + ",");
  }

  /** Each simulated board publishes as its PhotonVision starts, in the table's order. */
  @Test
  void theSimulatedBoardsPublishInTheTablesOrder() {
    CompiledTable two =
        new CompiledTable(
            new Table(24680, 5808, List.of(FRONT, BACK)),
            PhotonVersion.versionString,
            "recipe-1",
            Map.of());
    CoprocessorIOSim front = new CoprocessorIOSim(two, FRONT, Coprocessors.Settings.DEFAULTS);
    CoprocessorIOSim back = new CoprocessorIOSim(two, BACK, Coprocessors.Settings.DEFAULTS);
    front.wrongVersion(true);
    front.step(0);
    back.step(0);
    PhotonVisionIOSim versions = new PhotonVisionIOSim(List.of(front, back));
    PhotonVisionIOInputsAutoLogged inputs = new PhotonVisionIOInputsAutoLogged();

    versions.updateInputs(inputs);
    String[] first = inputs.published;
    versions.updateInputs(inputs);

    assertThat(first).containsExactly(CoprocessorIOSim.OTHER_VERSION, PhotonVersion.versionString);
    assertThat(inputs.reported).isEqualTo(PhotonVersion.versionString);
    assertThat(inputs.published).as("published once, as it started").isEmpty();
  }

  /** A version heard from a board is said until that board boots again, and publishes anew. */
  @Test
  void aWrongVersionIsSaidUntilItsBoardReboots() {
    build(table(""));
    agent.wrongVersion(true);
    run(2);
    agent.wrongVersion(false); // reflashed meanwhile, say
    run(2);
    boolean stillSaid =
        alerts().stream().anyMatch(alert -> alert.startsWith("HIGH: PhotonVision reports"));

    agent.reboot();
    run(2);

    assertThat(stillSaid).as("the same boot").isTrue();
    assertThat(alerts()).isEmpty();
  }

  /** The template's example table (team 0) watches nothing on a robot, and says what to set. */
  @Test
  void theTemplatesExampleTableWatchesNothingAndSaysSo() {
    CompiledTable example =
        new CompiledTable(
            new Table(0, 5808, List.of(FRONT)), PhotonVersion.versionString, "recipe-1", Map.of());
    CompiledTable none =
        new CompiledTable(
            new Table(0, 5808, List.of()), PhotonVersion.versionString, "recipe-1", Map.of());
    Coprocessors watching =
        new Coprocessors(
            example,
            new CoprocessorIO[] {},
            new PhotonVisionIO() {},
            Coprocessors.Settings.DEFAULTS);
    built.add(watching);

    watching.periodic(0, true);

    assertThat(Coprocessors.isExample(example)).isTrue();
    assertThat(Coprocessors.isExample(none)).as("none listed, on purpose").isFalse();
    assertThat(watching.names()).isEmpty();
    assertThat(watching.alerts())
        .containsExactly(
            "LOW: The coprocessor table is the template's example (team 0): a robot watches none"
                + " of its coprocessors, and the simulator only simulates them. Set your team"
                + " number and list your coprocessors in coprocessors/coprocessors.yaml, or"
                + " computers: [] for none");
  }

  /**
   * Restarts and errors from before the robot first heard a boot are on the card, not alerted: a
   * board up all day has them. New ones are alerted.
   */
  @Test
  void restartsAndErrorsAreAlertedAsTheyHappenNotFromBefore() {
    build(table(""));
    agent.restartPhotonVision(); // before the robot first hears this boot

    run(5);
    List<String> before = alerts();
    String card = line("Since boot").value();
    agent.restartPhotonVision();
    run(2);

    assertThat(before).isEmpty();
    assertThat(card).contains("restarted once", "1 error in its journal");
    assertThat(alerts())
        .contains(
            "MEDIUM: PhotonVision on vision-front restarted once since the robot first heard this"
                + " boot: read its journal (Pit, its card's Download logs)")
        .anySatisfy(alert -> assertThat(alert).contains("1 error to read"));
    assertThat(line("Since boot").value()).contains("restarted 2 times", "2 errors in its journal");
  }

  /** A USB, UVC, filesystem, memory, or heat error this boot is alerted at once, however old. */
  @Test
  void hardwareErrorsAreAlertedAtOnce() {
    build(table(""));
    agent.unplugCamera(1);
    agent.unplugCamera(-1); // plugged back in before the robot first heard it

    run(2);

    assertThat(alerts())
        .singleElement()
        .asString()
        .startsWith("MEDIUM: Coprocessor vision-front's journal has 1 error to read, the latest:")
        .contains("USB disconnect");
  }

  /** A new boot starts the count again. */
  @Test
  void aNewBootStartsTheCountAgain() {
    build(table(""));
    run(1);
    agent.restartPhotonVision();
    run(2);

    agent.reboot();
    run(2);

    assertThat(alerts()).isEmpty();
  }

  /**
   * Hot is said near the trip point, and unsaid only well clear of it (2 °C more) and after a while
   * (10 s), so a board hovering at the margin doesn't flicker the alert.
   */
  @Test
  void hotClearsOnlyWellClearAndAfterAWhile() {
    build(table(""));
    agent.heat(3);
    run(2);
    boolean hot = isHot();

    agent.heat(6); // past the margin, within 2 °C of it
    run(2);
    boolean warm = isHot();
    agent.heat(10);
    run(5);
    boolean cooling = isHot();
    run(6);

    assertThat(hot).isTrue();
    assertThat(warm).isTrue();
    assertThat(cooling).as("within 10 s of the last warm answer").isTrue();
    assertThat(isHot()).isFalse();
  }

  private boolean isHot() {
    return alerts().stream().anyMatch(alert -> alert.contains(" is hot: "));
  }

  /** One coprocessor's code failing is said, and the others are watched on. */
  @Test
  void aCoprocessorWhoseCodeFailsIsSaidAndTheOthersRunOn() {
    CompiledTable two =
        new CompiledTable(
            new Table(24680, 5808, List.of(FRONT, BACK)),
            PhotonVersion.versionString,
            "recipe-1",
            Map.of());
    agent = new CoprocessorIOSim(two, BACK, Coprocessors.Settings.DEFAULTS);
    CoprocessorIO broken =
        new CoprocessorIO() {
          @Override
          public void updateInputs(CoprocessorIOInputs inputs) {
            throw new IllegalStateException("boom");
          }
        };
    coprocessors =
        new Coprocessors(
            two,
            new CoprocessorIO[] {broken, agent},
            new PhotonVisionIO() {},
            Coprocessors.Settings.DEFAULTS);
    built.add(coprocessors);

    run(2);

    assertThat(coprocessors.isMissing("vision-back")).isFalse();
    assertThat(coprocessors.alerts())
        .contains("HIGH: Watching coprocessor vision-front failed: boom");
  }

  @Test
  void settingsHaveUpperBounds() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                CoprocessorsSettingsBuilder.builder(Coprocessors.Settings.DEFAULTS)
                    .pollPeriodSeconds(120)
                    .build())
        .withMessageContaining("pollPeriodSeconds");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                CoprocessorsSettingsBuilder.builder(Coprocessors.Settings.DEFAULTS)
                    .powerDownButton(99)
                    .build())
        .withMessageContaining("powerDownButton");
  }
}
