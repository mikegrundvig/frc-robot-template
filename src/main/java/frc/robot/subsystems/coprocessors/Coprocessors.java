package frc.robot.subsystems.coprocessors;

import static frc.robot.lib.config.Rules.positive;

import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import com.michaelgrundvig.frc.coprocessor.table.Table;
import com.michaelgrundvig.frc.pitwall.Devices;
import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;

/**
 * The robot's vision coprocessors, as their health agents report them: each one polled on a thread
 * of its IO's own, once a second, and judged against what this build expects of it (the table of
 * coprocessors compiled into the program). The loop reads only each one's latest answer and its
 * age, so a coprocessor that doesn't answer never holds the robot up.
 *
 * <p>Alerts, per coprocessor: missing (no answer for longer than {@link
 * Settings#staleAfterSeconds}: high while enabled, medium while disabled); the wrong image (another
 * PhotonVision, recipe, or name than this build's: high, naming both); two boards answering at its
 * address (high); a camera the table gives it missing; hot (its hottest zone within {@link
 * Settings#hotMarginCelsius} of its trip point, or its big cores throttled); PhotonVision restarted
 * since boot; errors in its journal; and its settings differing from Git (each medium). And for
 * them all: the PhotonVision version they publish against this program's PhotonLib, as exact
 * strings (high), which catches what PhotonLib's own check lets through.
 *
 * <p>Each coprocessor's inputs are logged under {@code Coprocessors/<name>}, PhotonVision's version
 * under {@code Coprocessors/PhotonVision}, so health sits in the match log and replays with it.
 * Each also gets a card on Pitwall's page ({@link #addTo}).
 */
public final class Coprocessors implements AutoCloseable {
  /**
   * How the robot asks its coprocessors, and when it raises alerts about them.
   *
   * @param pollPeriodSeconds how often each coprocessor is asked how it is, each on its own thread
   * @param connectTimeoutSeconds how long a poll waits for a connection to the agent
   * @param answerTimeoutSeconds how long a poll waits for the whole answer, once connected
   * @param staleAfterSeconds an answer older than this, or none at all, is a coprocessor missing
   * @param hotMarginCelsius a coprocessor whose hottest zone is this close to its trip point is hot
   * @param turnsWindowSeconds how far back MACs taking turns at one address count as two boards
   * @param downloadTimeoutSeconds how long a download from the agent (its settings backup, a page
   *     of its journal) may take, once connected
   */
  @ConfigRecord
  public record Settings(
      double pollPeriodSeconds,
      double connectTimeoutSeconds,
      double answerTimeoutSeconds,
      double staleAfterSeconds,
      double hotMarginCelsius,
      double turnsWindowSeconds,
      double downloadTimeoutSeconds) {
    /**
     * Once a second, within 250 ms to connect and 500 ms to answer; missing after 3 s; hot within 5
     * °C of a trip point.
     */
    public static final Settings DEFAULTS =
        builder()
            .pollPeriodSeconds(1)
            .connectTimeoutSeconds(0.25)
            .answerTimeoutSeconds(0.5)
            .staleAfterSeconds(3)
            .hotMarginCelsius(5)
            .turnsWindowSeconds(60)
            .downloadTimeoutSeconds(10)
            .build();

    public Settings {
      Checks.on(Settings.class)
          .field("pollPeriodSeconds", pollPeriodSeconds, positive())
          .field("connectTimeoutSeconds", connectTimeoutSeconds, positive())
          .field("answerTimeoutSeconds", answerTimeoutSeconds, positive())
          .field("staleAfterSeconds", staleAfterSeconds, positive())
          .field("hotMarginCelsius", hotMarginCelsius, positive())
          .field("turnsWindowSeconds", turnsWindowSeconds, positive())
          .field("downloadTimeoutSeconds", downloadTimeoutSeconds, positive())
          .check(
              "connectTimeoutSeconds",
              () ->
                  connectTimeoutSeconds + answerTimeoutSeconds <= pollPeriodSeconds
                      ? null
                      : "with answerTimeoutSeconds must fit in pollPeriodSeconds, so a poll that"
                          + " times out never delays the next")
          .check(
              "staleAfterSeconds",
              () ->
                  staleAfterSeconds > pollPeriodSeconds
                      ? null
                      : "must be longer than pollPeriodSeconds, or every coprocessor reads missing"
                          + " between its answers")
          .orThrow();
    }

    /** Starts building the settings, one named value at a time. */
    public static CoprocessorsSettingsBuilder.PollPeriodSecondsStage builder() {
      return CoprocessorsSettingsBuilder.builder();
    }
  }

  /** PhotonVision's own page on a coprocessor, which the card links to by address. */
  public static final int PHOTONVISION_PORT = 5800;

  private final List<Coprocessor> coprocessors;
  private final Alert noTable;
  private final PhotonVisionIO photonVision;
  private final PhotonVisionIOInputsAutoLogged versions = new PhotonVisionIOInputsAutoLogged();
  private final Alert photonVisionVersion;

  /**
   * @param table the table of coprocessors compiled into the program, with what this build expects
   *     of each
   * @param ios each coprocessor's door to its health agent, in the table's order
   * @param photonVision which PhotonVision the coprocessors say they run
   * @param settings how they're asked, and when they're alerted on
   */
  public Coprocessors(
      CompiledTable table, CoprocessorIO[] ios, PhotonVisionIO photonVision, Settings settings) {
    List<Computer> computers = table.table().computers();
    if (computers.size() != ios.length) {
      throw new IllegalArgumentException("one IO for each coprocessor");
    }
    List<Coprocessor> each = new ArrayList<>();
    for (int i = 0; i < ios.length; i++) {
      each.add(new Coprocessor(table, computers.get(i), ios[i], settings));
    }
    coprocessors = List.copyOf(each);
    this.photonVision = photonVision;
    photonVisionVersion = new Alert("Coprocessors", "photonVisionVersion", "", Alert.Level.HIGH);
    noTable =
        new Alert(
            "Coprocessors",
            "noTable",
            "This robot program has no coprocessor table, so no coprocessor is watched: build it"
                + " with Gradle, which compiles coprocessors/coprocessors.yaml in",
            Alert.Level.HIGH);
    noTable.set(table.photonvisionVersion().isEmpty());
  }

  /**
   * The table of coprocessors compiled into this program; a table of none, which raises an alert,
   * when the build left it out.
   */
  public static CompiledTable compiledTable() {
    try {
      return CompiledTable.load();
    } catch (IllegalStateException missing) {
      System.out.println("Coprocessors: " + missing.getMessage());
      return NO_TABLE;
    }
  }

  /** No table: no coprocessors, and nothing this build expects of them. */
  static final CompiledTable NO_TABLE =
      new CompiledTable(new Table(0, Table.DEFAULT_AGENT_PORT, List.of()), "", "", Map.of());

  /**
   * Reads every coprocessor's latest answer, logs it, and raises or clears the alerts. Once a loop;
   * it never waits on the network.
   *
   * @param enabled whether the robot is enabled: a coprocessor missing then is a high alert
   */
  public void periodic(boolean enabled) {
    for (Coprocessor coprocessor : coprocessors) {
      coprocessor.periodic(enabled);
    }
    photonVision.updateInputs(versions);
    Logger.processInputs("Coprocessors/PhotonVision", versions);
    judgeVersions();
  }

  /**
   * PhotonVision's version as it publishes it, against this program's PhotonLib, as exact strings:
   * naming the coprocessors whose stamp says the version published, since the topic doesn't say
   * which one published it.
   */
  private void judgeVersions() {
    Set<String> differing = new LinkedHashSet<>();
    if (!versions.reported.isEmpty() && !versions.reported.equals(versions.library)) {
      differing.add(versions.reported);
    }
    for (String published : versions.published) {
      if (!published.isEmpty() && !published.equals(versions.library)) {
        differing.add(published);
      }
    }
    if (!differing.isEmpty()) {
      List<String> said = new ArrayList<>();
      for (String version : differing) {
        List<String> whose =
            coprocessors.stream()
                .filter(c -> c.inputs.everAnswered && c.inputs.photonVisionVersion.equals(version))
                .map(Coprocessor::name)
                .toList();
        said.add(version + (whose.isEmpty() ? "" : " (" + String.join(", ", whose) + ")"));
      }
      String text =
          "PhotonVision reports "
              + String.join(" and ", said)
              + ", and this robot program's PhotonLib is "
              + versions.library
              + ": update the coprocessor's image, or the vendordep";
      if (!text.equals(photonVisionVersion.getText())) {
        photonVisionVersion.setText(text);
      }
    }
    photonVisionVersion.set(!differing.isEmpty());
  }

  /** Each coprocessor, by name, in the table's order. */
  public List<String> names() {
    return coprocessors.stream().map(Coprocessor::name).toList();
  }

  /** Whether a coprocessor is missing: no answer lately, or none at all. */
  public boolean isMissing(String name) {
    return coprocessor(name).isMissing();
  }

  /** A coprocessor's card, as Pitwall's page shows it, as of the last loop. */
  public Devices.Card card(String name) {
    return coprocessor(name).card();
  }

  /**
   * Puts a card for each coprocessor on Pitwall's page, each ending in what can be done about it:
   * open PhotonVision's own page, by address; download its settings backup; read and download its
   * journal. Pitwall asks the agent for those on its own threads, when the page asks; the page
   * never talks to a coprocessor.
   */
  public void addTo(Devices devices) {
    for (Coprocessor coprocessor : coprocessors) {
      String name = coprocessor.name();
      devices.add(
          Devices.Device.named(name, "Coprocessor", coprocessor::card)
              .link(
                  "Open PhotonVision", "http://" + coprocessor.address() + ":" + PHOTONVISION_PORT)
              .download("settings", "Download settings backup", coprocessor::settingsBackup)
              .log(coprocessor::journal));
    }
  }

  private Coprocessor coprocessor(String name) {
    return coprocessors.stream()
        .filter(c -> c.name().equals(name))
        .findFirst()
        .orElseThrow(() -> new IllegalArgumentException("no coprocessor is named " + name));
  }

  /**
   * The alerts up now, each as its level and text ({@code "HIGH: Coprocessor ..."}): one
   * coprocessor's, and the PhotonVision version's for them all.
   */
  List<String> alerts(String name) {
    List<String> up = new ArrayList<>(coprocessor(name).alerts());
    if (photonVisionVersion.get()) {
      up.add(Coprocessor.shown(photonVisionVersion));
    }
    return up;
  }

  @Override
  public void close() {
    for (Coprocessor coprocessor : coprocessors) {
      coprocessor.close();
    }
    photonVision.close();
    photonVisionVersion.close();
    noTable.close();
  }
}
