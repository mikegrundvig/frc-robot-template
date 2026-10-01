package frc.robot.subsystems.coprocessors;

import static frc.robot.lib.config.Rules.atLeast;
import static frc.robot.lib.config.Rules.positive;

import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import com.michaelgrundvig.frc.coprocessor.table.Table;
import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.pitwall.Notices.Notice;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import org.jspecify.annotations.Nullable;
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
   * @param powerDownTimeoutSeconds a coprocessor still answering this long after being asked to
   *     power down raises an alert naming it
   * @param haltedAfterSeconds this long after the coprocessors powered down, with the robot still
   *     on, the page says they're halted and the robot's power must be cycled to restart them
   * @param powerDownButton the digital input a power-down button is wired to, pressed when it reads
   *     false; -1 for none
   * @param powerDownButtonHoldSeconds how long the button must be held to power down, so a brush
   *     against it doesn't
   */
  @ConfigRecord
  public record Settings(
      double pollPeriodSeconds,
      double connectTimeoutSeconds,
      double answerTimeoutSeconds,
      double staleAfterSeconds,
      double hotMarginCelsius,
      double turnsWindowSeconds,
      double downloadTimeoutSeconds,
      double powerDownTimeoutSeconds,
      double haltedAfterSeconds,
      int powerDownButton,
      double powerDownButtonHoldSeconds) {
    /**
     * Once a second, within 250 ms to connect and 500 ms to answer; missing after 3 s; hot within 5
     * °C of a trip point; powered down within 30 s; no power-down button.
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
            .powerDownTimeoutSeconds(30)
            .haltedAfterSeconds(180)
            .powerDownButton(-1)
            .powerDownButtonHoldSeconds(1)
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
          .field("powerDownTimeoutSeconds", powerDownTimeoutSeconds, positive())
          .field("haltedAfterSeconds", haltedAfterSeconds, positive())
          .field("powerDownButton", powerDownButton, atLeast(-1))
          .field("powerDownButtonHoldSeconds", powerDownButtonHoldSeconds, positive())
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

  /** Pitwall's action that powers every coprocessor down, before the robot is switched off. */
  public static final String POWER_DOWN = "Power down coprocessors";

  private final List<Coprocessor> coprocessors;
  private final Settings settings;
  private final Alert noTable;
  private final PhotonVisionIO photonVision;
  private final PhotonVisionIOInputsAutoLogged versions = new PhotonVisionIOInputsAutoLogged();
  private final Alert photonVisionVersion;
  private final PowerButtonIO button;
  private final PowerButtonIOInputsAutoLogged buttonInputs = new PowerButtonIOInputsAutoLogged();
  private final Alert stillAnswering;

  // Powering down, on the robot's clock: when it was asked (-1 for never, or after it was
  // canceled), and since when every coprocessor has been gone (-1 while one isn't).
  private long nowNanos;
  private long powerDownAskedNanos = -1;
  private long goneSinceNanos = -1;

  /** Since when the button has been held: -1 while it isn't; Long.MAX_VALUE once it has acted. */
  private long buttonHeldNanos = -1;

  /** Asks Pitwall to start the power-down action, as the page's Run does: for the button. */
  private Runnable askPitwall = () -> {};

  /**
   * Coprocessors without a power-down button.
   *
   * @param table the table of coprocessors compiled into the program, with what this build expects
   *     of each
   * @param ios each coprocessor's door to its health agent, in the table's order
   * @param photonVision which PhotonVision the coprocessors say they run
   * @param settings how they're asked, and when they're alerted on
   */
  public Coprocessors(
      CompiledTable table, CoprocessorIO[] ios, PhotonVisionIO photonVision, Settings settings) {
    this(table, ios, photonVision, new PowerButtonIO() {}, settings);
  }

  /**
   * @param table the table of coprocessors compiled into the program, with what this build expects
   *     of each
   * @param ios each coprocessor's door to its health agent, in the table's order
   * @param photonVision which PhotonVision the coprocessors say they run
   * @param button a button on the robot that powers them down, or none
   * @param settings how they're asked, and when they're alerted on
   */
  public Coprocessors(
      CompiledTable table,
      CoprocessorIO[] ios,
      PhotonVisionIO photonVision,
      PowerButtonIO button,
      Settings settings) {
    List<Computer> computers = table.table().computers();
    if (computers.size() != ios.length) {
      throw new IllegalArgumentException("one IO for each coprocessor");
    }
    List<Coprocessor> each = new ArrayList<>();
    for (int i = 0; i < ios.length; i++) {
      each.add(new Coprocessor(table, computers.get(i), ios[i], settings));
    }
    coprocessors = List.copyOf(each);
    this.settings = settings;
    this.photonVision = photonVision;
    this.button = button;
    stillAnswering = new Alert("Coprocessors", "stillAnswering", "", Alert.Level.HIGH);
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
   * @param nowNanos the robot's time
   * @param enabled whether the robot is enabled: a coprocessor missing then is a high alert
   */
  public void periodic(long nowNanos, boolean enabled) {
    this.nowNanos = nowNanos;
    for (Coprocessor coprocessor : coprocessors) {
      coprocessor.periodic(enabled);
    }
    photonVision.updateInputs(versions);
    Logger.processInputs("Coprocessors/PhotonVision", versions);
    judgeVersions();
    judgePowerDown();
    button.updateInputs(buttonInputs);
    Logger.processInputs("Coprocessors/PowerButton", buttonInputs);
    readButton();
  }

  // ---- Powering down ----

  /**
   * Asks every coprocessor to power down, each on its IO's own thread, so they can shut down
   * cleanly before someone switches the robot off. Pitwall's action does this, while the robot is
   * disabled and off the field.
   */
  void powerDown() {
    for (Coprocessor coprocessor : coprocessors) {
      coprocessor.powerDown();
    }
    powerDownAskedNanos = nowNanos;
    goneSinceNanos = -1;
  }

  /** Whether every coprocessor asked to power down is gone, on both its ports. */
  private boolean allGone() {
    return coprocessors.stream().allMatch(Coprocessor::isGone);
  }

  /**
   * Since when every coprocessor has been gone; the alert naming any still answering past the
   * timeout.
   */
  private void judgePowerDown() {
    boolean asked = powerDownAskedNanos >= 0;
    if (asked && allGone()) {
      if (goneSinceNanos < 0) {
        goneSinceNanos = nowNanos;
      }
    } else {
      goneSinceNanos = -1;
    }
    List<String> answering =
        coprocessors.stream()
            .filter(c -> !c.isGone())
            .map(
                c ->
                    c.name()
                        + (c.powerDownAnswer().isEmpty() ? "" : " (" + c.powerDownAnswer() + ")"))
            .toList();
    boolean late =
        asked
            && !answering.isEmpty()
            && nowNanos - powerDownAskedNanos
                > Math.round(settings.powerDownTimeoutSeconds() * 1e9);
    if (late) {
      String text =
          String.format(
              Locale.ROOT,
              "Still answering %.0f s after being asked to power down: %s. Don't switch the robot"
                  + " off yet: power it down from its own page, or unplug it once it has been"
                  + " checked",
              settings.powerDownTimeoutSeconds(),
              String.join(", ", answering));
      if (!text.equals(stillAnswering.getText())) {
        stillAnswering.setText(text);
      }
    }
    stillAnswering.set(late);
  }

  /** The button, held long enough, asks Pitwall to power down, as the page's Run would. */
  private void readButton() {
    if (!buttonInputs.pressed) {
      buttonHeldNanos = -1;
      return;
    }
    if (buttonHeldNanos < 0) {
      buttonHeldNanos = nowNanos;
    }
    if (buttonHeldNanos != Long.MAX_VALUE
        && nowNanos - buttonHeldNanos >= Math.round(settings.powerDownButtonHoldSeconds() * 1e9)) {
      buttonHeldNanos = Long.MAX_VALUE; // once per press
      askPitwall.run();
    }
  }

  /** Whether powering down is under way: asked, and a coprocessor isn't gone yet. */
  boolean isPoweringDown() {
    return powerDownAskedNanos >= 0 && goneSinceNanos < 0;
  }

  /**
   * What the page says above everything else: the coprocessors powering down, then "Safe to switch
   * off" once every one is gone, then, if the robot stays on, that they're halted.
   */
  public @Nullable Notice notice() {
    if (powerDownAskedNanos < 0) {
      return null;
    }
    if (goneSinceNanos >= 0) {
      boolean halted = nowNanos - goneSinceNanos >= Math.round(settings.haltedAfterSeconds() * 1e9);
      return halted
          ? new Notice(
              "Coprocessors halted: cycle the robot's power to restart them", Devices.Level.WARNING)
          : new Notice("Safe to switch off", Devices.Level.OK);
    }
    List<String> waiting =
        coprocessors.stream().filter(c -> !c.isGone()).map(Coprocessor::name).toList();
    return new Notice(
        "Powering down coprocessors: waiting on " + String.join(", ", waiting),
        stillAnswering.get() ? Devices.Level.PROBLEM : Devices.Level.NOTE);
  }

  /** Pitwall's action: asks them all, and runs until every one is gone. */
  Actions.Routine powerDownAction() {
    return new Actions.Routine() {
      @Override
      public @Nullable String start() {
        if (coprocessors.isEmpty()) {
          return "the table has no coprocessors";
        }
        powerDown();
        return null;
      }

      @Override
      public boolean isRunning() {
        return isPoweringDown();
      }

      @Override
      public void cancel() {
        // What was asked was asked: this stops waiting, and the page saying so.
        powerDownAskedNanos = -1;
        goneSinceNanos = -1;
        stillAnswering.set(false);
      }
    };
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
   * Puts the coprocessors on Pitwall's page: a card for each ({@link #addTo(Devices)}), the action
   * that powers them down (only while the robot is disabled and off the field), and what powering
   * down says above everything else. A power-down button asks Pitwall, as the page's Run does.
   */
  public void addTo(Pitwall pitwall) {
    addTo(pitwall.devices());
    addTo(pitwall.actions());
    pitwall.notices().add("Coprocessors", this::notice);
  }

  /**
   * Adds the action that powers every coprocessor down, which runs only while the robot is disabled
   * and off the field; the button asks for it too.
   */
  void addTo(Actions actions) {
    actions.add(POWER_DOWN, powerDownAction(), Actions.When.DISABLED);
    askPitwall = () -> actions.requestStart(POWER_DOWN);
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
    for (Alert alert : List.of(photonVisionVersion, stillAnswering)) {
      if (alert.get()) {
        up.add(Coprocessor.shown(alert));
      }
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
    stillAnswering.close();
    button.close();
  }
}
