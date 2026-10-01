package frc.robot.subsystems.coprocessors;

import static frc.robot.lib.config.Rules.atMost;
import static frc.robot.lib.config.Rules.between;
import static frc.robot.lib.config.Rules.positive;

import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import com.michaelgrundvig.frc.coprocessor.table.Table;
import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.pitwall.Notices;
import com.michaelgrundvig.frc.pitwall.Notices.Notice;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
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
   * @param hotClearMarginCelsius a hot coprocessor stays hot until its hottest zone is this much
   *     farther from its trip point than the margin
   * @param hotHoldSeconds and until this long after its last answer that close
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
      double hotClearMarginCelsius,
      double hotHoldSeconds,
      double turnsWindowSeconds,
      double downloadTimeoutSeconds,
      double powerDownTimeoutSeconds,
      double haltedAfterSeconds,
      int powerDownButton,
      double powerDownButtonHoldSeconds) {
    /**
     * Once a second, within 250 ms to connect and 500 ms to answer; missing after 3 s; hot within 5
     * °C of a trip point, until 7 °C clear of it for 10 s; powered down within 30 s; no power-down
     * button.
     */
    public static final Settings DEFAULTS =
        builder()
            .pollPeriodSeconds(1)
            .connectTimeoutSeconds(0.25)
            .answerTimeoutSeconds(0.5)
            .staleAfterSeconds(3)
            .hotMarginCelsius(5)
            .hotClearMarginCelsius(2)
            .hotHoldSeconds(10)
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
          .field("pollPeriodSeconds", pollPeriodSeconds, atMost(60))
          .field("connectTimeoutSeconds", connectTimeoutSeconds, positive())
          .field("connectTimeoutSeconds", connectTimeoutSeconds, atMost(10))
          .field("answerTimeoutSeconds", answerTimeoutSeconds, positive())
          .field("answerTimeoutSeconds", answerTimeoutSeconds, atMost(10))
          .field("staleAfterSeconds", staleAfterSeconds, positive())
          .field("staleAfterSeconds", staleAfterSeconds, atMost(600))
          .field("hotMarginCelsius", hotMarginCelsius, positive())
          .field("hotMarginCelsius", hotMarginCelsius, atMost(50))
          .field("hotClearMarginCelsius", hotClearMarginCelsius, between(0, 20))
          .field("hotHoldSeconds", hotHoldSeconds, between(0, 600))
          .field("turnsWindowSeconds", turnsWindowSeconds, positive())
          .field("turnsWindowSeconds", turnsWindowSeconds, atMost(3600))
          .field("downloadTimeoutSeconds", downloadTimeoutSeconds, positive())
          .field("downloadTimeoutSeconds", downloadTimeoutSeconds, atMost(300))
          .field("powerDownTimeoutSeconds", powerDownTimeoutSeconds, positive())
          .field("powerDownTimeoutSeconds", powerDownTimeoutSeconds, atMost(600))
          .field("haltedAfterSeconds", haltedAfterSeconds, positive())
          .field("haltedAfterSeconds", haltedAfterSeconds, atMost(3600))
          .field("powerDownButton", powerDownButton, between(-1, MAX_DIGITAL_INPUT))
          .field("powerDownButtonHoldSeconds", powerDownButtonHoldSeconds, positive())
          .field("powerDownButtonHoldSeconds", powerDownButtonHoldSeconds, atMost(10))
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

    /** The highest digital input a button may be wired to: a generous bound, not Systemcore's. */
    static final int MAX_DIGITAL_INPUT = 31;

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
  private final Alert example;
  private final PhotonVisionIO photonVision;
  private final PhotonVisionIOInputsAutoLogged versions = new PhotonVisionIOInputsAutoLogged();
  private final Alert photonVisionVersion;
  private final PowerButtonIO button;
  private final PowerButtonIOInputsAutoLogged buttonInputs = new PowerButtonIOInputsAutoLogged();
  private final Alert buttonWiring;
  private final Alert notPoweredDown;

  /**
   * The versions published that aren't this program's PhotonLib's, latched for the run, each with
   * the coprocessors whose stamp carried it then and their boot: it's unsaid once every one of them
   * has booted again (and so published anew).
   */
  private final Map<String, Map<String, String>> wrongVersions = new LinkedHashMap<>();

  // Powering down, on the robot's clock: when it was asked (-1 for never, or after it was
  // canceled), and since when every coprocessor has been gone (-1 while one isn't).
  private long nowNanos;
  private long powerDownAskedNanos = -1;
  private long goneSinceNanos = -1;

  // The button: armed only once it has read released, so a stuck, shorted, or floating input
  // never powers anything down; since when it has been held (-1 while it isn't; Long.MAX_VALUE
  // once it has acted, until let go).
  private boolean buttonArmed;
  private boolean buttonRead;
  private long buttonHeldNanos = -1;

  /** Asks Pitwall to start the power-down action, as the page's Run does: for the button. */
  private Runnable askPitwall = () -> {};

  /**
   * Coprocessors without a power-down button.
   *
   * @param table the table of coprocessors compiled into the program, with what this build expects
   *     of each
   * @param ios each coprocessor's door to its health agent, in the table's order; none for the
   *     template's example table, which watches nothing ({@link #isExample})
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
   * @param ios each coprocessor's door to its health agent, in the table's order; none for the
   *     template's example table, which watches nothing ({@link #isExample})
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
    boolean watchNothing = ios.length == 0 && isExample(table);
    if (computers.size() != ios.length && !watchNothing) {
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
    notPoweredDown = new Alert("Coprocessors", "notPoweredDown", "", Alert.Level.HIGH);
    photonVisionVersion = new Alert("Coprocessors", "photonVisionVersion", "", Alert.Level.HIGH);
    buttonWiring =
        new Alert(
            "Coprocessors",
            "buttonWiring",
            "The coprocessors' power-down button has read pressed since the robot program started:"
                + " check its wiring. It works once it has read released",
            Alert.Level.MEDIUM);
    noTable =
        new Alert(
            "Coprocessors",
            "noTable",
            "This robot program has no coprocessor table, so no coprocessor is watched: build it"
                + " with Gradle, which compiles coprocessors/coprocessors.yaml in",
            Alert.Level.HIGH);
    noTable.set(table.photonvisionVersion().isEmpty());
    example =
        new Alert(
            "Coprocessors",
            "example",
            "The coprocessor table is the template's example (team 0): a robot watches none of"
                + " its coprocessors, and the simulator only simulates them. Set your team number"
                + " and list your coprocessors in coprocessors/coprocessors.yaml, or computers: []"
                + " for none",
            Alert.Level.LOW);
    example.set(isExample(table));
  }

  /**
   * Whether a table is the template's example: team 0, with computers listed. A robot watches none
   * of them (they'd all read missing), and an alert says what to set; a table with no computers
   * says none on purpose.
   */
  public static boolean isExample(CompiledTable table) {
    return table.table().team() == 0 && !table.table().computers().isEmpty();
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
   * it never waits on the network. One coprocessor's code failing is said, and the rest are watched
   * on.
   *
   * @param nowNanos the robot's time
   * @param enabled whether the robot is enabled: a coprocessor missing then is a high alert
   */
  public void periodic(long nowNanos, boolean enabled) {
    this.nowNanos = nowNanos;
    for (Coprocessor coprocessor : coprocessors) {
      try {
        coprocessor.periodic(enabled);
      } catch (RuntimeException e) {
        coprocessor.failed(e);
      }
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

  /** Since when every one has been gone; the alert naming any not gone past the timeout. */
  private void judgePowerDown() {
    if (powerDownAskedNanos < 0) {
      return;
    }
    if (coprocessors.stream().allMatch(Coprocessor::isGone)) {
      if (goneSinceNanos < 0) {
        goneSinceNanos = nowNanos;
      }
    } else {
      goneSinceNanos = -1;
    }
    boolean late =
        goneSinceNanos < 0
            && nowNanos - powerDownAskedNanos
                > Math.round(settings.powerDownTimeoutSeconds() * 1e9);
    if (late) {
      List<String> named = new ArrayList<>();
      for (Coprocessor coprocessor : coprocessors) {
        String trouble = coprocessor.powerDownTrouble();
        if (!coprocessor.isGone()) {
          named.add(coprocessor.name() + (trouble.isEmpty() ? "" : " (" + trouble + ")"));
        }
      }
      Coprocessor.text(
          notPoweredDown,
          String.format(
              Locale.ROOT,
              "Not powered down %.0f s after being asked: %s. Don't switch the robot off yet:"
                  + " check each first",
              settings.powerDownTimeoutSeconds(),
              String.join(", ", named)));
    }
    notPoweredDown.set(late);
  }

  /**
   * The button, held long enough, asks Pitwall to power down, as the page's Run would; Pitwall's
   * gate refuses it while the robot is enabled, and once the field system has connected in this
   * run. Only once it has read released: one reading pressed from the start is wiring (stuck,
   * shorted, or floating), and is said.
   */
  private void readButton() {
    boolean pressed = buttonInputs.pressed;
    if (!buttonRead) {
      buttonRead = true;
      buttonWiring.set(pressed);
    }
    if (!pressed) {
      buttonArmed = true;
      buttonWiring.set(false);
      buttonHeldNanos = -1;
      return;
    }
    if (!buttonArmed) {
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
   * What the page says above everything else: the coprocessors powering down; "Safe to switch off"
   * once every one has taken the request and gone; then, if the robot stays on, that they're
   * halted. One that couldn't be reached or refused is said at once, and one still answering past
   * the timeout: don't switch off.
   */
  public @Nullable Notice notice() {
    if (powerDownAskedNanos < 0) {
      return null;
    }
    if (goneSinceNanos >= 0) {
      boolean halted = nowNanos - goneSinceNanos >= Math.round(settings.haltedAfterSeconds() * 1e9);
      return halted
          ? new Notice(
              "Coprocessors halted: cycle the robot's power to restart them", Notices.Level.WARNING)
          : new Notice("Safe to switch off", Notices.Level.OK);
    }
    List<String> trouble = new ArrayList<>();
    List<String> waiting = new ArrayList<>();
    for (Coprocessor coprocessor : coprocessors) {
      if (coprocessor.isGone()) {
        continue;
      }
      waiting.add(coprocessor.name());
      String why = coprocessor.powerDownTrouble();
      if (why.startsWith("couldn't be reached")) {
        trouble.add(coprocessor.name() + " couldn't be reached");
      } else if (why.startsWith("refused")) {
        trouble.add(coprocessor.name() + " refused to power down");
      } else if (notPoweredDown.get()) {
        trouble.add(coprocessor.name() + " " + (why.isEmpty() ? "still answering" : why));
      }
    }
    if (!trouble.isEmpty()) {
      return new Notice(
          "Don't switch off: "
              + String.join(", ", trouble)
              + ": check "
              + (trouble.size() == 1 ? "it" : "each")
              + " before switching off",
          Notices.Level.PROBLEM);
    }
    return new Notice(
        "Powering down coprocessors: waiting on " + String.join(", ", waiting), Notices.Level.NOTE);
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
        // What was asked was asked: a board that took the request still powers down. This stops
        // asking the rest, watching them, and the page saying so.
        for (Coprocessor coprocessor : coprocessors) {
          coprocessor.stopPowerDown();
        }
        powerDownAskedNanos = -1;
        goneSinceNanos = -1;
        notPoweredDown.set(false);
      }
    };
  }

  /**
   * PhotonVision's version as it publishes it, against this program's PhotonLib, as exact strings.
   * Every coprocessor publishes to one topic, which keeps only the last, so each wrong version
   * heard is said for the rest of the run, naming the coprocessors whose stamp carried it, until
   * each of those has booted again.
   */
  private void judgeVersions() {
    // Unsaid once every coprocessor named has booted again; one that booted still wrong publishes
    // it again, and is named again below.
    wrongVersions
        .values()
        .removeIf(
            whose ->
                !whose.isEmpty()
                    && whose.entrySet().stream()
                        .allMatch(
                            named ->
                                coprocessors.stream()
                                    .anyMatch(
                                        c ->
                                            c.name().equals(named.getKey())
                                                && c.inputs.everAnswered
                                                && !c.inputs.bootId.equals(named.getValue()))));
    List<String> heard = new ArrayList<>(List.of(versions.published));
    heard.add(versions.reported);
    for (String version : heard) {
      if (!version.isEmpty() && !version.equals(versions.library)) {
        Map<String, String> whose =
            wrongVersions.computeIfAbsent(version, v -> new LinkedHashMap<>());
        for (Coprocessor coprocessor : coprocessors) {
          if (coprocessor.inputs.everAnswered
              && coprocessor.inputs.photonVisionVersion.equals(version)) {
            whose.putIfAbsent(coprocessor.name(), coprocessor.inputs.bootId);
          }
        }
      }
    }
    if (!wrongVersions.isEmpty()) {
      List<String> said = new ArrayList<>();
      wrongVersions.forEach(
          (version, whose) ->
              said.add(
                  version
                      + (whose.isEmpty() ? "" : " (" + String.join(", ", whose.keySet()) + ")")));
      Coprocessor.text(
          photonVisionVersion,
          "PhotonVision reports "
              + String.join(" and ", said)
              + ", and this robot program's PhotonLib is "
              + versions.library
              + ": update the coprocessor's image, or the vendordep");
    }
    photonVisionVersion.set(!wrongVersions.isEmpty());
  }

  /** Each coprocessor, by name, in the table's order. */
  public List<String> names() {
    return coprocessors.stream().map(Coprocessor::name).toList();
  }

  /** The robot's time at a coprocessor's boot, as best known: for tests. */
  long bootedAtRobotNanos(String name) {
    return coprocessor(name).bootedAtRobotNanos();
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
   * that powers them down (only while the robot is disabled, and until the field system first
   * connects), and what powering down says above everything else. A power-down button asks Pitwall,
   * as the page's Run does.
   */
  public void addTo(Pitwall pitwall) {
    addTo(pitwall.devices());
    addTo(pitwall.actions());
    pitwall.notices().add("Coprocessors", this::notice);
  }

  /**
   * Adds the action that powers every coprocessor down, which runs only while the robot is disabled
   * and until the field system first connects; the button asks for it too.
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

  /** The alerts for them all up now, each as its level and text: the table's, and each one's. */
  List<String> alerts() {
    List<String> up = new ArrayList<>();
    for (Coprocessor coprocessor : coprocessors) {
      up.addAll(coprocessor.alerts());
    }
    up.addAll(tableAlerts());
    return up;
  }

  /** One coprocessor's alerts up now, and those for them all. */
  List<String> alerts(String name) {
    List<String> up = new ArrayList<>(coprocessor(name).alerts());
    up.addAll(tableAlerts());
    return up;
  }

  private List<String> tableAlerts() {
    List<String> up = new ArrayList<>();
    for (Alert alert :
        List.of(noTable, example, photonVisionVersion, notPoweredDown, buttonWiring)) {
      if (alert.get()) {
        up.add(Coprocessor.shown(alert));
      }
    }
    return up;
  }

  /** The button's wiring alert, if it's up. */
  String buttonAlert() {
    return buttonWiring.get() ? Coprocessor.shown(buttonWiring) : "";
  }

  @Override
  public void close() {
    for (Coprocessor coprocessor : coprocessors) {
      coprocessor.close();
    }
    photonVision.close();
    for (Alert alert :
        List.of(noTable, example, photonVisionVersion, notPoweredDown, buttonWiring)) {
      alert.close();
    }
    button.close();
  }
}
