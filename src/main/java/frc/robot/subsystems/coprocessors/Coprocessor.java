package frc.robot.subsystems.coprocessors;

import com.michaelgrundvig.frc.coprocessor.api.JournalEntry;
import com.michaelgrundvig.frc.coprocessor.api.JournalPage;
import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.pitwall.Devices.Card;
import com.michaelgrundvig.frc.pitwall.Devices.Level;
import com.michaelgrundvig.frc.pitwall.Devices.Line;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;

/**
 * One coprocessor, as {@link Coprocessors} watches it: its IO and what it read this loop, what this
 * build expects of it, its alerts, and its card on the Pitwall page. Its inputs are logged under
 * {@code Coprocessors/<name>}. It's judged from its inputs alone, so a replay judges it as the
 * robot did.
 */
final class Coprocessor implements AutoCloseable {
  private final CompiledTable table;
  final Computer computer;
  private final String address;
  private final CoprocessorIO io;
  final CoprocessorIOInputsAutoLogged inputs = new CoprocessorIOInputsAutoLogged();
  private final String key;
  private final Coprocessors.Settings settings;

  private final Alert missingEnabled;
  private final Alert missingDisabled;
  private final Alert wrongImage;
  private final Alert duplicated;
  private final Alert cameraMissing;
  private final Alert hot;
  private final Alert restarted;
  private final Alert journalErrors;
  private final Alert settingsDiffer;

  // This loop's judgment of it, for its card.
  private boolean missing = true;
  private List<String> mismatches = List.of();
  private List<String> camerasMissing = List.of();
  private int hottest = -1;
  private boolean hotNow;
  private boolean throttled;

  /**
   * @param table the table compiled into the program, with what this build expects
   * @param computer the computer, from that table
   */
  Coprocessor(
      CompiledTable table, Computer computer, CoprocessorIO io, Coprocessors.Settings settings) {
    this.table = table;
    this.computer = computer;
    address = table.table().ip(computer);
    this.io = io;
    this.settings = settings;
    String name = computer.name();
    key = "Coprocessors/" + name;
    missingEnabled = alert(name + "MissingEnabled", Alert.Level.HIGH);
    missingDisabled = alert(name + "MissingDisabled", Alert.Level.MEDIUM);
    wrongImage = alert(name + "WrongImage", Alert.Level.HIGH);
    duplicated = alert(name + "Duplicated", Alert.Level.HIGH);
    cameraMissing = alert(name + "CameraMissing", Alert.Level.MEDIUM);
    hot = alert(name + "Hot", Alert.Level.MEDIUM);
    restarted = alert(name + "Restarted", Alert.Level.MEDIUM);
    journalErrors = alert(name + "JournalErrors", Alert.Level.MEDIUM);
    settingsDiffer = alert(name + "SettingsDiffer", Alert.Level.MEDIUM);
    settingsDiffer.setText(
        "Coprocessor "
            + name
            + "'s PhotonVision settings differ from Git: back them up and commit them, or reflash"
            + " it");
  }

  private Alert alert(String id, Alert.Level level) {
    return new Alert("Coprocessors", id, "Coprocessor " + name(), level);
  }

  String name() {
    return computer.name();
  }

  String address() {
    return address;
  }

  /** Reads its latest answer, logs it, and judges it. Once a loop: it never waits. */
  void periodic(boolean enabled) {
    io.updateInputs(inputs);
    Logger.processInputs(key, inputs);
    if (inputs.everAnswered) {
      // The robot's time at the coprocessor's boot, by its latest answer: a journal entry's
      // monotonic time after that is its time on the robot's clock.
      Logger.recordOutput(key + "/BootedAtRobotNanos", bootedAtRobotNanos());
    }
    judge(enabled);
  }

  private void judge(boolean enabled) {
    String name = name();
    missing = !inputs.everAnswered || inputs.ageSeconds > settings.staleAfterSeconds();
    if (missing) {
      // Powered down on purpose, it's missing for a reason: say so, not that it failed.
      text(
          enabled ? missingEnabled : missingDisabled,
          inputs.powerDownAsked
              ? "Coprocessor "
                  + name
                  + " ("
                  + address
                  + ") was powered down from Pitwall: cycle the robot's power to restart it"
              : "Coprocessor "
                  + name
                  + " ("
                  + address
                  + ") "
                  + (inputs.everAnswered
                      ? "stopped answering"
                      : "hasn't answered since the robot program started")
                  + reason());
    }
    missingEnabled.set(missing && enabled);
    missingDisabled.set(missing && !enabled);

    // Which image it runs, from its latest answer however old: a stamp changes only with a
    // reflash. Compared as the build compares it, every value named both ways.
    mismatches =
        inputs.everAnswered ? table.mismatches(computer, Answers.stamp(inputs)) : List.of();
    if (!mismatches.isEmpty()) {
      text(
          wrongImage,
          "Coprocessor "
              + name
              + " runs the wrong image: "
              + String.join("; ", mismatches)
              + ". Flash it with this build's release");
    }
    wrongImage.set(!mismatches.isEmpty());

    boolean twoBoards = inputs.macsTakingTurns.length > 1;
    if (twoBoards) {
      text(
          duplicated,
          "Two boards answer at "
              + address
              + " ("
              + name
              + "), MACs "
              + String.join(" and ", inputs.macsTakingTurns)
              + ": is a spare drive with its image in another board?");
    }
    duplicated.set(twoBoards);

    boolean answering = !missing;
    camerasMissing = answering ? missingCameras() : List.of();
    if (!camerasMissing.isEmpty()) {
      text(
          cameraMissing,
          "Coprocessor "
              + name
              + " is missing camera"
              + (camerasMissing.size() > 1 ? "s " : " ")
              + String.join(", ", camerasMissing)
              + ": is it plugged in, into the port its settings expect?");
    }
    cameraMissing.set(!camerasMissing.isEmpty());

    judgeHeat(answering);

    boolean restartedSinceBoot = answering && inputs.photonVisionRestarts > 0;
    if (restartedSinceBoot) {
      text(
          restarted,
          "PhotonVision on "
              + name
              + " restarted "
              + times(inputs.photonVisionRestarts)
              + " since boot: read its journal (Pit, its card's Download logs)");
    }
    restarted.set(restartedSinceBoot);

    boolean errors = answering && inputs.journalErrors > 0;
    if (errors) {
      text(
          journalErrors,
          "Coprocessor "
              + name
              + "'s journal has "
              + count(inputs.journalErrors, "error")
              + " since boot"
              + (inputs.latestErrors.length > 0
                  ? ", the latest: " + inputs.latestErrors[inputs.latestErrors.length - 1]
                  : ""));
    }
    journalErrors.set(errors);

    settingsDiffer.set(answering && settingsDiffer());
  }

  private String reason() {
    return inputs.error.isEmpty() ? "" : ": " + inputs.error;
  }

  /** The cameras the table gives it that aren't plugged in where its settings expect them. */
  private List<String> missingCameras() {
    List<String> present = Arrays.asList(inputs.camerasPresent);
    return computer.cameras().stream().filter(camera -> !present.contains(camera)).toList();
  }

  /** Hot: its hottest zone (nearest its trip point) within the margin, or its big cores capped. */
  private void judgeHeat(boolean answering) {
    hottest = -1;
    double closest = Double.POSITIVE_INFINITY;
    int zones = Math.min(inputs.zoneCelsius.length, inputs.zoneTripCelsius.length);
    for (int i = 0; i < zones; i++) {
      double below = inputs.zoneTripCelsius[i] - inputs.zoneCelsius[i];
      if (Double.isFinite(below) && below < closest) {
        hottest = i;
        closest = below;
      }
    }
    hotNow = answering && hottest >= 0 && closest <= settings.hotMarginCelsius();
    throttled = answering && inputs.bigCoreLimitMhz < inputs.bigCoreMaxMhz;
    if (hotNow || throttled) {
      List<String> why = new ArrayList<>();
      if (hotNow) {
        why.add(heat());
      }
      if (throttled) {
        why.add(capped());
      }
      text(
          hot,
          "Coprocessor "
              + name()
              + " is hot: "
              + String.join("; ", why)
              + ". PhotonVision slows as it throttles: cool it");
    }
    hot.set(hotNow || throttled);
  }

  /** Its hottest zone against its trip point: "bigcore0-thermal 81 °C, trips at 85 °C". */
  private String heat() {
    if (hottest < 0) {
      return "no thermal zones with a trip point";
    }
    return String.format(
        Locale.ROOT,
        "%s %.0f °C, trips at %.0f °C",
        hottest < inputs.zones.length ? inputs.zones[hottest] : "zone " + hottest,
        inputs.zoneCelsius[hottest],
        inputs.zoneTripCelsius[hottest]);
  }

  private String capped() {
    return String.format(
        Locale.ROOT,
        "big cores held to %.0f of %.0f MHz",
        inputs.bigCoreLimitMhz,
        inputs.bigCoreMaxMhz);
  }

  private boolean settingsDiffer() {
    String committed = table.settingsHash(computer);
    return !committed.isEmpty() && !inputs.settingsHash.equals(committed);
  }

  /** Asks it to power down, on its IO's own thread. */
  void powerDown() {
    io.powerDown();
  }

  /**
   * Whether it's gone, powered down: asked to, and since then neither its agent's port nor
   * PhotonVision's accepts a connection.
   */
  boolean isGone() {
    return inputs.powerDownAsked
        && inputs.portsTried
        && !inputs.agentPortOpen
        && !inputs.photonVisionPortOpen;
  }

  /** How its agent answered the request to power down: empty until it did. */
  String powerDownAnswer() {
    return inputs.powerDownAnswer;
  }

  /** The robot's time when it booted, by its latest answer. */
  private long bootedAtRobotNanos() {
    return inputs.answeredNanos - inputs.agentMonotonicNanos;
  }

  // ---- Its card ----

  /** Its card on the Pitwall page, from this loop's judgment. */
  Card card() {
    List<Line> lines = new ArrayList<>();
    boolean twoBoards = inputs.macsTakingTurns.length > 1;
    if (inputs.powerDownAsked) {
      lines.add(powerLine());
    }
    if (missing && !isGone()) {
      lines.add(
          new Line(
              "Answer",
              (inputs.everAnswered
                      ? String.format(Locale.ROOT, "none for %.0f s", Math.floor(inputs.ageSeconds))
                      : "none since the robot program started")
                  + reason()
                  + (inputs.everAnswered ? "; the rest is from its last answer" : ""),
              Level.PROBLEM));
    }
    if (twoBoards) {
      lines.add(
          new Line(
              "Address",
              "two boards answer at " + address + ": " + String.join(", ", inputs.macsTakingTurns),
              Level.PROBLEM));
    }
    if (inputs.everAnswered) {
      boolean versionRight = inputs.photonVisionVersion.equals(table.photonvisionVersion());
      lines.add(
          new Line(
              "Version",
              "PhotonVision "
                  + orNone(inputs.photonVisionVersion)
                  + (versionRight
                      ? ", as this build's"
                      : "; this build's is " + table.photonvisionVersion()),
              versionRight ? Level.OK : Level.PROBLEM));
      boolean stampRight =
          inputs.name.equals(computer.name())
              && inputs.team == table.table().team()
              && inputs.address.equals(address)
              && inputs.recipeHash.equals(table.recipeHash());
      lines.add(
          new Line(
              "Stamp",
              "release "
                  + orNone(inputs.release)
                  + ", recipe "
                  + shortHash(inputs.recipeHash)
                  + (stampRight
                      ? ", as this build's"
                      : "; stamped as "
                          + orNone(inputs.name)
                          + " at "
                          + orNone(inputs.address)
                          + ", and this build's is "
                          + computer.name()
                          + " at "
                          + address
                          + ", recipe "
                          + shortHash(table.recipeHash())),
              stampRight ? Level.OK : Level.PROBLEM));
      lines.add(settingsLine());
      lines.add(new Line("Heat", heat(), hotNow ? Level.WARNING : Level.OK));
      lines.add(
          new Line(
              "Load",
              String.format(Locale.ROOT, "big cores %.0f%% busy, ", inputs.bigCoreBusyPercent)
                  + (throttled
                      ? capped()
                      : String.format(
                          Locale.ROOT, "free to run at %.0f MHz", inputs.bigCoreMaxMhz)),
              throttled ? Level.WARNING : Level.OK));
      lines.add(camerasLine());
      lines.add(sinceBootLine());
    }
    Level level = Level.OK;
    for (Line line : lines) {
      level = line.level().compareTo(level) > 0 ? line.level() : level;
    }
    String state =
        isGone()
            ? "Powered down"
            : inputs.powerDownAsked
                ? "Powering down"
                : twoBoards ? "Two answer" : missing ? "Missing" : "Present";
    // Powered down on purpose, it's as it should be: the rest is its last answer.
    return new Card(state, isGone() ? Level.NOTE : level, lines);
  }

  /** Where powering it down stands: what its agent said, and which ports still answer. */
  private Line powerLine() {
    if (isGone()) {
      return new Line(
          "Power",
          "powered down from Pitwall: gone from its agent's port and PhotonVision's; cycle the"
              + " robot's power to restart it",
          Level.NOTE);
    }
    List<String> open = new ArrayList<>();
    if (inputs.agentPortOpen) {
      open.add("its agent");
    }
    if (inputs.photonVisionPortOpen) {
      open.add("PhotonVision");
    }
    return new Line(
        "Power",
        "asked to power down"
            + (inputs.powerDownAnswer.isEmpty() ? "" : ", " + inputs.powerDownAnswer)
            + (inputs.portsTried && !open.isEmpty()
                ? "; " + String.join(" and ", open) + " still answering"
                : ""),
        inputs.powerDownAnswer.isEmpty() || inputs.powerDownAnswer.equals("accepted")
            ? Level.NOTE
            : Level.WARNING);
  }

  private Line settingsLine() {
    if (table.settingsHash(computer).isEmpty()) {
      return new Line("Settings", "none in Git yet: download a backup, and commit it", Level.NOTE);
    }
    return settingsDiffer()
        ? new Line(
            "Settings",
            "differ from Git ("
                + shortHash(inputs.settingsHash)
                + "): back them up and commit them",
            Level.WARNING)
        : new Line("Settings", "as in Git (" + shortHash(inputs.settingsHash) + ")", Level.OK);
  }

  private Line camerasLine() {
    List<String> expected = computer.cameras();
    if (expected.isEmpty()) {
      return new Line(
          "Cameras",
          "none in the table; " + count(inputs.usbCameras.length, "USB camera") + " plugged in",
          Level.NOTE);
    }
    int present = expected.size() - camerasMissing.size();
    return new Line(
        "Cameras",
        present
            + " of "
            + expected.size()
            + (camerasMissing.isEmpty()
                ? ": " + String.join(", ", expected)
                : ", " + String.join(", ", camerasMissing) + " missing"),
        camerasMissing.isEmpty() ? Level.OK : Level.PROBLEM);
  }

  private Line sinceBootLine() {
    boolean unclean = inputs.lastShutdownKnown && !inputs.lastShutdownClean;
    boolean trouble = inputs.photonVisionRestarts > 0 || inputs.journalErrors > 0 || unclean;
    String text =
        "up "
            + uptime()
            + "; PhotonVision restarted "
            + (inputs.photonVisionRestarts == 0 ? "never" : times(inputs.photonVisionRestarts))
            + "; last shutdown "
            + (!inputs.lastShutdownKnown ? "unknown" : unclean ? "unclean (a power cut)" : "clean")
            + "; "
            + count(inputs.journalErrors, "error")
            + " in its journal";
    return new Line("Since boot", text, trouble ? Level.WARNING : Level.OK);
  }

  private String uptime() {
    long seconds = (long) inputs.uptimeSeconds;
    return seconds < 120
        ? seconds + " s"
        : seconds < 7200 ? seconds / 60 + " min" : seconds / 3600 + " h";
  }

  // ---- Downloads, on the asker's thread ----

  /** Its settings backup, fetched from its agent now. */
  Devices.File settingsBackup() throws IOException {
    return new Devices.File(name() + "-settings.zip", "application/zip", io.settingsBackup());
  }

  /**
   * A page of its journal, fetched from its agent now, each entry placed on the robot's clock when
   * it's from the boot the robot last heard of.
   */
  Devices.LogPage journal(Devices.LogQuery query) throws IOException {
    JournalPage page = io.journal(query.cursor(), query.priority(), query.unit());
    // Read once: the loop may move them on meanwhile.
    String boot = inputs.bootId;
    long bootedAt = bootedAtRobotNanos();
    List<Devices.LogEntry> entries = new ArrayList<>();
    for (JournalEntry entry : page.entries()) {
      entries.add(
          new Devices.LogEntry(
              when(entry, boot, bootedAt),
              entry.priority(),
              entry.unit().isEmpty() ? entry.identifier() : entry.unit(),
              entry.message()));
    }
    return new Devices.LogPage(entries, page.more() ? page.cursor() : "");
  }

  /**
   * When an entry was logged: on the robot's clock, in seconds, for this boot's; else since its own
   * boot, which the robot can't place.
   */
  static String when(JournalEntry entry, String boot, long bootedAtRobotNanos) {
    double sinceBoot = entry.monotonicMicros() / 1e6;
    if (!boot.isEmpty() && boot.equals(entry.bootId())) {
      return String.format(
          Locale.ROOT, "robot %.3f s", (bootedAtRobotNanos + entry.monotonicMicros() * 1000) / 1e9);
    }
    return String.format(Locale.ROOT, "boot +%.3f s", sinceBoot);
  }

  // ---- For tests ----

  boolean isMissing() {
    return missing;
  }

  /** Its alerts up now, each as its level and text. */
  List<String> alerts() {
    return Stream.of(
            missingEnabled,
            missingDisabled,
            wrongImage,
            duplicated,
            cameraMissing,
            hot,
            restarted,
            journalErrors,
            settingsDiffer)
        .filter(Alert::get)
        .map(Coprocessor::shown)
        .toList();
  }

  /** An alert as its level and text: {@code "HIGH: Coprocessor ..."}. */
  static String shown(Alert alert) {
    return alert.getLevel() + ": " + alert.getText();
  }

  /** Sets an alert's text only when it changes. */
  private static void text(Alert alert, String text) {
    if (!text.equals(alert.getText())) {
      alert.setText(text);
    }
  }

  private static String times(int count) {
    return count == 1 ? "once" : count + " times";
  }

  private static String count(int count, String thing) {
    return count + " " + thing + (count == 1 ? "" : "s");
  }

  private static String orNone(String text) {
    return text.isEmpty() ? "(none)" : text;
  }

  private static String shortHash(String hash) {
    return hash.isEmpty() ? "(none)" : hash.length() > 12 ? hash.substring(0, 12) : hash;
  }

  @Override
  public void close() {
    io.close();
    for (Alert alert :
        List.of(
            missingEnabled,
            missingDisabled,
            wrongImage,
            duplicated,
            cameraMissing,
            hot,
            restarted,
            journalErrors,
            settingsDiffer)) {
      alert.close();
    }
  }
}
