package frc.robot.subsystems.coprocessors;

import com.michaelgrundvig.frc.coprocessor.api.AgentApi;
import com.michaelgrundvig.frc.coprocessor.api.JournalEntry;
import com.michaelgrundvig.frc.coprocessor.api.JournalPage;
import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import com.michaelgrundvig.frc.pitwall.Devices;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Set;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;

/**
 * One coprocessor, as {@link Coprocessors} watches it: its IO and what it read this loop, what this
 * build expects of it, and its alerts. Its inputs are logged under {@code Coprocessors/<name>}.
 * It's judged from its inputs alone, so a replay judges it as the robot did; in full only when a
 * new answer arrives, or it goes missing or comes back. Its card is {@link CoprocessorCard}'s.
 */
final class Coprocessor implements AutoCloseable {
  /**
   * The kinds of journal error alerted as soon as they're seen, however old in the boot: hardware
   * and the system failing. The rest ({@code error}) are alerted only as they grow.
   */
  static final Set<String> AT_ONCE = Set.of("usb", "uvc", "filesystem", "oom", "thermal");

  /**
   * The coprocessor's boot as the robot last heard it, for placing its journal on the robot's
   * clock: read on Pitwall's threads, written whole in the loop.
   *
   * @param bootId the boot's ID
   * @param bootedAtRobotNanos the robot's time when it booted
   */
  private record Clock(String bootId, long bootedAtRobotNanos) {}

  final CompiledTable table;
  final Computer computer;
  private final String address;
  private final CoprocessorIO io;
  final CoprocessorIOInputsAutoLogged inputs = new CoprocessorIOInputsAutoLogged();
  private final String key;
  private final String bootedKey;
  final Coprocessors.Settings settings;

  private final Alert missingEnabled;
  private final Alert missingDisabled;
  private final Alert wrongImage;
  private final Alert otherRecipe;
  private final Alert duplicated;
  private final Alert cameraMissing;
  private final Alert hot;
  private final Alert restarted;
  private final Alert journalErrors;
  private final Alert settingsDiffer;
  private final Alert failed;

  private volatile Clock clock = new Clock("", 0);

  // This loop's judgment of it, for its alerts and card.
  boolean missing = true;
  List<CompiledTable.Mismatch> mismatches = List.of();
  List<String> camerasMissing = List.of();
  int hottest = -1;
  boolean hotNow;
  boolean throttled;

  /** The answer last judged in full, and whether it was missing then. */
  private long judgedAnswer = Long.MIN_VALUE;

  private boolean judgedMissing = true;

  // Hot, with hysteresis: when its last answer near enough its trip point to keep it hot came.
  private boolean hotAlerted;
  private long lastWarmNanos;

  // What this boot had when the robot first heard it: restarts and errors since are news.
  private String baselineBoot = "";
  private int baselineRestarts;
  private long baselineErrors;
  int newRestarts;
  long errorsToRead;

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
    bootedKey = key + "/BootedAtRobotNanos";
    missingEnabled = alert(name + "MissingEnabled", Alert.Level.HIGH);
    missingDisabled = alert(name + "MissingDisabled", Alert.Level.MEDIUM);
    wrongImage = alert(name + "WrongImage", Alert.Level.HIGH);
    otherRecipe = alert(name + "OtherRecipe", Alert.Level.MEDIUM);
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
    failed = alert(name + "Failed", Alert.Level.HIGH);
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
      // The robot's time at the coprocessor's boot: a journal entry's monotonic time after that is
      // its time on the robot's clock. An answer may carry a reading up to a poll old, its clock
      // read then, which puts the boot late by as much, never early: so this boot's earliest
      // estimate is its best.
      long estimate = inputs.answeredNanos - inputs.agentMonotonicNanos;
      Clock was = clock;
      Clock now =
          new Clock(
              inputs.bootId,
              was.bootId().equals(inputs.bootId)
                  ? Math.min(was.bootedAtRobotNanos(), estimate)
                  : estimate);
      if (!now.equals(was)) {
        clock = now;
      }
      Logger.recordOutput(bootedKey, now.bootedAtRobotNanos());
    }
    judge(enabled);
    failed.set(false);
  }

  /** Watching it failed this loop: said, so the others are watched on. */
  void failed(RuntimeException e) {
    String why = e.getMessage();
    text(
        failed,
        "Watching coprocessor " + name() + " failed: " + (why != null ? why : e.toString()));
    failed.set(true);
  }

  private void judge(boolean enabled) {
    missing = !inputs.everAnswered || inputs.ageSeconds > settings.staleAfterSeconds();
    if (missing) {
      // Powered down on purpose, it's missing for a reason: say so, not that it failed.
      text(
          enabled ? missingEnabled : missingDisabled,
          "Coprocessor "
              + name()
              + " ("
              + address
              + ") "
              + (inputs.powerDownAccepted
                  ? "was powered down from Pitwall: cycle the robot's power to restart it"
                  : (inputs.everAnswered
                          ? "stopped answering"
                          : "hasn't answered since the robot program started")
                      + reason()));
    }
    missingEnabled.set(missing && enabled);
    missingDisabled.set(missing && !enabled);
    // The rest changes only with a new answer, or with it going missing or coming back.
    if (inputs.answeredNanos == judgedAnswer && missing == judgedMissing) {
      return;
    }
    judgedAnswer = inputs.answeredNanos;
    judgedMissing = missing;
    judgeAnswer();
  }

  private void judgeAnswer() {
    String name = name();
    // Which image it runs, from its latest answer however old: a stamp changes only with a
    // reflash. Compared as the build compares it (the table's own comparison), every value named
    // both ways: the wrong computer or PhotonVision is an error, an older recipe a warning, and a
    // recipe this build can't judge (no recipe hash) neither.
    mismatches = inputs.everAnswered ? table.compare(computer, Answers.stamp(inputs)) : List.of();
    List<String> errors = messages(CompiledTable.Severity.ERROR);
    if (!errors.isEmpty()) {
      text(
          wrongImage,
          "Coprocessor "
              + name
              + " runs the wrong image: "
              + String.join("; ", errors)
              + ". Flash it with this build's release");
    }
    wrongImage.set(!errors.isEmpty());
    List<String> warnings = messages(CompiledTable.Severity.WARNING);
    if (!warnings.isEmpty()) {
      text(
          otherRecipe,
          "Coprocessor "
              + name
              + "'s image was built from another recipe: "
              + String.join("; ", warnings)
              + ". Flash this build's release when convenient");
    }
    otherRecipe.set(!warnings.isEmpty());

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
    judgeNews(answering);
    settingsDiffer.set(answering && settingsDiffer());
  }

  /** The judgment's mismatches of one severity, as sentences naming both values. */
  List<String> messages(CompiledTable.Severity severity) {
    return mismatches.stream()
        .filter(mismatch -> mismatch.severity() == severity)
        .map(CompiledTable.Mismatch::message)
        .toList();
  }

  /**
   * Restarts and errors, against what this boot had when the robot first heard it: a board up all
   * day has some, and only new ones are news. Errors of the kinds alerted at once ({@link
   * #AT_ONCE}) are news however old.
   */
  private void judgeNews(boolean answering) {
    if (inputs.everAnswered && !inputs.bootId.equals(baselineBoot)) {
      baselineBoot = inputs.bootId;
      baselineRestarts = inputs.photonVisionRestarts;
      baselineErrors = count("error");
    }
    newRestarts = Math.max(0, inputs.photonVisionRestarts - baselineRestarts);
    long atOnce = 0;
    for (String kind : AT_ONCE) {
      atOnce += count(kind);
    }
    errorsToRead = atOnce + Math.max(0, count("error") - baselineErrors);

    boolean restartedLately = answering && newRestarts > 0;
    if (restartedLately) {
      text(
          restarted,
          "PhotonVision on "
              + name()
              + " restarted "
              + times(newRestarts)
              + " since the robot first heard this boot: read its journal (Pit, its card's"
              + " Download logs)");
    }
    restarted.set(restartedLately);

    boolean errors = answering && errorsToRead > 0;
    if (errors) {
      text(
          journalErrors,
          "Coprocessor "
              + name()
              + "'s journal has "
              + count(errorsToRead, "error")
              + " to read"
              + (inputs.latestErrors.length > 0
                  ? ", the latest: " + inputs.latestErrors[inputs.latestErrors.length - 1]
                  : ""));
    }
    journalErrors.set(errors);
  }

  /** How many errors of one kind its journal has this boot. */
  private long count(String kind) {
    int kinds = Math.min(inputs.journalCategories.length, inputs.journalCounts.length);
    for (int i = 0; i < kinds; i++) {
      if (inputs.journalCategories[i].equals(kind)) {
        return inputs.journalCounts[i];
      }
    }
    return 0;
  }

  String reason() {
    return inputs.error.isEmpty() ? "" : ": " + inputs.error;
  }

  /** The cameras the table gives it that aren't plugged in where its settings expect them. */
  private List<String> missingCameras() {
    List<String> present = Arrays.asList(inputs.camerasPresent);
    return computer.cameras().stream().filter(camera -> !present.contains(camera)).toList();
  }

  /**
   * Hot: its hottest zone (nearest its trip point) within the margin, or its big cores capped. Once
   * hot, it stays hot until its answers come in well clear of the margin (by {@link
   * Coprocessors.Settings#hotClearMarginCelsius}) for a while ({@link
   * Coprocessors.Settings#hotHoldSeconds}), so a board hovering at the margin doesn't flicker it.
   */
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
    boolean warm =
        answering
            && (throttled
                || (hottest >= 0
                    && closest <= settings.hotMarginCelsius() + settings.hotClearMarginCelsius()));
    if (warm) {
      lastWarmNanos = inputs.answeredNanos;
    }
    boolean held =
        hotAlerted
            && answering
            && inputs.answeredNanos - lastWarmNanos < Math.round(settings.hotHoldSeconds() * 1e9);
    hotAlerted = hotNow || throttled || (hotAlerted && warm) || held;
    if (hotAlerted) {
      List<String> why = new ArrayList<>();
      why.add(heat());
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
    hot.set(hotAlerted);
  }

  /** Its hottest zone against its trip point: "bigcore0-thermal 81 °C, trips at 85 °C". */
  String heat() {
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

  String capped() {
    return String.format(
        Locale.ROOT,
        "big cores held to %.0f of %.0f MHz",
        inputs.bigCoreLimitMhz,
        inputs.bigCoreMaxMhz);
  }

  boolean settingsDiffer() {
    String committed = table.settingsHash(computer);
    return !committed.isEmpty() && !inputs.settingsHash.equals(committed);
  }

  // ---- Powering down ----

  /** Asks it to power down, on its IO's own thread. */
  void powerDown() {
    io.powerDown();
  }

  /** Stops asking it, and watching it go. */
  void stopPowerDown() {
    io.stopPowerDown();
  }

  /**
   * Whether it's gone, powered down: its agent took the request, and since then neither its agent's
   * port nor PhotonVision's accepts a connection. One that never got the request isn't, however
   * quiet: it may be running, its network unplugged.
   */
  boolean isGone() {
    return inputs.powerDownAccepted
        && inputs.portsTried
        && !inputs.agentPortOpen
        && !inputs.photonVisionPortOpen;
  }

  /** Whether its agent refused the request: it won't power down from here. */
  boolean refusedPowerDown() {
    return inputs.powerDownAnswer.startsWith("refused");
  }

  /**
   * What stands in the way of its powering down, as the page says it: that it couldn't be reached,
   * refused, or (taken, but not gone) is still answering; empty when nothing does, or it hasn't
   * answered yet.
   */
  String powerDownTrouble() {
    if (!inputs.powerDownAsked || isGone()) {
      return "";
    }
    if (inputs.powerDownAccepted) {
      return "still answering";
    }
    if (refusedPowerDown()) {
      return inputs.powerDownAnswer;
    }
    return inputs.powerDownAnswer.isEmpty() ? "" : "couldn't be reached: " + inputs.powerDownAnswer;
  }

  // ---- Its card, and downloads on the asker's thread ----

  /** Its card on the Pitwall page, from this loop's judgment. */
  Devices.Card card() {
    return CoprocessorCard.of(this);
  }

  /** Its settings backup, streamed from its agent as it arrives. */
  Devices.File settingsBackup() throws IOException {
    CoprocessorIO.Backup backup = busyAsBusy(io::settingsBackup);
    return new Devices.File(
        name() + "-settings.zip", "application/zip", backup.length(), backup.body());
  }

  /** A call to its agent, its agent still busy said as a device busy, for the page's 503. */
  private static <T> T busyAsBusy(AgentHttp.Call<T> call) throws IOException {
    try {
      return call.call();
    } catch (AgentHttp.Answered answered) {
      if (answered.busy()) {
        throw new Devices.Busy("its agent is busy with another such request");
      }
      throw answered;
    }
  }

  /**
   * A page of its journal, fetched from its agent now, each entry placed on the robot's clock when
   * it's from the boot the robot last heard of.
   */
  Devices.LogPage journal(Devices.LogQuery query) throws IOException {
    if (!query.unit().isEmpty() && !AgentApi.JOURNAL_UNITS.contains(query.unit())) {
      throw new IOException(
          "its agent serves only these units' entries: "
              + String.join(", ", AgentApi.JOURNAL_UNITS));
    }
    CoprocessorIO.JournalPlace place =
        switch (query.from()) {
          case LATEST -> CoprocessorIO.JournalPlace.LATEST;
          case AFTER -> CoprocessorIO.JournalPlace.AFTER;
          case BEFORE -> CoprocessorIO.JournalPlace.BEFORE;
          case START -> CoprocessorIO.JournalPlace.BOOT;
        };
    JournalPage page =
        busyAsBusy(() -> io.journal(place, query.cursor(), query.priority(), query.unit()));
    Clock heard = clock; // read once: the loop replaces it whole
    List<Devices.LogEntry> entries = new ArrayList<>();
    for (JournalEntry entry : page.entries()) {
      entries.add(
          new Devices.LogEntry(
              when(entry, heard.bootId(), heard.bootedAtRobotNanos()),
              entry.priority(),
              entry.unit().isEmpty() ? entry.identifier() : entry.unit(),
              entry.message()));
    }
    List<JournalEntry> got = page.entries();
    // The agent's one cursor goes the way the page was asked; the other side's is an entry's.
    String newer =
        switch (place) {
          case BEFORE -> got.isEmpty() ? "" : got.getLast().cursor();
          default -> page.cursor();
        };
    String older =
        switch (place) {
          case BEFORE -> page.more() ? page.cursor() : "";
          case BOOT -> "";
          default -> got.isEmpty() ? "" : got.get(0).cursor();
        };
    return new Devices.LogPage(entries, newer, older);
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

  /** The robot's time at its boot, as best known. */
  long bootedAtRobotNanos() {
    return clock.bootedAtRobotNanos();
  }

  /** Its alerts up now, each as its level and text. */
  List<String> alerts() {
    return Stream.of(
            missingEnabled,
            missingDisabled,
            wrongImage,
            otherRecipe,
            duplicated,
            cameraMissing,
            hot,
            restarted,
            journalErrors,
            settingsDiffer,
            failed)
        .filter(Alert::get)
        .map(Coprocessor::shown)
        .toList();
  }

  /** An alert as its level and text: {@code "HIGH: Coprocessor ..."}. */
  static String shown(Alert alert) {
    return alert.getLevel() + ": " + alert.getText();
  }

  /** Sets an alert's text only when it changes. */
  static void text(Alert alert, String text) {
    if (!Objects.equals(text, alert.getText())) {
      alert.setText(text);
    }
  }

  static String times(long count) {
    return count == 1 ? "once" : count + " times";
  }

  static String count(long count, String thing) {
    return count + " " + thing + (count == 1 ? "" : "s");
  }

  @Override
  public void close() {
    io.close();
    for (Alert alert :
        List.of(
            missingEnabled,
            missingDisabled,
            wrongImage,
            otherRecipe,
            duplicated,
            cameraMissing,
            hot,
            restarted,
            journalErrors,
            settingsDiffer,
            failed)) {
      alert.close();
    }
  }
}
