package frc.robot.subsystems.coprocessors;

import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.pitwall.Devices.Card;
import com.michaelgrundvig.frc.pitwall.Devices.Level;
import com.michaelgrundvig.frc.pitwall.Devices.Line;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * A coprocessor's card on the Pitwall page, from its judgment this loop: whether it's there; its
 * PhotonVision and stamp against this build's, as the judgment's mismatches say; its settings
 * against Git's; heat, load, cameras, and its boot; and where powering it down stands. Its live
 * numbers are rounded (heat to 5 °C until near the trip point, load to 10%, uptime to the minute),
 * so an open page isn't sent a change every second.
 */
final class CoprocessorCard {
  /** How the table's comparison names the PhotonVision version, and the recipe. */
  private static final String PHOTONVISION = "photonvisionVersion";

  private static final String RECIPE = "recipeHash";

  private final Coprocessor coprocessor;
  private final CoprocessorIO.CoprocessorIOInputs inputs;

  private CoprocessorCard(Coprocessor coprocessor) {
    this.coprocessor = coprocessor;
    inputs = coprocessor.inputs;
  }

  /** A coprocessor's card now. */
  static Card of(Coprocessor coprocessor) {
    return new CoprocessorCard(coprocessor).card();
  }

  private Card card() {
    List<Line> lines = new ArrayList<>();
    boolean twoBoards = inputs.macsTakingTurns.length > 1;
    boolean gone = coprocessor.isGone();
    if (inputs.powerDownAsked) {
      lines.add(powerLine(gone));
    }
    if (coprocessor.missing && !gone) {
      lines.add(
          new Line(
              "Answer",
              (inputs.everAnswered
                      ? String.format(Locale.ROOT, "none for %.0f s", Math.floor(inputs.ageSeconds))
                      : "none since the robot program started")
                  + coprocessor.reason()
                  + (inputs.everAnswered ? "; the rest is from its last answer" : ""),
              Level.PROBLEM));
    }
    if (twoBoards) {
      lines.add(
          new Line(
              "Address",
              "two boards answer at "
                  + coprocessor.address()
                  + ": "
                  + String.join(", ", inputs.macsTakingTurns),
              Level.PROBLEM));
    }
    if (inputs.everAnswered) {
      lines.add(versionLine());
      lines.add(stampLine());
      lines.add(settingsLine());
      lines.add(
          new Line(
              "Heat",
              coprocessor.hotNow ? coprocessor.heat() : roughHeat(),
              coprocessor.hotNow ? Level.WARNING : Level.OK));
      lines.add(loadLine());
      lines.add(camerasLine());
      lines.add(sinceBootLine());
    }
    Level level = Level.OK;
    for (Line line : lines) {
      level = line.level().compareTo(level) > 0 ? line.level() : level;
    }
    String state =
        gone
            ? "Powered down"
            : inputs.powerDownAccepted
                ? "Powering down"
                : inputs.powerDownAsked && !inputs.powerDownAnswer.isEmpty()
                    ? "Not powered down"
                    : twoBoards ? "Two answer" : coprocessor.missing ? "Missing" : "Present";
    // Powered down on purpose, it's as it should be: the rest is its last answer.
    return new Card(state, gone ? Level.NOTE : level, lines);
  }

  /** Its PhotonVision, against this build's: wrong when the judgment's mismatches say so. */
  private Line versionLine() {
    List<CompiledTable.Mismatch> wrong =
        coprocessor.mismatches.stream().filter(m -> m.field().equals(PHOTONVISION)).toList();
    return wrong.isEmpty()
        ? new Line(
            "Version",
            "PhotonVision " + orNone(inputs.photonVisionVersion) + ", as this build's",
            Level.OK)
        : new Line("Version", messages(wrong), Level.PROBLEM);
  }

  /**
   * The rest of its stamp, against this build's, as the judgment's mismatches say: the wrong name,
   * team, or address is a problem; another recipe a warning; a recipe this build can't judge (it
   * has no recipe hash) a note.
   */
  private Line stampLine() {
    List<CompiledTable.Mismatch> rest =
        coprocessor.mismatches.stream().filter(m -> !m.field().equals(PHOTONVISION)).toList();
    String release =
        "release " + orNone(inputs.release) + ", recipe " + shortHash(inputs.recipeHash);
    if (rest.isEmpty()) {
      return new Line("Stamp", release + ", as this build's", Level.OK);
    }
    Level level = Level.NOTE;
    for (CompiledTable.Mismatch mismatch : rest) {
      Level its =
          switch (mismatch.severity()) {
            case ERROR -> Level.PROBLEM;
            case WARNING -> Level.WARNING;
            case UNKNOWN -> Level.NOTE;
          };
      level = its.compareTo(level) > 0 ? its : level;
    }
    List<String> said = new ArrayList<>();
    for (CompiledTable.Mismatch mismatch : rest) {
      said.add(
          mismatch.field().equals(RECIPE) && mismatch.severity() == CompiledTable.Severity.UNKNOWN
              ? "the recipe can't be checked: this build has no recipe hash"
              : mismatch.message());
    }
    return new Line("Stamp", release + "; " + String.join("; ", said), level);
  }

  /** Mismatches' sentences, joined. */
  private static String messages(List<CompiledTable.Mismatch> mismatches) {
    return String.join("; ", mismatches.stream().map(CompiledTable.Mismatch::message).toList());
  }

  /** Where powering it down stands: what its agent said, and which ports still answer. */
  private Line powerLine(boolean gone) {
    if (gone) {
      return new Line(
          "Power",
          "powered down from Pitwall: gone from its agent's port and PhotonVision's; cycle the"
              + " robot's power to restart it",
          Level.NOTE);
    }
    if (coprocessor.refusedPowerDown()) {
      return new Line(
          "Power",
          "refused to power down: "
              + inputs.powerDownAnswer.substring(inputs.powerDownAnswer.indexOf(':') + 2),
          Level.PROBLEM);
    }
    if (!inputs.powerDownAccepted && !inputs.powerDownAnswer.isEmpty()) {
      return new Line(
          "Power", "couldn't be asked to power down: " + inputs.powerDownAnswer, Level.PROBLEM);
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
            + (inputs.powerDownAccepted ? ", accepted" : "")
            + (inputs.portsTried && !open.isEmpty()
                ? "; " + String.join(" and ", open) + " still answering"
                : ""),
        Level.NOTE);
  }

  /**
   * Its hottest zone, to the nearest 5 °C: on the card, which a board's degree-by-degree wobble
   * would otherwise change every second. Near its trip point, the card says it exactly.
   */
  private String roughHeat() {
    int hottest = coprocessor.hottest;
    if (hottest < 0) {
      return coprocessor.heat();
    }
    return String.format(
        Locale.ROOT,
        "%s about %.0f °C, trips at %.0f °C",
        hottest < inputs.zones.length ? inputs.zones[hottest] : "zone " + hottest,
        Math.round(inputs.zoneCelsius[hottest] / 5) * 5.0,
        inputs.zoneTripCelsius[hottest]);
  }

  private Line loadLine() {
    boolean throttled = coprocessor.throttled;
    return new Line(
        "Load",
        // To the nearest 10%, so the card changes when the load does, not with every poll.
        String.format(
                Locale.ROOT,
                "big cores about %.0f%% busy, ",
                Math.round(inputs.bigCoreBusyPercent / 10) * 10.0)
            + (throttled
                ? coprocessor.capped()
                : String.format(Locale.ROOT, "free to run at %.0f MHz", inputs.bigCoreMaxMhz)),
        throttled ? Level.WARNING : Level.OK);
  }

  private Line settingsLine() {
    if (coprocessor.table.settingsHash(coprocessor.computer).isEmpty()) {
      return new Line("Settings", "none in Git yet: download a backup, and commit it", Level.NOTE);
    }
    return coprocessor.settingsDiffer()
        ? new Line(
            "Settings",
            "differ from Git ("
                + shortHash(inputs.settingsHash)
                + "): back them up and commit them",
            Level.WARNING)
        : new Line("Settings", "as in Git (" + shortHash(inputs.settingsHash) + ")", Level.OK);
  }

  private Line camerasLine() {
    List<String> expected = coprocessor.computer.cameras();
    List<String> missing = coprocessor.camerasMissing;
    if (expected.isEmpty()) {
      return new Line(
          "Cameras",
          "none in the table; "
              + Coprocessor.count(inputs.usbCameras.length, "USB camera")
              + " plugged in",
          Level.NOTE);
    }
    int present = expected.size() - missing.size();
    return new Line(
        "Cameras",
        present
            + " of "
            + expected.size()
            + (missing.isEmpty()
                ? ": " + String.join(", ", expected)
                : ", " + String.join(", ", missing) + " missing"),
        missing.isEmpty() ? Level.OK : Level.PROBLEM);
  }

  /** Its boot: the totals, which the alerts count only as they grow. */
  private Line sinceBootLine() {
    boolean unclean = inputs.lastShutdownKnown && !inputs.lastShutdownClean;
    boolean trouble = inputs.photonVisionRestarts > 0 || inputs.journalErrors > 0 || unclean;
    String text =
        "up "
            + uptime()
            + "; PhotonVision restarted "
            + (inputs.photonVisionRestarts == 0
                ? "never"
                : Coprocessor.times(inputs.photonVisionRestarts))
            + "; last shutdown "
            + (!inputs.lastShutdownKnown ? "unknown" : unclean ? "unclean (a power cut)" : "clean")
            + "; "
            + Coprocessor.count(inputs.journalErrors, "error")
            + " in its journal";
    return new Line("Since boot", text, trouble ? Level.WARNING : Level.OK);
  }

  /** How long it's been up, to the minute: the card changes once a minute for it, at most. */
  private String uptime() {
    long seconds = (long) inputs.uptimeSeconds;
    return seconds < 60
        ? "under a minute"
        : seconds < 7200 ? seconds / 60 + " min" : seconds / 3600 + " h";
  }

  private static String orNone(String text) {
    return text.isEmpty() ? "(none)" : text;
  }

  private static String shortHash(String hash) {
    return hash.isEmpty() ? "(none)" : hash.length() > 12 ? hash.substring(0, 12) : hash;
  }
}
