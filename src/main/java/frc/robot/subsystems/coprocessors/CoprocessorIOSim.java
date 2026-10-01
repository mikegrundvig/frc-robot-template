package frc.robot.subsystems.coprocessors;

import com.michaelgrundvig.frc.coprocessor.api.Boot;
import com.michaelgrundvig.frc.coprocessor.api.Cameras;
import com.michaelgrundvig.frc.coprocessor.api.Cpu;
import com.michaelgrundvig.frc.coprocessor.api.CpuCluster;
import com.michaelgrundvig.frc.coprocessor.api.Drive;
import com.michaelgrundvig.frc.coprocessor.api.ExpectedCamera;
import com.michaelgrundvig.frc.coprocessor.api.Health;
import com.michaelgrundvig.frc.coprocessor.api.JournalEntry;
import com.michaelgrundvig.frc.coprocessor.api.JournalPage;
import com.michaelgrundvig.frc.coprocessor.api.JournalSummary;
import com.michaelgrundvig.frc.coprocessor.api.Service;
import com.michaelgrundvig.frc.coprocessor.api.SettingsState;
import com.michaelgrundvig.frc.coprocessor.api.Stamp;
import com.michaelgrundvig.frc.coprocessor.api.ThermalZone;
import com.michaelgrundvig.frc.coprocessor.api.TripPoint;
import com.michaelgrundvig.frc.coprocessor.api.UsbCamera;
import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;
import org.jspecify.annotations.Nullable;

/**
 * A simulated coprocessor's health agent, for the simulator: it answers as a healthy Orange Pi 5
 * running this build's image would, once a poll period on the robot's clock, and its faults make it
 * answer as a troubled one would (or not at all). It builds the agent's own records ({@link
 * Health}), and the robot reads them through the same flattening a real answer goes through ({@link
 * Answers}), so the simulator exercises everything past the network.
 *
 * <p>Everything follows the robot's clock, stepped by the simulated world: the same run gives the
 * same answers. Its faults are declared to Pitwall's simulator controls by {@code
 * frc.robot.sim.Faults}.
 */
public final class CoprocessorIOSim implements CoprocessorIO {
  /** A PhotonVision version not this build's, as a coprocessor left on last season's would run. */
  public static final String OTHER_VERSION = "v2026.3.4";

  /** The trip point the simulated board's zones slow the cores at: mainline RK3588's, 85 °C. */
  static final double TRIP_CELSIUS = 85;

  /** How long the simulated board had been up when the robot program started. */
  private static final double UP_BEFORE_SECONDS = 40;

  /** How long PhotonVision takes to come back after a restart. */
  private static final long RESTART_NANOS = 3_000_000_000L;

  /** The big cores' most, and the little ones'. */
  private static final int BIG_MAX_MHZ = 2400;

  private static final int LITTLE_MAX_MHZ = 1800;

  private final CompiledTable table;
  private final Computer computer;
  private final long periodNanos;
  private final MacWatch macs;
  private final String mac;
  private final String otherMac;

  // The faults.
  private boolean unreachable;
  private boolean wrongVersion;
  private double belowTripCelsius = Double.NaN;
  private int unpluggedCamera = -1;
  private boolean twoBoards;
  private boolean settingsChanged;
  private int restarts;
  private long restartedAtNanos;

  // The robot's clock, as the world steps it, and the agent's polls on it.
  private long nowNanos;
  private long nextPollNanos;
  private long polls;
  private boolean answered;
  private String error = "not asked yet";
  private long answeredNanos;
  private boolean everAnswered;
  private Health latest;
  private List<String> takingTurns = List.of();
  private @Nullable Health filled;

  /** What its journal holds: the boot, then what its faults made happen. */
  private final List<JournalEntry> journal = new ArrayList<>();

  private final Map<String, Integer> errorCounts = new LinkedHashMap<>();

  /**
   * @param table the table compiled into the program, with what this build expects
   * @param computer the computer it simulates
   * @param settings how often it's asked
   */
  public CoprocessorIOSim(CompiledTable table, Computer computer, Coprocessors.Settings settings) {
    this.table = table;
    this.computer = computer;
    periodNanos = Math.round(settings.pollPeriodSeconds() * 1e9);
    macs = new MacWatch(settings.turnsWindowSeconds());
    int at = computer.address();
    mac = String.format("02:00:00:00:00:%02x", at & 0xff);
    otherMac = String.format("02:00:00:00:01:%02x", at & 0xff);
    log(0, 6, "", "kernel", "Booting Linux on physical CPU 0x0", "");
    log(2_000_000, 6, "systemd-journald.service", "systemd-journald", "Journal started", "");
    log(9_000_000, 6, "photonvision.service", "systemd", "Started photonvision.service", "");
    latest = health();
  }

  /** The robot's clock moves on: the agent answers if a poll is due. */
  public void step(long nowNanos) {
    this.nowNanos = nowNanos;
    if (nowNanos < nextPollNanos) {
      return;
    }
    nextPollNanos = nowNanos + periodNanos;
    polls++;
    if (unreachable) {
      answered = false;
      error = "connect timed out";
      return;
    }
    answered = true;
    error = "";
    everAnswered = true;
    answeredNanos = nowNanos;
    latest = health();
    macs.heard(nowNanos, latest.stamp().mac());
    takingTurns = macs.takingTurns();
  }

  @Override
  public void updateInputs(CoprocessorIOInputs inputs) {
    inputs.answered = answered;
    inputs.error = error;
    inputs.everAnswered = everAnswered;
    inputs.ageSeconds = everAnswered ? Math.max(0, (nowNanos - answeredNanos) / 1e9) : 0;
    if (everAnswered && filled != latest) {
      inputs.answeredNanos = answeredNanos;
      Answers.fill(inputs, latest);
      inputs.macsTakingTurns = takingTurns.toArray(String[]::new);
      filled = latest;
    }
  }

  /** Its health now, as its agent would answer. */
  private Health health() {
    double uptime = UP_BEFORE_SECONDS + nowNanos / 1e9;
    String version = wrongVersion ? OTHER_VERSION : table.photonvisionVersion();
    String settingsHash = table.settingsHash(computer);
    String liveHash = settingsChanged ? "changed-" + settingsHash : settingsHash;
    // Two boards answer by turns: each poll, the other one.
    String answering = twoBoards && polls % 2 == 0 ? otherMac : mac;
    Stamp stamp =
        new Stamp(
            computer.name(),
            table.table().team(),
            table.table().ip(computer),
            computer.board().id(),
            computer.cameras(),
            "simulated",
            table.recipeHash(),
            version,
            settingsHash,
            "sim-" + computer.name(),
            answering);
    boolean restarting = restarts > 0 && nowNanos - restartedAtNanos < RESTART_NANOS;
    return new Health(
        stamp,
        new Boot(
            stamp.bootId(), uptime, Math.round(uptime * 1e6), true, "/dev/nvme0n1p2", true, ""),
        cpu(uptime),
        thermal(uptime),
        new Service(
            "photonvision.service",
            restarting ? "activating" : "active",
            restarting ? "auto-restart" : "running",
            restarts > 0 ? "exit-code" : "success",
            restarts,
            Math.round((restarting || restarts == 0 ? 9 : uptime - 1) * 1e6)),
        cameras(),
        new JournalSummary(errorCounts, latestErrors()),
        new Drive("/dev/nvme0", 41 + Math.sin(uptime / 97), 1, 3, 40, 12, 0, 0),
        new SettingsState(settingsHash, liveHash),
        List.of());
  }

  /** Little cores lightly busy, the big ones running PhotonVision; capped when it's hot. */
  private Cpu cpu(double uptime) {
    boolean hotNow = !Double.isNaN(belowTripCelsius) && belowTripCelsius <= 0;
    int bigLimit = hotNow ? 1800 : BIG_MAX_MHZ;
    List<Double> busy = new ArrayList<>();
    for (int core = 0; core < 8; core++) {
      double wobble = 3 * Math.sin(uptime / 7 + core);
      busy.add(core < 4 ? 8 + wobble : 38 + wobble);
    }
    return new Cpu(
        periodNanos / 1e9,
        busy,
        List.of(
            new CpuCluster(List.of(0, 1, 2, 3), LITTLE_MAX_MHZ, LITTLE_MAX_MHZ, LITTLE_MAX_MHZ),
            new CpuCluster(List.of(4, 5), bigLimit, bigLimit, BIG_MAX_MHZ),
            new CpuCluster(List.of(6, 7), bigLimit, bigLimit, BIG_MAX_MHZ)));
  }

  /** The RK3588's zones, comfortably under their trip point unless it's made hot. */
  private List<ThermalZone> thermal(double uptime) {
    double big =
        Double.isNaN(belowTripCelsius)
            ? 56 + Math.sin(uptime / 31)
            : TRIP_CELSIUS - belowTripCelsius;
    List<TripPoint> trips =
        List.of(new TripPoint("passive", TRIP_CELSIUS), new TripPoint("critical", 115));
    return List.of(
        new ThermalZone("soc-thermal", big - 3, trips),
        new ThermalZone("bigcore0-thermal", big, trips),
        new ThermalZone("bigcore1-thermal", big - 1, trips),
        new ThermalZone("littlecore-thermal", big - 8, trips),
        new ThermalZone("center-thermal", big - 5, trips),
        new ThermalZone("gpu-thermal", big - 10, trips),
        new ThermalZone("npu-thermal", big - 9, trips));
  }

  /** Its cameras, each where its settings expect it, unless one is unplugged. */
  private Cameras cameras() {
    List<ExpectedCamera> expected = new ArrayList<>();
    List<UsbCamera> present = new ArrayList<>();
    List<String> names = computer.cameras();
    for (int i = 0; i < names.size(); i++) {
      String path = "platform-fc800000.usb-usb-0:1." + (i + 1) + ":1.0-video-index0";
      boolean plugged = i != unpluggedCamera;
      expected.add(new ExpectedCamera(names.get(i), path, plugged));
      if (plugged) {
        present.add(new UsbCamera(path, "1-1." + (i + 1), 5000, "32e4", "9422", "Arducam OV9281"));
      }
    }
    return new Cameras(expected, present);
  }

  private List<JournalEntry> latestErrors() {
    List<JournalEntry> errors =
        journal.stream().filter(entry -> !entry.category().isEmpty()).toList();
    return errors.subList(Math.max(0, errors.size() - 5), errors.size());
  }

  /** Adds an entry to its journal, at this many microseconds after its boot. */
  private void log(
      long monotonicMicros,
      int priority,
      String unit,
      String identifier,
      String message,
      String category) {
    journal.add(
        new JournalEntry(
            "sim-" + journal.size(),
            0,
            monotonicMicros,
            "sim-" + computer.name(),
            priority,
            unit,
            identifier,
            message,
            category));
    if (!category.isEmpty()) {
      errorCounts.merge(category, 1, Integer::sum);
    }
  }

  /** The agent's monotonic clock now, in microseconds since the simulated boot. */
  private long micros() {
    return Math.round((UP_BEFORE_SECONDS + nowNanos / 1e9) * 1e6);
  }

  // ---- Faults, as the simulator's controls apply them ----

  /** Its agent stops answering, as one unplugged or off the network. */
  public void unreachable(boolean on) {
    unreachable = on;
  }

  public boolean isUnreachable() {
    return unreachable;
  }

  /** Its image carries another PhotonVision, as a board left on last season's image would. */
  public void wrongVersion(boolean on) {
    wrongVersion = on;
  }

  public boolean hasWrongVersion() {
    return wrongVersion;
  }

  /**
   * Its big cores run this far below their trip point; at or past it, the kernel caps them.
   *
   * @param belowTripCelsius how far below; NaN for its usual temperature
   */
  public void heat(double belowTripCelsius) {
    this.belowTripCelsius = belowTripCelsius;
    if (!Double.isNaN(belowTripCelsius) && belowTripCelsius <= 0) {
      log(
          micros(),
          4,
          "",
          "kernel",
          "thermal thermal_zone1: critical temperature reached, throttling",
          "thermal");
    }
  }

  public boolean isHot() {
    return !Double.isNaN(belowTripCelsius);
  }

  /**
   * One of its cameras is unplugged.
   *
   * @param index which of its cameras, from 0; -1 for none
   */
  public void unplugCamera(int index) {
    if (index >= computer.cameras().size()) {
      throw new IllegalArgumentException(
          computer.name() + " has " + computer.cameras().size() + " cameras, not " + (index + 1));
    }
    if (index >= 0 && index != unpluggedCamera) {
      log(
          micros(),
          6,
          "",
          "kernel",
          "usb 1-1." + (index + 1) + ": USB disconnect, device number " + (index + 3),
          "usb");
    }
    unpluggedCamera = index;
  }

  public boolean isCameraUnplugged() {
    return unpluggedCamera >= 0;
  }

  /** How many cameras the table gives it. */
  public int cameraCount() {
    return computer.cameras().size();
  }

  /** PhotonVision exits, and systemd restarts it: it's back a few seconds later. */
  public void restartPhotonVision() {
    restarts++;
    restartedAtNanos = nowNanos;
    log(
        micros(),
        3,
        "photonvision.service",
        "systemd",
        "photonvision.service: Main process exited, code=exited, status=1/FAILURE",
        "error");
    log(
        micros() + 2_000_000,
        6,
        "photonvision.service",
        "systemd",
        "Started photonvision.service",
        "");
  }

  /** A second board with its image answers at its address too, by turns. */
  public void twoBoards(boolean on) {
    twoBoards = on;
  }

  public boolean hasTwoBoards() {
    return twoBoards;
  }

  /** Its PhotonVision settings change from those it was stamped with, as tuning in its UI does. */
  public void settingsChanged(boolean on) {
    settingsChanged = on;
  }

  public boolean hasSettingsChanged() {
    return settingsChanged;
  }

  /** Its latest answer, as its agent would serve it: for tests that serve it over HTTP. */
  Health answer() {
    return latest;
  }

  /** Its name, as the table gives it. */
  public String name() {
    return computer.name();
  }

  /** The PhotonVision version it would publish on NetworkTables, as it runs now. */
  String publishedVersion() {
    return latest.stamp().photonvisionVersion();
  }

  boolean isAnswering() {
    return !unreachable;
  }

  @Override
  public byte[] settingsBackup() throws IOException {
    if (unreachable) {
      throw new IOException("connect timed out");
    }
    ByteArrayOutputStream zip = new ByteArrayOutputStream();
    try (ZipOutputStream out = new ZipOutputStream(zip)) {
      out.putNextEntry(new ZipEntry("coprocessors/" + computer.name() + "/settings/README.txt"));
      out.write(
          ("A simulated coprocessor's settings backup: the simulator runs no PhotonVision, so"
                  + " there are no settings to back up.\n")
              .getBytes(StandardCharsets.UTF_8));
      out.closeEntry();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    return zip.toByteArray();
  }

  @Override
  public JournalPage journal(String cursor, int priority, String unit) throws IOException {
    if (unreachable) {
      throw new IOException("connect timed out");
    }
    int after = -1;
    if (!cursor.isEmpty()) {
      after = journal.stream().map(JournalEntry::cursor).toList().indexOf(cursor);
    }
    List<JournalEntry> matching = new ArrayList<>();
    for (int i = after + 1; i < journal.size(); i++) {
      JournalEntry entry = journal.get(i);
      if (entry.priority() <= priority && (unit.isEmpty() || unit.equals(entry.unit()))) {
        matching.add(entry);
      }
    }
    String last = matching.isEmpty() ? cursor : matching.get(matching.size() - 1).cursor();
    return new JournalPage(matching, last, false);
  }
}
