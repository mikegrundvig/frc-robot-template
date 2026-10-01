package com.michaelgrundvig.frc.coprocessor.agent;

import com.michaelgrundvig.frc.coprocessor.api.AgentApi;
import com.michaelgrundvig.frc.coprocessor.api.Boot;
import com.michaelgrundvig.frc.coprocessor.api.Cameras;
import com.michaelgrundvig.frc.coprocessor.api.Cpu;
import com.michaelgrundvig.frc.coprocessor.api.Drive;
import com.michaelgrundvig.frc.coprocessor.api.Health;
import com.michaelgrundvig.frc.coprocessor.api.JournalPage;
import com.michaelgrundvig.frc.coprocessor.api.JournalSummary;
import com.michaelgrundvig.frc.coprocessor.api.Service;
import com.michaelgrundvig.frc.coprocessor.api.SettingsState;
import com.michaelgrundvig.frc.coprocessor.api.Stamp;
import com.michaelgrundvig.frc.coprocessor.api.ThermalZone;
import com.michaelgrundvig.frc.coprocessor.table.Table;
import java.io.IOException;
import java.time.Duration;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.Executor;
import java.util.function.IntFunction;
import org.jspecify.annotations.Nullable;

/**
 * What the agent knows, assembled from its sources: the answers to the API's requests. A source
 * that fails costs only its part, which reads as unknown, with the failure in the health's
 * problems; nothing one source does stops the others.
 */
final class Agent {
  /** Health asked for again within this long is answered from the last reading. */
  static final long HEALTH_MICROS = 500_000;

  /** The most problems a health answer carries. */
  static final int MAX_PROBLEMS = 10;

  private final Host host;
  private final StampSource stamp;
  private final BootSource boot;
  private final CpuSource cpu;
  private final ThermalSource thermal;
  private final ServiceSource service;
  private final CameraSource cameras;
  private final JournalSource journal;
  private final DriveSource drive;
  private final SettingsSource settings;
  private final Shutdown shutdown;
  private final IntFunction<String> controller;
  private @Nullable Health lastHealth;
  private long lastHealthMicros;
  // Each part's last problem, so a problem that lasts is logged once, not every second.
  private final Map<String, String> logged = new HashMap<>();

  /**
   * An agent reporting on a host.
   *
   * @param host the computer
   * @param database PhotonVision's settings database, as a path on the computer
   * @param unit PhotonVision's systemd unit
   * @param timeout how long any one command may take
   * @param executor where the shutdown runs, once its request is answered
   * @param controller the robot controller's address, from the team number: 10.TE.AM.2
   */
  Agent(
      Host host,
      String database,
      String unit,
      Duration timeout,
      Executor executor,
      IntFunction<String> controller) {
    this.host = host;
    this.controller = controller;
    this.stamp = new StampSource(host);
    this.journal = new JournalSource(host, timeout);
    this.boot = new BootSource(host, stamp, journal);
    this.cpu = new CpuSource(host);
    this.thermal = new ThermalSource(host);
    this.service = new ServiceSource(host, unit, timeout);
    this.cameras = new CameraSource(host);
    this.drive = new DriveSource(host);
    this.settings = new SettingsSource(host, database);
    this.shutdown = new Shutdown(host, unit, executor);
  }

  /** The stamp, with this boot and the MAC address. */
  Stamp stamp() throws IOException {
    return stamp.stamp();
  }

  /** The stamp file as the image wrote it, with the port it says to serve on. */
  StampSource.StampFile stampFile() throws IOException {
    return stamp.file();
  }

  /** The computer's health; asked again within half a second, the same answer. */
  synchronized Health health() {
    long now = host.monotonicMicros();
    Health last = lastHealth;
    if (last != null && now - lastHealthMicros < HEALTH_MICROS) {
      return last;
    }
    List<String> problems = new ArrayList<>();
    Stamp stamped = part("stamp", stamp::stamp, Stamp.NONE, problems);
    Boot booted =
        part(
            "boot",
            boot::read,
            new Boot(stamped.bootId(), Double.NaN, now, null, "", false, ""),
            problems);
    Cpu load = part("cpu", cpu::read, Cpu.UNKNOWN, problems);
    List<ThermalZone> zones = part("thermal", thermal::read, List.of(), problems);
    Service photonvision =
        part("photonvision", service::read, new Service("", "", "", "", 0, 0), problems);
    Optional<SettingsSource.Snapshot> live =
        part("settings", settings::read, Optional.empty(), problems);
    Map<String, String> usbPaths = live.map(SettingsSource.Snapshot::usbPaths).orElse(Map.of());
    Cameras plugged =
        part("cameras", () -> cameras.read(stamped.cameras(), usbPaths), Cameras.UNKNOWN, problems);
    JournalSummary trouble =
        part("journal", () -> journal.summary(stamped.bootId()), JournalSummary.EMPTY, problems);
    Optional<Drive> nvme = part("drive", drive::read, Optional.empty(), problems);
    if (shutdown.requested()) {
      problems.add(0, "shutting down: asked for by the robot");
    }
    Health health =
        new Health(
            stamped,
            booted,
            load,
            zones,
            photonvision,
            plugged,
            trouble,
            nvme.orElse(null),
            new SettingsState(
                stamped.settingsHash(), live.map(SettingsSource.Snapshot::hash).orElse("")),
            problems.subList(0, Math.min(problems.size(), MAX_PROBLEMS)));
    lastHealth = health;
    lastHealthMicros = now;
    return health;
  }

  /** A page of the journal. */
  JournalPage journal(String cursor, int priority, String unit, int limit) throws IOException {
    return journal.page(cursor, priority, unit, limit);
  }

  /** PhotonVision's live settings; empty when it has no database yet. */
  Optional<SettingsSource.Snapshot> settings() throws IOException {
    return settings.read();
  }

  /**
   * Asks for a shutdown on behalf of {@code from}: whether this request started it. Only the robot
   * controller, 10.TE.AM.2, may ask; the caller checks with {@link #controller}.
   */
  boolean shutdown(String from) {
    return shutdown.request(from);
  }

  /** The robot controller's address on this computer's team's network: 10.TE.AM.2. */
  String controller() throws IOException {
    return controller.apply(stamp.file().stamp().team());
  }

  /** The robot controller's address on a team's network: 10.TE.AM.2. */
  static String controllerOf(int team) {
    return Table.ip(team, AgentApi.CONTROLLER);
  }

  private interface Reading<T> {
    T read() throws IOException;
  }

  private <T> T part(String name, Reading<T> reading, T unknown, List<String> problems) {
    try {
      T read = reading.read();
      logged.remove(name);
      return read;
    } catch (IOException | RuntimeException e) {
      String problem = name + ": " + e.getMessage();
      problems.add(problem);
      if (!problem.equals(logged.put(name, problem))) {
        host.log("Couldn't read " + name + ": " + e);
      }
      return unknown;
    }
  }
}
