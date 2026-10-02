package frc.robot.subsystems.coprocessors;

import com.michaelgrundvig.frc.coprocessor.api.CpuCluster;
import com.michaelgrundvig.frc.coprocessor.api.Drive;
import com.michaelgrundvig.frc.coprocessor.api.ExpectedCamera;
import com.michaelgrundvig.frc.coprocessor.api.Health;
import com.michaelgrundvig.frc.coprocessor.api.JournalEntry;
import com.michaelgrundvig.frc.coprocessor.api.Stamp;
import com.michaelgrundvig.frc.coprocessor.api.ThermalZone;
import com.michaelgrundvig.frc.coprocessor.api.TripPoint;
import frc.robot.subsystems.coprocessors.CoprocessorIO.CoprocessorIOInputs;
import java.util.List;
import java.util.Locale;

/**
 * An agent's answer as the robot logs it: the stamp and health flattened into a coprocessor's
 * inputs, and the stamp back out of them, so a replay judges a coprocessor from its log exactly as
 * the robot did. Every IO fills its inputs here, the simulated agent's included.
 */
final class Answers {
  private Answers() {}

  /** Fills the inputs with an answer's stamp and health; the poll's own fields are the IO's. */
  static void fill(CoprocessorIOInputs inputs, Health health) {
    Stamp stamp = health.stamp();
    inputs.name = stamp.name();
    inputs.team = stamp.team();
    inputs.address = stamp.address();
    inputs.board = stamp.board();
    inputs.role = stamp.cameras().toArray(String[]::new);
    inputs.release = stamp.release();
    inputs.recipeHash = stamp.recipeHash();
    inputs.photonVisionVersion = stamp.photonvisionVersion();
    inputs.stampedSettingsHash = stamp.settingsHash();
    inputs.mac = stamp.mac();

    inputs.bootId = health.boot().bootId();
    inputs.agentMonotonicNanos = health.boot().monotonicMicros() * 1000;
    inputs.uptimeSeconds = health.boot().uptimeSeconds();
    Boolean clean = health.boot().lastShutdownClean();
    inputs.lastShutdownKnown = clean != null;
    inputs.lastShutdownClean = clean == null || clean;
    inputs.rootDevice = health.boot().rootDevice();
    inputs.rootReadOnly = health.boot().rootReadOnly();

    List<Double> busy = health.cpu().busyPercent();
    inputs.coreBusyPercent = busy.stream().mapToDouble(Double::doubleValue).toArray();
    // The big cores are the clusters that run fastest at most: the RK3588's A76s. The slowest of
    // them now is the one that shows: one throttled is enough.
    List<CpuCluster> big = health.cpu().bigClusters();
    double busiest = 0;
    double current = 0;
    double limit = 0;
    double most = 0;
    for (int i = 0; i < big.size(); i++) {
      CpuCluster cluster = big.get(i);
      for (int core : cluster.cores()) {
        if (core >= 0 && core < busy.size()) {
          busiest = Math.max(busiest, busy.get(core));
        }
      }
      current = i == 0 ? cluster.currentMhz() : Math.min(current, cluster.currentMhz());
      limit = i == 0 ? cluster.limitMhz() : Math.min(limit, cluster.limitMhz());
      most = cluster.maxMhz();
    }
    inputs.bigCoreBusyPercent = busiest;
    inputs.bigCoreMhz = current;
    inputs.bigCoreLimitMhz = limit;
    inputs.bigCoreMaxMhz = most;

    List<ThermalZone> zones = health.thermal();
    inputs.zones = zones.stream().map(ThermalZone::type).toArray(String[]::new);
    inputs.zoneCelsius = zones.stream().mapToDouble(ThermalZone::celsius).toArray();
    inputs.zoneTripCelsius = zones.stream().mapToDouble(Answers::firstPassiveTrip).toArray();

    inputs.photonVisionState =
        health.photonvision().activeState()
            + (health.photonvision().subState().isEmpty()
                ? ""
                : "/" + health.photonvision().subState());
    inputs.photonVisionRestarts = health.photonvision().restarts();
    inputs.camerasPresent =
        health.cameras().expected().stream()
            .filter(ExpectedCamera::present)
            .map(ExpectedCamera::name)
            .toArray(String[]::new);
    inputs.usbCameras =
        health.cameras().present().stream()
            .map(
                usb ->
                    String.format(
                        Locale.ROOT,
                        "%s %s, %.0f Mb/s",
                        usb.port(),
                        usb.product().isEmpty()
                            ? usb.vendorId() + ":" + usb.productId()
                            : usb.product(),
                        usb.speedMbps()))
            .toArray(String[]::new);

    inputs.journalErrors =
        health.journal().counts().values().stream().mapToInt(Integer::intValue).sum();
    inputs.journalCategories = health.journal().counts().keySet().toArray(String[]::new);
    inputs.journalCounts =
        health.journal().counts().values().stream().mapToLong(Integer::longValue).toArray();
    inputs.latestErrors =
        health.journal().latest().stream().map(Answers::said).toArray(String[]::new);
    Drive drive = health.drive();
    inputs.driveFound = drive != null;
    inputs.driveCelsius = drive != null ? drive.celsius() : 0;
    inputs.drivePercentUsed = drive != null ? drive.percentUsed() : 0;
    inputs.unsafeShutdowns = drive != null ? drive.unsafeShutdowns() : 0;
    inputs.settingsHash = health.settings().liveHash();
    inputs.problems = health.problems().toArray(String[]::new);
  }

  /** The stamp, as the inputs logged it, with this boot and MAC. */
  static Stamp stamp(CoprocessorIOInputs inputs) {
    return new Stamp(
        inputs.name,
        inputs.team,
        inputs.address,
        inputs.board,
        List.of(inputs.role),
        inputs.release,
        inputs.recipeHash,
        inputs.photonVisionVersion,
        inputs.stampedSettingsHash,
        inputs.bootId,
        inputs.mac);
  }

  /** A zone's first passive trip point, where the kernel slows the cores; else its first. */
  static double firstPassiveTrip(ThermalZone zone) {
    List<TripPoint> trips = zone.trips();
    for (TripPoint trip : trips) {
      if (trip.type().equals("passive")) {
        return trip.celsius();
      }
    }
    return trips.isEmpty() ? Double.NaN : trips.get(0).celsius();
  }

  /** A journal entry in a line: what logged it, and what it said. */
  static String said(JournalEntry entry) {
    String who = entry.unit().isEmpty() ? entry.identifier() : entry.unit();
    return (who.isEmpty() ? "" : who + ": ") + entry.message();
  }
}
