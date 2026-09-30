package frc.robot;

import frc.robot.lib.timing.LoopMonitor;
import frc.robot.lib.timing.RobotLoop;
import java.io.File;
import java.util.Locale;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.util.Alert;

/**
 * Whether the robot loop keeps up, and its log with it. {@link Robot} starts and finishes each loop
 * here; the container marks its sections on {@link #monitor()}.
 *
 * <p>It raises three alerts:
 *
 * <ul>
 *   <li>loops that overran, in the robot's own code, or started late, from time lost between loops
 *       (writing the log, dashboards, garbage collection), naming where the time went;
 *   <li>a log that fell behind and dropped data: that log has gaps, so the alert stays raised for
 *       the rest of the session;
 *   <li>a log drive that's missing, can't be written, or is nearly full, checked every second away
 *       from the robot loop, so a drive that hangs never stalls it.
 * </ul>
 */
final class LoopHealth implements AutoCloseable {
  /** How long "lately" covers. */
  private static final double RECENT_SECONDS = 10;

  /** A log drive with less room than this is nearly full: about half an hour of logging. */
  static final long MIN_FREE_BYTES = 500L * 1024 * 1024;

  private final LoopMonitor monitor =
      new LoopMonitor(RobotLoop.PERIOD_SECONDS, RobotLoop.loops(RECENT_SECONDS));
  private final Alert overrunning =
      new Alert("Loop", "overrunning", "The loop is overrunning", Alert.Level.MEDIUM);
  private final Alert loggingBehind =
      new Alert(
          "Loop",
          "loggingBehind",
          "Logging fell behind and dropped data this session: this log has gaps",
          Alert.Level.HIGH);
  private final Alert logDrive = new Alert("Loop", "logDrive", "", Alert.Level.HIGH);
  private @Nullable ScheduledExecutorService driveWatch;
  private volatile @Nullable String logDriveProblem;
  private LoopMonitor.Report last = LoopMonitor.Report.NONE;
  private String lastOverrun = "";

  /** The loop's timing, for marking the sections of it. */
  LoopMonitor monitor() {
    return monitor;
  }

  /** A loop begins. */
  void start(long nowNanos) {
    monitor.start(nowNanos);
  }

  /**
   * The loop is done: logs its timing section by section, and updates the alerts.
   *
   * @param logDropped whether the log dropped data this loop (AdvantageKit's queue to its writers
   *     was full)
   */
  void finish(long nowNanos, boolean logDropped) {
    last = monitor.finish(nowNanos);
    Logger.recordOutput("Loop/Milliseconds", last.seconds() * 1000);
    Logger.recordOutput("Loop/OverrunsLately", last.overrunsLately());
    Logger.recordOutput("Loop/WorstLatelyMilliseconds", last.worstLatelySeconds() * 1000);
    Logger.recordOutput("Loop/LateLately", last.lateLately());
    Logger.recordOutput("Loop/OutsideMilliseconds", last.outsideSeconds() * 1000);
    for (LoopMonitor.Section section : last.sections()) {
      Logger.recordOutput(
          "Loop/Sections/" + section.name() + "Milliseconds", section.seconds() * 1000);
    }
    LoopMonitor.Section slowest = last.slowest();
    if (last.overran() && slowest != null) {
      lastOverrun =
          format(
              "; the last took %.0f ms, %.0f of them in %s",
              last.seconds() * 1000, slowest.seconds() * 1000, slowest.name());
    }
    boolean trouble = last.overrunsLately() > 0 || last.lateLately() > 0;
    if (trouble) {
      overrunning.setText(overrunText());
    }
    overrunning.set(trouble);
    if (logDropped) {
      loggingBehind.set(true); // stays raised: the log has gaps
    }
    String driveProblem = logDriveProblem;
    if (driveProblem != null && !driveProblem.equals(logDrive.getText())) {
      logDrive.setText(driveProblem);
    }
    logDrive.set(driveProblem != null);
  }

  private String overrunText() {
    String overran =
        last.overrunsLately() == 0
            ? ""
            : format(
                "The loop overran %.0f ms %d %s in the last %.0f s%s",
                RobotLoop.PERIOD_SECONDS * 1000,
                last.overrunsLately(),
                last.overrunsLately() == 1 ? "time" : "times",
                RECENT_SECONDS,
                lastOverrun);
    String late =
        last.lateLately() == 0
            ? ""
            : format(
                "%d %s started late in the last %.0f s: time lost between loops (writing the log,"
                    + " dashboards, garbage collection)",
                last.lateLately(), last.lateLately() == 1 ? "loop" : "loops", RECENT_SECONDS);
    return overran.isEmpty() ? late : late.isEmpty() ? overran : overran + "; " + late;
  }

  /** One line for the pit: how long loops take, and whether they keep up. */
  String describe() {
    return last.describe();
  }

  /**
   * Watches the drive the log goes to: now, then every second on a thread of its own, since a USB
   * drive that hangs would otherwise hang the robot loop. On the robot only.
   */
  void watchLogDrive(File drive) {
    logDriveProblem = logDriveProblem(drive, MIN_FREE_BYTES);
    ScheduledExecutorService watch =
        Executors.newSingleThreadScheduledExecutor(
            task -> {
              Thread thread = new Thread(task, "Log drive watch");
              thread.setDaemon(true);
              return thread;
            });
    watch.scheduleWithFixedDelay(
        () -> logDriveProblem = logDriveProblem(drive, MIN_FREE_BYTES), 1, 1, TimeUnit.SECONDS);
    driveWatch = watch;
  }

  /**
   * Why a session isn't being logged, or soon won't be: no drive where the log goes, one that can't
   * be written, or one nearly full.
   *
   * @return null when the drive is there, writable, and has room
   */
  static @Nullable String logDriveProblem(File drive, long minimumFreeBytes) {
    if (!drive.isDirectory()) {
      return "No USB drive at " + drive + ": this session isn't being logged";
    }
    if (!drive.canWrite()) {
      return "The USB drive at " + drive + " can't be written: this session isn't being logged";
    }
    long free = drive.getUsableSpace();
    if (free < minimumFreeBytes) {
      return format(
          "The USB drive at %s has only %d MB free: logging stops when it's full",
          drive, free >> 20);
    }
    return null;
  }

  boolean isOverrunAlertActive() {
    return overrunning.get();
  }

  String overrunAlertText() {
    return overrunning.getText();
  }

  boolean isLoggingBehindAlertActive() {
    return loggingBehind.get();
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }

  @Override
  public void close() {
    ScheduledExecutorService watch = driveWatch;
    if (watch != null) {
      watch.shutdownNow();
    }
    overrunning.close();
    loggingBehind.close();
    logDrive.close();
  }
}
