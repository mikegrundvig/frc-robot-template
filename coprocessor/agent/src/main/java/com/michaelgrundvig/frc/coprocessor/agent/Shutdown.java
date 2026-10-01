package com.michaelgrundvig.frc.coprocessor.agent;

import java.io.IOException;
import java.time.Duration;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * The agent's one action: a soft power-off, so a coprocessor kept powered after the robot is
 * switched off (by a battery pack, say) shuts down cleanly rather than losing power mid-write. It
 * stops PhotonVision first, so its settings are saved, then powers the computer off. It happens
 * once: asking again while it's under way does nothing more.
 *
 * <p>The agent runs unprivileged; a polkit rule on the image lets its user stop PhotonVision's unit
 * and power off, and nothing else (coprocessor/README.md).
 */
final class Shutdown {
  /** How long PhotonVision may take to stop before the computer powers off anyway. */
  static final Duration STOP_TIMEOUT = Duration.ofSeconds(120);

  private final Host host;
  private final String unit;
  private final Executor executor;
  private final AtomicBoolean requested = new AtomicBoolean();

  Shutdown(Host host, String unit, Executor executor) {
    this.host = host;
    this.unit = unit;
    this.executor = executor;
  }

  /** Whether a shutdown has been asked for. */
  boolean requested() {
    return requested.get();
  }

  /**
   * Starts the shutdown, unless it's under way: whether this request started it. Logs who asked
   * before anything is done.
   */
  boolean request(String from) {
    if (!requested.compareAndSet(false, true)) {
      host.log("Shutdown asked for again by " + from + ": already shutting down");
      return false;
    }
    host.log("Shutdown asked for by " + from + ": stopping " + unit + ", then powering off");
    executor.execute(this::run);
    return true;
  }

  private void run() {
    try {
      Commands.Output stopped =
          host.commands().run(List.of("systemctl", "stop", unit), STOP_TIMEOUT, 16, 4096);
      if (stopped.exit() != 0) {
        host.log("Stopping " + unit + " failed (" + describe(stopped) + "); powering off anyway");
      }
    } catch (IOException e) {
      host.log("Stopping " + unit + " failed (" + e.getMessage() + "); powering off anyway");
    }
    try {
      Commands.Output off =
          host.commands().run(List.of("systemctl", "poweroff"), Duration.ofSeconds(30), 16, 4096);
      if (off.exit() != 0) {
        host.log("Powering off failed (" + describe(off) + ")");
      }
    } catch (IOException e) {
      host.log("Powering off failed (" + e.getMessage() + ")");
    }
  }

  private static String describe(Commands.Output output) {
    return output.timedOut() ? "timed out" : "exit " + output.exit();
  }
}
