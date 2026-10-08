package com.michaelgrundvig.frc.pitwall;

import java.time.Duration;
import java.util.concurrent.locks.LockSupport;
import java.util.function.BooleanSupplier;

/**
 * Waits for what another thread does, such as the robot loop a test plays on a timer: a condition,
 * checked until it holds, for 10 s at most. A test waits for the state it needs, never for a time
 * that's usually enough: on a loaded machine, no time is.
 */
final class Await {
  /** Far longer than anything here takes: a test that reaches it has found a bug. */
  static final Duration DEADLINE = Duration.ofSeconds(10);

  private Await() {}

  /**
   * Returns once {@code condition} holds.
   *
   * @param what what's awaited, for the failure: "the field system connected"
   * @throws AssertionError if it doesn't hold within {@link #DEADLINE}
   */
  static void until(String what, BooleanSupplier condition) {
    long deadline = System.nanoTime() + DEADLINE.toNanos();
    while (!condition.getAsBoolean()) {
      if (System.nanoTime() - deadline > 0) {
        throw new AssertionError("Waited " + DEADLINE.toSeconds() + " s for " + what);
      }
      LockSupport.parkNanos(1_000_000); // a millisecond, against a loop every 5 ms
    }
  }
}
