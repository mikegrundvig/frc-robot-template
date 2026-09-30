package com.michaelgrundvig.frc.pitwall.experiment;

import java.util.OptionalDouble;

/**
 * One signal's samples during a run: each a time on the robot's clock, in nanoseconds, and a value
 * in the signal's unit.
 *
 * <p>The thread recording the run appends; any other thread may read at the same time, up to {@link
 * #count()}. The count only grows, and a sample never changes once counted, so readers need no lock
 * and never slow the recording down.
 *
 * <p>A series holds a fixed number of samples, set when it's made, and stops recording when full. A
 * sample that isn't a finite number, or isn't later than the one before it, is dropped and counted:
 * a bad reading costs that reading, never the run.
 */
public final class Series {
  private final long[] times;
  private final double[] values;
  // Written after the sample it counts, so a reader that sees the count sees the sample.
  private volatile int count;
  private volatile int dropped;
  private volatile boolean full;

  /**
   * @param capacity how many samples it can hold
   * @throws IllegalArgumentException if the capacity isn't positive
   */
  public Series(int capacity) {
    if (capacity <= 0) {
      throw new IllegalArgumentException("a series holds at least one sample: " + capacity);
    }
    times = new long[capacity];
    values = new double[capacity];
  }

  /**
   * Records a sample. Call it only from the thread recording the run.
   *
   * @param timeNanos when it was measured, on the robot's clock
   * @param value what was measured, in the signal's unit
   * @return whether it was kept: false when the value isn't finite, the time isn't later than the
   *     last sample's, or the series is full
   */
  public boolean add(long timeNanos, double value) {
    int n = count;
    if (!Double.isFinite(value) || (n > 0 && timeNanos <= times[n - 1])) {
      dropped++;
      return false;
    }
    if (n == times.length) {
      full = true;
      dropped++;
      return false;
    }
    times[n] = timeNanos;
    values[n] = value;
    count = n + 1;
    return true;
  }

  /** How many samples it holds. Safe from any thread; it only grows. */
  public int count() {
    return count;
  }

  /** How many it can hold. */
  public int capacity() {
    return times.length;
  }

  /** How many samples were dropped: not finite, out of order, or past its capacity. */
  public int dropped() {
    return dropped;
  }

  /** Whether it ran out of room, so the run's end wasn't recorded. */
  public boolean full() {
    return full;
  }

  /**
   * When sample {@code i} was measured, on the robot's clock in nanoseconds.
   *
   * @throws IndexOutOfBoundsException unless {@code 0 <= i < count()}
   */
  public long time(int i) {
    return times[checked(i)];
  }

  /**
   * Sample {@code i}'s value.
   *
   * @throws IndexOutOfBoundsException unless {@code 0 <= i < count()}
   */
  public double value(int i) {
    return values[checked(i)];
  }

  /**
   * The largest magnitude among the samples from {@code startNanos} to {@code endNanos}, inclusive,
   * such as the peak current during a step; empty when none fall in that span.
   */
  public OptionalDouble peakMagnitude(long startNanos, long endNanos) {
    int n = count;
    double peak = -1;
    for (int i = 0; i < n; i++) {
      if (times[i] >= startNanos && times[i] <= endNanos) {
        peak = Math.max(peak, Math.abs(values[i]));
      }
    }
    return peak < 0 ? OptionalDouble.empty() : OptionalDouble.of(peak);
  }

  private int checked(int i) {
    int n = count;
    if (i < 0 || i >= n) {
      throw new IndexOutOfBoundsException("sample " + i + " of " + n);
    }
    return i;
  }
}
