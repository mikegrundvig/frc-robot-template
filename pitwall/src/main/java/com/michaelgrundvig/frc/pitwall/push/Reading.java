package com.michaelgrundvig.frc.pitwall.push;

/**
 * A number the page shows to some resolution, such as the battery to the nearest 0.1 V:
 *
 * {@snippet :
 * Reading.of(battery).toNearest(0.1)
 * }
 *
 * <p>It counts as changed only once it has moved by at least its resolution from the value last
 * sent. Noise around a rounding edge (a battery sitting at 12.35 V) then sends nothing, where
 * comparing the rounded values would flip between 12.3 and 12.4 every loop. What the page shows
 * stays within one step of the truth.
 */
public final class Reading {
  private final double value;
  private final double resolution;

  private Reading(double value, double resolution) {
    if (!(resolution > 0) || !Double.isFinite(resolution)) {
      throw new IllegalArgumentException("a resolution is a positive number: " + resolution);
    }
    this.value = value;
    this.resolution = resolution;
  }

  /** The first step of a reading: its value. */
  public static Unrounded of(double value) {
    return resolution -> new Reading(value, resolution);
  }

  /** A value waiting for its resolution. */
  @FunctionalInterface
  public interface Unrounded {
    /**
     * @param resolution the smallest change worth sending, in the value's unit
     * @throws IllegalArgumentException if the resolution isn't a positive number
     */
    Reading toNearest(double resolution);
  }

  /** The value, unrounded. */
  public double value() {
    return value;
  }

  /** The smallest change worth sending. */
  public double resolution() {
    return resolution;
  }

  /**
   * Whether this differs enough from a value already sent to send again: by at least the
   * resolution, or by being or ceasing to be a number (NaN or infinite) at all.
   */
  public boolean differsFrom(Reading sent) {
    if (!Double.isFinite(value) || !Double.isFinite(sent.value)) {
      return Double.compare(value, sent.value) != 0;
    }
    return Math.abs(value - sent.value) >= resolution;
  }

  @Override
  public String toString() {
    return value + " (to the nearest " + resolution + ")";
  }
}
