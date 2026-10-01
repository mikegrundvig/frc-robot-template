package com.michaelgrundvig.frc.pitwall.experiment;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.units.Unit;

/**
 * How a mechanism answered one jump in its setpoint: the scores that tell one run from another.
 *
 * <p>Times are measured from the jump. The rise and the settling moment are interpolated between
 * samples, so a 50 Hz measurement still times a fast rise closely.
 *
 * @param riseSeconds from 10% to 90% of the way there; empty if it never got 90% of the way
 * @param overshoot how far past the target it went, as a fraction of the jump (0.25 is 25%); 0 if
 *     it never passed it
 * @param settlingSeconds until it stayed within {@link #SETTLING_BAND} of the jump around the
 *     target, for good; empty if it left the band during the window's last tenth (the tail the
 *     steady-state error averages over), since a mechanism still buzzing then hasn't settled,
 *     wherever its last sample happens to fall
 * @param steadyStateError how far short of the target it stayed, averaged over the last tenth of
 *     the window, in the measurement's unit: positive when short, negative when past
 * @param samples how many samples the window held
 */
public record StepResponse(
    OptionalDouble riseSeconds,
    double overshoot,
    OptionalDouble settlingSeconds,
    double steadyStateError,
    int samples) {
  /** How close counts as settled: within 2% of the jump, either side of the target. */
  public static final double SETTLING_BAND = 0.02;

  /**
   * Scores each jump in a run: each from its jump to the next one, or to the end of the run.
   *
   * @param measured what the mechanism did, in the profile's base unit
   * @param profile the motion the run commanded
   * @param runStartNanos when the run started, on the robot's clock
   * @return one entry per edge, in order; empty where the window held fewer than two samples
   */
  public static List<Optional<StepResponse>> of(
      Series measured, Profile profile, long runStartNanos) {
    return of(measured, profile, profile.baseUnit(), runStartNanos);
  }

  /**
   * Scores each jump in a run, measured in any unit of the profile's kind: degrees for a profile in
   * radians, say. The steady-state error comes out in that unit.
   *
   * @param measured what the mechanism did, in {@code unit}
   * @param unit a unit whose base unit is the profile's
   */
  public static List<Optional<StepResponse>> of(
      Series measured, Profile profile, Unit unit, long runStartNanos) {
    List<Profile.Edge> edges = profile.edges();
    List<Optional<StepResponse>> scores = new ArrayList<>();
    for (int e = 0; e < edges.size(); e++) {
      Profile.Edge edge = edges.get(e);
      double endSeconds =
          e + 1 < edges.size() ? edges.get(e + 1).atSeconds() : profile.durationSeconds();
      scores.add(
          of(
              measured,
              new Profile.Edge(
                  edge.atSeconds(), unit.fromBaseUnits(edge.from()), unit.fromBaseUnits(edge.to())),
              runStartNanos + nanos(edge.atSeconds()),
              runStartNanos + nanos(endSeconds)));
    }
    return scores;
  }

  /**
   * Scores one jump.
   *
   * @param measured what the mechanism did, in the unit of the edge's setpoints
   * @param edge the jump
   * @param edgeNanos when it happened, on the robot's clock
   * @param endNanos the end of the window to score, on the robot's clock
   * @return empty when the window held fewer than two samples
   */
  public static Optional<StepResponse> of(
      Series measured, Profile.Edge edge, long edgeNanos, long endNanos) {
    int first = -1;
    int last = -1;
    int n = measured.count();
    for (int i = 0; i < n; i++) {
      long t = measured.time(i);
      if (t >= edgeNanos && t <= endNanos) {
        if (first < 0) {
          first = i;
        }
        last = i;
      }
    }
    if (first < 0 || last == first) {
      return Optional.empty();
    }
    double span = edge.to() - edge.from();
    Progress progress = i -> (measured.value(i) - edge.from()) / span;

    OptionalDouble t10 = crossing(measured, progress, first, last, 0.1);
    OptionalDouble t90 = crossing(measured, progress, first, last, 0.9);
    OptionalDouble rise =
        t10.isPresent() && t90.isPresent()
            ? OptionalDouble.of(t90.getAsDouble() - t10.getAsDouble())
            : OptionalDouble.empty();

    double highest = Double.NEGATIVE_INFINITY;
    for (int i = first; i <= last; i++) {
      highest = Math.max(highest, progress.at(i));
    }

    // The window's last tenth: where the steady-state error is averaged, and where a mechanism that
    // has settled stays in the band.
    double tailStart = endNanos - 0.1 * (endNanos - edgeNanos);
    double shortfall = 0;
    int tail = 0;
    for (int i = first; i <= last; i++) {
      if (measured.time(i) >= tailStart) {
        shortfall += (edge.to() - measured.value(i)) * Math.signum(span);
        tail++;
      }
    }
    if (tail == 0) {
      shortfall = (edge.to() - measured.value(last)) * Math.signum(span);
      tail = 1;
    }

    return Optional.of(
        new StepResponse(
            rise,
            Math.max(0, highest - 1),
            settling(measured, progress, first, last, edgeNanos, tailStart),
            shortfall / tail,
            last - first + 1));
  }

  /** Progress toward the target at a sample: 0 at the start, 1 at the target. */
  @FunctionalInterface
  private interface Progress {
    double at(int sample);
  }

  /** When progress first reached {@code level}, in seconds on the robot's clock; interpolated. */
  private static OptionalDouble crossing(
      Series measured, Progress progress, int first, int last, double level) {
    for (int i = first; i <= last; i++) {
      if (progress.at(i) >= level) {
        if (i == first) {
          return OptionalDouble.of(seconds(measured.time(i)));
        }
        double before = progress.at(i - 1);
        double fraction = (level - before) / (progress.at(i) - before);
        return OptionalDouble.of(
            seconds(measured.time(i - 1))
                + fraction * seconds(measured.time(i) - measured.time(i - 1)));
      }
    }
    return OptionalDouble.empty();
  }

  /**
   * Seconds from the jump until it stayed within the band for good; interpolated. Empty if it was
   * outside the band at any sample from {@code tailStart} on, or at the last sample.
   */
  private static OptionalDouble settling(
      Series measured, Progress progress, int first, int last, long edgeNanos, double tailStart) {
    int outside = -1;
    for (int i = last; i >= first; i--) {
      if (Math.abs(progress.at(i) - 1) > SETTLING_BAND) {
        outside = i;
        break;
      }
    }
    if (outside == last || (outside >= 0 && measured.time(outside) >= tailStart)) {
      return OptionalDouble.empty();
    }
    if (outside < 0) {
      return OptionalDouble.of(seconds(measured.time(first) - edgeNanos));
    }
    double out = progress.at(outside);
    double in = progress.at(outside + 1);
    double edgeOfBand = 1 + Math.copySign(SETTLING_BAND, out - 1);
    double fraction = (edgeOfBand - out) / (in - out);
    long from = measured.time(outside);
    long to = measured.time(outside + 1);
    return OptionalDouble.of(seconds(from - edgeNanos) + fraction * seconds(to - from));
  }

  private static double seconds(long nanos) {
    return nanos / 1e9;
  }

  private static long nanos(double seconds) {
    return Math.round(seconds * 1e9);
  }
}
