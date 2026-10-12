package frc.robot.studies;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The estimator study's measures: how far one estimator's pose was from the truth over a run, all
 * computed from every loop's pose and the simulator's true pose.
 */
final class EstimatorScores {
  /** Back within this of the truth, in meters, is recovered. */
  static final double RECOVERED_METERS = 0.05;

  /** And within this heading, in degrees. */
  static final double RECOVERED_DEGREES = 1.0;

  /** Recovered means staying within both for this long: 0.2 s. */
  static final int RECOVERED_LOOPS = RobotLoop.loops(0.2);

  /** How long after a fault its own errors are measured: 2 s. */
  static final int AFTER_LOOPS = RobotLoop.loops(2.0);

  /** A stop's error at rest is the mean over this long at its end: 0.5 s. */
  static final int REST_LOOPS = RobotLoop.loops(0.5);

  private EstimatorScores() {}

  /**
   * What happened in a run, by loop, the same for every estimator in it.
   *
   * @param faults the loops in which a fault began
   * @param movedByHand the loops in which the simulator moved the robot at once (a shove, a hit, a
   *     carry): the truth jumps there, not the estimate, so they don't count as jumps
   * @param restsNear the last loop of each stop near a tag
   * @param restsFar the last loop of each stop far from every tag
   */
  record Marks(
      List<Integer> faults,
      List<Integer> movedByHand,
      List<Integer> restsNear,
      List<Integer> restsFar) {
    Marks() {
      this(new ArrayList<>(), new ArrayList<>(), new ArrayList<>(), new ArrayList<>());
    }
  }

  /** One estimator's pose and the truth, loop by loop. */
  static final class Track {
    private final List<Pose2d> estimates = new ArrayList<>();
    private final List<Pose2d> truths = new ArrayList<>();

    void add(Pose2d estimate, Pose2d truth) {
      estimates.add(estimate);
      truths.add(truth);
    }

    int loops() {
      return estimates.size();
    }

    /** Where the robot really was at a loop. */
    Pose2d truth(int loop) {
      return truths.get(loop);
    }

    /** How far the estimate is from the truth at a loop, in meters. */
    double meters(int loop) {
      return estimates.get(loop).getTranslation().getDistance(truths.get(loop).getTranslation());
    }

    /** How far its heading is from the truth's at a loop, in degrees, either way. */
    double degrees(int loop) {
      return Math.abs(
          estimates.get(loop).getRotation().minus(truths.get(loop).getRotation()).getDegrees());
    }

    /** How far the estimate moved in a loop beyond how far the robot did, in meters. */
    double jump(int loop) {
      if (loop == 0) {
        return 0;
      }
      Translation2d estimated =
          estimates.get(loop).getTranslation().minus(estimates.get(loop - 1).getTranslation());
      Translation2d moved =
          truths.get(loop).getTranslation().minus(truths.get(loop - 1).getTranslation());
      return estimated.minus(moved).getNorm();
    }
  }

  /**
   * One estimator's measures over one run. Positions in meters, headings in degrees, times in
   * seconds. Where a run has no fault or no stop, its measures are NaN; a recovery that never came
   * within the run is infinite.
   */
  record Measures(
      double rmsMeters,
      double p95Meters,
      double worstMeters,
      double rmsDegrees,
      double p95Degrees,
      double worstDegrees,
      double afterRmsMeters,
      double afterP95Meters,
      double afterWorstMeters,
      double afterRmsDegrees,
      double afterP95Degrees,
      double afterWorstDegrees,
      double recoverySeconds,
      double jumpMeters,
      double restNearMeters,
      double restNearDegrees,
      double restFarMeters) {
    /** The measures from their values in the report's order, as {@link #byName} gives them. */
    static Measures of(double[] v) {
      if (v.length != 17) {
        throw new IllegalArgumentException("a run has 17 measures, not " + v.length);
      }
      return new Measures(
          v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11], v[12], v[13],
          v[14], v[15], v[16]);
    }

    /** Each measure by its column's name, in the report's order. */
    Map<String, Double> byName() {
      Map<String, Double> named = new LinkedHashMap<>();
      named.put("rms_m", rmsMeters);
      named.put("p95_m", p95Meters);
      named.put("worst_m", worstMeters);
      named.put("rms_deg", rmsDegrees);
      named.put("p95_deg", p95Degrees);
      named.put("worst_deg", worstDegrees);
      named.put("after_rms_m", afterRmsMeters);
      named.put("after_p95_m", afterP95Meters);
      named.put("after_worst_m", afterWorstMeters);
      named.put("after_rms_deg", afterRmsDegrees);
      named.put("after_p95_deg", afterP95Degrees);
      named.put("after_worst_deg", afterWorstDegrees);
      named.put("recovery_s", recoverySeconds);
      named.put("jump_m", jumpMeters);
      named.put("rest_near_m", restNearMeters);
      named.put("rest_near_deg", restNearDegrees);
      named.put("rest_far_m", restFarMeters);
      return named;
    }
  }

  /** The measures' names, in the report's order. */
  static List<String> names() {
    double none = Double.NaN;
    return List.copyOf(
        new Measures(
                none, none, none, none, none, none, none, none, none, none, none, none, none, none,
                none, none, none)
            .byName()
            .keySet());
  }

  /** One estimator's measures over a run. */
  static Measures measure(Track track, Marks marks) {
    int loops = track.loops();
    double[] meters = new double[loops];
    double[] degrees = new double[loops];
    double jump = 0;
    for (int i = 0; i < loops; i++) {
      meters[i] = track.meters(i);
      degrees[i] = track.degrees(i);
      if (!marks.movedByHand().contains(i)) {
        jump = Math.max(jump, track.jump(i));
      }
    }
    List<Double> afterMeters = new ArrayList<>();
    List<Double> afterDegrees = new ArrayList<>();
    double recovery = marks.faults().isEmpty() ? Double.NaN : 0;
    for (int fault : marks.faults()) {
      for (int i = fault; i < Math.min(loops, fault + AFTER_LOOPS); i++) {
        afterMeters.add(meters[i]);
        afterDegrees.add(degrees[i]);
      }
      recovery = Math.max(recovery, recoverySeconds(meters, degrees, fault));
    }
    double[] am = afterMeters.stream().mapToDouble(Double::doubleValue).toArray();
    double[] ad = afterDegrees.stream().mapToDouble(Double::doubleValue).toArray();
    return new Measures(
        rms(meters),
        percentile(meters, 0.95),
        worst(meters),
        rms(degrees),
        percentile(degrees, 0.95),
        worst(degrees),
        rms(am),
        percentile(am, 0.95),
        worst(am),
        rms(ad),
        percentile(ad, 0.95),
        worst(ad),
        recovery,
        jump,
        atRest(meters, marks.restsNear()),
        atRest(degrees, marks.restsNear()),
        atRest(meters, marks.restsFar()));
  }

  /**
   * How long after a fault the estimate was back within 5 cm and 1° and stayed there 0.2 s: zero if
   * it never left, infinite if it never came back within the run.
   */
  static double recoverySeconds(double[] meters, double[] degrees, int fault) {
    int held = 0;
    for (int i = fault; i < meters.length; i++) {
      boolean within = meters[i] <= RECOVERED_METERS && degrees[i] <= RECOVERED_DEGREES;
      held = within ? held + 1 : 0;
      if (held >= RECOVERED_LOOPS) {
        return (i - RECOVERED_LOOPS + 1 - fault) * RobotLoop.PERIOD_SECONDS;
      }
    }
    return Double.POSITIVE_INFINITY;
  }

  /** The mean of a value over the last half second of each stop; NaN without a stop. */
  private static double atRest(double[] values, List<Integer> lastLoops) {
    double sum = 0;
    int count = 0;
    for (int last : lastLoops) {
      for (int i = Math.max(0, last - REST_LOOPS + 1);
          i <= Math.min(last, values.length - 1);
          i++) {
        sum += values[i];
        count++;
      }
    }
    return count == 0 ? Double.NaN : sum / count;
  }

  static double rms(double[] values) {
    if (values.length == 0) {
      return Double.NaN;
    }
    double sum = 0;
    for (double value : values) {
      sum += value * value;
    }
    return Math.sqrt(sum / values.length);
  }

  static double worst(double[] values) {
    return values.length == 0 ? Double.NaN : Arrays.stream(values).max().orElseThrow();
  }

  /** The value a share of the values are at or below: 0.5 is the median. NaN for none. */
  static double percentile(double[] values, double share) {
    if (values.length == 0) {
      return Double.NaN;
    }
    double[] sorted = values.clone();
    Arrays.sort(sorted);
    double at = share * (sorted.length - 1);
    int below = (int) Math.floor(at);
    int above = Math.min(sorted.length - 1, below + 1);
    if (Double.isInfinite(sorted[below]) || Double.isInfinite(sorted[above])) {
      // Between a number and "never", the nearer one.
      return at - below < 0.5 ? sorted[below] : sorted[above];
    }
    return sorted[below] + (at - below) * (sorted[above] - sorted[below]);
  }
}
