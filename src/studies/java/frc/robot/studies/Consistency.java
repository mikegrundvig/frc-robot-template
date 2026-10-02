package frc.robot.studies;

import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;
import org.wpilib.math.geometry.Pose2d;

/**
 * Whether an estimator's uncertainty tells the truth, measured where the truth is known: in the
 * simulator. An estimate that claims 3 mm while it's 10 cm off misleads everything that trusts it
 * (a scoring gate, a χ² test, a bound on the error), so this is checked before any of those.
 *
 * <p>Runs of one situation are added one at a time, each a sample per loop. Every run must have the
 * same loops, so that loop by loop, the runs are draws of the same moment. Three measures, from
 * Bar-Shalom, Li, and Kirubarajan (2001):
 *
 * <ul>
 *   <li><b>NEES</b>, the normalized estimation error squared: the error weighted by the inverse of
 *       the claimed covariance, eᵀP⁻¹e. For an honest estimator it averages the number of
 *       dimensions: 2 for position, 1 for heading. Averaged over N runs at each loop, it falls
 *       inside a band 95% of the time ({@link #band}); above the band the estimator is
 *       overconfident, below it timid (it trusts its measurements less than it could).
 *   <li><b>Exceedance,</b> two ways. How often the position error is outside the claimed
 *       covariance's 99.9% ellipse: its NEES past {@value #BOUND_TEXT}² = 13.8, the χ² of two
 *       dimensions at 0.999. For an honest estimator, once in a thousand loops. And how often it
 *       goes past {@value #BOUND_TEXT} times the claimed spread along its least sure direction, the
 *       circle a bound on the error would use: that circle holds the ellipse, so for an honest
 *       estimator it's at most once in a thousand loops, and fewer the longer the ellipse. The
 *       ellipse catches an estimator that claims too little doubt along its short axis, which the
 *       circle never sees.
 *   <li><b>The worst ratio</b> of the error to the long axis's spread, over every loop.
 * </ul>
 *
 * <p>A run's loops aren't independent draws. What a seed draws once (its gyro's scale, its cameras'
 * mounts, its tags' places) lasts the whole run, so a seed that drew a gyro 2.4 σ off is off by
 * about that much all run, and every loop of it can land outside the ellipse together. So the
 * seeds, not the loops, are the sample: how far a mean NEES may stray ({@link #band(int, int,
 * double)}) and how many exceedances to allow ({@link #allowedShare}) count each run as one draw.
 *
 * <p>Beside them, how accurate the estimate was: an honest estimator that got there by being worse
 * would be no improvement.
 */
public final class Consistency {
  /** How often an honest estimate's error may go past the bound: once in a thousand loops. */
  public static final double EXCEEDANCE = 0.001;

  /**
   * The bound, in standard deviations along the least sure direction: a two-dimensional Gaussian
   * puts e^(-k²/2) of itself beyond k of them, so k = √(-2 ln 0.001) = 3.72 for a thousandth.
   */
  public static final double BOUND = Math.sqrt(-2 * Math.log(EXCEEDANCE));

  static final String BOUND_TEXT = "3.72";

  /** The ellipse's bound on a position NEES: χ² of two dimensions at 0.999, 3.72 squared. */
  public static final double NEES_BOUND = BOUND * BOUND;

  /** How sure an allowance is that an honest estimator stays within it: z for 0.999. */
  static final double Z_999 = 3.09;

  /** The band's two-sided probability: 95%. */
  static final double BAND_Z = 1.959964;

  /**
   * One loop of one run: how far the estimate was from the truth, and how far it said it might be.
   *
   * @param errorX the estimate's x minus the truth's, meters
   * @param errorY and its y
   * @param errorHeading the estimate's heading minus the truth's, radians, wrapped to ±π
   * @param covariance what the estimate claimed
   * @param tracking whether the estimate was tracking: recent camera measurements agreed with it
   */
  public record Sample(
      double errorX,
      double errorY,
      double errorHeading,
      PoseCovariance covariance,
      boolean tracking) {
    /** A tracking estimate's errors and claim. */
    public Sample(double errorX, double errorY, double errorHeading, PoseCovariance covariance) {
      this(errorX, errorY, errorHeading, covariance, true);
    }

    /** An estimate against the truth. */
    public static Sample of(LocalizationEstimate estimate, Pose2d truth) {
      Pose2d pose = estimate.pose();
      return new Sample(
          pose.getX() - truth.getX(),
          pose.getY() - truth.getY(),
          pose.getRotation().minus(truth.getRotation()).getRadians(),
          estimate.covariance(),
          estimate.mode() == LocalizationEstimate.Mode.TRACKING);
    }

    /** The position error weighted by the claimed position covariance; infinite if it's flat. */
    double positionNees() {
      double determinant = covariance.xx() * covariance.yy() - covariance.xy() * covariance.xy();
      if (!(determinant > 0)) {
        return errorX == 0 && errorY == 0 ? 0 : Double.POSITIVE_INFINITY;
      }
      return (covariance.yy() * errorX * errorX
              - 2 * covariance.xy() * errorX * errorY
              + covariance.xx() * errorY * errorY)
          / determinant;
    }

    /** The heading error weighted by the claimed heading variance. */
    double headingNees() {
      double variance = covariance.heading();
      if (!(variance > 0)) {
        return errorHeading == 0 ? 0 : Double.POSITIVE_INFINITY;
      }
      return errorHeading * errorHeading / variance;
    }

    double errorMeters() {
      return Math.hypot(errorX, errorY);
    }

    /** How many claimed spreads, along the least sure direction, the position error is. */
    double ratio() {
      double spread = covariance.largestPositionStdDev();
      double error = errorMeters();
      return spread > 0 ? error / spread : error == 0 ? 0 : Double.POSITIVE_INFINITY;
    }
  }

  /**
   * What the runs show.
   *
   * @param runs how many runs
   * @param loops how many loops each
   * @param positionNees the mean position NEES over every loop of every run: 2 when honest
   * @param positionAbove the share of loops whose mean position NEES across the runs is above the
   *     band: about 2.5% when honest, more when overconfident
   * @param positionBelow and below it: about 2.5% when honest, more when timid
   * @param headingNees the mean heading NEES: 1 when honest
   * @param headingAbove the share of loops above the heading band
   * @param headingBelow and below it
   * @param exceedance the share of loops, over every run, whose position error is past {@link
   *     #BOUND} times the claimed spread along its least sure direction: at most {@link
   *     #EXCEEDANCE} when honest
   * @param ellipseExceedance the share whose position NEES is past {@link #NEES_BOUND}, outside the
   *     claimed covariance's 99.9% ellipse: {@link #EXCEEDANCE} when honest
   * @param allowedShare the most of either an honest estimator shows in this many runs ({@link
   *     #allowedShare})
   * @param worstRatio the most claimed spreads the position error ever was
   * @param meanErrorMeters the mean position error
   * @param worstErrorMeters the worst
   * @param meanHeadingErrorDegrees the mean heading error, either way
   * @param meanSpreadMeters the mean claimed spread along the least sure direction
   */
  public record Summary(
      int runs,
      int loops,
      double positionNees,
      double positionAbove,
      double positionBelow,
      double headingNees,
      double headingAbove,
      double headingBelow,
      double exceedance,
      double ellipseExceedance,
      double allowedShare,
      double worstRatio,
      double meanErrorMeters,
      double worstErrorMeters,
      double meanHeadingErrorDegrees,
      double meanSpreadMeters) {}

  private final List<List<Sample>> runs = new ArrayList<>();

  /**
   * Adds a run.
   *
   * @throws IllegalArgumentException if it has no loops, or not as many as the runs before it
   */
  public void add(List<Sample> run) {
    if (run.isEmpty()) {
      throw new IllegalArgumentException("a run has at least one loop");
    }
    if (!runs.isEmpty() && runs.get(0).size() != run.size()) {
      throw new IllegalArgumentException(
          "every run has the same loops: " + run.size() + " after " + runs.get(0).size());
    }
    runs.add(List.copyOf(run));
  }

  /** How many runs have been added. */
  public int runs() {
    return runs.size();
  }

  /**
   * The band a mean of N honest NEES values falls in 95% of the time: N times the mean is χ² with N
   * times the dimensions, whose quantiles come from the Wilson-Hilferty approximation (within a
   * percent from a few degrees of freedom up).
   *
   * @param dimensions 2 for position, 1 for heading
   * @param runs how many runs are averaged
   * @return the low and high ends
   */
  public static double[] band(int dimensions, int runs) {
    return band(dimensions, runs, BAND_Z);
  }

  /**
   * The band a mean of N honest NEES values falls in, but for as often as a standard normal goes
   * past z either way: z of 3.29 is 999 times in 1000.
   */
  public static double[] band(int dimensions, int runs, double z) {
    int freedom = dimensions * runs;
    return new double[] {chiSquared(freedom, -z) / runs, chiSquared(freedom, z) / runs};
  }

  /**
   * The most of its loops an honest estimator's error goes past a 1-in-1000 bound over this many
   * runs, but for once in a thousand times: a thousandth, plus 3.09 standard deviations of the
   * share among that many draws, each run one.
   */
  static double allowedShare(int runs) {
    return EXCEEDANCE + Z_999 * Math.sqrt(EXCEEDANCE * (1 - EXCEEDANCE) / Math.max(1, runs));
  }

  /** The share of every loop of every run that passes a test: how often a gate would let it by. */
  public double share(Predicate<Sample> passes) {
    long passed = 0;
    long all = 0;
    for (List<Sample> run : runs) {
      for (Sample sample : run) {
        passed += passes.test(sample) ? 1 : 0;
        all++;
      }
    }
    return all == 0 ? Double.NaN : (double) passed / all;
  }

  /** The χ² quantile for some degrees of freedom at a standard normal's z (Wilson-Hilferty). */
  static double chiSquared(int freedom, double z) {
    double a = 2.0 / (9.0 * freedom);
    double cube = 1 - a + z * Math.sqrt(a);
    return freedom * Math.max(0, cube * cube * cube);
  }

  /** The mean position NEES across the runs, loop by loop. */
  public double[] positionNeesByLoop() {
    return byLoop(Sample::positionNees);
  }

  /** The mean heading NEES across the runs, loop by loop. */
  public double[] headingNeesByLoop() {
    return byLoop(Sample::headingNees);
  }

  /** The mean position error across the runs, loop by loop, meters. */
  public double[] errorByLoop() {
    return byLoop(Sample::errorMeters);
  }

  /** The mean claimed spread along the least sure direction across the runs, loop by loop. */
  public double[] spreadByLoop() {
    return byLoop(sample -> sample.covariance().largestPositionStdDev());
  }

  private double[] byLoop(ToDoubleFunction<Sample> measure) {
    int loops = runs.isEmpty() ? 0 : runs.get(0).size();
    double[] means = new double[loops];
    for (List<Sample> run : runs) {
      for (int i = 0; i < loops; i++) {
        means[i] += measure.applyAsDouble(run.get(i));
      }
    }
    for (int i = 0; i < loops; i++) {
      means[i] /= runs.size();
    }
    return means;
  }

  /**
   * What the runs show.
   *
   * @throws IllegalStateException with no runs
   */
  public Summary summary() {
    if (runs.isEmpty()) {
      throw new IllegalStateException("no runs to summarize");
    }
    int loops = runs.get(0).size();
    double[] positionByLoop = positionNeesByLoop();
    double[] headingByLoop = headingNeesByLoop();
    double[] positionBand = band(2, runs.size());
    double[] headingBand = band(1, runs.size());
    int positionAbove = 0;
    int positionBelow = 0;
    int headingAbove = 0;
    int headingBelow = 0;
    for (int i = 0; i < loops; i++) {
      positionAbove += positionByLoop[i] > positionBand[1] ? 1 : 0;
      positionBelow += positionByLoop[i] < positionBand[0] ? 1 : 0;
      headingAbove += headingByLoop[i] > headingBand[1] ? 1 : 0;
      headingBelow += headingByLoop[i] < headingBand[0] ? 1 : 0;
    }
    double positionNees = 0;
    double headingNees = 0;
    int exceeded = 0;
    int outsideEllipse = 0;
    double worstRatio = 0;
    double error = 0;
    double worstError = 0;
    double headingError = 0;
    double spread = 0;
    for (List<Sample> run : runs) {
      for (Sample sample : run) {
        double nees = sample.positionNees();
        positionNees += nees;
        outsideEllipse += nees > NEES_BOUND ? 1 : 0;
        headingNees += sample.headingNees();
        double ratio = sample.ratio();
        exceeded += ratio > BOUND ? 1 : 0;
        worstRatio = Math.max(worstRatio, ratio);
        error += sample.errorMeters();
        worstError = Math.max(worstError, sample.errorMeters());
        headingError += Math.abs(sample.errorHeading());
        spread += sample.covariance().largestPositionStdDev();
      }
    }
    double samples = (double) loops * runs.size();
    return new Summary(
        runs.size(),
        loops,
        positionNees / samples,
        positionAbove / (double) loops,
        positionBelow / (double) loops,
        headingNees / samples,
        headingAbove / (double) loops,
        headingBelow / (double) loops,
        exceeded / samples,
        outsideEllipse / samples,
        allowedShare(runs.size()),
        worstRatio,
        error / samples,
        worstError,
        Math.toDegrees(headingError / samples),
        spread / samples);
  }
}
