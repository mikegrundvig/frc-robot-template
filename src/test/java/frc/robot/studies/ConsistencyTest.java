package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import frc.robot.lib.localization.PoseCovariance;
import java.util.ArrayList;
import java.util.List;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The consistency measures, on made-up estimators whose honesty is known: errors drawn from one
 * spread, and a covariance that claims that spread, or less, or more.
 */
class ConsistencyTest {
  /** An estimator less sure along x than y, and correlated, as a real one is after driving. */
  private static final PoseCovariance CLAIMED =
      new PoseCovariance(0.04 * 0.04, 0.0003, 0.02 * 0.02, 0.01 * 0.01);

  /**
   * Runs of an estimator whose errors are drawn from {@link #CLAIMED} scaled by a factor, while it
   * claims {@link #CLAIMED}: a factor of 1 is honest, 2 is overconfident, 0.5 timid.
   */
  private static Consistency runs(int runs, int loops, double factor, long seed) {
    SplittableRandom random = new SplittableRandom(seed);
    // The claimed covariance's Cholesky factor, to draw correlated errors from it.
    double l00 = Math.sqrt(CLAIMED.xx());
    double l10 = CLAIMED.xy() / l00;
    double l11 = Math.sqrt(CLAIMED.yy() - l10 * l10);
    Consistency consistency = new Consistency();
    for (int run = 0; run < runs; run++) {
      List<Consistency.Sample> samples = new ArrayList<>();
      for (int loop = 0; loop < loops; loop++) {
        double a = gaussian(random);
        double b = gaussian(random);
        samples.add(
            new Consistency.Sample(
                factor * l00 * a,
                factor * (l10 * a + l11 * b),
                factor * CLAIMED.headingStdDev() * gaussian(random),
                CLAIMED));
      }
      consistency.add(samples);
    }
    return consistency;
  }

  /** A standard normal draw (Box-Muller): SplittableRandom has none of its own. */
  private static double gaussian(SplittableRandom random) {
    return Math.sqrt(-2 * Math.log(1 - random.nextDouble()))
        * Math.cos(2 * Math.PI * random.nextDouble());
  }

  @Test
  void anHonestEstimatorAveragesItsDimensionsAndStaysInTheBand() {
    Consistency.Summary summary = runs(20, 5000, 1, 1).summary();

    assertThat(summary.positionNees()).isCloseTo(2, within(0.05));
    assertThat(summary.headingNees()).isCloseTo(1, within(0.05));
    assertThat(summary.positionAbove()).isCloseTo(0.025, within(0.01));
    assertThat(summary.positionBelow()).isCloseTo(0.025, within(0.01));
    assertThat(summary.headingAbove()).isCloseTo(0.025, within(0.01));
    assertThat(summary.headingBelow()).isCloseTo(0.025, within(0.01));
    // The bound is conservative: it takes the least sure direction's spread for both.
    assertThat(summary.exceedance()).isLessThanOrEqualTo(Consistency.EXCEEDANCE);
    assertThat(summary.runs()).isEqualTo(20);
    assertThat(summary.loops()).isEqualTo(5000);
  }

  @Test
  void anOverconfidentEstimatorIsAboveTheBandAndPastTheBoundOften() {
    Consistency.Summary summary = runs(20, 2000, 2, 2).summary();

    assertThat(summary.positionNees()).isCloseTo(8, within(0.3));
    assertThat(summary.positionAbove()).isGreaterThan(0.99);
    assertThat(summary.headingAbove()).isGreaterThan(0.95);
    assertThat(summary.exceedance()).isGreaterThan(0.05);
    assertThat(summary.worstRatio()).isGreaterThan(1.5 * Consistency.BOUND);
  }

  @Test
  void aTimidEstimatorIsBelowTheBand() {
    Consistency.Summary summary = runs(20, 2000, 0.5, 3).summary();

    assertThat(summary.positionNees()).isCloseTo(0.5, within(0.05));
    assertThat(summary.positionBelow()).isGreaterThan(0.99);
    assertThat(summary.exceedance()).isZero();
  }

  @Test
  void theBoundHoldsAThousandthOfACircularGaussiansTail() {
    // e^(-k²/2) = 0.001
    assertThat(Consistency.BOUND).isCloseTo(3.717, within(0.001));
    assertThat(String.format(java.util.Locale.ROOT, "%.2f", Consistency.BOUND))
        .isEqualTo(Consistency.BOUND_TEXT);
  }

  /** Bar-Shalom, Li, and Kirubarajan's values, and χ² tables'. */
  @Test
  void theBandsMatchTheTables() {
    assertThat(Consistency.band(2, 50)).containsExactly(new double[] {1.48, 2.59}, within(0.01));
    assertThat(Consistency.band(3, 50)).containsExactly(new double[] {2.36, 3.72}, within(0.01));
    // χ² with 24 degrees of freedom: 12.40 and 39.36.
    assertThat(Consistency.band(2, 12))
        .containsExactly(new double[] {12.40 / 12, 39.36 / 12}, within(0.01));
  }

  @Test
  void theMeasuresFollowTheirDefinitions() {
    PoseCovariance round = PoseCovariance.fromStdDevs(0.1, 0.1, 0.02);
    Consistency.Sample sample =
        Consistency.Sample.of(
            new Pose2d(1.3, 2.4, Rotation2d.fromDegrees(1)),
            round,
            new Pose2d(1.0, 2.0, Rotation2d.ZERO));

    // 0.5 m off where 0.1 m is claimed: NEES (0.3² + 0.4²) / 0.01 = 25, and 5 spreads.
    assertThat(sample.positionNees()).isCloseTo(25, within(1e-9));
    assertThat(sample.ratio()).isCloseTo(5, within(1e-9));
    assertThat(sample.headingNees()).isCloseTo(Math.pow(Math.toRadians(1) / 0.02, 2), within(1e-9));
    assertThat(sample.errorMeters()).isCloseTo(0.5, within(1e-9));
  }

  @Test
  void anEstimateClaimingCertaintyIsInfinitelyWrongUnlessExact() {
    PoseCovariance certain = new PoseCovariance(0, 0, 0, 0);
    Consistency.Sample off = new Consistency.Sample(0.01, 0, 0.01, certain);
    Consistency.Sample exact = new Consistency.Sample(0, 0, 0, certain);

    assertThat(off.positionNees()).isInfinite();
    assertThat(off.headingNees()).isInfinite();
    assertThat(off.ratio()).isInfinite();
    assertThat(exact.positionNees()).isZero();
    assertThat(exact.headingNees()).isZero();
    assertThat(exact.ratio()).isZero();
  }

  @Test
  void everyRunHasTheSameLoops() {
    Consistency consistency = new Consistency();
    PoseCovariance claimed = PoseCovariance.fromStdDevs(0.1, 0.1, 0.1);
    consistency.add(List.of(new Consistency.Sample(0, 0, 0, claimed)));

    assertThatThrownBy(() -> consistency.add(List.of()))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(
            () ->
                consistency.add(
                    List.of(
                        new Consistency.Sample(0, 0, 0, claimed),
                        new Consistency.Sample(0, 0, 0, claimed))))
        .hasMessageContaining("same loops");
    assertThatThrownBy(() -> new Consistency().summary()).isInstanceOf(IllegalStateException.class);
  }
}
