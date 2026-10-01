package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.SimulationTest;
import org.junit.jupiter.api.Test;

/**
 * The consistency study's runs, six seeds of each, on the whole simulated robot: does the pose
 * estimator's uncertainty tell the truth? Each seed draws its errors as the estimator's
 * configuration claims them ({@link ConsistencyRuns.Setup#MATCHED}): its cameras off their mounts,
 * its tags off the layout, its gyro's scale off, and the vision noise model as the simulated
 * cameras are. See docs/testing.md.
 *
 * <p>What a seed draws lasts its whole run, so the seeds, not the loops, are the sample, and six of
 * them can't pin a NEES down closely. So each run must land where six honest seeds do 999 times in
 * 1000: its mean NEES inside that band (0.30 to 5.8 for position, 0.04 to 4.1 for heading), and the
 * share of loops outside the claimed covariance's 99.9% ellipse within what six seeds allow (4%).
 * Far above, and the estimate claims more than it knows; far below, it claims less. And it must not
 * get there by being less accurate.
 *
 * <p>The estimator before the consistency work failed all four: a position NEES from 7 standing to
 * 107 after the gyro-scale procedure (with milder errors than these).
 */
@SimulationTest
class EstimatorConsistencySimTest {
  private static final int SEEDS = 6;

  private static Consistency.Summary play(String run) {
    Consistency consistency = new Consistency();
    for (long seed = 1; seed <= SEEDS; seed++) {
      consistency.add(
          ConsistencyRuns.play(ConsistencyRuns.named(run), seed, ConsistencyRuns.Setup.MATCHED));
    }
    return consistency.summary();
  }

  /** Honest, and the estimate within this of the truth on average. */
  private static void assertHonest(Consistency.Summary summary, double meanErrorMeters) {
    double[] position = Consistency.band(2, SEEDS, 3.29);
    double[] heading = Consistency.band(1, SEEDS, 3.29);
    assertThat(summary.positionNees())
        .as("position NEES, 2 if honest")
        .isBetween(position[0], position[1]);
    assertThat(summary.headingNees())
        .as("heading NEES, 1 if honest")
        .isBetween(heading[0], heading[1]);
    assertThat(summary.ellipseExceedance())
        .as("loops outside the 99.9% ellipse")
        .isLessThanOrEqualTo(summary.allowedShare());
    assertThat(summary.exceedance())
        .as("loops past 3.72 σ along the long axis")
        .isLessThanOrEqualTo(summary.allowedShare());
    assertThat(summary.meanErrorMeters()).as("mean error").isLessThan(meanErrorMeters);
  }

  /** The route there and back: the cameras' shifts, and the field's tags, change as it goes. */
  @Test
  void drivingTheUncertaintyCoversTheError() {
    assertHonest(play("driving"), 0.03);
  }

  /**
   * Standing still for 20 s after the route: the same tags in every image, so what they share
   * (their placement, the cameras' mounts) never averages out, and the estimate doesn't claim it
   * does.
   */
  @Test
  void standingStillTheUncertaintyCoversTheError() {
    assertHonest(play("standing"), 0.025);
  }

  /** Two turns in place: the gyro's scale error is the same every turn, and it adds up. */
  @Test
  void turningInPlaceTheUncertaintyCoversTheError() {
    assertHonest(play("turning"), 0.035);
  }

  /**
   * After the gyro-scale procedure's five turns the heading is off, and every position the cameras
   * fix with it is off by its tags' distance times that, the same way every image: the estimate
   * carries it, where it used to claim millimeters while 6 to 20 cm off.
   */
  @Test
  void afterTheGyroScaleProcedureTheUncertaintyCoversTheError() {
    assertHonest(play("gyro-scale"), 0.03);
  }
}
