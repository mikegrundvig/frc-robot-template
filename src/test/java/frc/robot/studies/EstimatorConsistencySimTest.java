package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.SimulationTest;
import org.junit.jupiter.api.Test;

/**
 * The consistency study's runs, a few seeds of each, on the whole simulated robot: does the pose
 * estimator's uncertainty tell the truth? (See docs/testing.md.)
 */
@SimulationTest
class EstimatorConsistencySimTest {
  private static final int SEEDS = 2;

  private static Consistency.Summary play(String run) {
    Consistency consistency = new Consistency();
    for (long seed = 1; seed <= SEEDS; seed++) {
      consistency.add(
          ConsistencyRuns.play(
              ConsistencyRuns.named(run), seed, ConsistencyRuns.MountErrors.CALIBRATED));
    }
    return consistency.summary();
  }

  /**
   * After the gyro-scale procedure's five turns the heading is off, and every position the cameras
   * fix with it is off by its tags' distance times that, the same way every image: the estimate
   * averages them as if they were independent, and claims millimeters while it's centimeters off.
   */
  @Test
  void afterTheGyroScaleProcedureTheEstimateClaimsFarLessThanItsError() {
    Consistency.Summary summary = play("gyro-scale");

    assertThat(summary.positionNees()).as("position NEES, 2 if honest").isGreaterThan(20);
    assertThat(summary.exceedance()).as("loops past 3.72 σ").isGreaterThan(0.2);
  }
}
