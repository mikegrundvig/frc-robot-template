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
              ConsistencyRuns.named(run), seed, ConsistencyRuns.MountErrors.CALIBRATED, 1));
    }
    return consistency.summary();
  }

  /**
   * After the gyro-scale procedure's five turns the heading is off, and every position the cameras
   * fix with it is off by its tags' distance times that, the same way every image. The estimator
   * carries that in each fix's uncertainty, so it claims no more than it knows: the error stays
   * within its claimed spread, where it used to claim millimeters while centimeters off (a position
   * NEES near 95, and past 3.72 σ in 59% of loops).
   */
  @Test
  void afterTheGyroScaleProcedureTheUncertaintyCoversTheError() {
    Consistency.Summary summary = play("gyro-scale");

    assertThat(summary.positionNees()).as("position NEES, 2 if honest").isLessThan(3);
    assertThat(summary.exceedance()).as("loops past 3.72 σ").isLessThan(Consistency.EXCEEDANCE);
    assertThat(summary.meanErrorMeters()).as("mean error").isLessThan(0.04);
  }

  /** Two turns in place: the same, while the heading is unsure and after (NEES 28 before). */
  @Test
  void turningInPlaceTheUncertaintyCoversTheError() {
    Consistency.Summary summary = play("turning");

    assertThat(summary.positionNees()).as("position NEES, 2 if honest").isLessThan(3);
    assertThat(summary.exceedance()).as("loops past 3.72 σ").isLessThan(Consistency.EXCEEDANCE);
    assertThat(summary.meanErrorMeters()).as("mean error").isLessThan(0.03);
  }
}
