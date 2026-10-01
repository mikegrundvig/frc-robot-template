package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.SimulationTest;
import org.junit.jupiter.api.Test;

/**
 * The consistency study's runs, two seeds of each, on the whole simulated robot: does the pose
 * estimator's uncertainty tell the truth? The cameras sit a little off their mounts and the field's
 * tags a little off its layout, as calibrated cameras on a field built to its tolerance do (see
 * docs/testing.md). Honest is a NEES near or below its number of dimensions, and an error never
 * past 3.72 times the spread the estimate claims; it must not get there by being less accurate.
 *
 * <p>The estimator these replaced claimed millimeters while centimeters off in every run here: a
 * position NEES from 7 standing to 107 after the gyro-scale procedure, past 3.72 σ in 15% to 72% of
 * loops, and a mean error 1.5 to 6 times today's.
 */
@SimulationTest
class EstimatorConsistencySimTest {
  private static final int SEEDS = 2;

  private static Consistency.Summary play(String run) {
    Consistency consistency = new Consistency();
    for (long seed = 1; seed <= SEEDS; seed++) {
      consistency.add(
          ConsistencyRuns.play(
              ConsistencyRuns.named(run), seed, ConsistencyRuns.Placement.CALIBRATED, 1));
    }
    return consistency.summary();
  }

  /** The uncertainty covers the error, and the estimate is within this of the truth on average. */
  private static void assertHonest(Consistency.Summary summary, double meanErrorMeters) {
    assertThat(summary.positionNees()).as("position NEES, 2 if honest").isLessThan(3);
    assertThat(summary.headingNees()).as("heading NEES, 1 if honest").isLessThan(2);
    assertThat(summary.worstRatio()).as("worst error, in claimed σ").isLessThan(Consistency.BOUND);
    assertThat(summary.meanErrorMeters()).as("mean error").isLessThan(meanErrorMeters);
  }

  /** The route there and back: the cameras' shifts, and the field's tags, change as it goes. */
  @Test
  void drivingTheUncertaintyCoversTheError() {
    assertHonest(play("driving"), 0.025);
  }

  /**
   * Standing still for 20 s after the route: the same tags in every image, so what they share
   * (their placement, the cameras' mounts) never averages out, and the estimate doesn't claim it
   * does.
   */
  @Test
  void standingStillTheUncertaintyCoversTheError() {
    assertHonest(play("standing"), 0.02);
  }

  /** Two turns in place: the gyro's scale error is the same every turn, and it adds up. */
  @Test
  void turningInPlaceTheUncertaintyCoversTheError() {
    assertHonest(play("turning"), 0.025);
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
