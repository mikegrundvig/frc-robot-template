package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.SimulationTest;
import org.junit.jupiter.api.Test;

/**
 * The consistency study's runs, six seeds of each, on the whole simulated robot: a guard against an
 * estimator that's grossly wrong about its own uncertainty, not a verdict on how honest it is. Each
 * seed draws its errors as the estimator's configuration claims them ({@link
 * ConsistencyRuns.Setup#MATCHED}): its cameras off their mounts, its tags off the layout, its
 * gyro's scale off, and the vision noise model as the simulated cameras are. See docs/testing.md.
 *
 * <p>Those draws are guesses until cameras, IMUs and fields are measured, so this test never pins
 * an exact NEES or an ellipse share: a number that moves when a guess moves would fail every change
 * that touches it, either way. It fails only far outside what an honest estimator could read, by a
 * factor of five overconfident or ten timid, with long-axis tails past 10% of loops, or less
 * accurate than it is today. The full study ({@code ./gradlew consistencyStudy}) is the report, run
 * before and after a change to the estimator.
 *
 * <p>The estimator before the consistency work failed it everywhere: a position NEES from 7
 * standing to 107 after the gyro-scale procedure (with milder errors than these).
 */
@SimulationTest
class EstimatorConsistencySimTest {
  private static final int SEEDS = 6;

  private static Consistency.Summary play(String run) {
    Consistency consistency = new Consistency();
    for (long seed = 1; seed <= SEEDS; seed++) {
      consistency.add(
          ConsistencyRuns.play(ConsistencyRuns.named(run), seed, ConsistencyRuns.Setup.MATCHED)
              .samples());
    }
    return consistency.summary();
  }

  /**
   * Not grossly wrong about its uncertainty, and the estimate within this of the truth on average.
   * An honest estimator's NEES is 2 for position and 1 for heading.
   */
  private static void assertNotGrosslyWrong(Consistency.Summary summary, double meanErrorMeters) {
    assertThat(summary.positionNees())
        .as("position NEES, 2 if honest: under 10, over 0.2")
        .isBetween(0.2, 10.0);
    assertThat(summary.headingNees())
        .as("heading NEES, 1 if honest: under 5, over 0.1")
        .isBetween(0.1, 5.0);
    assertThat(summary.exceedance())
        .as("loops past 3.72 σ along the long axis")
        .isLessThanOrEqualTo(0.10);
    assertThat(summary.meanErrorMeters()).as("mean error").isLessThan(meanErrorMeters);
  }

  /**
   * The route there and back: the cameras see other tags as it goes, each off the layout its own
   * way, and the same ones again on the way back, off as they were.
   */
  @Test
  void drivingTheUncertaintyCoversTheError() {
    assertNotGrosslyWrong(play("driving"), 0.03);
  }

  /**
   * Standing still for 20 s after the route: the same tags in every image, two cameras on each, so
   * what they share (each tag's placement, and each camera's mount) never averages out, and the
   * estimate doesn't claim it does.
   */
  @Test
  void standingStillTheUncertaintyCoversTheError() {
    assertNotGrosslyWrong(play("standing"), 0.025);
  }

  /** Two turns in place: the gyro's scale error is the same every turn, and it adds up. */
  @Test
  void turningInPlaceTheUncertaintyCoversTheError() {
    assertNotGrosslyWrong(play("turning"), 0.035);
  }

  /**
   * After the gyro-scale procedure's five turns the heading is off, and every position the cameras
   * fix with it is off by its tags' distance times that, the same way every image: the estimate
   * carries it, where it used to claim millimeters while 6 to 20 cm off.
   */
  @Test
  void afterTheGyroScaleProcedureTheUncertaintyCoversTheError() {
    assertNotGrosslyWrong(play("gyro-scale"), 0.03);
  }
}
