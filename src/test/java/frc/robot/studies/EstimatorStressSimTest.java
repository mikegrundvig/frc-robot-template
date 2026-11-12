package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.SimulationTest;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * The estimator study's stresses, as gates: the robot's own localizer (A) on one seed of each, held
 * to bounds set from the study's medians over ten seeds (seeds 1 to 10, 2026-11-10, with the
 * placement's heading in doubt until a camera backs it). The study is the report; this fails only
 * when a stress the localizer handled today gets far worse.
 *
 * <p>Each bound is twice the recorded median plus a centimeter, for position RMS and 95th
 * percentile alike: the seeds' spread at these medians is a few millimeters, so a bound that wide
 * is passed by every seed the study ran, and fails a regression that doubles the error. The
 * simulation repeats exactly by seed, so the same seed gives the same numbers on every run.
 *
 * <p>Two more things are held. The camera knocked at 5 s is doubted by the robot's mount check
 * (what raises its "out of calibration" alert) within {@link #KNOCK_FLAGGED_WITHIN_SECONDS}, and no
 * other camera is doubted at all. A robot set down 30° off its start's heading, or backwards, finds
 * its heading from the cameras, on PhotonVision's multi-tag solutions and on Visor's single tags
 * alike, within {@link #RECOVERED_WITHIN_SECONDS} and without a driver's reset: the study's medians
 * are 0.1 s on PhotonVision and 0.22 s on Visor.
 */
@SimulationTest
class EstimatorStressSimTest {
  /** The seed every row plays: one of the ten the medians came from. */
  private static final long SEED = 1;

  /** A recovered estimate is back within 5 cm and 1° of the truth and stays: this soon. */
  private static final double RECOVERED_WITHIN_SECONDS = 1.0;

  /**
   * The camera knocked at 5 s is doubted by this long after it. On every one of the ten seeds it
   * was 10.1 to 10.2 s after the knock: the mount check judges over its latest few hundred tags,
   * seen where the estimate is sure enough. Two seconds more is the margin.
   */
  private static final double KNOCK_FLAGGED_WITHIN_SECONDS = 12.0;

  /** When camera-knocked knocks its first camera, after the robot is enabled. */
  private static final double KNOCKED_AT_SECONDS = 5.0;

  private static final StudyRobot VISOR =
      new StudyRobot(StudyRobot.Software.VISOR, StudyRobot.Choice.DEFAULT);

  /** What a row expects of a fault the robot recovers from: a recovery time, or none. */
  enum Recovers {
    NOT_ASKED,
    WITHIN_A_SECOND
  }

  /**
   * One stress, and A's medians on it as the study recorded them.
   *
   * @param scenario the study's scenario
   * @param robot the robot's cameras' software and its localizer
   * @param rmsCentimeters A's median position error, RMS
   * @param p95Centimeters and its 95th percentile
   * @param recovers whether the row asks how soon it recovered
   */
  record Row(
      String scenario,
      StudyRobot robot,
      double rmsCentimeters,
      double p95Centimeters,
      Recovers recovers) {
    @Override
    public String toString() {
      return scenario + " on " + robot.describe();
    }
  }

  static Stream<Row> rows() {
    StudyRobot photonVision = StudyRobot.AS_BUILT;
    return Stream.of(
        new Row("wheel-slip", photonVision, 0.7, 1.2, Recovers.NOT_ASKED),
        new Row("camera-knocked", photonVision, 1.4, 3.5, Recovers.NOT_ASKED),
        new Row("held-two-cameras", photonVision, 1.2, 2.8, Recovers.NOT_ASKED),
        new Row("held-camera-range-bias", photonVision, 6.9, 11.2, Recovers.NOT_ASKED),
        new Row("camera-clock-off", photonVision, 1.4, 2.6, Recovers.NOT_ASKED),
        new Row("off-heading-30", photonVision, 0.7, 1.3, Recovers.WITHIN_A_SECOND),
        new Row("off-heading-180", photonVision, 0.7, 1.3, Recovers.WITHIN_A_SECOND),
        new Row("off-heading-30", VISOR, 0.8, 2.0, Recovers.WITHIN_A_SECOND),
        new Row("off-heading-180", VISOR, 0.8, 2.0, Recovers.WITHIN_A_SECOND));
  }

  /** Twice a recorded median, plus a centimeter, in meters. */
  private static double bound(double medianCentimeters) {
    return (2 * medianCentimeters + 1) / 100;
  }

  @ParameterizedTest
  @MethodSource("rows")
  void theLocalizerHoldsUpUnderEachStress(Row row) {
    EstimatorRuns.Played played =
        EstimatorRuns.play(EstimatorScenarios.named(row.scenario()), SEED, row.robot());
    EstimatorScores.Measures ours =
        Objects.requireNonNull(played.measures().get(EstimatorRuns.OURS), "A");

    assertThat(ours.rmsMeters()).as("position RMS").isLessThan(bound(row.rmsCentimeters()));
    assertThat(ours.p95Meters()).as("position p95").isLessThan(bound(row.p95Centimeters()));
    if (row.recovers() == Recovers.WITHIN_A_SECOND) {
      assertThat(ours.recoverySeconds()).as("recovered").isLessThan(RECOVERED_WITHIN_SECONDS);
    }
    if (row.scenario().equals("camera-knocked")) {
      assertKnockedCameraAloneIsFlagged(played.firstDoubted());
    }
  }

  /** The first camera, knocked, is doubted soon after; no other camera ever is. */
  private static void assertKnockedCameraAloneIsFlagged(Map<String, Double> firstDoubted) {
    String knocked = firstDoubted.keySet().iterator().next();
    assertThat(firstDoubted.get(knocked))
        .as("%s doubted, seconds after the robot was enabled", knocked)
        .isBetween(KNOCKED_AT_SECONDS, KNOCKED_AT_SECONDS + KNOCK_FLAGGED_WITHIN_SECONDS);
    firstDoubted.forEach(
        (camera, seconds) -> {
          if (!camera.equals(knocked)) {
            assertThat(seconds).as("%s, never knocked", camera).isInfinite();
          }
        });
  }
}
