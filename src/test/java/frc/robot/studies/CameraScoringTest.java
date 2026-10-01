package frc.robot.studies;

import static frc.robot.studies.StudyFixtures.BACK;
import static frc.robot.studies.StudyFixtures.FACING_AWAY;
import static frc.robot.studies.StudyFixtures.FACING_THE_WALL;
import static frc.robot.studies.StudyFixtures.FRONT;
import static frc.robot.studies.StudyFixtures.ONE_THREAD;
import static frc.robot.studies.StudyFixtures.WALL;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import frc.robot.Robots;
import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.VisionMeasurement;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Scoring camera layouts, on cases simple enough to check by hand: a wall of tags, a robot facing
 * it or facing away, and one camera or two.
 */
class CameraScoringTest {
  private static final Candidate AHEAD = new Candidate("ahead", List.of(FRONT));
  private static final Candidate AHEAD_AND_BACK =
      new Candidate("ahead and back", List.of(FRONT, BACK));

  private static List<Score> ranking(
      CameraStudyConfig config, List<Sample> samples, Candidate... candidates) {
    return new CameraScoring(Robots.SIM, WALL, List.of(candidates), config)
        .score(samples, done -> {})
        .ranking();
  }

  private static Score score(Candidate candidate, Sample... samples) {
    return ranking(ONE_THREAD, List.of(samples), candidate).get(0);
  }

  @Test
  void aCameraFacingAWallOfTagsSeesThemAndPlacesTheRobot() {
    Score score = score(AHEAD, FACING_THE_WALL);

    assertThat(score.oneTag()).isEqualTo(1.0);
    assertThat(score.twoTags()).as("all three tags, 3 m off, fit in the view").isEqualTo(1.0);
    assertThat(score.twoCameras()).isZero();
    assertThat(score.measured()).isEqualTo(1.0);
    assertThat(score.withinFiveCentimeters()).isEqualTo(1.0);
    assertThat(score.heading()).as("a multi-tag solve that fits corrects heading").isEqualTo(1.0);
    // A quarter degree of gyro error alone moves a solve 3 m from its tags 1.3 cm.
    assertThat(score.positionRmsMeters()).isLessThan(0.04);
    assertThat(score.headingRmsDegrees()).isLessThan(1);
  }

  @Test
  void aCameraFacingAwaySeesNothing() {
    Score score = score(AHEAD, FACING_AWAY);

    assertThat(score.oneTag()).isZero();
    assertThat(score.measured()).isZero();
    assertThat(score.withinFiveCentimeters()).isZero();
    assertThat(score.positionRmsMeters()).isNaN();
    assertThat(score.positionP95Meters()).as("nothing measured").isInfinite();
    assertThat(score.headingRmsDegrees()).isNaN();
  }

  @Test
  void aSecondCameraFacingBackSeesTheWallWhateverWayTheRobotFaces() {
    List<Score> ranking =
        ranking(ONE_THREAD, List.of(FACING_THE_WALL, FACING_AWAY), AHEAD, AHEAD_AND_BACK);

    assertThat(ranking)
        .extracting(score -> score.candidate().name())
        .containsExactly("ahead and back", "ahead");
    assertThat(ranking.get(0).oneTag()).isEqualTo(1.0);
    assertThat(ranking.get(0).twoCameras()).as("never both at once").isZero();
    assertThat(ranking.get(1).oneTag()).isEqualTo(0.5);
    assertThat(ranking.get(1).withinFiveCentimeters()).isEqualTo(0.5);
  }

  @Test
  void twoCamerasFacingTheSameTagsBothSeeThem() {
    Candidate pair =
        new Candidate(
            "a pair",
            List.of(StudyFixtures.camera("left", 0.3, 5), StudyFixtures.camera("right", 0.3, -5)));

    Score score = score(pair, FACING_THE_WALL);

    assertThat(score.twoCameras()).isEqualTo(1.0);
    assertThat(score.withinFiveCentimeters()).isEqualTo(1.0);
  }

  @Test
  void aSpotWeighingNothingDoesntCount() {
    Sample ignored = new Sample(FACING_AWAY.pose(), 0, 1, 0);

    assertThat(score(AHEAD, FACING_THE_WALL, ignored).oneTag()).isEqualTo(1.0);
  }

  @Test
  void aCameraOutsideTheRobotIsRefused() {
    Candidate outside =
        new Candidate("on a pole", List.of(FRONT, StudyFixtures.camera("pole", 1.0, 0)));

    assertThatThrownBy(
            () -> new CameraScoring(Robots.SIM, WALL, List.of(AHEAD, outside), ONE_THREAD))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining("on a pole")
        .hasMessageContaining("camera pole is outside the robot");
  }

  @Test
  void camerasSharedByLayoutsTakeTheirImagesOnce() {
    CameraScoring scoring =
        new CameraScoring(Robots.SIM, WALL, List.of(AHEAD, AHEAD_AND_BACK), ONE_THREAD);

    assertThat(scoring.distinctCameras()).isEqualTo(2);
  }

  @Test
  void theSameSeedGivesTheSameScoresOnAnyNumberOfThreads() {
    List<Sample> samples = new FieldGrid(WALL, Robots.SIM.body(), 2, 4, Weights.NONE, 1).samples();
    CameraStudyConfig threes =
        CameraStudyConfigBuilder.builder(ONE_THREAD).threads(3).frames(2).build();
    CameraStudyConfig otherSeed = CameraStudyConfigBuilder.builder(threes).seed(2).build();

    List<Score> once =
        ranking(
            CameraStudyConfigBuilder.builder(threes).threads(1).build(),
            samples,
            AHEAD,
            AHEAD_AND_BACK);

    assertThat(ranking(threes, samples, AHEAD, AHEAD_AND_BACK)).isEqualTo(once);
    assertThat(ranking(otherSeed, samples, AHEAD, AHEAD_AND_BACK)).isNotEqualTo(once);
  }

  @Test
  void aLayoutScoresTheSameWhateverIsTriedBesideIt() {
    List<Sample> samples = List.of(FACING_THE_WALL, FACING_AWAY);

    Score alone = ranking(ONE_THREAD, samples, AHEAD_AND_BACK).get(0);
    Score beside = ranking(ONE_THREAD, samples, AHEAD, AHEAD_AND_BACK).get(0);

    assertThat(beside).isEqualTo(alone);
  }

  /** Two cameras' measurements, one twice as sure as the other, combined as the estimator would. */
  @Test
  void measurementsCombineWeighedByHowSureEachIs() {
    Pose2d truth = new Pose2d(3, 4, Rotation2d.ZERO);
    VisionMeasurement sure =
        new VisionMeasurement(
            0,
            new Pose2d(3.03, 4, Rotation2d.ZERO),
            PoseCovariance.fromStdDevs(0.01, 0.01, Double.POSITIVE_INFINITY),
            "sure",
            List.of(1),
            VisionMeasurement.Solver.HEADING_CONSTRAINED);
    VisionMeasurement unsure =
        new VisionMeasurement(
            0,
            new Pose2d(2.97, 4, Rotation2d.fromDegrees(1)),
            PoseCovariance.fromStdDevs(0.02, 0.02, 0.01),
            "unsure",
            List.of(2, 3),
            VisionMeasurement.Solver.HEADING_CONSTRAINED);

    Outcome outcome =
        CameraScoring.combine(
            new int[] {0, 1, 2},
            new int[] {1, 2, 0},
            new VisionMeasurement[] {sure, unsure, null},
            truth);

    // x: (0.03 / 0.01² - 0.03 / 0.02²) / (1 / 0.01² + 1 / 0.02²) = 0.018
    assertThat(outcome.positionErrorMeters()).isCloseTo(0.018, within(1e-9));
    assertThat(outcome.headingErrorRadians())
        .as("only one says anything about heading")
        .isCloseTo(Math.toRadians(1), within(1e-9));
    assertThat(outcome.mostTagsInOneImage()).isEqualTo(2);
    assertThat(outcome.camerasSeeingTags()).isEqualTo(2);
  }

  /**
   * Two measurements whose spreads lean differently: one round, one stretched along the diagonal x
   * = y. Combined with their whole covariances, as the estimator's filter does, they land where
   * weighing x and y apart wouldn't.
   */
  @Test
  void measurementsCombineWithTheirWholeCovariance() {
    Pose2d truth = new Pose2d(3, 4, Rotation2d.ZERO);
    VisionMeasurement round =
        new VisionMeasurement(
            0,
            new Pose2d(3.02, 4, Rotation2d.ZERO),
            new PoseCovariance(1e-4, 0, 1e-4, Double.POSITIVE_INFINITY),
            "round",
            List.of(1),
            VisionMeasurement.Solver.HEADING_CONSTRAINED);
    VisionMeasurement leaning =
        new VisionMeasurement(
            0,
            new Pose2d(3, 4.02, Rotation2d.ZERO),
            new PoseCovariance(2e-4, 1e-4, 2e-4, Double.POSITIVE_INFINITY),
            "leaning",
            List.of(2),
            VisionMeasurement.Solver.HEADING_CONSTRAINED);

    Outcome outcome =
        CameraScoring.combine(
            new int[] {0, 1}, new int[] {1, 1}, new VisionMeasurement[] {round, leaning}, truth);

    // Information: 10^4 [1 0; 0 1] and (10^4 / 3) [2 -1; -1 2], summed (10^4 / 3) [5 -1; -1 5].
    // Information times miss: 10^4 (0.02, 0) + (10^4 / 3)(-0.02, 0.04) = (10^4 / 3)(0.04, 0.04).
    // So [5 -1; -1 5] miss = (0.04, 0.04), and the miss is (0.01, 0.01). Weighing x and y apart
    // would give (0.0133, 0.0067).
    assertThat(outcome.positionErrorMeters()).isCloseTo(0.01 * Math.sqrt(2), within(1e-12));
    assertThat(outcome.correctsHeading()).isFalse();
  }

  @Test
  void seedsDependOnEveryNumberMixedIn() {
    assertThat(Seeds.mix(1, 0.5, 2.0))
        .isEqualTo(Seeds.mix(1, 0.5, 2.0))
        .isNotEqualTo(Seeds.mix(1, 2.0, 0.5))
        .isNotEqualTo(Seeds.mix(2, 0.5, 2.0));
  }
}
