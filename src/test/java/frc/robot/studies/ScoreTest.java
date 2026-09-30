package frc.robot.studies;

import static frc.robot.studies.StudyFixtures.BACK;
import static frc.robot.studies.StudyFixtures.FRONT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;

/** A layout's score from outcomes written by hand. */
class ScoreTest {
  private static final Candidate ONE = new Candidate("one", List.of(FRONT));
  private static final Candidate TWO = new Candidate("two", List.of(FRONT, BACK));

  private static Sample weighing(double weight) {
    return new Sample(Pose2d.ZERO, weight, 0, 0);
  }

  private static Outcome placed(double errorMeters) {
    return new Outcome(2, 1, errorMeters, Double.NaN);
  }

  private static final Outcome BLIND = new Outcome(0, 0, Double.NaN, Double.NaN);

  @Test
  void everyShareIsOfThePosesWeighed() {
    List<Sample> samples = List.of(weighing(1), weighing(3));
    Outcome[][] outcomes = {
      {new Outcome(3, 2, 0.01, Math.toRadians(0.5))}, {new Outcome(1, 1, 0.08, Double.NaN)}
    };

    Score score = Score.of(TWO, samples, outcomes, 0.05);

    assertThat(score.oneTag()).isEqualTo(1.0);
    assertThat(score.twoTags()).isEqualTo(0.25);
    assertThat(score.twoCameras()).isEqualTo(0.25);
    assertThat(score.measured()).isEqualTo(1.0);
    assertThat(score.localized()).as("only the 1 cm one is within 5 cm").isEqualTo(0.25);
    assertThat(score.heading()).isEqualTo(0.25);
    assertThat(score.positionRmsMeters())
        .isCloseTo(Math.sqrt((0.01 * 0.01 + 3 * 0.08 * 0.08) / 4), within(1e-12));
    assertThat(score.positionP95Meters()).isEqualTo(0.08);
    assertThat(score.headingRmsDegrees()).isCloseTo(0.5, within(1e-12));
  }

  @Test
  void eachImageCountsItsShareOfItsPose() {
    Outcome[][] outcomes = {{placed(0.01), BLIND}};

    Score score = Score.of(ONE, List.of(weighing(1)), outcomes, 0.05);

    assertThat(score.measured()).isEqualTo(0.5);
    assertThat(score.positionRmsMeters()).isCloseTo(0.01, within(1e-12));
  }

  @Test
  void theNinetyFifthPercentileIsWhereTheWeightReachesIt() {
    List<Sample> samples = new ArrayList<>();
    Outcome[][] outcomes = new Outcome[20][];
    for (int i = 0; i < 20; i++) {
      samples.add(weighing(1));
      outcomes[i] = new Outcome[] {placed((i + 1) / 100.0)};
    }

    assertThat(Score.of(ONE, samples, outcomes, 0.05).positionP95Meters()).isEqualTo(0.19);
  }

  @Test
  void aFieldWeighingNothingCantBeScored() {
    Outcome[][] outcomes = {{placed(0.01)}};

    assertThatThrownBy(() -> Score.of(ONE, List.of(weighing(0)), outcomes, 0.05))
        .hasMessageContaining("weighs nothing");
  }

  @Test
  void theRankingPutsTheMostLocalizedFirstThenFewerCamerasThenTheSmallerError() {
    Score better = new Score(TWO, 0.9, 1, 1, 1, 1, 1, 0.02, 0.03, 0.1);
    Score cheaper = new Score(ONE, 0.8, 1, 1, 0, 1, 1, 0.02, 0.03, 0.1);
    Score dearer = new Score(TWO, 0.8, 1, 1, 1, 1, 1, 0.01, 0.03, 0.1);
    Score blind = new Score(ONE, 0.8, 0, 0, 0, 0, 0, Double.NaN, Double.NaN, Double.NaN);
    List<Score> scores = new ArrayList<>(List.of(dearer, blind, cheaper, better));

    scores.sort(Score.RANKING);

    assertThat(scores).containsExactly(better, cheaper, blind, dearer);
  }
}
