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
      {new Outcome(3, 2, 0.01, Math.toRadians(0.5))}, {new Outcome(1, 1, 0.03, Double.NaN)}
    };

    Score score = Score.of(TWO, samples, outcomes);

    assertThat(score.oneTag()).isEqualTo(1.0);
    assertThat(score.twoTags()).isEqualTo(0.25);
    assertThat(score.twoCameras()).isEqualTo(0.25);
    assertThat(score.measured()).isEqualTo(1.0);
    assertThat(score.withinFiveCentimeters()).isEqualTo(1.0);
    assertThat(score.withinTwoCentimeters()).as("only the 1 cm one").isEqualTo(0.25);
    assertThat(score.heading()).isEqualTo(0.25);
    assertThat(score.positionRmsMeters())
        .isCloseTo(Math.sqrt((0.01 * 0.01 + 3 * 0.03 * 0.03) / 4), within(1e-12));
    assertThat(score.positionP95Meters()).isEqualTo(0.03);
    assertThat(score.headingRmsDegrees()).isCloseTo(0.5, within(1e-12));
  }

  @Test
  void eachImageCountsItsShareOfItsPose() {
    Outcome[][] outcomes = {{placed(0.01), BLIND}};

    Score score = Score.of(ONE, List.of(weighing(1)), outcomes);

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

    assertThat(Score.of(ONE, samples, outcomes).positionP95Meters()).isEqualTo(0.19);
  }

  /** A spot without a measurement is missed by any distance: past 5% of them, there's none. */
  @Test
  void spotsWithoutAMeasurementCountAgainstTheNinetyFifthPercentile() {
    List<Sample> samples = new ArrayList<>();
    Outcome[][] oneBlind = new Outcome[20][];
    Outcome[][] twoBlind = new Outcome[20][];
    for (int i = 0; i < 20; i++) {
      samples.add(weighing(1));
      oneBlind[i] = new Outcome[] {i == 0 ? BLIND : placed(0.01)};
      twoBlind[i] = new Outcome[] {i < 2 ? BLIND : placed(0.01)};
    }

    assertThat(Score.of(ONE, samples, oneBlind).positionP95Meters()).isEqualTo(0.01);
    assertThat(Score.of(ONE, samples, twoBlind).positionP95Meters()).isInfinite();
  }

  /** 12.4 and 12.6 mm round to 12 and 13: not a tie, so fewer cameras don't decide it. */
  @Test
  void errorsThatRoundApartArentATie() {
    Score nearer = scored(TWO, 0.0124, 1);
    Score cheaper = scored(ONE, 0.0126, 1);
    List<Score> scores = new ArrayList<>(List.of(cheaper, nearer));

    scores.sort(Score.RANKING);

    assertThat(scores).containsExactly(nearer, cheaper);
  }

  @Test
  void aFieldWeighingNothingCantBeScored() {
    Outcome[][] outcomes = {{placed(0.01)}};

    assertThatThrownBy(() -> Score.of(ONE, List.of(weighing(0)), outcomes))
        .hasMessageContaining("weighs nothing");
  }

  private static Score scored(Candidate candidate, double p95, double withinFive) {
    return new Score(
        candidate, p95, withinFive, 0, 1, 1, 1, 1, 1, 0.01, 0.1, Double.NaN, Double.NaN);
  }

  /**
   * The smaller 95th percentile first, rounded to the millimeter; when two round alike, fewer
   * cameras; and those without one last, the most within 5 cm first.
   */
  @Test
  void theRankingGoesByTheNinetyFifthPercentileThenFewerCameras() {
    Score best = scored(TWO, 0.0201, 1);
    Score cheaperTie = scored(ONE, 0.0254, 0.99);
    Score dearerTie = scored(TWO, 0.0246, 0.99);
    Score worse = scored(ONE, 0.031, 0.99);
    Score blindButClose = scored(TWO, Double.POSITIVE_INFINITY, 0.9);
    Score blind = scored(ONE, Double.POSITIVE_INFINITY, 0.5);
    List<Score> scores =
        new ArrayList<>(List.of(blind, worse, dearerTie, blindButClose, cheaperTie, best));

    scores.sort(Score.RANKING);

    assertThat(scores).containsExactly(best, cheaperTie, dearerTie, worse, blindButClose, blind);
  }
}
