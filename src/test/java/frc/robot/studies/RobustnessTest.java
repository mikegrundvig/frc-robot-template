package frc.robot.studies;

import static frc.robot.studies.StudyFixtures.BACK;
import static frc.robot.studies.StudyFixtures.FRONT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.tuple;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import org.junit.jupiter.api.Test;

/** Ranking layouts by how they hold up across trials, from scores written by hand. */
class RobustnessTest {
  private static final Candidate SHARP = new Candidate("sharp", List.of(FRONT, BACK));
  private static final Candidate STEADY = new Candidate("steady", List.of(FRONT, BACK));
  private static final Candidate SINGLE = new Candidate("single", List.of(FRONT));
  private static final Candidate BLIND = new Candidate("blind", List.of(BACK));
  private static final List<Conditions> TWO_TRIALS =
      List.of(
          Conditions.trial(CameraStudyConfig.DEFAULTS, 0),
          Conditions.trial(CameraStudyConfig.DEFAULTS, 1));

  private static Score scored(Candidate candidate, double p95, double withinFive) {
    return new Score(candidate, p95, withinFive, 0, 1, 1, 1, 1, 1, 0.01, 0.1, null);
  }

  /**
   * "Sharp" is best with the simulator's cameras and falls apart with worse ones; "steady" is a
   * little behind and stays there. The robust ranking puts steady first.
   */
  @Test
  void aLayoutBestOnlyWithPerfectCamerasDoesntWin() {
    List<Score> nominal =
        List.of(scored(SHARP, 0.020, 1), scored(STEADY, 0.024, 1), scored(SINGLE, 0.05, 0.9));
    List<List<Score>> trials =
        List.of(
            List.of(scored(SHARP, 0.030, 1), scored(STEADY, 0.026, 1), scored(SINGLE, 0.06, 0.9)),
            List.of(
                scored(SINGLE, Double.POSITIVE_INFINITY, 0.5),
                scored(STEADY, 0.029, 1),
                scored(SHARP, 0.045, 0.97)));

    Robustness robustness = new Robustness(nominal, TWO_TRIALS, trials);

    List<Robustness.Layout> ranking = robustness.ranking();
    assertThat(ranking)
        .extracting(layout -> layout.candidate().name())
        .containsExactly("steady", "sharp", "single");
    Robustness.Layout steady = ranking.get(0);
    assertThat(steady.nominalRank()).isEqualTo(2);
    assertThat(steady.trialP95Meters()).containsExactly(0.026, 0.029);
    assertThat(steady.worstP95Meters()).isEqualTo(0.029);
    assertThat(steady.medianP95Meters()).isCloseTo(0.0275, within(1e-12));
    Robustness.Layout single = ranking.get(2);
    assertThat(single.worstP95Meters()).isInfinite();
    assertThat(single.worstWithinFiveCentimeters()).isEqualTo(0.5);
    // Ranks moved: steady 2 -> 1, sharp 1 -> 2, single stays 3: 1 - 6 x 2 / (3 x 8).
    assertThat(robustness.rankCorrelation()).isCloseTo(0.5, within(1e-12));
  }

  /**
   * Layouts that leave more than 5% of the field unmeasured in some trial have no worst 95% error:
   * they rank after the rest, the one placing more of the field within 5 cm in its worst trial
   * first, whatever their nominal order.
   */
  @Test
  void blindLayoutsRankByTheLeastOfTheFieldTheyPlaceWithinFiveCentimeters() {
    List<Score> nominal =
        List.of(scored(BLIND, 0.03, 0.95), scored(SINGLE, 0.04, 0.95), scored(STEADY, 0.025, 1));
    List<List<Score>> trials =
        List.of(
            List.of(
                scored(BLIND, Double.POSITIVE_INFINITY, 0.6),
                scored(SINGLE, Double.POSITIVE_INFINITY, 0.7),
                scored(STEADY, 0.03, 1)),
            List.of(scored(BLIND, 0.05, 0.9), scored(SINGLE, 0.04, 0.92), scored(STEADY, 0.03, 1)));

    Robustness robustness = new Robustness(nominal, TWO_TRIALS, trials);

    assertThat(robustness.ranking())
        .extracting(layout -> layout.candidate().name(), Robustness.Layout::nominalRank)
        .containsExactly(tuple("steady", 1), tuple("single", 3), tuple("blind", 2));
  }

  /** Worst trials that round to the same millimeter are a tie, and a tie goes to fewer cameras. */
  @Test
  void aTieToTheMillimeterGoesToFewerCameras() {
    List<Score> nominal = List.of(scored(SHARP, 0.020, 1), scored(SINGLE, 0.021, 1));
    List<List<Score>> trials =
        List.of(
            List.of(scored(SHARP, 0.0251, 1), scored(SINGLE, 0.0254, 1)),
            List.of(scored(SHARP, 0.022, 1), scored(SINGLE, 0.022, 1)));

    Robustness robustness = new Robustness(nominal, TWO_TRIALS, trials);

    assertThat(robustness.ranking())
        .extracting(layout -> layout.candidate().name())
        .containsExactly("single", "sharp");
  }

  @Test
  void everyLayoutMustBeScoredInEveryTrial() {
    List<Score> nominal = List.of(scored(SHARP, 0.020, 1), scored(SINGLE, 0.021, 1));

    assertThatThrownBy(
            () ->
                new Robustness(
                    nominal,
                    TWO_TRIALS,
                    List.of(List.of(scored(SHARP, 0.02, 1)), List.of(scored(SHARP, 0.02, 1)))))
        .hasMessageContaining("not scored in every trial");
    assertThatThrownBy(() -> new Robustness(nominal, List.of(), List.of()))
        .hasMessageContaining("a trial at least");
  }
}
