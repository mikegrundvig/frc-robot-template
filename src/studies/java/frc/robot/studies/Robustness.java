package frc.robot.studies;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;

/**
 * How every layout holds up when the cameras and the robot err more, or less, than the simulator
 * assumes: its 95% error in each robustness trial's {@link Conditions}, and the worst of them,
 * which ranks it. Every layout faces the same trials, so they're compared fairly; a layout that's
 * best only with perfect cameras falls behind one that stays good when they aren't.
 */
public final class Robustness {
  /**
   * One layout across the trials.
   *
   * @param nominal its score in the nominal conditions
   * @param nominalRank its rank there
   * @param trialP95Meters its 95% error in each trial, in order: infinite where more than 5% of the
   *     field went unmeasured
   * @param medianP95Meters the median of those
   * @param worstP95Meters the worst of those: the robust ranking's score
   * @param worstWithinFiveCentimeters the least of the field it placed within 5 cm in any trial
   */
  public record Layout(
      Score nominal,
      int nominalRank,
      List<Double> trialP95Meters,
      double medianP95Meters,
      double worstP95Meters,
      double worstWithinFiveCentimeters) {
    public Layout {
      trialP95Meters = List.copyOf(trialP95Meters);
    }

    public Candidate candidate() {
      return nominal.candidate();
    }

    public int cameras() {
      return nominal.cameras();
    }

    /** For a layout without a worst 95th percentile, more of the field within 5 cm first. */
    private double blindByWithin() {
      return Double.isInfinite(worstP95Meters) ? -worstWithinFiveCentimeters : 0;
    }
  }

  /**
   * The robust ranking: the smallest worst-trial 95% error, to the millimeter; a tie to fewer
   * cameras, then the smaller worst error, the smaller median, and the name. A layout with no 95%
   * error in some trial ranks after the rest, by the least of the field it placed within 5 cm.
   */
  public static final Comparator<Layout> RANKING =
      Comparator.comparingDouble((Layout layout) -> Math.round(layout.worstP95Meters() * 1000))
          .thenComparingDouble(Layout::blindByWithin)
          .thenComparingInt(Layout::cameras)
          .thenComparingDouble(Layout::worstP95Meters)
          .thenComparingDouble(Layout::medianP95Meters)
          .thenComparing(layout -> layout.candidate().name());

  private final List<Conditions> trials;
  private final List<Layout> ranking;

  /**
   * @param nominal every layout's score in the nominal conditions, in any order
   * @param trials each trial's conditions
   * @param trialScores each trial's scores, every layout's, in any order: a layout is known by its
   *     own object, as scored, so two layouts alike in every way stay two
   */
  public Robustness(List<Score> nominal, List<Conditions> trials, List<List<Score>> trialScores) {
    if (trials.isEmpty() || trials.size() != trialScores.size()) {
      throw new IllegalArgumentException("one list of scores for each trial, and a trial at least");
    }
    List<Score> ranked = new ArrayList<>(nominal);
    ranked.sort(Score.RANKING);
    this.trials = List.copyOf(trials);
    List<Map<Candidate, Score>> byTrial = new ArrayList<>();
    for (List<Score> scores : trialScores) {
      Map<Candidate, Score> byCandidate = new IdentityHashMap<>();
      scores.forEach(score -> byCandidate.put(score.candidate(), score));
      byTrial.add(byCandidate);
    }
    List<Layout> layouts = new ArrayList<>();
    for (int rank = 0; rank < ranked.size(); rank++) {
      Score score = ranked.get(rank);
      List<Double> p95s = new ArrayList<>();
      double worstWithin = 1;
      for (Map<Candidate, Score> trial : byTrial) {
        Score inTrial = trial.get(score.candidate());
        if (inTrial == null) {
          throw new IllegalArgumentException("not scored in every trial: " + score.candidate());
        }
        p95s.add(inTrial.positionP95Meters());
        worstWithin = Math.min(worstWithin, inTrial.withinFiveCentimeters());
      }
      List<Double> sorted = new ArrayList<>(p95s);
      sorted.sort(Comparator.naturalOrder());
      int middle = sorted.size() / 2;
      double median =
          sorted.size() % 2 == 1
              ? sorted.get(middle)
              : (sorted.get(middle - 1) + sorted.get(middle)) / 2;
      layouts.add(
          new Layout(score, rank + 1, p95s, median, sorted.get(sorted.size() - 1), worstWithin));
    }
    layouts.sort(RANKING);
    ranking = List.copyOf(layouts);
  }

  public List<Conditions> trials() {
    return trials;
  }

  /** Every layout, the most robust first. */
  public List<Layout> ranking() {
    return ranking;
  }

  /**
   * How alike this ranking and the nominal one are: Spearman's rank correlation, 1 when they're the
   * same, near 0 when one says nothing about the other.
   */
  public double rankCorrelation() {
    double n = ranking.size();
    if (n < 2) {
      return 1;
    }
    double squares = 0;
    for (int rank = 0; rank < ranking.size(); rank++) {
      double moved = ranking.get(rank).nominalRank() - (rank + 1);
      squares += moved * moved;
    }
    return 1 - 6 * squares / (n * (n * n - 1));
  }
}
