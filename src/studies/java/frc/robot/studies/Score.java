package frc.robot.studies;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

/**
 * How well a layout of cameras does over the whole field. Each share is of the poses, weighted, and
 * of every set of images taken at them.
 *
 * @param candidate the layout
 * @param positionP95Meters the position error that 95% of the field is within: the ranking's score.
 *     A spot with no measurement counts as missed by any distance, so a layout that measures less
 *     than 95% of the field has none (infinite).
 * @param withinFiveCentimeters the share where the combined measurement is within 5 cm of where the
 *     robot really is
 * @param withinTwoCentimeters the share where it's within 2 cm
 * @param oneTag the share where a camera sees at least one tag
 * @param twoTags the share where one camera sees two or more at once: a multi-tag solve needs two,
 *     and so does correcting the gyro's heading
 * @param twoCameras the share where two or more cameras see tags, so one blocked or knocked camera
 *     doesn't leave the robot blind
 * @param measured the share where an image makes a measurement for the estimator
 * @param heading the share where an image corrects the gyro's heading
 * @param positionRmsMeters the position error where measured, root mean square; NaN if never
 * @param headingRmsDegrees the heading error where corrected, root mean square; NaN if never
 */
public record Score(
    Candidate candidate,
    double positionP95Meters,
    double withinFiveCentimeters,
    double withinTwoCentimeters,
    double oneTag,
    double twoTags,
    double twoCameras,
    double measured,
    double heading,
    double positionRmsMeters,
    double headingRmsDegrees) {
  static final double FIVE_CENTIMETERS = 0.05;
  static final double TWO_CENTIMETERS = 0.02;

  /**
   * The ranking: the smallest error that 95% of the field is within, to the millimeter, which is
   * about the sampling's own noise. A tie goes to fewer cameras, which cost less and weigh less,
   * then to the smaller error, then by name, so ties always fall the same way. A layout that
   * measures less than 95% of the field has no such error; those rank after the rest, by how much
   * of the field they place within 5 cm.
   */
  public static final Comparator<Score> RANKING =
      Comparator.comparingDouble((Score score) -> Math.round(score.positionP95Meters() * 1000))
          .thenComparingDouble(Score::blindByWithin)
          .thenComparingInt(Score::cameras)
          .thenComparingDouble(Score::positionP95Meters)
          .thenComparing(score -> score.candidate().name());

  /** How many cameras the layout has. */
  public int cameras() {
    return candidate.cameras().size();
  }

  /** For a layout without a 95th percentile, more of the field within 5 cm first; else nothing. */
  private double blindByWithin() {
    return Double.isInfinite(positionP95Meters) ? -withinFiveCentimeters : 0;
  }

  /**
   * A layout's score from its outcomes.
   *
   * @param outcomes for each sample, one outcome per set of images taken there
   */
  public static Score of(Candidate candidate, List<Sample> samples, Outcome[][] outcomes) {
    int count = 0;
    for (Outcome[] here : outcomes) {
      count += here.length;
    }
    double[] errors = new double[count];
    double[] weights = new double[count];
    double total = 0;
    double withinFive = 0;
    double withinTwo = 0;
    double oneTag = 0;
    double twoTags = 0;
    double twoCameras = 0;
    double measured = 0;
    double heading = 0;
    double squaredPosition = 0;
    double squaredHeading = 0;
    int next = 0;
    for (int i = 0; i < samples.size(); i++) {
      Outcome[] here = outcomes[i];
      double weight = samples.get(i).weight() / here.length;
      for (Outcome outcome : here) {
        total += weight;
        oneTag += outcome.camerasSeeingTags() >= 1 ? weight : 0;
        twoTags += outcome.mostTagsInOneImage() >= 2 ? weight : 0;
        twoCameras += outcome.camerasSeeingTags() >= 2 ? weight : 0;
        withinFive += outcome.localized(FIVE_CENTIMETERS) ? weight : 0;
        withinTwo += outcome.localized(TWO_CENTIMETERS) ? weight : 0;
        errors[next] = Double.POSITIVE_INFINITY;
        weights[next] = weight;
        if (outcome.measured()) {
          double error = outcome.positionErrorMeters();
          errors[next] = error;
          measured += weight;
          squaredPosition += weight * error * error;
        }
        if (outcome.correctsHeading()) {
          heading += weight;
          squaredHeading += weight * Math.pow(Math.toDegrees(outcome.headingErrorRadians()), 2);
        }
        next++;
      }
    }
    if (!(total > 0)) {
      throw new IllegalArgumentException("every pose weighs nothing: nothing left to score");
    }
    return new Score(
        candidate,
        percentile(errors, weights, 0.95),
        withinFive / total,
        withinTwo / total,
        oneTag / total,
        twoTags / total,
        twoCameras / total,
        measured / total,
        heading / total,
        measured > 0 ? Math.sqrt(squaredPosition / measured) : Double.NaN,
        heading > 0 ? Math.sqrt(squaredHeading / heading) : Double.NaN);
  }

  /** The error that this share of the weight falls at or below. */
  static double percentile(double[] errors, double[] weights, double share) {
    Integer[] order = new Integer[errors.length];
    double total = 0;
    for (int i = 0; i < order.length; i++) {
      order[i] = i;
      total += weights[i];
    }
    Arrays.sort(order, Comparator.comparingDouble(i -> errors[i]));
    double reached = 0;
    for (int i : order) {
      reached += weights[i];
      if (reached >= share * total) {
        return errors[i];
      }
    }
    return errors[order[order.length - 1]];
  }
}
