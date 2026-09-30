package frc.robot.studies;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * How well a layout of cameras does over the whole field. Each share is of the poses, weighted, and
 * of every set of images taken at them.
 *
 * @param candidate the layout
 * @param localized the share where the combined measurement is within the target of where the robot
 *     really is: the ranking's score
 * @param oneTag the share where a camera sees at least one tag
 * @param twoTags the share where one camera sees two or more at once: a multi-tag solve needs two,
 *     and so does correcting the gyro's heading
 * @param twoCameras the share where two or more cameras see tags, so one blocked or knocked camera
 *     doesn't leave the robot blind
 * @param measured the share where an image makes a measurement for the estimator
 * @param heading the share where an image corrects the gyro's heading
 * @param positionRmsMeters the position error where measured, root mean square
 * @param positionP95Meters the position error where measured, 95th percentile
 * @param headingRmsDegrees the heading error where corrected, root mean square; NaN if never
 */
public record Score(
    Candidate candidate,
    double localized,
    double oneTag,
    double twoTags,
    double twoCameras,
    double measured,
    double heading,
    double positionRmsMeters,
    double positionP95Meters,
    double headingRmsDegrees) {
  /**
   * The ranking: the most of the field localized first; then fewer cameras, which cost less and
   * weigh less; then the smaller position error; then by name, so ties always fall the same way.
   */
  public static final Comparator<Score> RANKING =
      Comparator.comparingDouble(Score::localized)
          .reversed()
          .thenComparingInt(Score::cameras)
          .thenComparingDouble(Score::positionRmsMeters) // none at all sorts last
          .thenComparing(score -> score.candidate().name());

  /** How many cameras the layout has. */
  public int cameras() {
    return candidate.cameras().size();
  }

  /**
   * A layout's score from its outcomes.
   *
   * @param outcomes for each sample, one outcome per set of images taken there
   * @param targetMeters how close counts as localized
   */
  public static Score of(
      Candidate candidate, List<Sample> samples, Outcome[][] outcomes, double targetMeters) {
    double total = 0;
    double localized = 0;
    double oneTag = 0;
    double twoTags = 0;
    double twoCameras = 0;
    double measured = 0;
    double heading = 0;
    double squaredPosition = 0;
    double squaredHeading = 0;
    List<double[]> errors = new ArrayList<>(); // each measured image set's error, with its weight
    for (int i = 0; i < samples.size(); i++) {
      Outcome[] here = outcomes[i];
      double weight = samples.get(i).weight() / here.length;
      for (Outcome outcome : here) {
        total += weight;
        oneTag += outcome.camerasSeeingTags() >= 1 ? weight : 0;
        twoTags += outcome.mostTagsInOneImage() >= 2 ? weight : 0;
        twoCameras += outcome.camerasSeeingTags() >= 2 ? weight : 0;
        if (outcome.measured()) {
          double error = outcome.positionErrorMeters();
          measured += weight;
          localized += error <= targetMeters ? weight : 0;
          squaredPosition += weight * error * error;
          errors.add(new double[] {error, weight});
        }
        if (outcome.correctsHeading()) {
          heading += weight;
          squaredHeading += weight * Math.pow(Math.toDegrees(outcome.headingErrorRadians()), 2);
        }
      }
    }
    if (!(total > 0)) {
      throw new IllegalArgumentException("every pose weighs nothing: nothing left to score");
    }
    return new Score(
        candidate,
        localized / total,
        oneTag / total,
        twoTags / total,
        twoCameras / total,
        measured / total,
        heading / total,
        measured > 0 ? Math.sqrt(squaredPosition / measured) : Double.NaN,
        percentile(errors, 0.95),
        heading > 0 ? Math.sqrt(squaredHeading / heading) : Double.NaN);
  }

  /** The error that this share of the weight falls at or below; NaN with none. */
  private static double percentile(List<double[]> errors, double share) {
    if (errors.isEmpty()) {
      return Double.NaN;
    }
    errors.sort(Comparator.comparingDouble(error -> error[0]));
    double total = 0;
    for (double[] error : errors) {
      total += error[1];
    }
    double reached = 0;
    for (double[] error : errors) {
      reached += error[1];
      if (reached >= share * total) {
        return error[0];
      }
    }
    return errors.get(errors.size() - 1)[0];
  }
}
