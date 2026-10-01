package frc.robot.studies;

/**
 * What one set of images tells the robot at one pose: an image from each of a layout's cameras, all
 * taken at once.
 *
 * @param mostTagsInOneImage the most tags any one camera saw
 * @param camerasSeeingTags how many cameras saw at least one tag
 * @param positionErrorMeters how far the cameras' combined measurement is from where the robot
 *     really is; NaN when no image made a measurement
 * @param headingErrorRadians how far the combined heading correction is from the robot's real
 *     heading; NaN when no image corrected heading
 */
public record Outcome(
    int mostTagsInOneImage,
    int camerasSeeingTags,
    double positionErrorMeters,
    double headingErrorRadians) {
  /** Whether any image made a measurement for the estimator. */
  public boolean measured() {
    return !Double.isNaN(positionErrorMeters);
  }

  /** Whether the combined measurement is within a distance of where the robot really is. */
  public boolean localized(double withinMeters) {
    return measured() && positionErrorMeters <= withinMeters;
  }

  /** Whether any image corrected the gyro's heading. */
  public boolean correctsHeading() {
    return !Double.isNaN(headingErrorRadians);
  }
}
