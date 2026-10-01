package frc.robot.lib.localization;

import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A robot pose worked out from a camera's view of AprilTags, ready for the estimator: where the
 * robot was when the image was taken, how uncertain that is, and where the pose came from, so every
 * accepted or rejected measurement can be traced to its camera, tags, and solver.
 *
 * @param timestampNanos when the image was captured; the estimator applies the measurement at that
 *     moment, not when it arrived
 * @param pose the robot's pose on the field
 * @param covariance its uncertainty. A solve that took its heading from the gyro says nothing new
 *     about heading, so its heading variance is infinite.
 * @param robotToTags from the robot's center to the tags its position was solved from, on the
 *     field: their middle, weighted as the solve weighed them. A position solved with the gyro's
 *     heading placed the robot by turning its view of the tags by that heading, so a heading off by
 *     an angle puts the position off by that angle times this, square to it, the same way in every
 *     image: the estimator carries that in the measurement's uncertainty. Zero for a solve that
 *     found its own heading (multi-tag, single tag, external), whose position doesn't lean on the
 *     gyro's.
 * @param camera which camera
 * @param tagIds the tags it used
 * @param solver how it was solved
 */
public record VisionMeasurement(
    long timestampNanos,
    Pose2d pose,
    PoseCovariance covariance,
    Translation2d robotToTags,
    String camera,
    List<Integer> tagIds,
    Solver solver) {
  /** How a pose was solved from the tags. */
  public enum Solver {
    /** Position only, with the heading taken from the gyro: the default, and the most robust. */
    HEADING_CONSTRAINED,
    /** Every visible tag solved at once, on the coprocessor. */
    MULTI_TAG,
    /** One tag alone: ambiguous unless the heading settles it. */
    SINGLE_TAG,
    /** A pose the vision system solved its own way, such as a Limelight's. */
    EXTERNAL
  }

  public VisionMeasurement {
    if (timestampNanos < 0) {
      throw new IllegalArgumentException("a time on the robot's clock: " + timestampNanos);
    }
    if (camera.isBlank()) {
      throw new IllegalArgumentException("a vision measurement needs its camera's name");
    }
    if (tagIds.isEmpty()) {
      throw new IllegalArgumentException("a vision measurement comes from at least one tag");
    }
    if (!Double.isFinite(robotToTags.getX()) || !Double.isFinite(robotToTags.getY())) {
      throw new IllegalArgumentException("the robot's offset to its tags: " + robotToTags);
    }
    tagIds = List.copyOf(tagIds);
  }
}
