package frc.robot.lib.localization;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Whether a camera still sits where its configuration says, from where its tags land: each tag it
 * sees, placed on the field through the estimated pose and the camera's configured mount, should
 * land on the tag. Noise, and the estimate's own small errors, put tags off every which way, and
 * average out. A camera bumped since it was calibrated puts them off the same way every time, in
 * the robot's frame, where its mount moved.
 *
 * <p>So the check averages each tag's miss in the robot's frame, over the latest tags: that average
 * is how far off the camera's mount is. Beyond a tolerance, the camera is out of calibration. Only
 * the level part of the miss moves the camera's measurements, since an image fixes where the robot
 * stands on the floor: that part, beyond the tolerance, is how much less the camera is trusted
 * ({@link #doubtMeters}). A camera knocked straight down still places the robot right on level
 * ground; it is out of calibration all the same, and every tag it sees lands high.
 *
 * <p>The check measures against the estimate, so it needs other cameras to hold the estimate: with
 * one camera, the estimate follows that camera, and its tags land where they should whatever its
 * mount. And a heading error in the estimate (after a gyro's scale error over many turns, say) puts
 * every camera's tags off as if each were turned on its mount: the cameras correct the heading, and
 * the offsets fade with it.
 */
public final class MountCheck {
  private final int sightings;
  private int seen;
  private double x;
  private double y;
  private double z;

  /**
   * @param sightings how many of the latest tags the camera is judged by: enough that noise and the
   *     estimate's errors average out, few enough that a bump shows within seconds
   */
  public MountCheck(int sightings) {
    if (sightings < 1) {
      throw new IllegalArgumentException("judge by at least one sighting: " + sightings);
    }
    this.sightings = sightings;
  }

  /**
   * A tag the camera saw.
   *
   * @param robot where the robot was when the image was taken, raised and tilted as it was
   * @param seen the tag, placed through that pose and the camera's configured mount
   * @param real where the tag really is
   */
  public void add(Pose3d robot, Pose3d seen, Pose3d real) {
    Translation3d miss =
        seen.getTranslation().minus(real.getTranslation()).rotateBy(robot.getRotation().inverse());
    // The average of every tag so far, until there are enough; from then on each new tag counts
    // as one of the latest, and the oldest fade.
    this.seen++;
    double weight = 1.0 / Math.min(this.seen, sightings);
    x += weight * (miss.getX() - x);
    y += weight * (miss.getY() - y);
    z += weight * (miss.getZ() - z);
  }

  /** Whether the camera has seen enough tags to judge its mount by. */
  public boolean judged() {
    return seen >= sightings;
  }

  /**
   * How far off, on average, the camera's tags land: a camera mounted 4 in lower than configured
   * puts every tag 4 in high. NaN until it has seen enough tags to judge.
   */
  public double offsetMeters() {
    return judged() ? Math.sqrt(x * x + y * y + z * z) : Double.NaN;
  }

  /** Whether the camera's tags land farther off than a calibrated camera's do. */
  public boolean outOfCalibration(double toleranceMeters) {
    return offsetMeters() > toleranceMeters;
  }

  /**
   * How far the camera's measurements may be off from where its mount moved, in meters: none while
   * its tags land within the tolerance. The tolerance stands for what a calibrated camera shows
   * (mostly the estimate's own error, which isn't the camera's), so it comes off the level offset
   * the way independent errors add: in squares.
   */
  public double doubtMeters(double toleranceMeters) {
    double level = x * x + y * y;
    if (!judged() || level <= toleranceMeters * toleranceMeters) {
      return 0;
    }
    return Math.sqrt(level - toleranceMeters * toleranceMeters);
  }
}
