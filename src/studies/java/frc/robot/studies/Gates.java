package frc.robot.studies;

import com.michaelgrundvig.frc.localization.LocalizationGate;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.runtime.ExampleRobot;

/**
 * The robot code's tests on the estimate's claimed uncertainty, as the consistency study counts
 * them: each says whether a loop's estimate would pass. An overconfident estimator passes them when
 * it shouldn't; a timid one fails them when it needn't.
 */
final class Gates {
  /** How sure the pose must be to calibrate a camera against: Vision's 5 cm. */
  static final double CALIBRATING_METERS = 0.05;

  /** The distance a tag is judged at, for the mount check's test: a typical one. */
  static final double JUDGING_DISTANCE_METERS = 3;

  /** How sure of its heading a robot placed on its start claims to be: RobotContainer's 2°. */
  static final double PLACED_HEADING_RADIANS = Math.toRadians(2);

  private static final VisionConfig VISION = ExampleRobot.SIM.vision();

  private Gates() {}

  /** Vision calibrates a camera only while the pose is tracking and sure within 5 cm. */
  static boolean calibrating(Consistency.Sample sample) {
    return sample.tracking() && sample.covariance().largestPositionStdDev() <= CALIBRATING_METERS;
  }

  /**
   * Vision counts a tag toward its camera's mount check only while the pose is tracking, and its
   * position and heading together would put a tag at that distance within the tolerance. (It also
   * needs the robot turning slowly and another camera agreeing, which the estimate doesn't say.)
   */
  static boolean judgingMounts(Consistency.Sample sample) {
    return sample.tracking()
        && Math.hypot(
                sample.covariance().largestPositionStdDev(),
                JUDGING_DISTANCE_METERS * sample.covariance().headingStdDev())
            <= VISION.mountToleranceMeters();
  }

  /** The scoring gate's test on the position's uncertainty. */
  static boolean scoring(Consistency.Sample sample) {
    return sample.covariance().largestPositionStdDev()
        <= LocalizationGate.SCORING.maxPositionStdDevMeters();
  }

  /** The vision pipeline solves with the gyro's heading only while the heading is this sure. */
  static boolean constraining(Consistency.Sample sample) {
    return sample.covariance().headingStdDev() <= VISION.maxHeadingStdDevToConstrain();
  }

  /** The heading as sure as a robot placed on its start claims. */
  static boolean asPlaced(Consistency.Sample sample) {
    return sample.covariance().headingStdDev() <= PLACED_HEADING_RADIANS;
  }
}
