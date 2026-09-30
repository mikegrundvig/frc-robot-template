package frc.robot.lib.localization;

import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;

/**
 * What the cameras need from localization, and no more: where the robot was when an image was
 * taken, and where to send what they measured. The {@link PoseEstimator} is one.
 */
public interface CameraLocalizer {
  /** Where the robot was at a moment, if the estimate reaches back that far. */
  Optional<Pose2d> poseAt(long timestampNanos);

  /** A camera's measurement, applied at the moment its image was taken on the next update. */
  void addVision(VisionMeasurement measurement);
}
