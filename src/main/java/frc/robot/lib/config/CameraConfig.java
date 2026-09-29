package frc.robot.lib.config;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.math.geometry.Transform3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Time;

/**
 * A camera on the robot, for AprilTag pose estimation. The transform is what turns what the camera
 * sees into where the robot is, so measure it carefully. The rest describes the camera for
 * simulation.
 *
 * @param name unique on the robot; use the name the vision software knows the camera by
 * @param robotToCamera where the camera sits and points, from the robot's origin (the frame's
 *     center, on the floor). Rotation is roll, pitch, yaw in WPILib's convention: a negative pitch
 *     tilts the camera up, a positive yaw turns it to the left.
 * @param widthPixels width of the images the vision software processes
 * @param heightPixels height of those images
 * @param diagonalFieldOfView the angle the camera sees across the image's diagonal
 * @param frameRate images processed per second
 * @param latency the average time from taking an image to its result reaching the robot
 */
public record CameraConfig(
    String name,
    Transform3d robotToCamera,
    int widthPixels,
    int heightPixels,
    Angle diagonalFieldOfView,
    Frequency frameRate,
    Time latency) {
  public CameraConfig {
    Require.notBlank("camera name", name);
    Require.positive(name + " widthPixels", widthPixels);
    Require.positive(name + " heightPixels", heightPixels);
    Require.positive(name + " diagonalFieldOfView", diagonalFieldOfView);
    if (diagonalFieldOfView.gte(Degrees.of(180))) {
      throw new IllegalArgumentException(
          name
              + " diagonalFieldOfView must be under 180 degrees, got "
              + diagonalFieldOfView.toShortString());
    }
    Require.positive(name + " frameRate", frameRate);
    Require.nonNegative(name + " latency", latency);
  }
}
