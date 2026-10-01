package frc.robot.lib.config;

import static frc.robot.lib.config.Rules.below;
import static frc.robot.lib.config.Rules.finite;
import static frc.robot.lib.config.Rules.nonNegative;
import static frc.robot.lib.config.Rules.notBlank;
import static frc.robot.lib.config.Rules.positive;
import static org.wpilib.units.Units.Degrees;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Time;

/**
 * A camera on the robot, for AprilTag pose estimation. Where it sits and points is what turns what
 * the camera sees into where the robot is, so measure it carefully. The sensor describes the camera
 * itself, for simulation.
 *
 * <p>Positions are from the robot's origin: the frame's center, on the floor. Angles apply in the
 * order yaw, tilt, roll, each about the camera's own axes.
 *
 * @param name unique on the robot; use the name the vision software knows the camera by
 * @param forward how far ahead of the center the lens sits (negative: behind it)
 * @param left how far left of the center (negative: to the right)
 * @param up how far above the floor
 * @param roll rotation about the lens's axis; zero for a camera mounted level
 * @param tiltUp how far the camera is tilted up from level (negative: down)
 * @param yaw which way it faces: zero is straight ahead, positive turns it to the left
 * @param sensor the camera itself, often shared by several cameras on a robot
 * @param noiseScale how much noisier this camera's results are than a typical one's: 1 for typical,
 *     2 to trust it half as much. Fit it from the camera's logged residuals.
 */
@ConfigRecord
public record CameraConfig(
    String name,
    Distance forward,
    Distance left,
    Distance up,
    Angle roll,
    Angle tiltUp,
    Angle yaw,
    Sensor sensor,
    double noiseScale) {
  public CameraConfig {
    Checks.on(CameraConfig.class, name)
        .field("name", name, notBlank())
        .field("forward", forward, finite())
        .field("left", left, finite())
        .field("up", up, finite())
        .field("roll", roll, finite())
        .field("tiltUp", tiltUp, finite())
        .field("yaw", yaw, finite())
        .field("noiseScale", noiseScale, positive())
        .orThrow();
  }

  /** Starts building a camera, one named value at a time. */
  public static CameraConfigBuilder.NameStage builder() {
    return CameraConfigBuilder.builder();
  }

  /** Where the lens sits, from the robot's origin. */
  public Translation3d position() {
    return new Translation3d(forward, left, up);
  }

  /**
   * Where the camera sits and points, in the form WPILib's vision code takes. WPILib's pitch is
   * positive downward, so the tilt up becomes a negative pitch.
   */
  public Transform3d robotToCamera() {
    return new Transform3d(position(), new Rotation3d(roll, tiltUp.unaryMinus(), yaw));
  }

  /**
   * The camera itself, as the vision software runs it.
   *
   * @param widthPixels width of the images the vision software processes
   * @param heightPixels height of those images
   * @param diagonalFieldOfView the angle the camera sees across the image's diagonal
   * @param frameRate images processed per second
   * @param latency the average time from taking an image to its result reaching the robot. Images
   *     that take much longer are thrown out, so measure it: the Vision log's {@code
   *     LatencySeconds} for the camera, less the wait for the robot's loop. The simulator's cameras
   *     take this long too.
   * @param sightRange the farthest a tag is found in most images: farther tags are too small in the
   *     image to read. The simulator's cameras detect tags no farther than this, and AdvantageScope
   *     draws each camera's field of view out to it.
   */
  @ConfigRecord
  public record Sensor(
      int widthPixels,
      int heightPixels,
      Angle diagonalFieldOfView,
      Frequency frameRate,
      Time latency,
      Distance sightRange) {
    public Sensor {
      Checks.on(Sensor.class)
          .field("widthPixels", widthPixels, positive())
          .field("heightPixels", heightPixels, positive())
          .field("diagonalFieldOfView", diagonalFieldOfView, positive())
          .field("diagonalFieldOfView", diagonalFieldOfView, below(Degrees.of(180)))
          .field("frameRate", frameRate, positive())
          .field("latency", latency, nonNegative())
          .field("sightRange", sightRange, positive())
          .orThrow();
    }

    /** Starts building a camera sensor, one named value at a time. */
    public static CameraConfigSensorBuilder.WidthPixelsStage builder() {
      return CameraConfigSensorBuilder.builder();
    }
  }
}
