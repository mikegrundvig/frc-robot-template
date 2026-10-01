package frc.robot.studies;

import static frc.robot.lib.config.Rules.atLeast;
import static frc.robot.lib.config.Rules.nonNegative;
import static frc.robot.lib.config.Rules.positive;
import static frc.robot.lib.config.Rules.shown;
import static org.wpilib.units.Units.Meters;

import frc.robot.Robots;
import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;
import frc.robot.subsystems.vision.VisionSim;

/**
 * How the camera placement study samples the field, how the cameras and robot err, and how it
 * judges what the cameras see (see docs/camera-placement.md).
 *
 * @param gridMeters how far apart the robot positions are, both ways
 * @param headings how many headings the robot takes at each position, evenly spaced
 * @param frames how many images each camera takes at each pose, each with its own pixel noise
 * @param cornerNoiseMeanPixels how far each tag corner lands from where a perfect camera would put
 *     it, on average, in pixels: the simulator's own by default
 * @param cornerNoiseStdDevPixels and that distance's spread: the simulator's own by default
 * @param sightMeters tags farther than this aren't detected: by default the sight range of the
 *     robot's cameras ({@code CameraConfig.Sensor.sightRange}), which the simulator's use too
 * @param gyroErrorDegrees how far off the gyro's heading typically is (one standard deviation):
 *     every solve takes its heading from the gyro, so its error moves every position solved
 * @param mountErrorMillimeters how far each camera really sits from its configured place, at most,
 *     along each axis: none by default, as in the simulator
 * @param mountErrorDegrees how far each camera is really turned, at most, about each axis
 * @param otherRobotsLeast the fewest other robots in the way of the cameras, around each pose
 * @param otherRobotsMost the most, five at most: each set of images has a number between, drawn
 *     from the seed
 * @param otherRobotsHeightMeters how tall the other robots are; 0, the default, makes them as tall
 *     as the robot being designed
 * @param trials how many robustness trials, each drawing all of these errors from plausible ranges;
 *     0 scores the field once, with the errors as set here
 * @param mostCameras the most cameras a layout may have: larger layouts from the family, or from
 *     CAD's places, are left out
 * @param seed where every random draw starts: the same seed gives the same report
 * @param threads how many threads share the work
 */
@ConfigRecord
public record CameraStudyConfig(
    double gridMeters,
    int headings,
    int frames,
    double cornerNoiseMeanPixels,
    double cornerNoiseStdDevPixels,
    double sightMeters,
    double gyroErrorDegrees,
    double mountErrorMillimeters,
    double mountErrorDegrees,
    int otherRobotsLeast,
    int otherRobotsMost,
    double otherRobotsHeightMeters,
    int trials,
    int mostCameras,
    long seed,
    int threads) {
  /** A robustness run's trials by default: under ten minutes on two threads. */
  static final int ROBUST_TRIALS = 12;

  /**
   * A run of about a minute: every half meter, every 45 degrees, two images per camera, with the
   * simulator's own camera errors and the gyro a quarter degree off. Two threads, since the
   * computer may be shared.
   */
  public static final CameraStudyConfig DEFAULTS =
      builder()
          .gridMeters(0.5)
          .headings(8)
          .frames(2)
          .cornerNoiseMeanPixels(VisionSim.CORNER_NOISE_MEAN_PIXELS)
          .cornerNoiseStdDevPixels(VisionSim.CORNER_NOISE_STD_DEV_PIXELS)
          .sightMeters(Robots.current().cameras().get(0).sensor().sightRange().in(Meters))
          .gyroErrorDegrees(0.25)
          .mountErrorMillimeters(0)
          .mountErrorDegrees(0)
          .otherRobotsLeast(0)
          .otherRobotsMost(0)
          .otherRobotsHeightMeters(0)
          .trials(0)
          .mostCameras(4)
          .seed(1)
          .threads(2)
          .build();

  public CameraStudyConfig {
    Checks.on(CameraStudyConfig.class)
        .field("gridMeters", gridMeters, positive())
        .field("headings", headings, atLeast(1))
        .field("frames", frames, atLeast(1))
        .field("cornerNoiseMeanPixels", cornerNoiseMeanPixels, nonNegative())
        .field("cornerNoiseStdDevPixels", cornerNoiseStdDevPixels, nonNegative())
        .field("sightMeters", sightMeters, positive())
        .field("gyroErrorDegrees", gyroErrorDegrees, nonNegative())
        .field("mountErrorMillimeters", mountErrorMillimeters, nonNegative())
        .field("mountErrorDegrees", mountErrorDegrees, nonNegative())
        .field("otherRobotsLeast", otherRobotsLeast, atLeast(0))
        .field(
            "otherRobotsMost",
            otherRobotsMost,
            most ->
                most >= 0 && most <= OtherRobots.MOST
                    ? null
                    : shown(most) + " must be from 0 to 5: an FRC field has five other robots")
        .field("otherRobotsHeightMeters", otherRobotsHeightMeters, nonNegative())
        .field("trials", trials, atLeast(0))
        .field("mostCameras", mostCameras, atLeast(1))
        .field("threads", threads, atLeast(1))
        .check(
            "otherRobotsMost",
            () ->
                otherRobotsMost >= otherRobotsLeast
                    ? null
                    : otherRobotsMost
                        + " must be at least otherRobotsLeast ("
                        + otherRobotsLeast
                        + ")")
        .orThrow();
  }

  /** Starts building the study's settings, one named value at a time. */
  public static CameraStudyConfigBuilder.GridMetersStage builder() {
    return CameraStudyConfigBuilder.builder();
  }
}
