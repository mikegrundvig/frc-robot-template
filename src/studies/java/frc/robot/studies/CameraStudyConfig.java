package frc.robot.studies;

import static frc.robot.lib.config.Rules.atLeast;
import static frc.robot.lib.config.Rules.nonNegative;
import static frc.robot.lib.config.Rules.positive;

import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;

/**
 * How the camera placement study samples the field and judges what the cameras see (see
 * docs/camera-placement.md).
 *
 * @param gridMeters how far apart the robot positions are, both ways
 * @param headings how many headings the robot takes at each position, evenly spaced
 * @param frames how many images each camera takes at each pose, each with its own pixel noise
 * @param gyroErrorDegrees how far off the gyro's heading typically is (one standard deviation):
 *     every solve takes its heading from the gyro, so its error moves every position solved
 * @param mostCameras the most cameras a layout has: the family's, or those made from CAD's places,
 *     with more are left out
 * @param seed where every random draw starts: the same seed gives the same report
 * @param threads how many threads share the work
 */
@ConfigRecord
public record CameraStudyConfig(
    double gridMeters,
    int headings,
    int frames,
    double gyroErrorDegrees,
    int mostCameras,
    long seed,
    int threads) {
  /**
   * A run of under a minute: every half meter, every 45 degrees, two images per camera. Two
   * threads, since the computer may be shared.
   */
  public static final CameraStudyConfig DEFAULTS =
      builder()
          .gridMeters(0.5)
          .headings(8)
          .frames(2)
          .gyroErrorDegrees(0.25)
          .mostCameras(4)
          .seed(1)
          .threads(2)
          .build();

  public CameraStudyConfig {
    Checks.on(CameraStudyConfig.class)
        .field("gridMeters", gridMeters, positive())
        .field("headings", headings, atLeast(1))
        .field("frames", frames, atLeast(1))
        .field("gyroErrorDegrees", gyroErrorDegrees, nonNegative())
        .field("mostCameras", mostCameras, atLeast(1))
        .field("threads", threads, atLeast(1))
        .orThrow();
  }

  /** Starts building the study's settings, one named value at a time. */
  public static CameraStudyConfigBuilder.GridMetersStage builder() {
    return CameraStudyConfigBuilder.builder();
  }
}
