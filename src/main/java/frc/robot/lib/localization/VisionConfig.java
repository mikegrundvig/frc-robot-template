package frc.robot.lib.localization;

import frc.robot.lib.config.ConfigRecord;

/**
 * How the vision pipeline filters camera results and how much it trusts them. The defaults start
 * from 2611's 2026 code (itself AdvantageKit's template) and what top teams did in 2025 and 2026;
 * fit them from logged residuals, never from one run.
 *
 * @param maxAmbiguity a single tag's result is used only below this ambiguity (2026: 0.2)
 * @param maxHeightErrorMeters how far a solve's height may miss the floor, or a tag's known height,
 *     before the result is thrown out (2026: 0.2)
 * @param maxDistanceMeters results from tags farther than this, on average, are thrown out: the
 *     noise grows with distance squared
 * @param maxYawRateRadiansPerSecond results taken while the robot turned faster than this, lately,
 *     are thrown out: motion blur and rolling shutter (254: 5 rad/s over the last 0.3 s)
 * @param linearStdDevBaseline position uncertainty in meters for one tag at one meter (2026: 0.02)
 * @param angularStdDevBaseline heading uncertainty in radians for one tag at one meter, used only
 *     when a result may correct heading (2026: 0.03)
 * @param distanceExponent uncertainty grows with the tags' distance to this power (2026: 2; teams
 *     used 2 to 4, so fit it)
 * @param headingConstrainedFactor how much less uncertain a position is when the gyro's heading
 *     fixes the orientation (6328's factor for the same idea: 0.5)
 * @param speedStdDevFactor uncertainty grows by this fraction per meter per second of robot speed
 * @param yawRateStdDevFactor and by this fraction per radian per second of turning
 * @param maxHeadingStdDevToConstrain the gyro's heading is used to solve only while the estimate is
 *     this sure of it, in radians; until then (a robot not yet placed), full solves find it
 * @param minTagsForHeading a result may correct heading only from a multi-tag solution with at
 *     least this many tags
 * @param maxYawRateForHeading and only while the robot turns slower than this
 * @param maxReprojectionErrorForHeadingPixels and only when the solution fits its tags this well
 */
@ConfigRecord
public record VisionConfig(
    double maxAmbiguity,
    double maxHeightErrorMeters,
    double maxDistanceMeters,
    double maxYawRateRadiansPerSecond,
    double linearStdDevBaseline,
    double angularStdDevBaseline,
    double distanceExponent,
    double headingConstrainedFactor,
    double speedStdDevFactor,
    double yawRateStdDevFactor,
    double maxHeadingStdDevToConstrain,
    int minTagsForHeading,
    double maxYawRateForHeading,
    double maxReprojectionErrorForHeadingPixels) {
  /** Starting points: 2611's 2026 values where it had them, top teams' where it didn't. */
  public static final VisionConfig DEFAULTS =
      builder()
          .maxAmbiguity(0.2)
          .maxHeightErrorMeters(0.2)
          .maxDistanceMeters(6.0)
          .maxYawRateRadiansPerSecond(5.0)
          .linearStdDevBaseline(0.02)
          .angularStdDevBaseline(0.03)
          .distanceExponent(2.0)
          .headingConstrainedFactor(0.5)
          .speedStdDevFactor(0.25)
          .yawRateStdDevFactor(0.5)
          .maxHeadingStdDevToConstrain(Math.toRadians(5))
          .minTagsForHeading(2)
          .maxYawRateForHeading(0.5)
          .maxReprojectionErrorForHeadingPixels(1.0)
          .build();

  public VisionConfig {
    double[] positive = {
      maxAmbiguity,
      maxHeightErrorMeters,
      maxDistanceMeters,
      maxYawRateRadiansPerSecond,
      linearStdDevBaseline,
      angularStdDevBaseline,
      distanceExponent,
      headingConstrainedFactor,
      maxHeadingStdDevToConstrain,
      maxYawRateForHeading,
      maxReprojectionErrorForHeadingPixels
    };
    for (double value : positive) {
      if (!(value > 0) || Double.isInfinite(value)) {
        throw new IllegalArgumentException("vision thresholds and uncertainties must be positive");
      }
    }
    if (!(speedStdDevFactor >= 0) || !(yawRateStdDevFactor >= 0)) {
      throw new IllegalArgumentException("motion factors must be zero or more");
    }
    if (minTagsForHeading < 2) {
      throw new IllegalArgumentException("heading comes only from two or more tags");
    }
  }

  /** Starts building vision settings, one named value at a time. */
  public static VisionConfigBuilder.MaxAmbiguityStage builder() {
    return VisionConfigBuilder.builder();
  }
}
