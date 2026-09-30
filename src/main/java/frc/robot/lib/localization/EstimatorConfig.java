package frc.robot.lib.localization;

import frc.robot.lib.config.ConfigRecord;

/**
 * How the pose estimator weighs odometry, the gyro, and the cameras. The defaults are reasonable
 * starting points, not measurements: fit them from logged residuals (the calibration procedures),
 * never from one run.
 *
 * @param translationStdDevPerMeter how far odometry's position may be off after one meter of
 *     travel, in meters. Its variance grows with distance.
 * @param translationStdDevPerRadian how far odometry's position may be off after turning one
 *     radian, in meters: wheels scrub as they turn
 * @param residualScale how much of the wheels' disagreement (the kinematic residual) counts as
 *     suspect travel: 1 counts all of it
 * @param tractionLimitedSlipFraction while the wheels are at the edge of their grip, this share of
 *     their travel counts as suspect
 * @param tiltStartDegrees below this tilt the wheels are trusted fully
 * @param tiltZeroDegrees at and above this tilt the wheels' travel counts for nothing: a robot
 *     climbing a bump or a ramp has wheels in the air (6328 scaled to zero at 25 degrees). Between
 *     the two, their travel counts for less, and the share left out counts as suspect.
 * @param slipCorrelationSeconds how long slip keeps its direction. Suspect travel within this time
 *     adds up before it's squared: a slip that builds one way for a quarter second grows the
 *     uncertainty as one error, not as many small ones that would average out.
 * @param collisionStdDevMeters how far a collision may have moved the robot beyond what its wheels
 *     saw: added at once, since the robot has already moved by the time the jolt is read
 * @param collisionWindowSeconds how long after a jolt the robot counts as just hit: more jolts
 *     within it are the same collision, and a scoring gate waits it out
 * @param handlingStdDevPerSqrtSecond while the robot is disabled, how far people may carry it where
 *     its wheels can't see, in meters after one second; the variance grows with time
 * @param headingDriftDegreesPerMinute how far the gyro's heading may drift in a minute
 * @param headingStdDevDegreesPerTurn how far the gyro's heading may be off after a full turn: its
 *     scale error
 * @param wheelHeadingStdDevDegreesPerTurn the same for heading from the wheels, used only while the
 *     gyro is lost
 * @param historySeconds how far back a late camera measurement can still be applied at its capture
 *     time
 * @param futureToleranceSeconds how far a measurement may be stamped after the newest odometry
 *     sample (clocks differ a little) before it's rejected
 * @param visionTimeoutSeconds how long without an accepted camera measurement before the estimate
 *     counts as dead reckoning
 * @param disagreementWindowSeconds how far back the health's vision-odometry disagreement looks
 * @param gateChiSquaredPosition the gate for a position measurement: its innovation's squared
 *     Mahalanobis distance may not exceed this (13.8 passes 99.9% of good measurements, 2 degrees
 *     of freedom)
 * @param gateChiSquaredPose the same for a measurement with heading (16.3: 99.9%, 3 degrees)
 * @param gateChiSquaredAxis the same for a landmark, which measures one axis (10.8: 99.9%, 1
 *     degree)
 * @param recoveryMeasurements how many rejected measurements must agree with each other, within the
 *     recovery window, before odometry is judged wrong and the estimate jumps to them
 * @param recoveryWindowSeconds how recent those rejected measurements must be
 * @param recoveryAgreementMeters how closely their innovations must agree
 * @param recoveryStdDevMeters the position uncertainty after a recovery
 * @param recoveryAgreementDegrees how closely rejected measurements' headings must agree before the
 *     gyro's heading is judged wrong (a bad heading reset, a robot placed backwards, a gyro that
 *     jumped) and the estimate turns to them
 * @param recoveryHeadingStdDevDegrees the heading uncertainty after a heading recovery
 * @param lostPositionStdDevMeters the position uncertainty after a heading recovery: the position
 *     was worked out with the wrong heading, so it could be far off
 */
@ConfigRecord
public record EstimatorConfig(
    double translationStdDevPerMeter,
    double translationStdDevPerRadian,
    double residualScale,
    double tractionLimitedSlipFraction,
    double tiltStartDegrees,
    double tiltZeroDegrees,
    double slipCorrelationSeconds,
    double collisionStdDevMeters,
    double collisionWindowSeconds,
    double handlingStdDevPerSqrtSecond,
    double headingDriftDegreesPerMinute,
    double headingStdDevDegreesPerTurn,
    double wheelHeadingStdDevDegreesPerTurn,
    double historySeconds,
    double futureToleranceSeconds,
    double visionTimeoutSeconds,
    double disagreementWindowSeconds,
    double gateChiSquaredPosition,
    double gateChiSquaredPose,
    double gateChiSquaredAxis,
    int recoveryMeasurements,
    double recoveryWindowSeconds,
    double recoveryAgreementMeters,
    double recoveryStdDevMeters,
    double recoveryAgreementDegrees,
    double recoveryHeadingStdDevDegrees,
    double lostPositionStdDevMeters) {
  /** Starting points for a swerve robot with a good gyro. */
  public static final EstimatorConfig DEFAULTS =
      builder()
          .translationStdDevPerMeter(0.03)
          .translationStdDevPerRadian(0.02)
          .residualScale(1.0)
          .tractionLimitedSlipFraction(0.25)
          .tiltStartDegrees(5)
          .tiltZeroDegrees(25)
          .slipCorrelationSeconds(0.5)
          .collisionStdDevMeters(0.25)
          .collisionWindowSeconds(0.25)
          .handlingStdDevPerSqrtSecond(0.02)
          .headingDriftDegreesPerMinute(0.5)
          .headingStdDevDegreesPerTurn(0.5)
          .wheelHeadingStdDevDegreesPerTurn(10)
          .historySeconds(1.5)
          .futureToleranceSeconds(0.05)
          .visionTimeoutSeconds(1.0)
          .disagreementWindowSeconds(0.5)
          .gateChiSquaredPosition(13.816)
          .gateChiSquaredPose(16.266)
          .gateChiSquaredAxis(10.828)
          .recoveryMeasurements(3)
          .recoveryWindowSeconds(1.0)
          .recoveryAgreementMeters(0.25)
          .recoveryStdDevMeters(0.25)
          .recoveryAgreementDegrees(5)
          .recoveryHeadingStdDevDegrees(2)
          .lostPositionStdDevMeters(1.0)
          .build();

  public EstimatorConfig {
    double[] nonNegative = {
      translationStdDevPerMeter,
      translationStdDevPerRadian,
      residualScale,
      tiltStartDegrees,
      collisionStdDevMeters,
      handlingStdDevPerSqrtSecond,
      headingDriftDegreesPerMinute,
      headingStdDevDegreesPerTurn,
      wheelHeadingStdDevDegreesPerTurn,
      futureToleranceSeconds,
      recoveryAgreementMeters
    };
    for (double value : nonNegative) {
      if (!(value >= 0) || Double.isInfinite(value)) {
        throw new IllegalArgumentException("estimator settings must be finite, and zero or more");
      }
    }
    double[] positive = {
      slipCorrelationSeconds,
      collisionWindowSeconds,
      historySeconds,
      visionTimeoutSeconds,
      disagreementWindowSeconds,
      gateChiSquaredPosition,
      gateChiSquaredPose,
      gateChiSquaredAxis,
      recoveryWindowSeconds,
      recoveryStdDevMeters,
      recoveryAgreementDegrees,
      recoveryHeadingStdDevDegrees,
      lostPositionStdDevMeters
    };
    for (double value : positive) {
      if (!(value > 0) || Double.isInfinite(value)) {
        throw new IllegalArgumentException(
            "estimator windows, gates, and spreads must be positive");
      }
    }
    if (!(tractionLimitedSlipFraction >= 0 && tractionLimitedSlipFraction <= 1)) {
      throw new IllegalArgumentException("the slip fraction is from 0 to 1");
    }
    if (!(tiltZeroDegrees > tiltStartDegrees) || !(tiltZeroDegrees < 90)) {
      throw new IllegalArgumentException(
          "wheels must stop counting at a tilt above where they start to count less, and below 90");
    }
    if (recoveryMeasurements < 2) {
      throw new IllegalArgumentException("recovery needs at least 2 agreeing measurements");
    }
  }

  /** Starts building estimator settings, one named value at a time. */
  public static EstimatorConfigBuilder.TranslationStdDevPerMeterStage builder() {
    return EstimatorConfigBuilder.builder();
  }
}
