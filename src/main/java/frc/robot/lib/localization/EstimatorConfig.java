package frc.robot.lib.localization;

/**
 * How the pose estimator weighs odometry, the gyro, and the cameras. The defaults are reasonable
 * starting points, not measurements: fit them from logged residuals (the calibration procedures),
 * never from one run.
 *
 * @param translationStdDevPerMeter how far odometry's position may be off after one meter of
 *     travel, in meters. Its variance grows with distance.
 * @param translationStdDevPerRadian how far odometry's position may be off after turning one
 *     radian, in meters: wheels scrub as they turn
 * @param residualScale how much the wheels' disagreement (the kinematic residual) adds to a step's
 *     uncertainty: 1 adds the residual itself as a standard deviation
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
 * @param gateChiSquaredPosition the gate for a position measurement: its innovation's squared
 *     Mahalanobis distance may not exceed this (13.8 passes 99.9% of good measurements, 2 degrees
 *     of freedom)
 * @param gateChiSquaredPose the same for a measurement with heading (16.3: 99.9%, 3 degrees)
 * @param recoveryMeasurements how many rejected measurements must agree with each other, within the
 *     recovery window, before odometry is judged wrong and the estimate jumps to them
 * @param recoveryWindowSeconds how recent those rejected measurements must be
 * @param recoveryAgreementMeters how closely their innovations must agree
 * @param recoveryStdDevMeters the position uncertainty after a recovery
 */
public record EstimatorConfig(
    double translationStdDevPerMeter,
    double translationStdDevPerRadian,
    double residualScale,
    double headingDriftDegreesPerMinute,
    double headingStdDevDegreesPerTurn,
    double wheelHeadingStdDevDegreesPerTurn,
    double historySeconds,
    double futureToleranceSeconds,
    double visionTimeoutSeconds,
    double gateChiSquaredPosition,
    double gateChiSquaredPose,
    int recoveryMeasurements,
    double recoveryWindowSeconds,
    double recoveryAgreementMeters,
    double recoveryStdDevMeters) {
  /** Starting points for a swerve robot with a good gyro. */
  public static final EstimatorConfig DEFAULTS =
      new EstimatorConfig(
          0.03, 0.02, 1.0, 0.5, 0.5, 10.0, 1.5, 0.05, 1.0, 13.816, 16.266, 3, 1.0, 0.25, 0.25);

  public EstimatorConfig {
    double[] nonNegative = {
      translationStdDevPerMeter,
      translationStdDevPerRadian,
      residualScale,
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
      historySeconds,
      visionTimeoutSeconds,
      gateChiSquaredPosition,
      gateChiSquaredPose,
      recoveryWindowSeconds,
      recoveryStdDevMeters
    };
    for (double value : positive) {
      if (!(value > 0) || Double.isInfinite(value)) {
        throw new IllegalArgumentException(
            "estimator windows, gates, and spreads must be positive");
      }
    }
    if (recoveryMeasurements < 2) {
      throw new IllegalArgumentException("recovery needs at least 2 agreeing measurements");
    }
  }
}
