package com.michaelgrundvig.frc.localization;

/**
 * Differential (tank) kinematics for localization: forward travel is the average of the two sides.
 * Rotation from the wheels is only a fallback for when the gyro is lost, because a tank drive's
 * wheels skid sideways whenever it turns; the estimator measures their disagreement with the gyro.
 * Samples hold two wheels, left then right, at angle zero.
 */
public final class DifferentialKinematicsModel implements DriveKinematicsModel {
  private final double trackWidthMeters;

  /**
   * @param trackWidthMeters between the left and right wheels' contact patches
   */
  public DifferentialKinematicsModel(double trackWidthMeters) {
    if (!(trackWidthMeters > 0) || Double.isInfinite(trackWidthMeters)) {
      throw new IllegalArgumentException("the track width must be positive: " + trackWidthMeters);
    }
    this.trackWidthMeters = trackWidthMeters;
  }

  @Override
  public BodyMotion motion(OdometrySample from, OdometrySample to) {
    if (from.wheels() != 2 || to.wheels() != 2) {
      throw new IllegalArgumentException("a differential drive's samples hold 2 wheels");
    }
    double left = to.distanceMeters(0) - from.distanceMeters(0);
    double right = to.distanceMeters(1) - from.distanceMeters(1);
    // Two sides always fit a rigid motion exactly, so there's no residual to report.
    return new BodyMotion((left + right) / 2, 0, (right - left) / trackWidthMeters, 0);
  }
}
