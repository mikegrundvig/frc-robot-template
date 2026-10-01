package com.michaelgrundvig.frc.localization;

/**
 * How the robot moved between two odometry samples, in its own frame, as its wheels tell it. The
 * drivetrain's kinematics (swerve, differential, mecanum) produces it; the estimator turns it by
 * the gyro's heading and adds it to the pose.
 *
 * @param dxMeters forward
 * @param dyMeters to the left
 * @param dthetaRadians counterclockwise. The estimator takes rotation from the gyro, and uses this
 *     only to check it, and when the gyro is lost.
 * @param residualMeters how far the wheels' disagreement with any one rigid motion of the robot may
 *     have put this motion off: zero when they all agree, more when a wheel slips, lifts, or is
 *     pushed. The estimator doubts the motion by as much.
 * @param oneWheelMeters how much of that one wheel alone explains, up to all of it: a wheel
 *     slipping on its own, whose error keeps its direction while it slips, so the estimator lets it
 *     add up from step to step. The rest is spread over the wheels (scrub, noise, wheels reaching a
 *     new speed each at its own pace), each step's own, and averages out.
 */
public record BodyMotion(
    double dxMeters,
    double dyMeters,
    double dthetaRadians,
    double residualMeters,
    double oneWheelMeters) {
  /**
   * A motion whose residual can't be told apart: counted as one wheel's, which keeps its direction,
   * the more doubtful reading of it.
   */
  public BodyMotion(double dxMeters, double dyMeters, double dthetaRadians, double residualMeters) {
    this(dxMeters, dyMeters, dthetaRadians, residualMeters, residualMeters);
  }

  public BodyMotion {
    if (!Double.isFinite(dxMeters)
        || !Double.isFinite(dyMeters)
        || !Double.isFinite(dthetaRadians)) {
      throw new IllegalArgumentException(
          "a motion must be numbers: " + dxMeters + ", " + dyMeters + ", " + dthetaRadians);
    }
    if (!(residualMeters >= 0) || Double.isInfinite(residualMeters)) {
      throw new IllegalArgumentException("the residual must be finite, and zero or more");
    }
    if (!(oneWheelMeters >= 0) || !(oneWheelMeters <= residualMeters)) {
      throw new IllegalArgumentException(
          "one wheel explains some or all of the residual, never more: " + oneWheelMeters);
    }
  }
}
