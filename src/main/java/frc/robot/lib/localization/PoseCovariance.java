package frc.robot.lib.localization;

import org.wpilib.math.geometry.Rotation2d;

/**
 * How uncertain a pose is: the covariance of its position (x and y, in square meters, and how they
 * vary together) and the variance of its heading (square radians). Heading stands apart from
 * position, as in the estimator: the gyro owns heading.
 *
 * @param xx the variance along the field's x
 * @param xy the covariance of x and y
 * @param yy the variance along y
 * @param heading the variance of the heading; infinite when the pose says nothing about heading
 */
public record PoseCovariance(double xx, double xy, double yy, double heading) {
  public PoseCovariance {
    if (!Double.isFinite(xx) || !Double.isFinite(yy) || !Double.isFinite(xy) || xx < 0 || yy < 0) {
      throw new IllegalArgumentException(
          "the position variances must be finite, and zero or more: " + xx + ", " + yy);
    }
    // No real spread of positions has x and y varying together by more than their own spreads
    // allow (the matrix would not be positive semidefinite); a little slack absorbs rounding.
    if (xy * xy - xx * yy > 1e-9 * Math.max(xx * yy, xy * xy) + Double.MIN_NORMAL) {
      throw new IllegalArgumentException(
          "the covariance of x and y, " + xy + ", is more than their variances allow");
    }
    if (!(heading >= 0)) {
      throw new IllegalArgumentException(
          "the heading variance must be zero or more (infinite when unknown): " + heading);
    }
  }

  /**
   * Independent standard deviations: meters along x and along y, and radians of heading ({@code
   * Double.POSITIVE_INFINITY} when the heading is unknown).
   */
  public static PoseCovariance fromStdDevs(double x, double y, double heading) {
    return new PoseCovariance(x * x, 0, y * y, heading * heading);
  }

  public double xStdDev() {
    return Math.sqrt(xx);
  }

  public double yStdDev() {
    return Math.sqrt(yy);
  }

  public double headingStdDev() {
    return Math.sqrt(heading);
  }

  /** Whether the pose says anything about heading. */
  public boolean hasHeading() {
    return Double.isFinite(heading);
  }

  /**
   * The same uncertainty in a frame turned by an angle: a camera's "less certain along my line of
   * sight", written in the robot's frame, turned into the field's by the robot's heading.
   */
  public PoseCovariance rotatedBy(Rotation2d rotation) {
    double c = rotation.getCos();
    double s = rotation.getSin();
    // R C R^T, for the rotation R = [c -s; s c].
    return new PoseCovariance(
        c * c * xx - 2 * c * s * xy + s * s * yy,
        c * s * (xx - yy) + (c * c - s * s) * xy,
        s * s * xx + 2 * c * s * xy + c * c * yy,
        heading);
  }
}
