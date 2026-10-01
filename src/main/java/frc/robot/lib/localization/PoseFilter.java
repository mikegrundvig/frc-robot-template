package frc.robot.lib.localization;

import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * The pose estimator's filter: the robot's position and a heading offset (the field heading is the
 * gyro's yaw plus the offset), with a 3x3 covariance of all three and how they relate. It predicts
 * through odometry steps and corrects with measurements; what to feed it, when, and whether to
 * believe a measurement is the estimator's business.
 *
 * <p>A camera's position solved with the gyro's heading leans on that heading: off by an angle, the
 * position is off by the angle times the distance to the tags, square to them, the same way in
 * every image ({@link VisionMeasurement#robotToTags}). So each such fix measures the position and
 * the heading's error together, and the update says so (its H is [I | J d], d the robot-to-tags
 * vector and J a quarter turn). Without that, the filter would take the heading's error, repeated
 * in every image, for noise that averages out, and grow sure of a position that's off.
 *
 * <p>A measurement with no heading of its own is applied as a Schmidt "consider" update (Schmidt
 * 1966; Zanetti and D'Souza 2015): it corrects the position, with the heading's doubt carried into
 * its weight and into the position's uncertainty, but never the heading itself, which stays the
 * gyro's (and the measurements with a heading's). The heading's variance is left as it was, and the
 * position's ties to it are updated as the gain left them.
 *
 * <p>The covariance is row-major: x, y, offset.
 */
final class PoseFilter {
  /** An uncertainty as big as the field: the estimate knows nothing. */
  private static final PoseCovariance NO_IDEA = PoseCovariance.fromStdDevs(100, 100, Math.PI);

  private double x;
  private double y;
  private double offset;
  private final double[] covariance = new double[9];

  // Scratch space for a camera measurement, from its innovation to its correction: two rows for
  // its position, a third for its heading if it has one. Row-major, three columns each.
  private int rows;
  private final double[] innovation3 = new double[3];
  private final double[] h3 = new double[9]; // H: what each row measures of x, y, and the offset
  private final double[] ph3 = new double[9]; // P H^T: 3 x rows
  private final double[] s3 = new double[9]; // S = H P H^T + R: rows x rows
  private final double[] sInverse3 = new double[9];
  private final double[] gain = new double[9]; // K = P H^T S^-1: 3 x rows
  private final double[] product = new double[9];

  /**
   * How far a camera's measurement is from the estimate, and how surprising that is.
   *
   * @param x the position's innovation, meters
   * @param y the position's innovation, meters
   * @param withHeading whether the measurement has a heading
   * @param heading the heading's innovation, radians; NaN for a measurement without heading
   * @param chiSquared the innovation's squared distance under its uncertainty, all of it
   * @param positionChiSquared the same for the position alone
   */
  record Innovation(
      double x,
      double y,
      boolean withHeading,
      double heading,
      double chiSquared,
      double positionChiSquared) {
    double meters() {
      return Math.hypot(x, y);
    }
  }

  /**
   * How far a landmark is from the estimate, along its one axis.
   *
   * @param meters the innovation along the axis
   * @param variance its variance, estimate and landmark together
   * @param chiSquared its squared distance under that variance
   */
  record AxisInnovation(
      double meters, double variance, double chiSquared, double ph0, double ph1, double ph2) {}

  double x() {
    return x;
  }

  double y() {
    return y;
  }

  double offset() {
    return offset;
  }

  Pose2d pose(double yaw) {
    return new Pose2d(x, y, new Rotation2d(yaw + offset));
  }

  /** Takes a reset's position and covariance, and its heading if it gives one. */
  void set(PoseReset reset, double yaw) {
    x = reset.pose().getX();
    y = reset.pose().getY();
    PoseCovariance given = reset.covariance();
    if (given.hasHeading()) {
      offset = reset.pose().getRotation().getRadians() - yaw;
      covariance[8] = given.heading();
    }
    setPosition(given.xx(), given.xy(), given.yy());
  }

  /** A new heading, as sure as given, the position left where it is and unrelated to it. */
  void resetHeading(double newOffset, double variance) {
    offset = newOffset;
    covariance[8] = variance;
    clearCrossTerms();
  }

  /** Moves the position, unrelated to the heading and as sure as given in each direction. */
  void shift(double dx, double dy, double variance) {
    x += dx;
    y += dy;
    setPosition(variance, 0, variance);
  }

  /** Turns the heading, with a fresh uncertainty for it and for the position. */
  void turn(double radians, double positionVariance, double headingVariance) {
    offset += radians;
    covariance[8] = headingVariance;
    setPosition(positionVariance, 0, positionVariance);
  }

  /**
   * Moves through part of an odometry step: the wheels' travel along the arc the gyro turned
   * through, starting at a yaw, and the uncertainty the move adds.
   *
   * @param startYaw the gyro's yaw where this part of the step starts
   * @param turn how far the gyro turned over it
   * @param dx the wheels' travel forward over it, in the robot's frame at its start
   * @param dy and to the left
   * @param positionNoise variance to add to each position axis
   * @param headingNoise variance to add to the heading
   */
  void predict(
      double startYaw,
      double turn,
      double dx,
      double dy,
      double positionNoise,
      double headingNoise) {
    double heading = startYaw + offset;
    double along = dx;
    double across = dy;
    if (Math.abs(turn) > 1e-9) {
      double s = Math.sin(turn) / turn;
      double c = (1 - Math.cos(turn)) / turn;
      along = s * dx - c * dy;
      across = c * dx + s * dy;
    }
    double cos = Math.cos(heading);
    double sin = Math.sin(heading);
    double moveX = cos * along - sin * across;
    double moveY = sin * along + cos * across;
    x += moveX;
    y += moveY;

    // P = F P F^T, where F = I plus a heading column: turning the offset swings the move.
    double a = -moveY;
    double b = moveX;
    double p02 = covariance[2];
    double p12 = covariance[5];
    double p22 = covariance[8];
    covariance[0] += 2 * a * p02 + a * a * p22;
    covariance[1] += a * p12 + b * p02 + a * b * p22;
    covariance[3] = covariance[1];
    covariance[4] += 2 * b * p12 + b * b * p22;
    covariance[2] += a * p22;
    covariance[6] = covariance[2];
    covariance[5] += b * p22;
    covariance[7] = covariance[5];

    covariance[0] += positionNoise;
    covariance[4] += positionNoise;
    covariance[8] += headingNoise;
  }

  /**
   * A camera measurement's innovation against the estimate, readying {@link #correct(Innovation)}
   * to apply it.
   *
   * @param yawThen the gyro's yaw when the image was taken
   * @return null if its uncertainty and the estimate's together are degenerate
   */
  @Nullable Innovation innovation(VisionMeasurement measurement, double yawThen) {
    PoseCovariance noise = measurement.covariance();
    rows = noise.hasHeading() ? 3 : 2;
    // H: each position row sees its own axis, and the heading through the tags' lever: a heading
    // off by dθ moves the fix dθ times the robot-to-tags vector, turned a quarter turn.
    double leverX = measurement.robotToTags().getX();
    double leverY = measurement.robotToTags().getY();
    h3[0] = 1;
    h3[1] = 0;
    h3[2] = -leverY;
    h3[3] = 0;
    h3[4] = 1;
    h3[5] = leverX;
    h3[6] = 0;
    h3[7] = 0;
    h3[8] = 1;
    innovation3[0] = measurement.pose().getX() - x;
    innovation3[1] = measurement.pose().getY() - y;
    innovation3[2] =
        rows == 3
            ? MathUtil.angleModulus(
                measurement.pose().getRotation().getRadians() - (yawThen + offset))
            : Double.NaN;
    // P H^T, then S = H P H^T + R.
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < rows; j++) {
        ph3[i * 3 + j] =
            covariance[i * 3] * h3[j * 3]
                + covariance[i * 3 + 1] * h3[j * 3 + 1]
                + covariance[i * 3 + 2] * h3[j * 3 + 2];
      }
    }
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < rows; j++) {
        s3[i * 3 + j] =
            h3[i * 3] * ph3[j] + h3[i * 3 + 1] * ph3[3 + j] + h3[i * 3 + 2] * ph3[6 + j];
      }
    }
    s3[0] += noise.xx();
    s3[1] += noise.xy();
    s3[3] += noise.xy();
    s3[4] += noise.yy();
    if (rows == 3) {
      s3[8] += noise.heading();
    }
    // The position's own agreement, from the position block of S.
    double s00 = s3[0];
    double s01 = (s3[1] + s3[3]) / 2;
    double s11 = s3[4];
    double determinant = s00 * s11 - s01 * s01;
    if (!(determinant > 0)) {
      return null;
    }
    double positionChiSquared =
        (s11 * innovation3[0] * innovation3[0]
                - 2 * s01 * innovation3[0] * innovation3[1]
                + s00 * innovation3[1] * innovation3[1])
            / determinant;
    double chiSquared = positionChiSquared;
    if (rows == 3) {
      if (!Matrices.invert3(s3, sInverse3)) {
        return null;
      }
      chiSquared = Matrices.quadratic3(sInverse3, innovation3);
    } else {
      sInverse3[0] = s11 / determinant;
      sInverse3[1] = -s01 / determinant;
      sInverse3[3] = -s01 / determinant;
      sInverse3[4] = s00 / determinant;
    }
    // K = P H^T S^-1. A measurement without a heading of its own leaves the heading alone: its gain
    // for the offset is zero, the Schmidt "consider" update.
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < rows; j++) {
        double sum = 0;
        for (int k = 0; k < rows; k++) {
          sum += ph3[i * 3 + k] * sInverse3[k * 3 + j];
        }
        gain[i * 3 + j] = i == 2 && rows == 2 ? 0 : sum;
      }
    }
    return new Innovation(
        innovation3[0], innovation3[1], rows == 3, innovation3[2], chiSquared, positionChiSquared);
  }

  /**
   * Applies the camera measurement last given to {@link #innovation}, as a Kalman update, or a
   * consider update for one without heading.
   */
  void correct(Innovation innovation) {
    for (int i = 0; i < 3; i++) {
      double move = 0;
      for (int j = 0; j < rows; j++) {
        move += gain[i * 3 + j] * innovation3[j];
      }
      if (i == 0) {
        x += move;
      } else if (i == 1) {
        y += move;
      } else {
        offset += move;
      }
    }
    // P = P - K C^T - C K^T + K S K^T, with C = P H^T: right for any gain, so for the consider
    // update's too. (For the full Kalman gain, the last two terms cancel to P - K C^T.)
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < rows; j++) {
        double sum = 0;
        for (int k = 0; k < rows; k++) {
          sum += gain[i * 3 + k] * s3[k * 3 + j];
        }
        product[i * 3 + j] = sum; // K S
      }
    }
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        double change = 0;
        for (int k = 0; k < rows; k++) {
          change +=
              -gain[i * 3 + k] * ph3[j * 3 + k]
                  - ph3[i * 3 + k] * gain[j * 3 + k]
                  + product[i * 3 + k] * gain[j * 3 + k];
        }
        covariance[i * 3 + j] += change;
      }
    }
    symmetrize();
  }

  /**
   * A landmark's innovation along its one axis: it measures the estimate across its line, and says
   * nothing along it.
   *
   * @return null if the variance across the line is degenerate
   */
  @Nullable AxisInnovation innovationAlong(LandmarkEvent landmark) {
    double nx = landmark.across().getCos();
    double ny = landmark.across().getSin();
    double innovation = -landmark.offset(new Translation2d(x, y));
    // With H = [nx, ny, 0]: P H^T, then S = H P H^T + R.
    double ph0 = covariance[0] * nx + covariance[1] * ny;
    double ph1 = covariance[3] * nx + covariance[4] * ny;
    double ph2 = covariance[6] * nx + covariance[7] * ny;
    double s = nx * ph0 + ny * ph1 + landmark.stdDevMeters() * landmark.stdDevMeters();
    if (!(s > 0)) {
      return null;
    }
    return new AxisInnovation(innovation, s, innovation * innovation / s, ph0, ph1, ph2);
  }

  /** Applies a landmark, as a Kalman update along its one axis. */
  void correct(AxisInnovation innovation) {
    double s = innovation.variance();
    double[] k = {innovation.ph0() / s, innovation.ph1() / s, innovation.ph2() / s};
    x += k[0] * innovation.meters();
    y += k[1] * innovation.meters();
    offset += k[2] * innovation.meters();
    // P = P - K H P, and H P is (P H^T) transposed: K K^T S.
    for (int row = 0; row < 3; row++) {
      for (int column = 0; column < 3; column++) {
        covariance[row * 3 + column] -= k[row] * k[column] * s;
      }
    }
    symmetrize();
  }

  /** The covariance, as the estimate reports it. */
  PoseCovariance covariance() {
    if (!Double.isFinite(covariance[0])
        || !Double.isFinite(covariance[1])
        || !Double.isFinite(covariance[4])
        || !Double.isFinite(covariance[8])) {
      // Only a bug gets here (every input is checked on the way in): say the estimate knows
      // nothing, rather than throw in the robot loop.
      return NO_IDEA;
    }
    double xx = Math.max(0, covariance[0]);
    double yy = Math.max(0, covariance[4]);
    double limit = Math.sqrt(xx * yy);
    double xy = Math.clamp(covariance[1], -limit, limit);
    return new PoseCovariance(xx, xy, yy, Math.max(0, covariance[8]));
  }

  void copyCovarianceTo(double[] into, int at) {
    System.arraycopy(covariance, 0, into, at, 9);
  }

  void restore(double x, double y, double offset, double[] covariances, int at) {
    this.x = x;
    this.y = y;
    this.offset = offset;
    System.arraycopy(covariances, at, covariance, 0, 9);
  }

  /** The position's own block, and nothing relating it to the heading. */
  private void setPosition(double xx, double xy, double yy) {
    covariance[0] = xx;
    covariance[1] = xy;
    covariance[3] = xy;
    covariance[4] = yy;
    clearCrossTerms();
  }

  /** The heading unrelated to the position. */
  private void clearCrossTerms() {
    covariance[2] = 0;
    covariance[6] = 0;
    covariance[5] = 0;
    covariance[7] = 0;
  }

  private void symmetrize() {
    covariance[1] = covariance[3] = (covariance[1] + covariance[3]) / 2;
    covariance[2] = covariance[6] = (covariance[2] + covariance[6]) / 2;
    covariance[5] = covariance[7] = (covariance[5] + covariance[7]) / 2;
  }
}
