package frc.robot.lib.localization;

import java.util.Arrays;
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
 * <p><b>Errors it considers but never estimates.</b> Some errors repeat: in every image a camera
 * takes, or in every turn the gyro makes. Averaged as if each image's or turn's error were new,
 * they would shrink the uncertainty to millimeters while the estimate stays centimeters off. So the
 * filter keeps track of them as Schmidt "consider" parameters (Schmidt 1966; Zanetti and D'Souza
 * 2015): each has a fixed uncertainty, the filter never corrects it, and it follows how the
 * estimate's own errors have come to depend on it.
 *
 * <ul>
 *   <li><b>The heading, for a position a camera solved with it.</b> Such a position leans on the
 *       gyro's heading: off by an angle, it is off by the angle times the distance to the tags,
 *       square to them ({@link VisionMeasurement#robotToTags}). So each fix measures the position
 *       and the heading's error together (its H is [I | J d], d the robot-to-tags vector and J a
 *       quarter turn), but a fix never corrects the heading: the gyro owns it, and only a
 *       measurement with a heading of its own corrects its drift.
 *   <li><b>Each IMU's scale:</b> a gyro that reads 0.3% too much turn is 1.1° off after a turn and
 *       5.4° after five, the same way each time, not a little more at random. The backup IMU's is
 *       its own: three turns one way on the gyro and three back on the backup don't cancel.
 *   <li><b>Each camera's turn on its mount</b>, what's left after calibrating it: it moves the
 *       camera's positions just as a heading error does, and its headings by the angle itself.
 *   <li><b>Each camera's shift:</b> the rest of what its positions and headings share from image to
 *       image: its mount's offset, and the tags' placement on the field. Two tags a third of a
 *       meter apart, each half an inch from where the layout says, turn a multi-tag heading by up
 *       to 4°. Unlike a turn, a shift changes as the robot moves and turns and the camera sees
 *       other tags from elsewhere, so the filter lets go of it over a correlation distance and
 *       angle.
 * </ul>
 *
 * <p>Standing still, a camera seeing the same tags image after image still follows any change in
 * where the robot is, but can't place it better than its own repeated error allows; several
 * cameras, each with its own, together do better.
 *
 * <p>The covariance is row-major: x, y, offset. How the estimate relates to the considered errors
 * is a 3 by {@link #CONSIDERED} block, row-major too: their own variances stay fixed, and two
 * considered errors stay unrelated, since nothing ever corrects them.
 */
final class PoseFilter {
  /** The most cameras whose errors the filter keeps apart; any more are treated as independent. */
  static final int MAX_CAMERAS = 8;

  /** How many errors the filter considers for each camera: its turn, and its shift's three. */
  private static final int PER_CAMERA = 4;

  /**
   * How many errors the filter considers: the gyro's scale and the backup IMU's, then each camera's
   * turn and its shift (along x, along y, and of its headings).
   */
  static final int CONSIDERED = 2 + PER_CAMERA * MAX_CAMERAS;

  /**
   * How many numbers the filter's uncertainty takes to store: the covariance, and its relations.
   */
  static final int UNCERTAINTY = 9 + 3 * CONSIDERED;

  private static final int SCALE = 0;
  private static final int BACKUP_SCALE = 1;

  /** An uncertainty as big as the field: the estimate knows nothing. */
  private static final PoseCovariance NO_IDEA = PoseCovariance.fromStdDevs(100, 100, Math.PI);

  /**
   * How unsure the filter is of the errors it considers.
   *
   * @param gyroScaleStdDev the gyro's scale error: radians of heading per radian it turns
   * @param backupScaleStdDev the backup IMU's
   * @param cameraTurnStdDevRadians each camera's turn on its mount
   * @param cameraShiftStdDevMeters each camera's shift, along each axis
   * @param cameraShiftStdDevRadians and of its headings
   * @param shiftCorrelationMeters how far the robot moves before a camera's shift is mostly a new
   *     one: what the filter keeps of it falls by a factor of e over this distance
   * @param shiftCorrelationRadians and over this much turning
   */
  record Considered(
      double gyroScaleStdDev,
      double backupScaleStdDev,
      double cameraTurnStdDevRadians,
      double cameraShiftStdDevMeters,
      double cameraShiftStdDevRadians,
      double shiftCorrelationMeters,
      double shiftCorrelationRadians) {}

  private final double scaleVariance;
  private final double backupScaleVariance;
  private final double turnVariance;
  private final double shiftVariance;
  private final double headingShiftVariance;
  private final double shiftMeters;
  private final double shiftRadians;

  private double x;
  private double y;
  private double offset;
  private final double[] covariance = new double[9];

  /** How each of x, y, and the offset relates to each considered error: 3 by CONSIDERED. */
  private final double[] related = new double[3 * CONSIDERED];

  // Scratch space for a camera measurement, from its innovation to its correction: two rows for
  // its position, a third for its heading if it has one. Row-major, three columns each.
  private int rows;
  private int camera;
  private final double[] innovation3 = new double[3];
  private final double[] h3 = new double[9]; // H: what each row measures of x, y, and the offset
  // and of the camera's turn and shift (x, y, heading): rows x PER_CAMERA
  private final double[] hCamera = new double[3 * PER_CAMERA];
  private final double[] ph3 = new double[9]; // P H^T: 3 x rows
  private final double[] s3 = new double[9]; // S = H P H^T + R: rows x rows
  private final double[] sInverse3 = new double[9];
  private final double[] gain = new double[9]; // K = P H^T S^-1: 3 x rows
  private final double[] product = new double[9];
  private final double[] column = new double[3];

  // A landmark's direction, from its innovation to its correction.
  private double landmarkX;
  private double landmarkY;

  PoseFilter(Considered considered) {
    scaleVariance = square(considered.gyroScaleStdDev());
    backupScaleVariance = square(considered.backupScaleStdDev());
    turnVariance = square(considered.cameraTurnStdDevRadians());
    shiftVariance = square(considered.cameraShiftStdDevMeters());
    headingShiftVariance = square(considered.cameraShiftStdDevRadians());
    shiftMeters = considered.shiftCorrelationMeters();
    shiftRadians = considered.shiftCorrelationRadians();
  }

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
      unrelate(2);
    }
    setPosition(given.xx(), given.xy(), given.yy());
  }

  /** A new heading, as sure as given, the position left where it is and unrelated to it. */
  void resetHeading(double newOffset, double variance) {
    offset = newOffset;
    covariance[8] = variance;
    clearCrossTerms();
    unrelate(2);
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
    unrelate(2);
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
   * @param turnedBy what turned it: an IMU's scale error grows the heading's error by the turn; the
   *     wheels' own turning errors are in the heading noise
   */
  void predict(
      double startYaw,
      double turn,
      double dx,
      double dy,
      double positionNoise,
      double headingNoise,
      YawSource turnedBy) {
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
    // The same F for how the position relates to the considered errors.
    for (int k = 0; k < CONSIDERED; k++) {
      double withHeading = related[2 * CONSIDERED + k];
      related[k] += a * withHeading;
      related[CONSIDERED + k] += b * withHeading;
    }

    // An IMU reading its turns a little large or small adds its scale error times the turn to the
    // heading's error: the same error every turn, so it is related to all the others.
    int scale = turnedBy == YawSource.GYRO ? SCALE : BACKUP_SCALE;
    double variance = turnedBy == YawSource.GYRO ? scaleVariance : backupScaleVariance;
    if (turnedBy != YawSource.WHEELS && variance > 0) {
      covariance[8] += 2 * turn * related[2 * CONSIDERED + scale] + turn * turn * variance;
      covariance[2] += turn * related[scale];
      covariance[6] = covariance[2];
      covariance[5] += turn * related[CONSIDERED + scale];
      covariance[7] = covariance[5];
      related[2 * CONSIDERED + scale] += turn * variance;
    }

    // As the robot moves and turns, each camera's shift becomes a different one.
    double kept = Math.exp(-Math.hypot(moveX, moveY) / shiftMeters - Math.abs(turn) / shiftRadians);
    if (kept < 1) {
      for (int slot = 0; slot < MAX_CAMERAS; slot++) {
        for (int row = 0; row < 3; row++) {
          for (int c = 1; c < PER_CAMERA; c++) {
            related[row * CONSIDERED + turnColumn(slot) + c] *= kept;
          }
        }
      }
    }

    covariance[0] += positionNoise;
    covariance[4] += positionNoise;
    covariance[8] += headingNoise;
  }

  /**
   * A camera measurement's innovation against the estimate, readying {@link #correct(Innovation)}
   * to apply it.
   *
   * @param yawThen the gyro's yaw when the image was taken
   * @param camera which camera, from 0 to {@link #MAX_CAMERAS} - 1, whose turn and shift it shares
   *     with the camera's other images; -1 for one without a place, whose turn and shift are taken
   *     as new in each image
   * @return null if its uncertainty and the estimate's together are degenerate
   */
  @Nullable Innovation innovation(VisionMeasurement measurement, double yawThen, int camera) {
    PoseCovariance noise = measurement.covariance();
    rows = noise.hasHeading() ? 3 : 2;
    this.camera = camera;
    // H: each position row sees its own axis, and the heading through the tags' lever: a heading
    // off by dθ moves the fix dθ times the robot-to-tags vector, turned a quarter turn. The
    // camera's own turn moves it the same way, and its heading by the turn itself; its shift moves
    // the position, or the heading.
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
    Arrays.fill(hCamera, 0);
    hCamera[0] = -leverY; // x: the turn's lever, and the shift along x
    hCamera[1] = 1;
    hCamera[PER_CAMERA] = leverX; // y: the turn's lever, and the shift along y
    hCamera[PER_CAMERA + 2] = 1;
    hCamera[2 * PER_CAMERA] = 1; // heading: the turn, and the headings' shift
    hCamera[2 * PER_CAMERA + 3] = 1;
    innovation3[0] = measurement.pose().getX() - x;
    innovation3[1] = measurement.pose().getY() - y;
    innovation3[2] =
        rows == 3
            ? MathUtil.angleModulus(
                measurement.pose().getRotation().getRadians() - (yawThen + offset))
            : Double.NaN;
    // P H^T, for the estimate's rows: through the estimate's own covariance, and through how it
    // relates to the camera's turn and shift.
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < rows; j++) {
        double sum =
            covariance[i * 3] * h3[j * 3]
                + covariance[i * 3 + 1] * h3[j * 3 + 1]
                + covariance[i * 3 + 2] * h3[j * 3 + 2];
        if (camera >= 0) {
          for (int c = 0; c < PER_CAMERA; c++) {
            sum += related[i * CONSIDERED + turnColumn(camera) + c] * hCamera[j * PER_CAMERA + c];
          }
        }
        ph3[i * 3 + j] = sum;
      }
    }
    // S = H P H^T + R: the estimate's part, then the camera's errors': how they relate to the
    // estimate, and their own variances. A camera without a place among the considered errors
    // still has them, in every image: they're taken as new in each, unrelated to anything.
    for (int i = 0; i < rows; i++) {
      for (int j = 0; j < rows; j++) {
        double sum = h3[i * 3] * ph3[j] + h3[i * 3 + 1] * ph3[3 + j] + h3[i * 3 + 2] * ph3[6 + j];
        for (int c = 0; c < PER_CAMERA; c++) {
          double hc = hCamera[i * PER_CAMERA + c];
          if (hc != 0) {
            double own = ownVariance(c) * hCamera[j * PER_CAMERA + c];
            if (camera >= 0) {
              int k = turnColumn(camera) + c;
              own +=
                  related[k] * h3[j * 3]
                      + related[CONSIDERED + k] * h3[j * 3 + 1]
                      + related[2 * CONSIDERED + k] * h3[j * 3 + 2];
            }
            sum += hc * own;
          }
        }
        s3[i * 3 + j] = sum;
      }
    }
    s3[0] += noise.xx();
    s3[1] += noise.xy();
    s3[3] += noise.xy();
    s3[4] += noise.yy();
    if (rows == 3) {
      s3[8] += noise.heading();
    }
    symmetrizeS();
    // The position's own agreement, from the position block of S.
    double s00 = s3[0];
    double s01 = s3[1];
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
    // for the offset is zero, as it is for every considered error.
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
   * Applies the camera measurement last given to {@link #innovation}: a Kalman update of what it
   * may correct, considering the rest.
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
    // P = P - K C^T - C K^T + K S K^T, with C = P H^T: right for any gain, so for one that leaves
    // the heading or a considered error alone too. (For the full Kalman gain, the last two terms
    // cancel to P - K C^T.)
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
    // How the estimate relates to each considered error changes by the gain times how the
    // measurement related to it: C's considered rows, H times P's considered columns.
    for (int k = 0; k < CONSIDERED; k++) {
      int own = camera >= 0 ? k - turnColumn(camera) : -1;
      boolean ofThisCamera = own >= 0 && own < PER_CAMERA;
      for (int r = 0; r < rows; r++) {
        double sum =
            related[k] * h3[r * 3]
                + related[CONSIDERED + k] * h3[r * 3 + 1]
                + related[2 * CONSIDERED + k] * h3[r * 3 + 2];
        if (ofThisCamera) {
          sum += ownVariance(own) * hCamera[r * PER_CAMERA + own];
        }
        column[r] = sum;
      }
      for (int i = 0; i < 3; i++) {
        double change = 0;
        for (int r = 0; r < rows; r++) {
          change += gain[i * 3 + r] * column[r];
        }
        related[i * CONSIDERED + k] -= change;
      }
    }
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
    landmarkX = nx;
    landmarkY = ny;
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
    for (int c = 0; c < CONSIDERED; c++) {
      double across = landmarkX * related[c] + landmarkY * related[CONSIDERED + c];
      for (int row = 0; row < 3; row++) {
        related[row * CONSIDERED + c] -= k[row] * across;
      }
    }
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

  /** Copies the uncertainty, {@link #UNCERTAINTY} numbers, for the history to keep. */
  void copyUncertaintyTo(double[] into, int at) {
    System.arraycopy(covariance, 0, into, at, 9);
    System.arraycopy(related, 0, into, at + 9, related.length);
  }

  void restore(double x, double y, double offset, double[] uncertainties, int at) {
    this.x = x;
    this.y = y;
    this.offset = offset;
    System.arraycopy(uncertainties, at, covariance, 0, 9);
    System.arraycopy(uncertainties, at + 9, related, 0, related.length);
  }

  /** A camera's turn's column; its shift's x, y, and heading follow it. */
  private static int turnColumn(int camera) {
    return 2 + PER_CAMERA * camera;
  }

  /**
   * A camera's own considered variance: its turn's (0), its shift's along x (1) or y (2), or its
   * headings' shift (3).
   */
  private double ownVariance(int which) {
    return switch (which) {
      case 0 -> turnVariance;
      case 3 -> headingShiftVariance;
      default -> shiftVariance;
    };
  }

  /** The position's own block, and nothing relating it to the heading or to considered errors. */
  private void setPosition(double xx, double xy, double yy) {
    covariance[0] = xx;
    covariance[1] = xy;
    covariance[3] = xy;
    covariance[4] = yy;
    clearCrossTerms();
    unrelate(0);
    unrelate(1);
  }

  /** The heading unrelated to the position. */
  private void clearCrossTerms() {
    covariance[2] = 0;
    covariance[6] = 0;
    covariance[5] = 0;
    covariance[7] = 0;
  }

  /** One of x, y, or the offset, unrelated to every considered error: it was just set afresh. */
  private void unrelate(int row) {
    Arrays.fill(related, row * CONSIDERED, (row + 1) * CONSIDERED, 0);
  }

  private void symmetrize() {
    covariance[1] = covariance[3] = (covariance[1] + covariance[3]) / 2;
    covariance[2] = covariance[6] = (covariance[2] + covariance[6]) / 2;
    covariance[5] = covariance[7] = (covariance[5] + covariance[7]) / 2;
  }

  private void symmetrizeS() {
    for (int i = 0; i < rows; i++) {
      for (int j = i + 1; j < rows; j++) {
        s3[i * 3 + j] = s3[j * 3 + i] = (s3[i * 3 + j] + s3[j * 3 + i]) / 2;
      }
    }
  }

  private static double square(double value) {
    return value * value;
  }
}
