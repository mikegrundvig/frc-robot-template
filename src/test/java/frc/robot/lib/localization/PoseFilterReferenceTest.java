package frc.robot.lib.localization;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * {@link PoseFilter} against the textbook it shortcuts: one Schmidt-Kalman filter over the whole
 * augmented state (x, y, the heading offset, and every considered error), each update in Joseph
 * form, written plainly with full matrices. The filter keeps only what it needs of that (the 3x3
 * covariance, and how the estimate relates to each considered error), so random sequences of every
 * operation it has must leave both with the same estimate and the same covariance, and the joint
 * covariance must stay positive semidefinite.
 */
class PoseFilterReferenceTest {
  private static final int C = PoseFilter.CONSIDERED;
  private static final int N = 3 + C;
  private static final int PER_CAMERA = 4;

  private static final double SCALE = Math.toRadians(1) / (2 * Math.PI);
  private static final double BACKUP_SCALE = Math.toRadians(2) / (2 * Math.PI);
  private static final double TURN = Math.toRadians(0.25);
  private static final double SHIFT = 0.01;
  private static final double HEADING_SHIFT = Math.toRadians(2);
  private static final double SHIFT_METERS = 1.0;
  private static final double SHIFT_RADIANS = Math.toRadians(45);

  /** The reference: the augmented state's mean (just x, y, offset: the rest stay zero) and P. */
  private final double[] mean = new double[3];

  private double[][] p = new double[N][N];

  private static double ownVariance(int which) {
    return which == 0 ? TURN * TURN : which == 3 ? HEADING_SHIFT * HEADING_SHIFT : SHIFT * SHIFT;
  }

  /** A camera's considered error's index in the augmented state, after the two IMUs' scales. */
  private static int column(int camera, int which) {
    return 3 + 2 + PER_CAMERA * camera + which;
  }

  /** A considered error's own variance, by its index among them. */
  private static double consideredVariance(int k) {
    return k == 0
        ? SCALE * SCALE
        : k == 1 ? BACKUP_SCALE * BACKUP_SCALE : ownVariance((k - 2) % PER_CAMERA);
  }

  private void startConsidered() {
    p = new double[N][N];
    for (int k = 0; k < C; k++) {
      p[3 + k][3 + k] = consideredVariance(k);
    }
  }

  private void setPosition(double xx, double xy, double yy) {
    for (int j = 0; j < N; j++) {
      p[0][j] = p[j][0] = 0;
      p[1][j] = p[j][1] = 0;
    }
    p[0][0] = xx;
    p[0][1] = p[1][0] = xy;
    p[1][1] = yy;
  }

  private void unrelateHeading() {
    for (int j = 0; j < N; j++) {
      if (j != 2) {
        p[2][j] = p[j][2] = 0;
      }
    }
  }

  private void predict(
      double startYaw,
      double turn,
      double dx,
      double dy,
      double positionNoise,
      double headingNoise,
      YawSource turnedBy) {
    double heading = startYaw + mean[2];
    double along = dx;
    double across = dy;
    if (Math.abs(turn) > 1e-9) {
      double s = Math.sin(turn) / turn;
      double c = (1 - Math.cos(turn)) / turn;
      along = s * dx - c * dy;
      across = c * dx + s * dy;
    }
    double moveX = Math.cos(heading) * along - Math.sin(heading) * across;
    double moveY = Math.sin(heading) * along + Math.cos(heading) * across;
    mean[0] += moveX;
    mean[1] += moveY;
    double[][] f = identity(N);
    f[0][2] = -moveY;
    f[1][2] = moveX;
    double[][] g = identity(N);
    if (turnedBy == YawSource.GYRO) {
      g[2][3] = turn; // the gyro's scale error, times the turn, goes into the heading's
    } else if (turnedBy == YawSource.BACKUP) {
      g[2][4] = turn; // and the backup IMU's, its own
    }
    // Each camera's shift fades as the robot moves and turns, to a fresh one of the same variance.
    double remains =
        Math.exp(-Math.hypot(moveX, moveY) / SHIFT_METERS - Math.abs(turn) / SHIFT_RADIANS);
    double[][] fade = identity(N);
    double[][] q = new double[N][N];
    for (int camera = 0; camera < PoseFilter.MAX_CAMERAS; camera++) {
      for (int which = 1; which < PER_CAMERA; which++) {
        int k = column(camera, which);
        fade[k][k] = remains;
        q[k][k] = (1 - remains * remains) * ownVariance(which);
      }
    }
    double[][] phi = multiply(fade, multiply(g, f));
    p = add(multiply(multiply(phi, p), transpose(phi)), q);
    p[0][0] += positionNoise;
    p[1][1] += positionNoise;
    p[2][2] += headingNoise;
  }

  private void camera(VisionMeasurement measurement, double yawThen, int camera) {
    PoseCovariance noise = measurement.covariance();
    int rows = noise.hasHeading() ? 3 : 2;
    double leverX = measurement.lever().robotToTags().getX();
    double leverY = measurement.lever().robotToTags().getY();
    double[][] h = new double[rows][N];
    h[0][0] = 1;
    h[0][2] = -leverY;
    h[1][1] = 1;
    h[1][2] = leverX;
    if (camera >= 0) {
      h[0][column(camera, 0)] = -leverY;
      h[0][column(camera, 1)] = 1;
      h[1][column(camera, 0)] = leverX;
      h[1][column(camera, 2)] = 1;
    }
    if (rows == 3) {
      h[2][2] = 1;
      if (camera >= 0) {
        h[2][column(camera, 0)] = 1;
        h[2][column(camera, 3)] = 1;
      }
    }
    double[][] r = new double[rows][rows];
    r[0][0] = noise.xx();
    r[0][1] = r[1][0] = noise.xy();
    r[1][1] = noise.yy();
    if (rows == 3) {
      r[2][2] = noise.heading();
    }
    if (camera < 0) {
      // A camera without a place: its turn and shift, new in each image, are just more noise.
      double[][] own = new double[rows][PER_CAMERA];
      own[0][0] = -leverY;
      own[0][1] = 1;
      own[1][0] = leverX;
      own[1][2] = 1;
      if (rows == 3) {
        own[2][0] = 1;
        own[2][3] = 1;
      }
      for (int i = 0; i < rows; i++) {
        for (int j = 0; j < rows; j++) {
          for (int which = 0; which < PER_CAMERA; which++) {
            r[i][j] += own[i][which] * ownVariance(which) * own[j][which];
          }
        }
      }
    }
    // Predicted: the position, turned through the lever by how far the heading it was solved with
    // is from the estimate's then.
    double solvedOff =
        MathUtil.angleModulus(measurement.lever().heading().getRadians() - (yawThen + mean[2]));
    double[] innovation = new double[rows];
    innovation[0] = measurement.pose().getX() - mean[0] - solvedOff * leverY;
    innovation[1] = measurement.pose().getY() - mean[1] + solvedOff * leverX;
    if (rows == 3) {
      innovation[2] =
          MathUtil.angleModulus(
              measurement.pose().getRotation().getRadians() - (yawThen + mean[2]));
    }
    double[][] gain =
        multiply(
            multiply(p, transpose(h)), inverse(add(multiply(multiply(h, p), transpose(h)), r)));
    // Schmidt: the considered errors are never corrected; a position alone never turns the heading.
    for (int i = 3; i < N; i++) {
      for (int j = 0; j < rows; j++) {
        gain[i][j] = 0;
      }
    }
    if (rows == 2) {
      gain[2][0] = gain[2][1] = 0;
    }
    joseph(gain, h, r, innovation);
  }

  private void landmark(LandmarkEvent landmark) {
    double[][] h = new double[1][N];
    h[0][0] = landmark.across().getCos();
    h[0][1] = landmark.across().getSin();
    double[][] r = {{landmark.stdDevMeters() * landmark.stdDevMeters()}};
    double[][] gain =
        multiply(
            multiply(p, transpose(h)), inverse(add(multiply(multiply(h, p), transpose(h)), r)));
    for (int i = 3; i < N; i++) {
      gain[i][0] = 0;
    }
    joseph(gain, h, r, new double[] {-landmark.offset(new Translation2d(mean[0], mean[1]))});
  }

  /** x += K ν, and P = (I - K H) P (I - K H)ᵀ + K R Kᵀ: right for any gain. */
  private void joseph(double[][] gain, double[][] h, double[][] r, double[] innovation) {
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < innovation.length; j++) {
        mean[i] += gain[i][j] * innovation[j];
      }
    }
    double[][] keep = add(identity(N), scale(multiply(gain, h), -1));
    p =
        add(
            multiply(multiply(keep, p), transpose(keep)),
            multiply(multiply(gain, r), transpose(gain)));
  }

  private final double[] kept = new double[PoseFilter.UNCERTAINTY];

  /** How far the filter is from the reference, relative to the estimate's own variances. */
  private double difference(PoseFilter filter) {
    filter.copyUncertaintyTo(kept, 0);
    double scale = 1e-12;
    for (int i = 0; i < 3; i++) {
      scale = Math.max(scale, Math.abs(p[i][i]));
    }
    double worst = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        worst = Math.max(worst, Math.abs(kept[i * 3 + j] - p[i][j]) / scale);
      }
      for (int k = 0; k < C; k++) {
        worst = Math.max(worst, Math.abs(kept[9 + i * C + k] - p[i][3 + k]) / scale);
      }
    }
    double moved =
        Math.max(
            Math.abs(filter.x() - mean[0]),
            Math.max(Math.abs(filter.y() - mean[1]), Math.abs(filter.offset() - mean[2])));
    return Math.max(worst, 1e-3 * moved / Math.sqrt(scale));
  }

  /**
   * The joint covariance is positive semidefinite exactly when the estimate's covariance, less what
   * the considered errors explain of it, is: its smallest eigenvalue, over its trace.
   */
  private double leastEigenvalue(PoseFilter filter) {
    filter.copyUncertaintyTo(kept, 0);
    double[][] schur = new double[3][3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        double value = kept[i * 3 + j];
        for (int k = 0; k < C; k++) {
          double variance = consideredVariance(k);
          if (variance > 0) {
            value -= kept[9 + i * C + k] * kept[9 + j * C + k] / variance;
          }
        }
        schur[i][j] = value;
      }
    }
    return smallestEigenvalue(schur) / Math.max(kept[0] + kept[4] + kept[8], 1e-300);
  }

  @Test
  void everyOperationMatchesTheAugmentedSchmidtKalmanFilter() {
    PoseFilter.Considered considered =
        new PoseFilter.Considered(
            SCALE, BACKUP_SCALE, TURN, SHIFT, HEADING_SHIFT, SHIFT_METERS, SHIFT_RADIANS);
    double worstDifference = 0;
    double worstEigenvalue = Double.POSITIVE_INFINITY;
    int operations = 0;
    for (int sequence = 0; sequence < 60; sequence++) {
      SplittableRandom random = new SplittableRandom(sequence * 7919L + 1);
      PoseFilter filter = new PoseFilter(considered);
      double start = logUniform(random, 0.01, 100);
      double startHeading = logUniform(random, 1e-3, 3);
      filter.set(
          new PoseReset(
              0,
              new Pose2d(1, 2, new Rotation2d(0.3)),
              PoseCovariance.fromStdDevs(start, start, startHeading),
              "x"),
          0.1);
      mean[0] = 1;
      mean[1] = 2;
      mean[2] = 0.2;
      startConsidered();
      setPosition(start * start, 0, start * start);
      p[2][2] = startHeading * startHeading;
      double yaw = 0.1;
      for (int op = 0; op < 300; op++) {
        double u = random.nextDouble();
        if (u < 0.55) {
          double turn =
              random.nextDouble() < 0.9
                  ? 0.06 * (random.nextDouble() - 0.5)
                  : 2 * (random.nextDouble() - 0.5);
          double dx = 0.04 * (random.nextDouble() - 0.5);
          double dy = 0.04 * (random.nextDouble() - 0.5);
          double positionNoise = logUniform(random, 1e-9, 1e-4);
          double headingNoise = logUniform(random, 1e-12, 1e-5);
          double which = random.nextDouble();
          YawSource turnedBy =
              which < 0.6 ? YawSource.GYRO : which < 0.85 ? YawSource.BACKUP : YawSource.WHEELS;
          filter.predict(yaw, turn, dx, dy, positionNoise, headingNoise, turnedBy);
          predict(yaw, turn, dx, dy, positionNoise, headingNoise, turnedBy);
          yaw += turn;
        } else if (u < 0.90) {
          VisionMeasurement measurement = measurement(random, filter);
          int slot = random.nextInt(10);
          int camera = slot < PoseFilter.MAX_CAMERAS ? slot : -1;
          double yawThen = yaw - 0.01 * random.nextDouble();
          PoseFilter.Innovation innovation = filter.innovation(measurement, yawThen, camera);
          if (innovation == null) {
            continue;
          }
          filter.correct(innovation);
          camera(measurement, yawThen, camera);
        } else if (u < 0.94) {
          LandmarkEvent event =
              new LandmarkEvent(
                  1,
                  "bump",
                  new Translation2d(filter.x() + 0.02, filter.y()),
                  new Rotation2d(6 * random.nextDouble()),
                  logUniform(random, 0.005, 0.1));
          PoseFilter.AxisInnovation innovation = filter.innovationAlong(event);
          if (innovation == null) {
            continue;
          }
          filter.correct(innovation);
          landmark(event);
        } else if (u < 0.95) {
          double variance = logUniform(random, 1e-4, 1);
          boolean withHeading = random.nextBoolean();
          double headingVariance = logUniform(random, 1e-6, 1);
          PoseReset reset =
              new PoseReset(
                  0,
                  new Pose2d(filter.x() + 0.1, filter.y(), new Rotation2d(0.2)),
                  new PoseCovariance(
                      variance,
                      0,
                      variance,
                      withHeading ? headingVariance : Double.POSITIVE_INFINITY),
                  "reset");
          filter.set(reset, yaw);
          mean[0] = reset.pose().getX();
          mean[1] = reset.pose().getY();
          if (withHeading) {
            mean[2] = 0.2 - yaw;
            p[2][2] = headingVariance;
            unrelateHeading();
          }
          setPosition(variance, 0, variance);
        } else if (u < 0.96) {
          double variance = logUniform(random, 1e-6, 1);
          double offset = random.nextDouble();
          filter.resetHeading(offset, variance);
          mean[2] = offset;
          p[2][2] = variance;
          unrelateHeading();
        } else if (u < 0.97) {
          double variance = logUniform(random, 1e-4, 1);
          filter.shift(0.1, -0.1, variance);
          mean[0] += 0.1;
          mean[1] -= 0.1;
          setPosition(variance, 0, variance);
        } else if (u < 0.98) {
          double positionVariance = logUniform(random, 1e-4, 1);
          double headingVariance = logUniform(random, 1e-6, 1);
          filter.turn(0.05, positionVariance, headingVariance);
          mean[2] += 0.05;
          p[2][2] = headingVariance;
          unrelateHeading();
          setPosition(positionVariance, 0, positionVariance);
        } else {
          // What the history keeps, it gives back exactly.
          double[] stored = new double[PoseFilter.UNCERTAINTY];
          filter.copyUncertaintyTo(stored, 0);
          filter.restore(filter.x(), filter.y(), filter.offset(), stored, 0);
        }
        operations++;
        worstDifference = Math.max(worstDifference, difference(filter));
        worstEigenvalue = Math.min(worstEigenvalue, leastEigenvalue(filter));
      }
    }

    assertThat(operations).isGreaterThan(15_000);
    assertThat(worstDifference).as("relative difference from the reference").isLessThan(1e-5);
    assertThat(worstEigenvalue).as("least eigenvalue, over the trace").isGreaterThan(-1e-9);
  }

  /**
   * Thousands of very sure fixes from two cameras, leaning on the heading, a few with a heading of
   * their own, each a micrometer from the estimate: the covariance shrinks toward its floors, where
   * rounding would show first. The filter must stay positive semidefinite and on its measurements.
   * (Fixes a million times surer than the floors outrun the plain reference's own arithmetic, which
   * drifts off after a few hundred of them, so this checks the filter alone.)
   */
  @Test
  void manyVerySureFixesKeepTheCovariancePositive() {
    PoseFilter filter =
        new PoseFilter(
            new PoseFilter.Considered(
                SCALE, BACKUP_SCALE, TURN, SHIFT, HEADING_SHIFT, SHIFT_METERS, SHIFT_RADIANS));
    filter.set(new PoseReset(0, Pose2d.ZERO, PoseCovariance.fromStdDevs(1, 1, 0.1), "x"), 0);
    double yaw = 0;
    double worstEigenvalue = Double.POSITIVE_INFINITY;
    double worstMiss = 0;
    for (int i = 0; i < 3000; i++) {
      double x = filter.x() + 1e-6;
      VisionMeasurement sure =
          new VisionMeasurement(
              1,
              new Pose2d(x, filter.y(), new Rotation2d(yaw + filter.offset())),
              PoseCovariance.fromStdDevs(1e-6, 1e-6, i % 7 == 0 ? 1e-6 : Double.POSITIVE_INFINITY),
              new VisionMeasurement.Lever(
                  new Rotation2d(yaw + filter.offset()), new Translation2d(4, 1)),
              "camera",
              List.of(1),
              VisionMeasurement.Solver.HEADING_CONSTRAINED);
      PoseFilter.Innovation innovation = filter.innovation(sure, yaw, i % 3 == 0 ? 1 : 0);
      if (innovation == null) {
        continue;
      }
      filter.correct(innovation);
      worstMiss = Math.max(worstMiss, Math.abs(filter.x() - x));
      if (i % 5 == 0) {
        filter.predict(yaw, 0.001, 0.001, 0, 1e-12, 1e-14, YawSource.GYRO);
        yaw += 0.001;
      }
      worstEigenvalue = Math.min(worstEigenvalue, leastEigenvalue(filter));
    }

    assertThat(worstEigenvalue).as("least eigenvalue, over the trace").isGreaterThan(-1e-9);
    assertThat(worstMiss).as("how far the estimate strays from its measurements").isLessThan(1e-3);
    assertThat(filter.covariance().xx()).isPositive();
  }

  /** A camera's measurement, near the estimate: with or without heading, leaning or not. */
  private static VisionMeasurement measurement(SplittableRandom random, PoseFilter filter) {
    double spread = logUniform(random, 1e-3, 0.3);
    double heading =
        random.nextBoolean() ? Double.POSITIVE_INFINITY : logUniform(random, 1e-3, 0.2);
    VisionMeasurement.Lever lever =
        random.nextBoolean()
            ? VisionMeasurement.Lever.NONE
            : new VisionMeasurement.Lever(
                new Rotation2d(random.nextDouble()),
                new Translation2d(
                    logUniform(random, 0.5, 6), new Rotation2d(6.28 * random.nextDouble())));
    double correlation = 0.9 * random.nextDouble() - 0.45;
    return new VisionMeasurement(
        1,
        new Pose2d(
            filter.x() + 0.05 * (random.nextDouble() - 0.5),
            filter.y() + 0.05 * (random.nextDouble() - 0.5),
            new Rotation2d(random.nextDouble())),
        new PoseCovariance(
            spread * spread, correlation * spread * spread, spread * spread, heading * heading),
        lever,
        "camera",
        List.of(1),
        VisionMeasurement.Solver.HEADING_CONSTRAINED);
  }

  private static double logUniform(SplittableRandom random, double low, double high) {
    return Math.exp(Math.log(low) + random.nextDouble() * (Math.log(high) - Math.log(low)));
  }

  // Plain matrices, as small as the reference needs them.

  private static double[][] identity(int n) {
    double[][] result = new double[n][n];
    for (int i = 0; i < n; i++) {
      result[i][i] = 1;
    }
    return result;
  }

  private static double[][] multiply(double[][] a, double[][] b) {
    double[][] result = new double[a.length][b[0].length];
    for (int i = 0; i < a.length; i++) {
      for (int k = 0; k < b.length; k++) {
        double value = a[i][k];
        if (value != 0) {
          for (int j = 0; j < b[0].length; j++) {
            result[i][j] += value * b[k][j];
          }
        }
      }
    }
    return result;
  }

  private static double[][] add(double[][] a, double[][] b) {
    double[][] result = new double[a.length][a[0].length];
    for (int i = 0; i < a.length; i++) {
      for (int j = 0; j < a[0].length; j++) {
        result[i][j] = a[i][j] + b[i][j];
      }
    }
    return result;
  }

  private static double[][] scale(double[][] a, double factor) {
    double[][] result = new double[a.length][a[0].length];
    for (int i = 0; i < a.length; i++) {
      for (int j = 0; j < a[0].length; j++) {
        result[i][j] = factor * a[i][j];
      }
    }
    return result;
  }

  private static double[][] transpose(double[][] a) {
    double[][] result = new double[a[0].length][a.length];
    for (int i = 0; i < a.length; i++) {
      for (int j = 0; j < a[0].length; j++) {
        result[j][i] = a[i][j];
      }
    }
    return result;
  }

  /** Gauss-Jordan with partial pivoting: for the small innovation covariances only. */
  private static double[][] inverse(double[][] a) {
    int n = a.length;
    double[][] work = new double[n][2 * n];
    for (int i = 0; i < n; i++) {
      System.arraycopy(a[i], 0, work[i], 0, n);
      work[i][n + i] = 1;
    }
    for (int column = 0; column < n; column++) {
      int pivot = column;
      for (int row = column + 1; row < n; row++) {
        if (Math.abs(work[row][column]) > Math.abs(work[pivot][column])) {
          pivot = row;
        }
      }
      double[] swap = work[column];
      work[column] = work[pivot];
      work[pivot] = swap;
      double divisor = work[column][column];
      for (int j = 0; j < 2 * n; j++) {
        work[column][j] /= divisor;
      }
      for (int row = 0; row < n; row++) {
        if (row != column) {
          double factor = work[row][column];
          for (int j = 0; j < 2 * n; j++) {
            work[row][j] -= factor * work[column][j];
          }
        }
      }
    }
    double[][] result = new double[n][n];
    for (int i = 0; i < n; i++) {
      System.arraycopy(work[i], n, result[i], 0, n);
    }
    return result;
  }

  /** A symmetric 3x3 matrix's smallest eigenvalue, by Jacobi rotations. */
  private static double smallestEigenvalue(double[][] a) {
    double[][] w = new double[3][];
    for (int i = 0; i < 3; i++) {
      w[i] = a[i].clone();
    }
    for (int sweep = 0; sweep < 50; sweep++) {
      for (int i = 0; i < 2; i++) {
        for (int j = i + 1; j < 3; j++) {
          if (Math.abs(w[i][j]) < 1e-300) {
            continue;
          }
          double theta = (w[j][j] - w[i][i]) / (2 * w[i][j]);
          double t =
              theta == 0
                  ? 1
                  : Math.signum(theta) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
          double c = 1 / Math.sqrt(t * t + 1);
          double s = t * c;
          for (int k = 0; k < 3; k++) {
            double ki = w[k][i];
            double kj = w[k][j];
            w[k][i] = c * ki - s * kj;
            w[k][j] = s * ki + c * kj;
          }
          for (int k = 0; k < 3; k++) {
            double ik = w[i][k];
            double jk = w[j][k];
            w[i][k] = c * ik - s * jk;
            w[j][k] = s * ik + c * jk;
          }
        }
      }
    }
    return Math.min(w[0][0], Math.min(w[1][1], w[2][2]));
  }
}
