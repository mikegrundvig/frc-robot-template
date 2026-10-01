package frc.robot.lib.localization;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * Swerve kinematics for localization. Each module's travel between two samples is a small vector
 * (how far it rolled, at the angle it pointed); the robot's motion is the one rigid motion that
 * best fits all of them (least squares, as WPILib's odometry does). How badly it fits is the
 * residual: modules that disagree with every rigid motion are slipping, lifted, or being pushed,
 * which is when odometry deserves the least trust (6800's skid ratio, 2056's wheel scores).
 *
 * <p>The residual is reported as how far the disagreement may have put the fitted travel off: the
 * modules' misses, spread over what the fit leaves free (two numbers a module, less the motion's
 * three), carried into the travel as least squares carries each module's error into it. One module
 * of four slipping puts the fit off by a quarter of its slip, and the residual says that much,
 * within about a tenth, whichever way it slips (exactly, for a square drivetrain's wheel slipping
 * forward or sideways).
 *
 * <p>It also says how much of that one wheel alone explains ({@link BodyMotion#oneWheelMeters}), as
 * a navigation receiver tells one bad satellite from noise spread over all of them. One wheel
 * slipping on its own (spinning past the floor, lifted) leaves misses that lie as that wheel's
 * alone would: the share of their squares the best single wheel explains is all of them. Misses
 * spread over the wheels (scrub, noise, wheels reaching a new speed each at its own pace) leave
 * every wheel a part: with four modules, the best one explains about 0.7 of them by chance, half
 * the time. How far the best explains past that chance level, out of what's left above it, is the
 * share counted as one wheel's: all of it for one wheel slipping alone, a quarter of it on average
 * for misses spread evenly. With two modules no wheel can be told from the other, and it all counts
 * as one wheel's.
 */
public final class SwerveKinematicsModel implements DriveKinematicsModel {
  private final double[] moduleX;
  private final double[] moduleY;

  /** (A^T A)^-1 A^T for the rigid-motion model: 3 rows of 2n, row-major. */
  private final double[] pseudoInverse;

  /**
   * How the modules' misses, squared, carry into the fitted travel's error, squared: the travel's
   * share of (A^T A)^-1, over the fit's spare degrees of freedom, 2n - 3.
   */
  private final double missToTravel;

  /**
   * For each module, the inverse of how its own misses carry into all of them (E_i^T P E_i, where P
   * takes the travels to the fit's misses): 4 numbers a module, row-major. Empty with two modules.
   */
  private final double[] alone;

  /**
   * The share of the misses' squares the best single module explains by chance half the time, when
   * the misses are spread evenly over all of them.
   */
  private final double byChance;

  /**
   * @param modules where each module sits, from the robot's center, in the samples' order
   */
  public SwerveKinematicsModel(Translation2d... modules) {
    int n = modules.length;
    if (n < 2) {
      throw new IllegalArgumentException("swerve needs at least 2 modules: " + n);
    }
    moduleX = new double[n];
    moduleY = new double[n];
    double sumX = 0;
    double sumY = 0;
    double sumSquares = 0;
    for (int i = 0; i < n; i++) {
      moduleX[i] = modules[i].getX();
      moduleY[i] = modules[i].getY();
      sumX += moduleX[i];
      sumY += moduleY[i];
      sumSquares += moduleX[i] * moduleX[i] + moduleY[i] * moduleY[i];
    }
    // A has two rows per module: [1, 0, -y] and [0, 1, x], mapping (dx, dy, dtheta) to that
    // module's travel. Its normal matrix A^T A:
    double[] normal = {n, 0, -sumY, 0, n, sumX, -sumY, sumX, sumSquares};
    double[] inverse = new double[9];
    if (!Matrices.invert3(normal, inverse)) {
      throw new IllegalArgumentException("the modules must not all sit at one point");
    }
    missToTravel = (inverse[0] + inverse[4]) / (2 * n - 3);
    alone = n > 2 ? aloneInverses(inverse) : new double[0];
    // Spread evenly over the 2n - 3 directions the misses can take, the share one module's two
    // explain is Beta(1, (2n - 5) / 2): the median of the largest of n of them.
    byChance = n > 2 ? 1 - Math.pow(1 - Math.pow(0.5, 1.0 / n), 2.0 / (2 * n - 5)) : 1;
    pseudoInverse = new double[3 * 2 * n];
    for (int row = 0; row < 3; row++) {
      for (int i = 0; i < n; i++) {
        // Column 2i of A^T is (1, 0, -y_i); column 2i + 1 is (0, 1, x_i).
        pseudoInverse[row * 2 * n + 2 * i] = inverse[row * 3] - inverse[row * 3 + 2] * moduleY[i];
        pseudoInverse[row * 2 * n + 2 * i + 1] =
            inverse[row * 3 + 1] + inverse[row * 3 + 2] * moduleX[i];
      }
    }
  }

  @Override
  public BodyMotion motion(OdometrySample from, OdometrySample to) {
    int n = moduleX.length;
    if (from.wheels() != n || to.wheels() != n) {
      throw new IllegalArgumentException(
          "samples of " + from.wheels() + " and " + to.wheels() + " wheels, for " + n + " modules");
    }
    double[] travel = new double[2 * n];
    for (int i = 0; i < n; i++) {
      double distance = to.distanceMeters(i) - from.distanceMeters(i);
      // The module may have steered during the interval: take the angle halfway through it.
      double angle =
          from.angleRadians(i)
              + MathUtil.angleModulus(to.angleRadians(i) - from.angleRadians(i)) / 2;
      travel[2 * i] = distance * Math.cos(angle);
      travel[2 * i + 1] = distance * Math.sin(angle);
    }
    double dx = 0;
    double dy = 0;
    double dtheta = 0;
    for (int k = 0; k < 2 * n; k++) {
      dx += pseudoInverse[k] * travel[k];
      dy += pseudoInverse[2 * n + k] * travel[k];
      dtheta += pseudoInverse[4 * n + k] * travel[k];
    }
    double[] misses = new double[2 * n];
    double squares = 0;
    for (int i = 0; i < n; i++) {
      misses[2 * i] = travel[2 * i] - (dx - moduleY[i] * dtheta);
      misses[2 * i + 1] = travel[2 * i + 1] - (dy + moduleX[i] * dtheta);
      squares += misses[2 * i] * misses[2 * i] + misses[2 * i + 1] * misses[2 * i + 1];
    }
    double residual = Math.sqrt(squares * missToTravel);
    return new BodyMotion(dx, dy, dtheta, residual, oneWheelShare(misses, squares) * residual);
  }

  /**
   * How much of the misses one module alone explains, past what the best of them explains by
   * chance, from none to all.
   */
  private double oneWheelShare(double[] misses, double squares) {
    if (alone.length == 0) {
      return 1;
    }
    if (!(squares > 0)) {
      return 0;
    }
    double best = 0;
    for (int i = 0; i < alone.length / 4; i++) {
      double x = misses[2 * i];
      double y = misses[2 * i + 1];
      // The misses' squares a slip of this module alone would leave: m_i^T (E_i^T P E_i)^-1 m_i.
      double explained =
          x * (alone[4 * i] * x + alone[4 * i + 1] * y)
              + y * (alone[4 * i + 2] * x + alone[4 * i + 3] * y);
      best = Math.max(best, explained / squares);
    }
    return Math.clamp((best - byChance) / (1 - byChance), 0.0, 1.0);
  }

  /**
   * For each module, (E_i^T P E_i)^-1: E_i^T P E_i is how a slip of that module alone shows in its
   * own misses, the identity less A_i (A^T A)^-1 A_i^T, where A_i is its two rows of A.
   */
  private double[] aloneInverses(double[] inverse) {
    int n = moduleX.length;
    double[] out = new double[4 * n];
    for (int i = 0; i < n; i++) {
      double[][] rows = {{1, 0, -moduleY[i]}, {0, 1, moduleX[i]}};
      double[] g = new double[4];
      for (int r = 0; r < 2; r++) {
        for (int c = 0; c < 2; c++) {
          double carried = 0;
          for (int k = 0; k < 3; k++) {
            for (int l = 0; l < 3; l++) {
              carried += rows[r][k] * inverse[k * 3 + l] * rows[c][l];
            }
          }
          g[2 * r + c] = (r == c ? 1 : 0) - carried;
        }
      }
      double det = g[0] * g[3] - g[1] * g[2];
      if (!(det > 1e-12)) {
        throw new IllegalArgumentException("a module the others can't check: " + i);
      }
      out[4 * i] = g[3] / det;
      out[4 * i + 1] = -g[1] / det;
      out[4 * i + 2] = -g[2] / det;
      out[4 * i + 3] = g[0] / det;
    }
    return out;
  }
}
