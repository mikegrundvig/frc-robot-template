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
    double squares = 0;
    for (int i = 0; i < n; i++) {
      double missX = travel[2 * i] - (dx - moduleY[i] * dtheta);
      double missY = travel[2 * i + 1] - (dy + moduleX[i] * dtheta);
      squares += missX * missX + missY * missY;
    }
    return new BodyMotion(dx, dy, dtheta, Math.sqrt(squares * missToTravel));
  }
}
