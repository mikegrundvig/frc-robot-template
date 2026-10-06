package com.michaelgrundvig.frc.world.physical;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Mass;

/**
 * How hard a part is to turn, about its center of mass, in its own frame: the six terms of its
 * inertia tensor, in kg·m². They're the tensor's own entries ({@code [ixx ixy ixz; ixy iyy iyz; ixz
 * iyz izz]}), as URDF, ODE and MuJoCo take them, so a product of inertia is minus the integral of,
 * say, x times y over the mass. A CAD program's mass-properties report gives them about the center
 * of mass, but check its sign: some report the integral itself.
 *
 * <p>All zero is a part whose inertia nobody has given ({@link #ZERO}): a point mass, or no mass at
 * all. What must turn as a body, the robot itself, asks for a solid one ({@link #whyNotSolid}).
 *
 * @param ixx about the part's x axis, kg·m²
 * @param iyy about its y axis, kg·m²
 * @param izz about its z axis, kg·m²
 * @param ixy the tensor's entry for x and y, kg·m²: zero when the part is symmetric across either
 *     axis
 * @param ixz its entry for x and z, kg·m²
 * @param iyz its entry for y and z, kg·m²
 */
@ConfigRecord
public record Inertia(double ixx, double iyy, double izz, double ixy, double ixz, double iyz) {
  /** None: a part whose inertia isn't known, or a point mass. Turning it takes nothing. */
  public static final Inertia ZERO =
      InertiaBuilder.builder().ixx(0).iyy(0).izz(0).ixy(0).ixz(0).iyz(0).build();

  /**
   * How far a tensor built from others' sums may miss a bound, as a share of its size: a few
   * roundings, which a zero or a point mass turned on its frame comes out with.
   */
  private static final double ROUNDING = 1e-12;

  public Inertia {
    Checks.on(Inertia.class)
        .field("ixx", ixx, nonNegative())
        .field("iyy", iyy, nonNegative())
        .field("izz", izz, nonNegative())
        .field("ixy", ixy, finite())
        .field("ixz", ixz, finite())
        .field("iyz", iyz, finite())
        .check(
            "ixx",
            () -> {
              double slack = ROUNDING * (ixx + iyy + izz);
              return ixx <= iyy + izz + slack
                      && iyy <= ixx + izz + slack
                      && izz <= ixx + iyy + slack
                  ? null
                  : "("
                      + shown(ixx)
                      + ", "
                      + shown(iyy)
                      + ", "
                      + shown(izz)
                      + ") can't be a body's: each must be at most the sum of the other two";
            })
        .check(
            "ixy",
            () -> {
              // Positive semidefinite, by its principal minors: no axis turns with less than no
              // effort. Zero is a part with none given; a point mass has none about its center.
              double size = ixx + iyy + izz;
              double slack = ROUNDING * size * size;
              double xy = ixx * iyy - ixy * ixy;
              double xz = ixx * izz - ixz * ixz;
              double yz = iyy * izz - iyz * iyz;
              return xy >= -slack && xz >= -slack && yz >= -slack && determinant() >= -slack * size
                  ? null
                  : "the products of inertia are too large for the moments: no body turns that way";
            })
        .orThrow();
  }

  /**
   * Why it isn't a solid body's, or null when it is: one that takes effort to turn about every
   * axis, as a whole robot does. A part may have none ({@link #ZERO}); the robot can't, or its
   * drivetrain couldn't turn it.
   */
  public @Nullable String whyNotSolid() {
    return ixx > 0 && iyy > 0 && izz > 0 && ixx * iyy - ixy * ixy > 0 && determinant() > 0
        ? null
        : "("
            + shown(ixx)
            + ", "
            + shown(iyy)
            + ", "
            + shown(izz)
            + ") turns too easily: a whole body takes effort to turn about every axis";
  }

  /** Whether it's all zero: none given. */
  public boolean isZero() {
    return ixx == 0 && iyy == 0 && izz == 0 && ixy == 0 && ixz == 0 && iyz == 0;
  }

  private double determinant() {
    return ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz);
  }

  /** Starts describing an inertia, one named value at a time. */
  public static InertiaBuilder.IxxStage builder() {
    return InertiaBuilder.builder();
  }

  /**
   * The same body's inertia along turned axes: its own axes turned by a rotation, as a part turned
   * on its mount is seen from what it's mounted on ({@code R I R}<sup>T</sup>).
   */
  public Inertia rotated(Rotation3d rotation) {
    double[] r = rotation(rotation);
    double[] tensor = entries();
    double[] turned = new double[9];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        double sum = 0;
        for (int k = 0; k < 3; k++) {
          for (int l = 0; l < 3; l++) {
            sum += r[3 * i + k] * tensor[3 * k + l] * r[3 * j + l];
          }
        }
        turned[3 * i + j] = sum;
      }
    }
    return of(turned);
  }

  /** How hard it is to turn about an axis through its center of mass, in kg·m². */
  public double about(Vector3 axis) {
    Vector3 a = axis.unit();
    double[] t = entries();
    double[] v = {a.x(), a.y(), a.z()};
    double sum = 0;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        sum += v[i] * t[3 * i + j] * v[j];
      }
    }
    return sum;
  }

  /** The tensor's nine entries, row by row. */
  double[] entries() {
    return new double[] {ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz};
  }

  /**
   * An inertia from a tensor's nine entries, row by row, made symmetric.
   *
   * @throws com.michaelgrundvig.frc.world.config.ConfigException if no body has it
   */
  static Inertia of(double[] t) {
    return builder()
        .ixx(t[0])
        .iyy(t[4])
        .izz(t[8])
        .ixy((t[1] + t[3]) / 2)
        .ixz((t[2] + t[6]) / 2)
        .iyz((t[5] + t[7]) / 2)
        .build();
  }

  /** A rotation's matrix, row by row. */
  static double[] rotation(Rotation3d rotation) {
    var matrix = rotation.toMatrix();
    double[] r = new double[9];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        r[3 * i + j] = matrix.get(i, j);
      }
    }
    return r;
  }

  /**
   * A solid box's, of a mass spread evenly through it, about its center, its axes along its sides:
   * the provisional stand-in until CAD or a measurement gives the real one. A robot or a link isn't
   * solid and even, so this is a guess, and its register row says so.
   *
   * @param size its length along x, width along y, and height along z, in meters
   */
  public static Inertia box(Mass mass, Translation3d size) {
    double m = mass.in(Kilograms) / 12;
    double x = size.getX() * size.getX();
    double y = size.getY() * size.getY();
    double z = size.getZ() * size.getZ();
    return builder()
        .ixx(m * (y + z))
        .iyy(m * (x + z))
        .izz(m * (x + y))
        .ixy(0)
        .ixz(0)
        .iyz(0)
        .build();
  }
}
