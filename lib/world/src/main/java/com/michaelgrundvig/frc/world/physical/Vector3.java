package com.michaelgrundvig.frc.world.physical;

/**
 * A point or a direction in 3D, in meters for a point. Computed with {@link StrictMath}, so the
 * same numbers come out on every computer: a generated file that differed by a rounding on a
 * teammate's laptop would read as an edit made outside the build.
 *
 * @param x along the frame's x: forward on the robot, toward the red alliance wall on the field
 * @param y along its y: left on the robot, to the blue drivers' left on the field
 * @param z along its z: up
 */
public record Vector3(double x, double y, double z) {
  public static final Vector3 ZERO = new Vector3(0, 0, 0);

  /** Straight up: the floor's normal. */
  public static final Vector3 UP = new Vector3(0, 0, 1);

  public Vector3 plus(Vector3 other) {
    return new Vector3(x + other.x, y + other.y, z + other.z);
  }

  public Vector3 minus(Vector3 other) {
    return new Vector3(x - other.x, y - other.y, z - other.z);
  }

  public Vector3 times(double factor) {
    return new Vector3(x * factor, y * factor, z * factor);
  }

  public double dot(Vector3 other) {
    return x * other.x + y * other.y + z * other.z;
  }

  public Vector3 cross(Vector3 other) {
    return new Vector3(
        y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
  }

  public double length() {
    return StrictMath.sqrt(dot(this));
  }

  /** The same direction, a meter long; not a number for the zero vector, which has none. */
  public Vector3 unit() {
    return times(1 / length());
  }
}
