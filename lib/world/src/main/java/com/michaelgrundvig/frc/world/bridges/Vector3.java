package com.michaelgrundvig.frc.world.bridges;

/**
 * A point or a direction in 3D, in meters. The writers' own, computed with {@link StrictMath}, so a
 * generated file comes out byte for byte the same on every computer: a file that differed by a
 * rounding on a teammate's laptop would read as an edit made outside the build.
 */
record Vector3(double x, double y, double z) {
  static final Vector3 ZERO = new Vector3(0, 0, 0);

  Vector3 plus(Vector3 other) {
    return new Vector3(x + other.x, y + other.y, z + other.z);
  }

  Vector3 minus(Vector3 other) {
    return new Vector3(x - other.x, y - other.y, z - other.z);
  }

  Vector3 times(double factor) {
    return new Vector3(x * factor, y * factor, z * factor);
  }

  double dot(Vector3 other) {
    return x * other.x + y * other.y + z * other.z;
  }

  Vector3 cross(Vector3 other) {
    return new Vector3(
        y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
  }

  double length() {
    return StrictMath.sqrt(dot(this));
  }

  /** The same direction, a meter long. */
  Vector3 unit() {
    return times(1 / length());
  }
}
