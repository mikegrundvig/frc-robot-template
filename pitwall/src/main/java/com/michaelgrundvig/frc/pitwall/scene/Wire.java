package com.michaelgrundvig.frc.pitwall.scene;

import java.math.BigDecimal;
import java.util.List;
import java.util.regex.Pattern;

/**
 * Numbers and names as a scene sends them. Lengths go in meters to the millimeter and angles in
 * degrees to the tenth, so a description stays small and a pose that moves less than that sends
 * nothing; a whole number goes without its ".0".
 */
final class Wire {
  /** A length's resolution, in meters: a millimeter. */
  static final double METER_STEP = 0.001;

  /** An angle's resolution, in degrees: a tenth. */
  static final double DEGREE_STEP = 0.1;

  /**
   * What a scene's names may be: its own id, its layers', frames', values', and shapes'. Short, and
   * safe in an address ({@code /api/scenes/world/inspect/ray.front-left.7}) and as a key.
   */
  static final Pattern NAME = Pattern.compile("[A-Za-z0-9][A-Za-z0-9._-]{0,63}");

  private Wire() {}

  /** Checks a name, saying what it names when it isn't one. */
  static String name(String what, String name) {
    if (!NAME.matcher(name).matches()) {
      throw new IllegalArgumentException(
          what
              + " \""
              + name
              + "\" isn't a name: up to 64 letters, digits, dots, hyphens, and underscores,"
              + " starting with a letter or digit");
    }
    return name;
  }

  /** A length in meters, to the millimeter. */
  static Number meters(double meters) {
    return stepped(finite(meters), 1000);
  }

  /** An angle given in radians, in degrees to the tenth, between -180 and 180. */
  static Number degrees(double radians) {
    return stepped(wrapDegrees(Math.toDegrees(finite(radians))), 10);
  }

  /** A number to a resolution: a multiple of it, as few digits as that needs. */
  static Number toResolution(double value, double resolution) {
    double steps = Math.rint(finite(value) / resolution);
    double rounded = steps * resolution;
    // The resolution's own decimals, so 0.1's multiples print as 0.3, not 0.30000000000000004.
    int decimals = Math.max(0, BigDecimal.valueOf(resolution).stripTrailingZeros().scale());
    return stepped(rounded, Math.pow(10, Math.min(decimals, 9)));
  }

  /** A point, {@code [x, y, z]} in meters. */
  static List<Number> point(double x, double y, double z) {
    return List.of(meters(x), meters(y), meters(z));
  }

  /**
   * A pose: {@code [x, y, yaw]} when it's level on the floor (no height, roll, or pitch), else
   * {@code [x, y, z, roll, pitch, yaw]}; meters and degrees.
   */
  static List<Number> pose(double x, double y, double z, double roll, double pitch, double yaw) {
    Number height = meters(z);
    Number rolled = degrees(roll);
    Number pitched = degrees(pitch);
    if (isZero(height) && isZero(rolled) && isZero(pitched)) {
      return List.of(meters(x), meters(y), degrees(yaw));
    }
    return List.of(meters(x), meters(y), height, rolled, pitched, degrees(yaw));
  }

  /** An angle in degrees, wrapped into (-180, 180]. */
  static double wrapDegrees(double degrees) {
    double wrapped = degrees % 360;
    if (wrapped > 180) {
      wrapped -= 360;
    } else if (wrapped <= -180) {
      wrapped += 360;
    }
    return wrapped;
  }

  /** How far apart two angles are, in radians, the short way round. */
  static double angleBetween(double a, double b) {
    return Math.abs(Math.IEEEremainder(a - b, 2 * Math.PI));
  }

  private static boolean isZero(Number number) {
    return number.doubleValue() == 0;
  }

  /** A number rounded to 1/scale, whole numbers as whole, and no negative zero. */
  private static Number stepped(double value, double scale) {
    double steps = Math.rint(value * scale);
    if (steps == 0) {
      return 0L;
    }
    if (steps % scale == 0 && Math.abs(steps / scale) < Long.MAX_VALUE) {
      return (long) (steps / scale);
    }
    return steps / scale;
  }

  private static double finite(double value) {
    if (!Double.isFinite(value)) {
      throw new IllegalArgumentException("a scene's numbers are finite: " + value);
    }
    return value;
  }
}
