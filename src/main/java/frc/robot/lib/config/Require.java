package frc.robot.lib.config;

import java.math.BigDecimal;
import java.math.MathContext;
import org.wpilib.units.Measure;

/** Value checks shared by the configuration records. Each names the value it rejects. */
final class Require {
  private Require() {}

  static void positive(String name, Measure<?> value) {
    positive(name, value.baseUnitMagnitude(), shown(value));
  }

  static void positive(String name, double value) {
    positive(name, value, Double.toString(value));
  }

  static void nonNegative(String name, Measure<?> value) {
    nonNegative(name, value.baseUnitMagnitude(), shown(value));
  }

  static void nonNegative(String name, double value) {
    nonNegative(name, value, Double.toString(value));
  }

  static void finite(String name, Measure<?> value) {
    if (!Double.isFinite(value.baseUnitMagnitude())) {
      throw new IllegalArgumentException(name + " must be a number, got " + shown(value));
    }
  }

  /** A measure as people write it, "4.729 m/s" rather than "4.729e+00 m/s". */
  static String shown(Measure<?> value) {
    return shown(value.magnitude()) + " " + value.unit().symbol();
  }

  /** A number to four significant digits, without an exponent. */
  static String shown(double value) {
    if (!Double.isFinite(value)) {
      return Double.toString(value);
    }
    return new BigDecimal(value).round(new MathContext(4)).stripTrailingZeros().toPlainString();
  }

  static void notBlank(String name, String value) {
    if (value.isBlank()) {
      throw new IllegalArgumentException(name + " is required");
    }
  }

  private static void positive(String name, double magnitude, String shown) {
    if (!Double.isFinite(magnitude) || magnitude <= 0) {
      throw new IllegalArgumentException(name + " must be positive, got " + shown);
    }
  }

  private static void nonNegative(String name, double magnitude, String shown) {
    if (!Double.isFinite(magnitude) || magnitude < 0) {
      throw new IllegalArgumentException(name + " must be zero or more, got " + shown);
    }
  }
}
