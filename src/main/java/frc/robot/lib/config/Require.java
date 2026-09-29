package frc.robot.lib.config;

import org.wpilib.units.Measure;

/** Value checks shared by the configuration records. Each names the value it rejects. */
final class Require {
  private Require() {}

  static void positive(String name, Measure<?> value) {
    positive(name, value.baseUnitMagnitude(), value.toShortString());
  }

  static void positive(String name, double value) {
    positive(name, value, Double.toString(value));
  }

  static void nonNegative(String name, Measure<?> value) {
    nonNegative(name, value.baseUnitMagnitude(), value.toShortString());
  }

  static void nonNegative(String name, double value) {
    nonNegative(name, value, Double.toString(value));
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
