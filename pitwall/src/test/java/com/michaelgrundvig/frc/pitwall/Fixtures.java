package com.michaelgrundvig.frc.pitwall;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;

/**
 * A small configuration to tune: a machine with a drive, whose top speed can't exceed its motor's
 * (a check across records), and a range of travel, whose ends are valid only together.
 */
final class Fixtures {
  private Fixtures() {}

  /** A fact about the hardware: not tunable. */
  record Motor(double freeSpeedMetersPerSecond) {}

  /**
   * The drive's limits.
   *
   * @param current the most current through the motor
   */
  record DriveLimits(Current current, LinearVelocity maxSpeed) {
    DriveLimits {
      if (!(current.baseUnitMagnitude() > 0)) {
        throw new IllegalArgumentException("current must be positive");
      }
    }
  }

  /**
   * How the drive is controlled. Tuned on a running machine, then committed.
   *
   * <p>Start {@code kP} low.
   *
   * @param kP volts per meter per second of error
   * @param lookahead how far ahead the drive aims
   */
  record Gains(double kP, Time lookahead) {
    Gains {
      if (!(kP >= 0)) {
        throw new IllegalArgumentException("kP must be zero or more, got " + kP);
      }
    }
  }

  /**
   * A drive.
   *
   * @param gains how it's controlled
   */
  record Drive(Motor motor, DriveLimits limits, Gains gains) {
    Drive {
      if (limits.maxSpeed().in(MetersPerSecond) > motor.freeSpeedMetersPerSecond()) {
        throw new IllegalArgumentException("maxSpeed is faster than the motor's free speed");
      }
    }
  }

  record Range(double min, double max) {
    Range {
      if (!(min < max)) {
        throw new IllegalArgumentException("min must be below max, got " + min + " and " + max);
      }
    }
  }

  record Machine(String name, Drive drive, Range travel) {}

  /**
   * What a configuration writes that JSON must escape, or that isn't ASCII.
   *
   * @param heading the "zero" heading, in ° from C:\field's corner
   * @param resistance in Ω, or &Omega;
   */
  record Symbols(double heading, double resistance) {}

  /** A stage of a mechanism, of whichever kind: a component declared by its interface. */
  interface Stage {}

  /**
   * A stage that lifts.
   *
   * @param kP volts per meter of error
   */
  record Lift(double kP) implements Stage {}

  /**
   * A carriage, on whichever stage.
   *
   * @param stage the stage it rides
   */
  record Carriage(Stage stage) {}

  /**
   * A 40 A, 4 m/s drive (its motor is good for 4.5 m/s) with kP 16 and a 30 ms lookahead, and
   * travel from 0 to 10.
   */
  static Machine machine() {
    return new Machine(
        "test",
        new Drive(
            new Motor(4.5),
            new DriveLimits(Amps.of(40), MetersPerSecond.of(4)),
            new Gains(16, Seconds.of(0.03))),
        new Range(0, 10));
  }
}
