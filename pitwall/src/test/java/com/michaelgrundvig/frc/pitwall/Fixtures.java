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

  record Limits(@Tunable Current current, @Tunable LinearVelocity maxSpeed) {
    Limits {
      if (!(current.baseUnitMagnitude() > 0)) {
        throw new IllegalArgumentException("current must be positive");
      }
    }
  }

  record Gains(@Tunable double kP, @Tunable Time lookahead) {
    Gains {
      if (!(kP >= 0)) {
        throw new IllegalArgumentException("kP must be zero or more, got " + kP);
      }
    }
  }

  record Drive(Motor motor, Limits limits, Gains gains) {
    Drive {
      if (limits.maxSpeed().in(MetersPerSecond) > motor.freeSpeedMetersPerSecond()) {
        throw new IllegalArgumentException("maxSpeed is faster than the motor's free speed");
      }
    }
  }

  record Range(@Tunable double min, @Tunable double max) {
    Range {
      if (!(min < max)) {
        throw new IllegalArgumentException("min must be below max, got " + min + " and " + max);
      }
    }
  }

  record Machine(String name, Drive drive, Range travel) {}

  /**
   * A 40 A, 4 m/s drive (its motor is good for 4.5 m/s) with kP 16 and a 30 ms lookahead, and
   * travel from 0 to 10.
   */
  static Machine machine() {
    return new Machine(
        "test",
        new Drive(
            new Motor(4.5),
            new Limits(Amps.of(40), MetersPerSecond.of(4)),
            new Gains(16, Seconds.of(0.03))),
        new Range(0, 10));
  }
}
