package com.michaelgrundvig.frc.pitwall.experiment;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.LiveConfig;

/**
 * A mechanism whose closed loop is a first-order lag: it turns toward its setpoint at kP times the
 * error, so its time constant is 1/kP. Its angle is read at 250 Hz, five readings a loop, the way a
 * swerve module's is. Its answer to a step is known exactly: a rise of ln 9 / kP, settling in ln 50
 * / kP, and no overshoot.
 */
final class Lag {
  /** The robot loop: 20 ms. */
  static final long LOOP_NANOS = 20_000_000;

  /** Readings per loop: 250 Hz. */
  static final int READINGS = 5;

  // Public, so Pitwall (in another package) can read and rebuild them, as it does a robot's.
  public record Gains(double kP, double kD) {
    public Gains {
      if (!(kP >= 0)) {
        throw new IllegalArgumentException("kP must be zero or more, got " + kP);
      }
    }
  }

  public record Arm(Gains gains) {}

  /** A step to 90° and back, 0.8 s at each, after the usual half-second lead-in. */
  static final Profile STEP_AND_BACK =
      Profiles.stepAndBack().from(Degrees.of(0)).to(Degrees.of(90)).hold(Seconds.of(0.8));

  final LiveConfig<Arm> config =
      LiveConfig.of("arm", new Arm(new Gains(10, 0)))
          .number("gains.kP", Limits.between(0, 50))
          .number("gains.kD", Limits.between(0, 1));

  /** The setpoint it was last sent, and its angle, in radians. */
  double setpoint;

  double angle;

  private long now;

  /** This loop's readings, as a swerve module's inputs hold them: replaced every loop. */
  private long[] times = new long[0];

  private double[] angles = new double[0];

  /** A rig on it: setpoints in radians, its angle measured in degrees, and its "voltage". */
  Rig rig() {
    return Rig.builder()
        .name("Arm")
        .gains(config, "gains.kP", "gains.kD")
        .setpoint(Radians, radians -> setpoint = radians)
        .limits(Degrees.of(-100), Degrees.of(100))
        .measurement("Angle", Degrees, this::readings)
        .signal("Voltage", Volts, () -> config.get().gains().kP() * (setpoint - angle))
        .build();
  }

  /**
   * Moves on to the next loop: five readings, each exactly where a lag with this setpoint would be.
   *
   * @return the new loop's time
   */
  long nextLoop() {
    double kP = config.get().gains().kP();
    long[] readTimes = new long[READINGS];
    double[] readAngles = new double[READINGS];
    for (int i = 0; i < READINGS; i++) {
      long step = LOOP_NANOS / READINGS;
      angle = setpoint + (angle - setpoint) * Math.exp(-kP * step / 1e9);
      now += step;
      readTimes[i] = now;
      readAngles[i] = Math.toDegrees(angle);
    }
    times = readTimes;
    angles = readAngles;
    return now;
  }

  /** This loop's readings, as a swerve module hands over its odometry. */
  private void readings(long nowNanos, Series series) {
    for (int i = 0; i < times.length; i++) {
      series.add(times[i], angles[i]);
    }
  }
}
