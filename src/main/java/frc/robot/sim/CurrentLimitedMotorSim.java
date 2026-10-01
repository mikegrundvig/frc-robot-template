package frc.robot.sim;

import org.wpilib.math.system.DCMotor;

/**
 * A motor turning a load through its gearing, its motor controller holding the current through it
 * to a limit: a swerve module's steering.
 *
 * <p>Its motion is worked out exactly rather than stepped, because a steering motor is quick: with
 * the simulated inertia, its speed closes on the speed its voltage gives with a time constant under
 * a millisecond, well inside one of the simulator's steps. WPILib's {@code DCMotorSim} holds a
 * voltage through each step, so a current limit set at the step's start would hold only at that
 * instant, and the motor would give far less torque than its limit allows. A motor controller holds
 * the limit the whole time. So here, at the limit, the motor gives a steady torque until its back
 * EMF leaves no more than the limit's worth of voltage across its windings; below the limit, the
 * load's speed closes on the speed the voltage gives, exponentially, exactly as in {@code
 * DCMotorSim}. The drive's motor is about sixty times slower, so the drive keeps {@code
 * DCMotorSim}, with its limit set once a step.
 *
 * <p>Friction is left out: a real module's wheel scrubs on the carpet as it turns, and its gearbox
 * drags, so a real one coasts less than this one. Both need measuring on a real module before they
 * belong here. Once measured, add them in {@link #run} as a steady torque against the motion: it
 * takes {@code torque / (gearing * Kt)} amps of the motor's current, both at the limit and below
 * it, and the module stays put while the motor's torque can't beat it.
 */
final class CurrentLimitedMotorSim {
  private final DCMotor motor;
  private final double gearing;

  /** The load's acceleration per amp through the motor, in radians per second squared. */
  private final double accelerationPerAmp;

  /**
   * How quickly the load's speed closes on the speed its voltage gives: one over a time constant.
   */
  private final double settlingRate;

  private double positionRadians;
  private double velocityRadPerSec;
  private double appliedVolts;

  /**
   * @param motor the motor
   * @param inertiaKgM2 the load's moment of inertia
   * @param gearing motor rotations per rotation of the load
   */
  CurrentLimitedMotorSim(DCMotor motor, double inertiaKgM2, double gearing) {
    this.motor = motor;
    this.gearing = gearing;
    accelerationPerAmp = gearing * motor.Kt / inertiaKgM2;
    settlingRate = gearing * gearing * motor.Kt / (motor.Kv * motor.R * inertiaKgM2);
  }

  /**
   * Runs the motor for a time, its controller asking for a voltage and holding the current to a
   * limit.
   *
   * @param volts the voltage the controller asks for
   * @param limitAmps the most current it lets through the motor, either way
   * @param seconds how long
   * @return the most current through the motor in that time, in amps: the current at the start,
   *     since it only falls as the motor's speed catches up with its voltage
   */
  double run(double volts, double limitAmps, double seconds) {
    double amps = motor.getCurrent(velocityRadPerSec * gearing, volts);
    double remaining = seconds;
    if (Math.abs(amps) > limitAmps) {
      // At the limit: a steady torque, until the speed leaves no more than the limit's current.
      double limit = Math.copySign(limitAmps, amps);
      double acceleration = limit * accelerationPerAmp;
      double releaseSpeed = (volts - limit * motor.R) * motor.Kv / gearing;
      double atLimit =
          Math.clamp((releaseSpeed - velocityRadPerSec) / acceleration, 0.0, remaining);
      positionRadians += (velocityRadPerSec + acceleration * atLimit / 2) * atLimit;
      velocityRadPerSec += acceleration * atLimit;
      remaining -= atLimit;
      if (remaining <= 0) {
        appliedVolts = velocityRadPerSec * gearing / motor.Kv + limit * motor.R;
        return limitAmps;
      }
    }
    // Below the limit: the speed closes on the voltage's free speed, exponentially.
    double freeSpeed = volts * motor.Kv / gearing;
    double decay = Math.exp(-settlingRate * remaining);
    positionRadians +=
        freeSpeed * remaining + (velocityRadPerSec - freeSpeed) * (1 - decay) / settlingRate;
    velocityRadPerSec = freeSpeed + (velocityRadPerSec - freeSpeed) * decay;
    appliedVolts = volts;
    return Math.min(Math.abs(amps), limitAmps);
  }

  /** The load's angle, in radians from where it started. */
  double positionRadians() {
    return positionRadians;
  }

  /** The load's speed, in radians per second. */
  double velocityRadPerSec() {
    return velocityRadPerSec;
  }

  /** The voltage across the motor now: less than the controller asks for, at the limit. */
  double appliedVolts() {
    return appliedVolts;
  }

  /** The current through the motor now, in amps. */
  double currentAmps() {
    return motor.getCurrent(velocityRadPerSec * gearing, appliedVolts);
  }
}
