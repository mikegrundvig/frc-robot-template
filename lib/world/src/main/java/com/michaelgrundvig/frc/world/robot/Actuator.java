package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.NewtonMeters;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import org.wpilib.math.system.DCMotor;

/**
 * What drives a mechanism. An open set of kinds rather than a list of parts: any DC motor, from its
 * datasheet, through any gearing.
 *
 * <p>Only motors so far, which is what the drivetrain's modules are driven by. Other kinds join as
 * the simulator comes to read them: a pneumatic cylinder, a spring that counterbalances an arm, a
 * team's own law for what nothing else describes. A mechanism with more than one kind lists one
 * actuator of each.
 */
public sealed interface Actuator {
  /**
   * One or more identical motors driving the same thing through one gearing, such as two motors on
   * an arm's shaft.
   *
   * @param motor the motor, the same for each
   * @param count how many drive it together
   * @param gearing between the motors and what they drive
   */
  @ConfigRecord
  record Motors(Motor motor, int count, Gearing gearing) implements Actuator {
    public Motors {
      Checks.on(Motors.class).field("count", count, positive()).orThrow();
    }

    /** Starts describing the motors, one named value at a time. */
    public static ActuatorMotorsBuilder.MotorStage builder() {
      return ActuatorMotorsBuilder.builder();
    }

    /**
     * The motors together, as WPILib's simulation models them: their torque, less what the gearing
     * loses, at the motors' shaft. The gearing's ratio isn't in it: whatever turns the load applies
     * the ratio itself, as WPILib's motor simulations take it apart from the motor.
     */
    public DCMotor model() {
      return new DCMotor(
          motor.nominalVoltage().in(Volts),
          motor.stallTorque().in(NewtonMeters) * gearing.efficiency(),
          motor.stallCurrent().in(Amps),
          motor.freeCurrent().in(Amps),
          motor.freeSpeed().in(RadiansPerSecond),
          count);
    }
  }
}
