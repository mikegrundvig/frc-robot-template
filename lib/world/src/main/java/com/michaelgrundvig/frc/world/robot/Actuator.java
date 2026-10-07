package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.NewtonMeters;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import org.wpilib.math.system.DCMotor;

/**
 * What makes a mechanism's force: motors, or a servo. Only the force: where it goes is its {@link
 * Transmission}, and a {@link Drive} pairs the two. Pneumatic cylinders, springs and a team's own
 * force law come back when a robot needs one (archived 2026-10-05).
 *
 * <p>Each is a law of force along its transmission's coordinate ({@link #forceOrTorque}): on a line
 * between two points, newtons along it as its length changes; on a geared shaft, newton meters as
 * the shaft turns. The simulator runs the motors'; it doesn't simulate servos yet.
 */
public sealed interface Actuator {
  /**
   * The force, or the torque, it applies along its transmission's coordinate.
   *
   * @param command what it's told: volts for motors; for a servo, how far along its travel, 0 to 1
   * @param position its coordinate: the length between its ends, in meters, or its shaft's angle,
   *     in radians
   * @param velocity how fast that's changing
   * @return newtons along the line, positive pushing its ends apart; or newton meters on the shaft,
   *     positive turning it forward
   */
  double forceOrTorque(double command, double position, double velocity);

  /**
   * One or more identical motors turning one shaft together, such as two motors on an arm's
   * gearbox. What they turn is the drive's transmission.
   *
   * @param motor the motor, the same for each
   * @param count how many turn it together
   */
  @ConfigRecord
  record Motors(Motor motor, int count) implements Actuator {
    public Motors {
      Checks.on(Motors.class).field("count", count, positive()).orThrow();
    }

    /** Starts describing the motors, one named value at a time. */
    public static ActuatorMotorsBuilder.MotorStage builder() {
      return ActuatorMotorsBuilder.builder();
    }

    /** The motors together, as WPILib models them: their torque, current and speed at the shaft. */
    public DCMotor model() {
      return model(1);
    }

    /**
     * The motors together through a transmission that loses some of their torque: what reaches past
     * it, still at the motors' shaft. The losses come off the torque, not the speed or the current,
     * as WPILib's simulations take a gearing's.
     *
     * @param efficiency the share of the torque that gets through, above 0 and at most 1
     */
    public DCMotor model(double efficiency) {
      return new DCMotor(
          motor.nominalVoltage().in(Volts),
          motor.stallTorque().in(NewtonMeters) * efficiency,
          motor.stallCurrent().in(Amps),
          motor.freeCurrent().in(Amps),
          motor.freeSpeed().in(RadiansPerSecond),
          count);
    }

    /** At a voltage and shaft speed: the motors' torque together, by WPILib's model. */
    @Override
    public double forceOrTorque(double command, double position, double velocity) {
      DCMotor model = model();
      return model.stallTorque / model.nominalVoltage * (command - velocity / model.Kv);
    }
  }

  /**
   * A servo on PWM: a linear servo along a line between two parts, or a hobby servo's shaft. Its
   * own loop drives it where it's told along its travel, as hard as it can push and no faster than
   * it can move, and nothing reads back where it is: robot code tells it a share of its travel
   * ({@link #command}), the share of its pulse between the shortest and the longest.
   *
   * <p>Its loop is the servo's own, so only how it's told and how hard and fast it goes are here.
   * Its force law takes the loop as a stiff one ({@link #forceOrTorque}): its full push until
   * within a fiftieth of its travel of where it's told, less as it nears its top speed.
   *
   * @param from its transmission's coordinate at the shortest pulse, the command 0: its length eye
   *     to eye, in meters, fully retracted for a linear servo; its shaft's angle, in radians
   * @param to its coordinate at the longest pulse, the command 1: fully extended
   * @param force the most it pushes, in newtons, or turns, in newton meters: its datasheet's
   * @param speed how fast it moves with nothing on it, in meters or radians per second: its
   *     datasheet's
   */
  @ConfigRecord
  record Servo(double from, double to, double force, double speed) implements Actuator {
    /** The share of its travel within which its own loop eases off its full push. */
    private static final double BAND = 0.02;

    public Servo {
      Checks.on(Servo.class)
          .field("from", from, finite())
          .field("to", to, finite())
          .field("force", force, positive())
          .field("speed", speed, positive())
          .check("to", () -> to != from ? null : shown(to) + " must differ from where it starts")
          .orThrow();
    }

    /** Starts describing a servo, one named value at a time. */
    public static ActuatorServoBuilder.FromStage builder() {
      return ActuatorServoBuilder.builder();
    }

    /**
     * What to tell it to put it at a coordinate: the share of its travel, 0 at {@code from} and 1
     * at {@code to}: its pulse that share of the way from its shortest to its longest, which WPILib
     * 2027 sends as a {@code PWM}'s pulse in microseconds. Outside 0 to 1 it can't reach.
     */
    public double command(double coordinate) {
      return (coordinate - from) / (to - from);
    }

    /**
     * Its push toward where it's told, at a position and speed along its coordinate: all of its
     * force until within a fiftieth of its travel, and less as it nears its top speed that way.
     *
     * @param command the share of its travel it's told, held to 0 to 1
     */
    @Override
    public double forceOrTorque(double command, double position, double velocity) {
      double target = from + Math.max(0, Math.min(1, command)) * (to - from);
      double band = BAND * Math.abs(to - from);
      double push = Math.max(-1, Math.min(1, (target - position) / band));
      return force * Math.max(-1, Math.min(1, push - velocity / speed));
    }
  }
}
