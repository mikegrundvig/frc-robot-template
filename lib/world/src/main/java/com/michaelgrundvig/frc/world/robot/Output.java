package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

/**
 * What a joint is run at, by name: a roller's "intaking" and "outtaking", a feeder's "feeding".
 * It's the joint's own ({@code Joint.outputs}), of one of three kinds, as teams run their rollers:
 *
 * <ul>
 *   <li>a velocity, which a velocity loop holds, in the joint's units per second: radians per
 *       second for a revolute joint, meters per second for a prismatic one;
 *   <li>a voltage across its motors;
 *   <li>a duty cycle, a share of the battery's voltage, from -1 to 1.
 * </ul>
 *
 * <p>Negative runs it backward. Every joint has one more, built in, which it doesn't name: {@link
 * #STOPPED}, its motors at neutral, braking or coasting as its control says. The definition holds
 * each to what the joint's motors can do: a velocity within their free speed through the gearing
 * ({@code RobotMath.freeSpeed}), a voltage within their nominal voltage.
 *
 * @param name what code, logs and Pitwall call it: unique among its joint's positions and outputs
 * @param kind a velocity, a voltage or a duty cycle
 * @param value how much: in the joint's units per second, in volts, or as a share from -1 to 1
 */
@ConfigRecord
public record Output(String name, Kind kind, double value) {
  /** What an output holds its joint's motors to. */
  public enum Kind {
    /** A speed, in the joint's units per second, which a velocity loop holds. */
    VELOCITY,
    /** A voltage across the motors. */
    VOLTAGE,
    /** A share of the battery's voltage, from -1 to 1. */
    DUTY_CYCLE
  }

  /** The built-in output's name: every joint has it, and none names its own. */
  public static final String STOPPED_NAME = "stopped";

  /** Every joint's built-in output: no duty cycle, its motors at neutral. */
  public static final Output STOPPED =
      builder().name(STOPPED_NAME).kind(Kind.DUTY_CYCLE).value(0).build();

  public Output {
    double share = value;
    Checks.on(Output.class, name)
        .field("name", name, notBlank())
        .field("value", value, finite())
        .check(
            "value",
            () ->
                kind == Kind.DUTY_CYCLE && !(Math.abs(share) <= 1)
                    ? shown(share) + " must be a share of the battery's voltage, from -1 to 1"
                    : null)
        .orThrow();
  }

  /** Starts describing an output, one named value at a time. */
  public static OutputBuilder.NameStage builder() {
    return OutputBuilder.builder();
  }

  /** A revolute joint's speed, in its turns a minute, as teams' code gives them. */
  public static Output rpm(String name, double rpm) {
    return builder().name(name).kind(Kind.VELOCITY).value(rpm * 2 * Math.PI / 60).build();
  }

  /** A revolute joint's speed. */
  public static Output of(String name, AngularVelocity speed) {
    return builder().name(name).kind(Kind.VELOCITY).value(speed.in(RadiansPerSecond)).build();
  }

  /** A prismatic joint's speed. */
  public static Output of(String name, LinearVelocity speed) {
    return builder().name(name).kind(Kind.VELOCITY).value(speed.in(MetersPerSecond)).build();
  }

  /** A voltage across the joint's motors. */
  public static Output of(String name, Voltage voltage) {
    return builder().name(name).kind(Kind.VOLTAGE).value(voltage.in(Volts)).build();
  }

  /** A voltage across the joint's motors, in volts, as teams' code gives them. */
  public static Output volts(String name, double volts) {
    return builder().name(name).kind(Kind.VOLTAGE).value(volts).build();
  }

  /** A share of the battery's voltage, from -1 to 1: a team's {@code DutyCycleOut}. */
  public static Output dutyCycle(String name, double share) {
    return builder().name(name).kind(Kind.DUTY_CYCLE).value(share).build();
  }

  /** Whether it runs the joint at all: anything but zero. */
  public boolean runs() {
    return value != 0;
  }
}
