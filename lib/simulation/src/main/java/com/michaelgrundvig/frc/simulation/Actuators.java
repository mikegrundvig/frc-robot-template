package com.michaelgrundvig.frc.simulation;

/**
 * What drives the physics' wheels, worked out in closed form beside it: the motor controllers'
 * loops and limits, the motors, and the battery. The {@link Physics} asks at the start of each of
 * its sub-steps, telling the truth as it stands (a wheel's speed, for a velocity loop and a motor's
 * back EMF), and the actuators say what drives each wheel over the sub-step ({@link WheelDrive}).
 *
 * <p>Two couplings, chosen so an engine whose wheels are stiff, light bodies can still take
 * sub-steps of a millisecond or more:
 *
 * <ul>
 *   <li><b>A wheel's drive is a motor's line, not a torque.</b> Its controller gives a voltage
 *       across the motor, its current limit, and how the voltage moves with the wheel's speed (its
 *       velocity loop's gain): together, the motor's torque against the wheel's speed, a straight
 *       line capped at the limit's torque. An engine that integrates implicitly applies it as a
 *       joint motor, a damped velocity target, through the whole sub-step, where a torque held from
 *       its start would ring at the wheel's sub-millisecond time constant while it slips. Today's
 *       engine holds the voltage through the sub-step.
 *   <li><b>A wheel's steering stays closed form.</b> The steering settles in under a millisecond,
 *       so the actuators turn it themselves ({@code CurrentLimitedMotorSim}) and give the angle;
 *       the physics follows it, and returns what the floor's scrub pushes back with ({@link
 *       WheelTruth#steerLoadNewtonMeters}) for the next sub-step.
 * </ul>
 */
@FunctionalInterface
public interface Actuators {
  /**
   * Runs the motor controllers and the steering over the next sub-step, from the truth as it
   * starts, and says what drives each wheel over it.
   *
   * @param seconds the sub-step's length
   * @param physics the world as the sub-step starts
   * @param drives one for each wheel, in the physics' order, to fill in
   */
  void actuate(double seconds, Physics physics, WheelDrive[] drives);
}
