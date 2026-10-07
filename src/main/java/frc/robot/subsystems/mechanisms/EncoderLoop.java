package frc.robot.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.ControllerPlan;

/**
 * The loop a joint's mechanism closes itself, on an absolute encoder the robot reads on a
 * Systemcore port ({@code MechanismWiring.PortEncoder#closesLoop}): each loop, the volts that hold
 * the joint at its profile's point, worked out from where the encoder says the joint is, and sent
 * to the motor controller as a voltage. It's the law a SPARK runs with the feedforward the robot
 * works out for it ({@code RevMechanism}), the plan's ({@link ControllerPlan#feedforward}), its kP
 * worked out here rather than on the controller, and its speed the motor's own encoder's. Here it
 * runs at the robot's 50 Hz, not a motor controller's 1 kHz, so it's coarser: a kP that a motor
 * controller holds steady may be too stiff here.
 *
 * @param plan how the program drives the joint: its gains and feedforward, in its units
 */
record EncoderLoop(ControllerPlan plan) {
  /**
   * Volts to hold the joint at a point of its profile, everything in the joint's units.
   *
   * @param position where the profile is
   * @param velocity how fast it moves there
   * @param feedforwardVolts what the program adds of its own, the profile's acceleration times kA
   * @param measured where the encoder says the joint is
   * @param measuredVelocity how fast the motor's encoder says it moves
   */
  double volts(
      double position,
      double velocity,
      double feedforwardVolts,
      double measured,
      double measuredVelocity) {
    return plan.kP() * (position - measured)
        + plan.feedforward(position, velocity, 0)
        + plan.kD() * (velocity - measuredVelocity)
        + feedforwardVolts;
  }
}
