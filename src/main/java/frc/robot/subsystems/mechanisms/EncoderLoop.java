package frc.robot.subsystems.mechanisms;

import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.lib.phoenix.PhoenixUnits;

/**
 * The loop a joint's mechanism closes itself, on an absolute encoder the robot reads on a
 * Systemcore port ({@code MechanismWiring.PortEncoder#closesLoop}): each loop, the volts that hold
 * the joint at its profile's point, worked out from where the encoder says the joint is, and sent
 * to the motor controller as a voltage. It's the law a SPARK runs with the feedforward the robot
 * works out for it ({@code RevMechanism}), its kP worked out here rather than on the controller,
 * and its speed the motor's own encoder's. Here it runs at the robot's 50 Hz, not a motor
 * controller's 1 kHz, so it's coarser: a kP that a motor controller holds steady may be too stiff
 * here.
 *
 * @param units the joint's units as its motor controller's, which the gains are in
 * @param kP volts per rotation of error
 * @param feedforward the slot's feedforward and kD, as the robot works them out
 */
record EncoderLoop(PhoenixUnits units, double kP, PhoenixMechanism.Feedforward feedforward) {
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
    double at = units.toPhoenix(position);
    return kP * (at - units.toPhoenix(measured))
        + feedforward.atPosition(at, units.toPhoenix(velocity), units.toPhoenix(measuredVelocity))
        + feedforwardVolts;
  }
}
