package frc.robot.sim;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.subsystems.mechanisms.AbsoluteEncoderIO;
import org.jspecify.annotations.Nullable;

/**
 * A joint's absolute encoder in the simulator: it reads where the joint really is, as its port
 * would, its offset and direction undone, so the joint's mechanism seeds from it as from a real
 * one. Unplugged on a test's word ({@link JointSimulation#unplug}).
 */
final class AbsoluteEncoderIOSim implements AbsoluteEncoderIO {
  private final JointIOSim joint;

  /** Its signal's turns for each of the joint's units: radians, or meters. */
  private final double perUnit;

  private final MechanismWiring.@Nullable PortEncoder port;

  boolean connected = true;

  AbsoluteEncoderIOSim(
      JointIOSim joint, Sensor.Encoder model, MechanismWiring.@Nullable PortEncoder port) {
    this.joint = joint;
    double perTurn = joint.joint() instanceof Joint.Revolute ? 2 * Math.PI : 1;
    perUnit = Math.abs(model.ratio(joint.joint())) / perTurn;
    this.port = port;
  }

  @Override
  public void updateInputs(AbsoluteEncoderIOInputs inputs) {
    inputs.connected = connected;
    double reading = joint.position() * perUnit;
    MechanismWiring.PortEncoder wired = port;
    inputs.share = wired != null ? wired.raw(reading) : reading - Math.floor(reading);
  }
}
