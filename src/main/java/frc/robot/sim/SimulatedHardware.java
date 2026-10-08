package frc.robot.sim;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.hardware.Hardware;
import frc.robot.subsystems.drive.DriveHardware;
import frc.robot.subsystems.mechanisms.AbsoluteEncoderIO;
import frc.robot.subsystems.mechanisms.JointIO;
import frc.robot.subsystems.mechanisms.PdhIO;
import frc.robot.subsystems.mechanisms.SensorIO;
import frc.robot.subsystems.vision.CameraIO;
import java.util.List;

/**
 * The simulator's hardware: the simulated world's drivetrain, joints, sensors, PDH and cameras,
 * each part the robot's definition has, wired or not, built with the world from the definition. The
 * world stays its maker's to step and close.
 */
public final class SimulatedHardware implements Hardware {
  private final SimulatedWorld world;

  /**
   * @param world the world the robot drives in, simulated from the robot's definition
   */
  public SimulatedHardware(SimulatedWorld world) {
    this.world = world;
  }

  @Override
  public DriveHardware drive() {
    return world.drivetrainHardware();
  }

  @Override
  public JointIO joint(Joint joint) {
    return world.joints().io(joint);
  }

  @Override
  public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
    return world.joints().absoluteEncoder(joint);
  }

  @Override
  public SensorIO sensor(Sensor.Placed sensor) {
    return world.sensors().io(sensor);
  }

  @Override
  public PdhIO pdh() {
    return world.pdh();
  }

  @Override
  public List<CameraIO> cameras() {
    return List.of(world.cameras());
  }
}
