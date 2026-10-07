package frc.robot.sim;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
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
 * each part the robot's definition has, wired or not. The world stays its maker's to step and
 * close.
 */
public final class SimulatedHardware implements Hardware {
  private final SimulatedWorld world;
  private final RobotDefinition definition;
  private final RobotMath math;

  /**
   * @param world the world the robot drives in
   * @param definition what the robot is: the world simulates its joints, sensors and PDH from it
   */
  public SimulatedHardware(SimulatedWorld world, RobotDefinition definition) {
    this.world = world;
    this.definition = definition;
    math = definition.math();
  }

  @Override
  public DriveHardware drive() {
    return world.drivetrainHardware();
  }

  @Override
  public JointIO joint(Joint joint) {
    return world.joint(math, joint);
  }

  @Override
  public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
    return world.absoluteEncoder(joint);
  }

  @Override
  public SensorIO sensor(Sensor.Placed sensor) {
    return world.sensor(sensor);
  }

  @Override
  public PdhIO pdh() {
    return world.pdh(definition);
  }

  @Override
  public List<CameraIO> cameras() {
    return List.of(world.cameras());
  }
}
