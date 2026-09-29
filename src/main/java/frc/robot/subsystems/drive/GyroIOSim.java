package frc.robot.subsystems.drive;

import frc.robot.lib.swerve.SwerveSim;

/** The physics simulation's gyro, seen through {@link GyroIO}: it reports the true heading. */
public final class GyroIOSim implements GyroIO {
  private final SwerveSim sim;

  public GyroIOSim(SwerveSim sim) {
    this.sim = sim;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yaw = sim.yaw();
    inputs.yawVelocityRadPerSec = sim.yawVelocityRadPerSec();
  }
}
