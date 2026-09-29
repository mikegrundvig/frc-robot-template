package frc.robot.subsystems.drive;

import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.swerve.SwerveSim;
import org.wpilib.math.geometry.Rotation2d;

/** A module in the physics simulation, seen through {@link ModuleIO}. */
public final class ModuleIOSim implements ModuleIO {
  private final SwerveSim.Module module;

  public ModuleIOSim(SwerveSim.Module module) {
    this.module = module;
  }

  /** The four modules of a simulated robot: front left, front right, back left, back right. */
  public static ModuleIO[] of(SwerveSim sim) {
    return new ModuleIO[] {
      new ModuleIOSim(sim.module(0)),
      new ModuleIOSim(sim.module(1)),
      new ModuleIOSim(sim.module(2)),
      new ModuleIOSim(sim.module(3))
    };
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = module.connected();
    inputs.drivePositionRad = module.drivePositionRad();
    inputs.driveVelocityRadPerSec = module.driveVelocityRadPerSec();
    inputs.driveAppliedVolts = module.driveAppliedVolts();
    inputs.driveStatorCurrentAmps = module.driveStatorAmps();
    inputs.driveSupplyCurrentAmps = module.driveSupplyAmps();
    inputs.steerConnected = module.connected();
    inputs.steerAngle = module.steerAngle();
    inputs.steerVelocityRadPerSec = module.steerVelocityRadPerSec();
    inputs.steerAppliedVolts = module.steerAppliedVolts();
    inputs.steerStatorCurrentAmps = module.steerStatorAmps();
  }

  @Override
  public void configure(SwerveConfig.Gains gains, SwerveConfig.Limits limits) {
    module.configure(gains, limits);
  }

  @Override
  public void setDriveVelocity(double wheelRadPerSec, double feedforwardVolts) {
    module.setDriveVelocity(wheelRadPerSec, feedforwardVolts);
  }

  @Override
  public void setDriveVoltage(double volts) {
    module.setDriveVoltage(volts);
  }

  @Override
  public void setSteerAngle(Rotation2d angle) {
    module.setSteerAngle(angle);
  }

  @Override
  public void stop() {
    module.stop();
  }
}
