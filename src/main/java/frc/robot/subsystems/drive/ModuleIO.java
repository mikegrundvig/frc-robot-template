package frc.robot.subsystems.drive;

import frc.robot.lib.config.SwerveConfig;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

/**
 * One swerve module's door to hardware: a simulated module, a real one, or nothing (in replay,
 * where the inputs come from the log). Drive values are at the wheel, in radians, so the wheel
 * radius (which changes with wear) lives in one place: {@link Module}.
 */
public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveStatorCurrentAmps = 0.0;
    public double driveSupplyCurrentAmps = 0.0;

    public boolean steerConnected = false;

    /** From the absolute encoder: valid at power-on, no homing needed. */
    public Rotation2d steerAngle = Rotation2d.ZERO;

    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerStatorCurrentAmps = 0.0;
  }

  default void updateInputs(ModuleIOInputs inputs) {}

  /**
   * Sets the motor controllers' gains and current limits: at startup, and again whenever tuning
   * changes them.
   */
  default void configure(SwerveConfig.Gains gains, SwerveConfig.Limits limits) {}

  /** Closed-loop wheel velocity on the motor controller, plus robot-computed feedforward. */
  default void setDriveVelocity(double wheelRadPerSec, double feedforwardVolts) {}

  /** Open-loop drive voltage (characterization). */
  default void setDriveVoltage(double volts) {}

  /** Closed-loop steer angle on the motor controller. */
  default void setSteerAngle(Rotation2d angle) {}

  /** Stop both motors. */
  default void stop() {}
}
