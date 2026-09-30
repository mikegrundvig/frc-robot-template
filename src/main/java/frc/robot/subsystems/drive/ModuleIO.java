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

    /**
     * Whether the drive motor hit its stator current limit since the last loop: the torque it can
     * give is capped, and a wheel at that edge of its grip slips.
     */
    public boolean driveTorqueLimited = false;

    public boolean steerConnected = false;

    /** From the absolute encoder: valid at power-on, no homing needed. */
    public Rotation2d steerAngle = Rotation2d.ZERO;

    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerStatorCurrentAmps = 0.0;

    /**
     * Readings taken since the last loop, oldest first, faster than the loop runs (250 Hz on a
     * robot) so odometry sees every turn of the wheel: when each was taken, on the robot's clock,
     * and the wheel's position and angle then. Angles are plain radians, not Rotation2d: a replay
     * must hand the estimator the very same numbers, and a Rotation2d is rebuilt from its cosine
     * and sine, a bit off.
     */
    public long[] odometryTimestampsNanos = new long[] {};

    public double[] odometryDrivePositionsRad = new double[] {};
    public double[] odometrySteerAnglesRad = new double[] {};

    /**
     * The module's devices that didn't take their configuration at startup, such as "drive motor 3,
     * CANcoder 9", or empty: they run on their defaults, without the configured directions,
     * gearing, or current limits.
     */
    public String unconfigured = "";
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
