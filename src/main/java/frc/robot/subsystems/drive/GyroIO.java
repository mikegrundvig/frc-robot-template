package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

/** The gyro's door to hardware. The gyro owns the robot's heading. */
public interface GyroIO {
  @AutoLog
  class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yaw = Rotation2d.ZERO;
    public double yawVelocityRadPerSec = 0.0;

    /** Yaw readings taken since the last loop, with the modules' readings (see ModuleIO). */
    public long[] odometryTimestampsNanos = new long[] {};

    public double[] odometryYawsRad = new double[] {};
  }

  default void updateInputs(GyroIOInputs inputs) {}
}
