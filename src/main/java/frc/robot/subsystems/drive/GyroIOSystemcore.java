package frc.robot.subsystems.drive;

import frc.robot.lib.localization.Timestamps;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.RobotController;

/**
 * Systemcore's onboard IMU as a gyro: the backup to the main gyro, and its cross-check. Read once a
 * loop (its fused yaw updates at 400 Hz, by the alpha spec); the drivetrain interpolates between
 * readings when it has to turn odometry with it.
 *
 * <p>WPILib reports no connection status for it, so it counts as connected; a failure shows as the
 * drivetrain's cross-check disagreeing with the gyro. Its pitch and roll are WPILib's angles about
 * the board's axes for the chosen mount, and its accelerometer reads the board's axes, so it
 * reports acceleration only when mounted flat. Unconfirmed on a real unit until checked against
 * another IMU: its axes, signs, scale, and drift.
 */
public final class GyroIOSystemcore implements GyroIO {
  private final OnboardIMU imu;
  private final boolean flat;
  private boolean read;
  private double lastYaw;
  private long lastTime;

  /**
   * @param mount how Systemcore is mounted on the robot
   */
  public GyroIOSystemcore(OnboardIMU.MountOrientation mount) {
    imu = new OnboardIMU(mount);
    flat = mount == OnboardIMU.MountOrientation.FLAT;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    long now = RobotController.getTime();
    double yaw = imu.getYawRadians();
    inputs.connected = true;
    inputs.yaw = new Rotation2d(yaw);
    // From successive readings, which works however the board is mounted.
    inputs.yawVelocityRadPerSec =
        read && now > lastTime
            ? MathUtil.angleModulus(yaw - lastYaw) / Timestamps.toSeconds(now - lastTime)
            : 0;
    read = true;
    lastYaw = yaw;
    lastTime = now;
    inputs.pitchRad = imu.getAngleY();
    inputs.rollRad = imu.getAngleX();
    inputs.accelerationXMetersPerSecSq = flat ? imu.getAccelX() : 0;
    inputs.accelerationYMetersPerSecSq = flat ? imu.getAccelY() : 0;
    inputs.odometryTimestampsNanos = new long[] {now};
    inputs.odometryYawsRad = new double[] {yaw};
  }
}
