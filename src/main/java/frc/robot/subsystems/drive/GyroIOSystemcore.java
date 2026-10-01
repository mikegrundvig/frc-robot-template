package frc.robot.subsystems.drive;

import frc.robot.lib.localization.Timestamps;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.MathUtil;
import org.wpilib.system.RobotController;

/**
 * Systemcore's onboard IMU as a gyro: the backup to the main gyro, and its cross-check. Read once a
 * loop (its fused yaw updates at 400 Hz, by the alpha spec); the drivetrain interpolates between
 * readings when it has to turn odometry with it.
 *
 * <p>WPILib reports no connection status for it, so it counts as connected; a failure shows as the
 * drivetrain's cross-check disagreeing with the gyro. Its pitch and roll are WPILib's angles about
 * the board's axes for the chosen mount. Its accelerometer reads the board's axes, which the
 * board's orientation on the robot, from the robot's definition ({@code ImuMount.orientation}),
 * turns into the robot's. Unconfirmed on a real unit until checked against another IMU: its axes,
 * signs, scale, and drift.
 */
public final class GyroIOSystemcore implements GyroIO {
  private final OnboardIMU imu;
  private final Rotation3d orientation;
  private boolean read;
  private double lastYaw;
  private long lastTime;

  /**
   * @param mount how Systemcore is mounted on the robot, which sets the axis WPILib takes its yaw
   *     about
   * @param orientation how the board is turned on the robot, from the robot's definition: the
   *     rotation that takes the board's axes into the robot's
   */
  public GyroIOSystemcore(OnboardIMU.MountOrientation mount, Rotation3d orientation) {
    imu = new OnboardIMU(mount);
    this.orientation = orientation;
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
    Translation3d acceleration =
        robotAxes(orientation, imu.getAccelX(), imu.getAccelY(), imu.getAccelZ());
    inputs.accelerationXMetersPerSecSq = acceleration.getX();
    inputs.accelerationYMetersPerSecSq = acceleration.getY();
    inputs.odometryTimestampsNanos = new long[] {now};
    inputs.odometryYawsRad = new double[] {yaw};
  }

  /** What the board's accelerometer reads, along the robot's axes. */
  static Translation3d robotAxes(Rotation3d orientation, double x, double y, double z) {
    return new Translation3d(x, y, z).rotateBy(orientation);
  }
}
