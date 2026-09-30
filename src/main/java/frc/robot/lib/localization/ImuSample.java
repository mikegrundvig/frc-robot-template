package frc.robot.lib.localization;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * What an IMU reads at one moment.
 *
 * @param timestampNanos when it was read
 * @param orientation the robot's attitude. Yaw is what the IMU has integrated since it started, not
 *     the field heading: the estimator keeps the difference. Pitch and roll come from gravity, so
 *     they are true to the floor.
 * @param angularVelocity radians per second about the robot's x, y, and z axes: the roll, pitch,
 *     and yaw rates
 * @param acceleration meters per second squared along the robot's axes, gravity included
 * @param connected false once the IMU stops reporting; its other values are then stale
 * @param source which IMU, when there are several
 */
public record ImuSample(
    long timestampNanos,
    Rotation3d orientation,
    Translation3d angularVelocity,
    Translation3d acceleration,
    boolean connected,
    String source) {
  public ImuSample {
    if (timestampNanos < 0) {
      throw new IllegalArgumentException("a time on the robot's clock: " + timestampNanos);
    }
    if (source.isBlank()) {
      throw new IllegalArgumentException("an IMU sample needs its source");
    }
  }
}
