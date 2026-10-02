package com.michaelgrundvig.frc.pitwall.scene;

import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;

/**
 * What moves in a scene, as a layer says in {@link Layer#update}: frames' poses, values, shapes'
 * styles, and the layer's shapes of the moment. Each is kept until it's said again, so a layer says
 * only what it has to; and each is sent to the page only once it has changed by what the page can
 * show (a millimeter, a tenth of a degree, a value's resolution), so a robot standing still sends
 * nothing.
 *
 * <p>It's the robot loop's: a layer calls it from {@code update}, and nowhere else. A name the
 * scene doesn't have is a mistake in the layer, and throws.
 */
public interface SceneState {
  /** The robot's clock now, in nanoseconds. */
  long nowNanos();

  /**
   * A frame's pose in its parent, level: on the floor, turned about z.
   *
   * @param yaw radians, counterclockwise
   */
  default void pose(String frame, double x, double y, double yaw) {
    pose(frame, x, y, 0, 0, 0, yaw);
  }

  /**
   * A frame's pose in its parent. A number that isn't finite leaves the pose as it was.
   *
   * @param roll radians, as WPILib's {@code Rotation3d} turns: about x, then pitch about y, then
   *     yaw about z
   */
  void pose(String frame, double x, double y, double z, double roll, double pitch, double yaw);

  /** A frame's pose in its parent, level: a robot's pose on the field, say. */
  default void pose(String frame, Pose2d pose) {
    pose(frame, pose.getX(), pose.getY(), pose.getRotation().getRadians());
  }

  /** A frame's pose in its parent. */
  default void pose(String frame, Pose3d pose) {
    pose(
        frame,
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getX(),
        pose.getRotation().getY(),
        pose.getRotation().getZ());
  }

  /** A value a shape's size follows. A number that isn't finite leaves it as it was. */
  void value(String name, double value);

  /**
   * Draws a shape of the scene another way: a camera's frustum in {@link Style#BAD} while it's
   * unplugged, say. Null draws it as described again.
   */
  void style(String shape, @Nullable Style style);

  /**
   * Starts this layer's shapes of the moment over: the shapes added to what this returns replace
   * the ones before, whole, as this {@code update} returns. A layer that doesn't call it keeps the
   * ones it has, so it can call it only when something changed.
   */
  Shapes shapes();
}
