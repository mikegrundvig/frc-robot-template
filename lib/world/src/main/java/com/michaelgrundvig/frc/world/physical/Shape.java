package com.michaelgrundvig.frc.world.physical;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Translation3d;

/**
 * A solid's shape, set where it is in its owner's frame (a link's, for part of a mechanism), and
 * what it's made of: what the robot's extension checks and the views use. Only a {@link Box} so
 * far; other kinds come back when a robot needs one.
 */
public sealed interface Shape {
  /** Where it is, and how it's turned, in its owner's frame: each kind says from which point. */
  Pose3d pose();

  /** What it's made of. */
  Material material();

  /** How far a point in its owner's frame is from it, in meters: zero inside it or on it. */
  double distance(Translation3d point);

  /** Its middle, the center of its volume, in its owner's frame. */
  Translation3d center();

  /** Whether a point in its owner's frame is inside it, or on it. */
  default boolean contains(Translation3d point) {
    return distance(point) == 0;
  }

  /**
   * Points on it, in its owner's frame, that reach as far as it does in every direction: its
   * corners, or around a cylinder's ends closely enough to fall short by a part in a thousand. How
   * far it reaches out of the robot, or up, is how far the farthest of them does.
   */
  List<Translation3d> points();

  /**
   * A box, centered on its pose, its sides along the pose's axes.
   *
   * @param pose its center, and how it's turned
   * @param size its length along the pose's x, width along y, and height along z, in meters
   * @param material what it's made of
   */
  @ConfigRecord
  record Box(Pose3d pose, Translation3d size, Material material) implements Shape {
    public Box {
      Checks.on(Box.class)
          .field("pose", pose, Shape::notAPose)
          .field(
              "size",
              size,
              each ->
                  each.getX() > 0
                          && each.getY() > 0
                          && each.getZ() > 0
                          && Double.isFinite(each.getX() + each.getY() + each.getZ())
                      ? null
                      : each + " must be positive along each axis")
          .orThrow();
    }

    /** Starts describing a box, one named value at a time. */
    public static ShapeBoxBuilder.PoseStage builder() {
      return ShapeBoxBuilder.builder();
    }

    @Override
    public double distance(Translation3d point) {
      Translation3d local = local(pose, point);
      return Math.sqrt(
          square(outside(Math.abs(local.getX()) - size.getX() / 2))
              + square(outside(Math.abs(local.getY()) - size.getY() / 2))
              + square(outside(Math.abs(local.getZ()) - size.getZ() / 2)));
    }

    @Override
    public Translation3d center() {
      return pose.getTranslation();
    }

    @Override
    public List<Translation3d> points() {
      List<Translation3d> corners = new ArrayList<>();
      for (int i = 0; i < 8; i++) {
        corners.add(
            placed(
                pose,
                new Translation3d(
                    ((i & 1) == 0 ? -1 : 1) * size.getX() / 2,
                    ((i & 2) == 0 ? -1 : 1) * size.getY() / 2,
                    ((i & 4) == 0 ? -1 : 1) * size.getZ() / 2)));
      }
      return corners;
    }
  }

  /** A point in a shape's own frame, in its owner's. */
  private static Translation3d placed(Pose3d pose, Translation3d local) {
    return local.rotateBy(pose.getRotation()).plus(pose.getTranslation());
  }

  /** A point in a shape's own frame. */
  private static Translation3d local(Pose3d pose, Translation3d point) {
    return point.minus(pose.getTranslation()).rotateBy(pose.getRotation().inverse());
  }

  /** How far past a side a coordinate is: zero inside it. */
  private static double outside(double past) {
    return Math.max(past, 0);
  }

  private static double square(double value) {
    return value * value;
  }

  /** What's wrong with a pose, or null when it's a pose, numbers all. */
  private static @Nullable String notAPose(Pose3d pose) {
    Quaternion turn = pose.getRotation().getQuaternion();
    return Double.isFinite(
            pose.getX()
                + pose.getY()
                + pose.getZ()
                + turn.getW()
                + turn.getX()
                + turn.getY()
                + turn.getZ())
        ? null
        : pose + " must be a pose, numbers all";
  }
}
