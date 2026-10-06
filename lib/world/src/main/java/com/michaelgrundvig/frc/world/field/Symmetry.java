package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * How the red alliance's half of the field relates to the blue alliance's. Fields have been one of
 * two kinds, and which one changes by season, so each season's field model declares it. WPILib's
 * tag layouts show 2022, 2025, and 2026 rotated, and 2023 and 2024 mirrored.
 *
 * <p>Each applies to a whole pose, position and heading together, and applying it twice gives back
 * what it started with. It turns nominal geometry, written for blue, into red's: places, zones,
 * surfaces, elements, and the tags' official poses. What was measured, a survey's tags, is never
 * turned: a survey measures each tag where it stands, on both halves.
 */
public enum Symmetry {
  /** Red's half is blue's reflected across the center line, x = length / 2: 2023 and 2024. */
  MIRRORED,

  /** Red's half is blue's turned half a turn about the center of the field: 2022, 2025, 2026. */
  ROTATED;

  /** Where a point on one alliance's half is on the other's. */
  public Translation2d apply(Translation2d point, double length, double width) {
    return switch (this) {
      case MIRRORED -> new Translation2d(length - point.getX(), point.getY());
      case ROTATED -> new Translation2d(length - point.getX(), width - point.getY());
    };
  }

  /** Which way a heading on one alliance's half points on the other's. */
  public Rotation2d apply(Rotation2d heading) {
    return switch (this) {
      case MIRRORED -> Rotation2d.PI.minus(heading);
      case ROTATED -> heading.plus(Rotation2d.PI);
    };
  }

  /** Where a pose on one alliance's half is on the other's. */
  public Pose2d apply(Pose2d pose, double length, double width) {
    return new Pose2d(apply(pose.getTranslation(), length, width), apply(pose.getRotation()));
  }

  /**
   * Where a pose in 3D is on the other half, at the same height: its x axis turned as a heading is
   * ({@link #apply(Rotation2d)}), so a tag's image faces where the tag's face would. A mirror image
   * of a frame is a left-handed one; its y axis is turned back, so the image is a pose still, with
   * its x and z the mirror images of the pose's, and its y opposite its mirror image.
   */
  public Pose3d apply(Pose3d pose, double length, double width) {
    Translation2d image = apply(pose.getTranslation().toTranslation2d(), length, width);
    return new Pose3d(image.getX(), image.getY(), pose.getZ(), apply(pose.getRotation()));
  }

  /**
   * Which way a frame in 3D is turned on the other half: as a pose's is ({@link #apply(Pose3d,
   * double, double)}), its x axis turned as a heading is, and on a mirror image its y axis turned
   * back.
   */
  public Rotation3d apply(Rotation3d rotation) {
    return switch (this) {
      // A reflection across x = 0 turns a rotation's axis and reverses its sense: (w, x, -y, -z).
      // Then its y axis turned back, with z as it is: a half turn about z, first.
      case MIRRORED -> {
        Quaternion q = rotation.getQuaternion();
        yield HALF_TURN.rotateBy(
            new Rotation3d(new Quaternion(q.getW(), q.getX(), -q.getY(), -q.getZ())));
      }
      case ROTATED -> rotation.rotateBy(HALF_TURN);
    };
  }

  /**
   * An offset within a frame, as it is within the frame's image on the other half: what takes a
   * pose's image to the image of the pose it took the pose to. The same on a turned field; on a
   * mirrored one, mirrored across the frame's x-z plane, since the image's y axis runs the other
   * way. So something placed from a tag on blue's half lands, placed from the tag's partner, on its
   * image.
   */
  public Transform3d inImage(Transform3d offset) {
    if (!mirrors()) {
      return offset;
    }
    Translation3d move = offset.getTranslation();
    // The frame's y axis reversed, on both sides of the turn: its axis's x and z reverse, and its
    // sense too, so (w, -x, y, -z).
    Quaternion q = offset.getRotation().getQuaternion();
    return new Transform3d(
        new Translation3d(move.getX(), -move.getY(), move.getZ()),
        new Rotation3d(new Quaternion(q.getW(), -q.getX(), q.getY(), -q.getZ())));
  }

  /** Where a point in 3D is on the other half: as on the floor, at the same height. */
  public Vector3 apply(Vector3 point, double length, double width) {
    Translation2d image = apply(new Translation2d(point.x(), point.y()), length, width);
    return new Vector3(image.getX(), image.getY(), point.z());
  }

  /**
   * An outline on the other half. A mirror image runs the other way round, so its corners are then
   * listed backwards to stay counterclockwise, the first still first: its edge {@code i} is the
   * image of edge {@code n - 1 - i}.
   */
  public Polygon2d apply(Polygon2d polygon, double length, double width) {
    List<Translation2d> corners = new ArrayList<>();
    for (int i = 0; i < polygon.corners().size(); i++) {
      corners.add(apply(polygon.corner(mirrors() ? -i : i), length, width));
    }
    return new Polygon2d(corners);
  }

  /** A shape on the other half, its triangles still facing out. */
  public TriangleMesh apply(TriangleMesh mesh, double length, double width) {
    return mesh.map(corner -> apply(corner, length, width), mirrors());
  }

  /**
   * A crossing on the other half. A mirror image swaps its ends, so the way across it is still the
   * image of the way across: a ramp's top still faces up the ramp.
   */
  public Crossing apply(Crossing crossing, double length, double width) {
    Translation2d start = apply(crossing.start(), length, width);
    Translation2d end = apply(crossing.end(), length, width);
    return mirrors()
        ? new Crossing(crossing.name(), end, start, crossing.kind())
        : new Crossing(crossing.name(), start, end, crossing.kind());
  }

  /** A surface on the other half, under the same name. */
  public Surface apply(Surface surface, double length, double width) {
    return switch (surface) {
      case Bump bump -> {
        Crossing crest = apply(bump.crest(), length, width);
        yield new Bump(
            bump.name(),
            crest.start(),
            crest.end(),
            bump.heightMeters(),
            bump.depthMeters(),
            bump.material());
      }
      case Ramp ramp ->
          new Ramp(
              ramp.name(),
              apply(ramp.foot(), length, width),
              apply(ramp.top(), length, width),
              ramp.widthMeters(),
              ramp.heightMeters(),
              ramp.material());
      case Platform platform -> {
        List<Platform.Side> sides = new ArrayList<>(platform.sides());
        if (mirrors()) {
          Collections.reverse(sides);
        }
        yield new Platform(
            platform.name(),
            apply(platform.top(), length, width),
            platform.heightMeters(),
            sides,
            platform.material());
      }
      case MeshSurface mesh ->
          new MeshSurface(
              mesh.name(),
              apply(mesh.mesh(), length, width),
              mesh.crossings().stream().map(each -> apply(each, length, width)).toList(),
              mesh.material());
    };
  }

  private static final Rotation3d HALF_TURN = new Rotation3d(0, 0, Math.PI);

  /** Whether it's a mirror image, which turns counterclockwise into clockwise. */
  boolean mirrors() {
    return this == MIRRORED;
  }
}
