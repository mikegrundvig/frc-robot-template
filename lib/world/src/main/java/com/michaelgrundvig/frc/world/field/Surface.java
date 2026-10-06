package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import org.wpilib.math.geometry.Translation2d;

/**
 * Raised ground robots drive on: a {@link Bump}, a {@link Ramp}, a {@link Platform}'s flat top, or
 * a shape from the field's CAD ({@link MeshSurface}). The simulator rides the robot over it,
 * localization tells a tilt the ground explains from one it doesn't and takes its {@link
 * #crossings} as landmarks, and the views draw it. Off every surface the ground is the floor, at
 * height zero.
 *
 * <p>Each surface describes its shape twice, and the two agree: exactly, as heights and slopes
 * ({@link #heightAt}, {@link #normalAt}), and as the triangles that make it ({@link #mesh}). The
 * tests sample every mesh against its heights, within a millimeter. Pure and immutable: every
 * answer comes from the description alone.
 */
public sealed interface Surface permits Bump, Ramp, Platform, MeshSurface {
  /** What the log and the views call it. */
  String name();

  /** What its top is covered in: the field's carpet, unless it says otherwise. */
  Material material();

  /** Where it is, seen from above, in the field frame. */
  Polygon2d footprint();

  /** How high the ground is at a point, in meters: zero off the surface. */
  double heightAt(Translation2d point);

  /**
   * Which way the ground faces at a point: a unit vector, straight up off the surface and on its
   * flat parts. How far it leans is the slope a robot standing there tilts by, which its IMU should
   * feel. On a fold (a crest, a ramp's foot), one side's or straight up.
   */
  Vector3 normalAt(Translation2d point);

  /**
   * Its triangles, in the field frame, counterclockwise seen from outside: the ground on top, and
   * each wall down to the floor. What a physics engine collides with and the views draw.
   */
  TriangleMesh mesh();

  /** The lines on it that place a robot crossing them: a crest, a ramp's top. */
  List<Crossing> crossings();

  /** How steep its steepest slope is, from level: what a robot on it can tilt by, walls aside. */
  double steepestSlopeRadians();

  /**
   * Whether a point is on the surface, or within a margin of its footprint.
   *
   * @param marginMeters how far outside the footprint still counts, for a point that may be off
   */
  default boolean covers(Translation2d point, double marginMeters) {
    return footprint().distance(point) <= marginMeters;
  }
}
