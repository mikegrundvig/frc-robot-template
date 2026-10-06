package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A raised platform: a flat top at a height, each of its sides either a vertical edge or a ramp
 * down to the floor. Each ramp runs straight out from its side, as wide as the side, so where two
 * ramps meet at a corner the floor shows between them. A ramp's top edge is a crossing.
 *
 * @param name for the log
 * @param top the flat top's outline, seen from above, in the field frame: convex, so every ramp
 *     runs out from it clear of the others
 * @param heightMeters how high the top is above the floor
 * @param sides each side of the top in turn: side {@code i} is the top's edge from its corner
 *     {@code i} to the next
 * @param material what its top and ramps are covered in
 */
public record Platform(
    String name, Polygon2d top, double heightMeters, List<Side> sides, Material material)
    implements Surface {
  /**
   * One side of a platform's top: a vertical edge, or a ramp down to the floor.
   *
   * @param rampMeters how far a ramp runs out from the side to its foot, seen from above; zero for
   *     a vertical edge
   */
  public record Side(double rampMeters) {
    /** A vertical edge: the top drops straight to the floor. */
    public static final Side EDGE = new Side(0);

    public Side {
      if (!(rampMeters >= 0) || Double.isInfinite(rampMeters)) {
        throw new IllegalArgumentException(
            "a ramp runs some distance out from its side, or none for an edge: " + rampMeters);
      }
    }

    /** A ramp from the side down to the floor, its foot this far out from the side. */
    public static Side ramp(double runMeters) {
      if (!(runMeters > 0)) {
        throw new IllegalArgumentException("a ramp runs some distance out: " + runMeters);
      }
      return new Side(runMeters);
    }

    /** Whether a ramp leads down from it, rather than a vertical edge. */
    public boolean ramped() {
      return rampMeters > 0;
    }
  }

  public Platform {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a platform needs a name");
    }
    if (!top.convex()) {
      throw new IllegalArgumentException(name + ": a platform's top must be convex");
    }
    if (!(heightMeters > 0) || Double.isInfinite(heightMeters)) {
      throw new IllegalArgumentException(name + ": a platform's height must be positive");
    }
    sides = List.copyOf(sides);
    if (sides.size() != top.corners().size()) {
      throw new IllegalArgumentException(
          name
              + ": a platform needs a side for each of its top's "
              + top.corners().size()
              + " edges, not "
              + sides.size());
    }
    for (int i = 0; i < sides.size(); i++) {
      int before = Math.floorMod(i - 1, sides.size());
      Translation2d in = top.corner(i).minus(top.corner(i - 1));
      Translation2d out = top.corner(i + 1).minus(top.corner(i));
      if (sides.get(before).ramped()
          && sides.get(i).ramped()
          && in.cross(out) <= 1e-9 * in.getNorm() * out.getNorm()) {
        throw new IllegalArgumentException(
            name
                + ": the top's corner "
                + i
                + " goes straight on between two ramps: make them one side, or one an edge");
      }
    }
  }

  /** A platform covered in the field's carpet. */
  public Platform(String name, Polygon2d top, double heightMeters, List<Side> sides) {
    this(name, top, heightMeters, sides, Material.CARPET);
  }

  @Override
  public double heightAt(Translation2d point) {
    if (top.contains(point)) {
      return heightMeters;
    }
    double height = 0;
    for (int i = 0; i < sides.size(); i++) {
      height = Math.max(height, rampHeight(i, point));
    }
    return height;
  }

  /** Straight up on the top; leaning out from the side on a ramp; straight up off it. */
  @Override
  public Vector3 normalAt(Translation2d point) {
    if (top.contains(point)) {
      return Vector3.UP;
    }
    // The ramp the ground is highest on there, as heightAt takes it.
    int highest = -1;
    double height = Double.NEGATIVE_INFINITY;
    for (int i = 0; i < sides.size(); i++) {
      if (onRamp(i, point) && rampHeight(i, point) > height) {
        height = rampHeight(i, point);
        highest = i;
      }
    }
    if (highest < 0) {
      return Vector3.UP;
    }
    double lean = heightMeters / sides.get(highest).rampMeters();
    Rotation2d out = outward(highest);
    return new Vector3(lean * out.getCos(), lean * out.getSin(), 1).unit();
  }

  @Override
  public double steepestSlopeRadians() {
    double steepest = 0;
    for (Side side : sides) {
      if (side.ramped()) {
        steepest = Math.max(steepest, Math.atan2(heightMeters, side.rampMeters()));
      }
    }
    return steepest;
  }

  /** The top, and each ramp out from it; the floor between two ramps at a corner left out. */
  @Override
  public Polygon2d footprint() {
    List<Translation2d> outline = new ArrayList<>();
    for (int i = 0; i < sides.size(); i++) {
      outline.add(top.corner(i));
      if (sides.get(i).ramped()) {
        Translation2d out = new Translation2d(sides.get(i).rampMeters(), outward(i));
        outline.add(top.corner(i).plus(out));
        outline.add(top.corner(i + 1).plus(out));
      }
    }
    return new Polygon2d(outline);
  }

  /** Its top, and for each side a wall down to the floor, or a ramp with a triangle at each end. */
  @Override
  public TriangleMesh mesh() {
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    int corners = top.corners().size();
    for (int i = 0; i < corners; i++) {
      mesh.corner(top.corner(i).getX(), top.corner(i).getY(), heightMeters);
    }
    for (var triangle : top.triangles()) {
      mesh.triangle(triangle.a(), triangle.b(), triangle.c());
    }
    for (int i = 0; i < corners; i++) {
      int from = i;
      int to = (i + 1) % corners;
      int fromFloor = floor(mesh, top.corner(i));
      int toFloor = floor(mesh, top.corner(i + 1));
      Side side = sides.get(i);
      if (!side.ramped()) {
        mesh.quad(fromFloor, toFloor, to, from);
        continue;
      }
      Translation2d out = new Translation2d(side.rampMeters(), outward(i));
      int fromFoot = floor(mesh, top.corner(i).plus(out));
      int toFoot = floor(mesh, top.corner(i + 1).plus(out));
      mesh.quad(from, fromFoot, toFoot, to)
          .triangle(from, fromFloor, fromFoot)
          .triangle(to, toFoot, toFloor);
    }
    return mesh.build();
  }

  /** Each ramp's top edge, named for its side: the way across it is the way up. */
  @Override
  public List<Crossing> crossings() {
    List<Crossing> crossings = new ArrayList<>();
    for (int i = 0; i < sides.size(); i++) {
      if (sides.get(i).ramped()) {
        crossings.add(
            new Crossing(
                name + ", side " + (i + 1), top.corner(i), top.corner(i + 1), Crossing.Kind.TOP));
      }
    }
    return List.copyOf(crossings);
  }

  /** The way out of the top across a side: a quarter turn right of along it. */
  private Rotation2d outward(int side) {
    return top.corner(side + 1)
        .minus(top.corner(side))
        .getAngle()
        .orElseThrow()
        .minus(Rotation2d.CCW_PI_2);
  }

  /** How high a side's ramp is at a point: zero off it, or for a side with no ramp. */
  private double rampHeight(int side, Translation2d point) {
    if (!onRamp(side, point)) {
      return 0;
    }
    double out = outOffset(side, point);
    return heightMeters * (1 - out / sides.get(side).rampMeters());
  }

  /** Whether a point is over a side's ramp, edges included. */
  private boolean onRamp(int side, Translation2d point) {
    if (!sides.get(side).ramped()) {
      return false;
    }
    Translation2d edge = top.corner(side + 1).minus(top.corner(side));
    double along = point.minus(top.corner(side)).dot(edge) / edge.getNorm();
    double out = outOffset(side, point);
    return along >= 0 && along <= edge.getNorm() && out >= 0 && out <= sides.get(side).rampMeters();
  }

  /** How far a point is out from a side's line. */
  private double outOffset(int side, Translation2d point) {
    return point.minus(top.corner(side)).rotateBy(outward(side).unaryMinus()).getX();
  }

  private static int floor(TriangleMesh.Builder mesh, Translation2d point) {
    return mesh.corner(point.getX(), point.getY(), 0);
  }
}
