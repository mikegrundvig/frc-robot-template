package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A bump robots drive over: a ridge along a straight crest, with ramps down to the floor on both
 * sides (REBUILT's were 6.5 in tall, with 15° ramps), and flat ends. Crossing one puts the robot's
 * center on the crest line at the top: a landmark across the bump, though not along it.
 *
 * @param name for the log
 * @param crestStart one end of the crest line, in the field frame
 * @param crestEnd the other end
 * @param heightMeters how high the crest stands above the floor
 * @param depthMeters from the foot of one ramp to the foot of the other, across the crest
 * @param material what it's covered in
 */
public record Bump(
    String name,
    Translation2d crestStart,
    Translation2d crestEnd,
    double heightMeters,
    double depthMeters,
    Material material)
    implements Surface {
  public Bump {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a bump needs a name");
    }
    if (!(crestStart.getDistance(crestEnd) > 0)) {
      throw new IllegalArgumentException(name + ": a crest runs between two different points");
    }
    if (!(heightMeters > 0)
        || !(depthMeters > 0)
        || Double.isInfinite(heightMeters)
        || Double.isInfinite(depthMeters)) {
      throw new IllegalArgumentException(name + ": a bump's height and depth must be positive");
    }
  }

  /** A bump covered in the field's carpet. */
  public Bump(
      String name,
      Translation2d crestStart,
      Translation2d crestEnd,
      double heightMeters,
      double depthMeters) {
    this(name, crestStart, crestEnd, heightMeters, depthMeters, Material.CARPET);
  }

  /** Its crest, the bump's one crossing, named as the bump is. */
  public Crossing crest() {
    return new Crossing(name, crestStart, crestEnd, Crossing.Kind.CREST);
  }

  /** How long the crest is. */
  public double length() {
    return crest().length();
  }

  /** The way along the crest, from its start to its end. */
  public Rotation2d along() {
    return crest().along();
  }

  /** The way across the crest: its crossing axis, a quarter turn left of along it. */
  public Rotation2d across() {
    return crest().across();
  }

  /** How far a point is from the crest line, across it: positive on the {@link #across} side. */
  public double acrossOffset(Translation2d point) {
    return crest().acrossOffset(point);
  }

  /** How far along the crest a point is, from its start. */
  public double alongOffset(Translation2d point) {
    return crest().alongOffset(point);
  }

  /** Whether a point is beside the crest, not past either end of it, give or take a margin. */
  public boolean spans(Translation2d point, double marginMeters) {
    return crest().spans(point, marginMeters);
  }

  /**
   * Whether a point is on the bump, or within a margin of its footprint: of its sides, and of its
   * ends, so the margin squares the footprint's corners off rather than rounding them.
   */
  @Override
  public boolean covers(Translation2d point, double marginMeters) {
    return spans(point, marginMeters)
        && Math.abs(acrossOffset(point)) <= depthMeters / 2 + marginMeters;
  }

  /** How steep the ramps are. */
  public double slopeRadians() {
    return Math.atan2(heightMeters, depthMeters / 2);
  }

  @Override
  public double steepestSlopeRadians() {
    return slopeRadians();
  }

  /** How high the bump is at a point: the crest's height, down the ramps to the floor. */
  @Override
  public double heightAt(Translation2d point) {
    if (!spans(point, 0)) {
      return 0;
    }
    return heightMeters * Math.max(0, 1 - Math.abs(acrossOffset(point)) / (depthMeters / 2));
  }

  /** Up the ramp a point is on, leaning away from the crest; straight up on the crest itself. */
  @Override
  public Vector3 normalAt(Translation2d point) {
    double offset = acrossOffset(point);
    if (!spans(point, 0) || offset == 0 || Math.abs(offset) > depthMeters / 2) {
      return Vector3.UP;
    }
    // The ground falls away from the crest at the ramps' slope, so it faces away from the crest.
    double lean = Math.signum(offset) * heightMeters / (depthMeters / 2);
    Rotation2d across = across();
    return new Vector3(lean * across.getCos(), lean * across.getSin(), 1).unit();
  }

  /** The rectangle from one ramp's foot to the other's, between the crest's ends. */
  @Override
  public Polygon2d footprint() {
    Translation2d side = new Translation2d(depthMeters / 2, across());
    return new Polygon2d(
        List.of(
            crestStart.minus(side),
            crestEnd.minus(side),
            crestEnd.plus(side),
            crestStart.plus(side)));
  }

  /** Its two ramps, and a triangle at each end of the crest. */
  @Override
  public TriangleMesh mesh() {
    Translation2d side = new Translation2d(depthMeters / 2, across());
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    int startBack = floor(mesh, crestStart.minus(side));
    int startFront = floor(mesh, crestStart.plus(side));
    int startTop = mesh.corner(crestStart.getX(), crestStart.getY(), heightMeters);
    int endBack = floor(mesh, crestEnd.minus(side));
    int endFront = floor(mesh, crestEnd.plus(side));
    int endTop = mesh.corner(crestEnd.getX(), crestEnd.getY(), heightMeters);
    return mesh.quad(startBack, endBack, endTop, startTop)
        .quad(startFront, startTop, endTop, endFront)
        .triangle(startBack, startTop, startFront)
        .triangle(endBack, endFront, endTop)
        .build();
  }

  @Override
  public List<Crossing> crossings() {
    return List.of(crest());
  }

  private static int floor(TriangleMesh.Builder mesh, Translation2d point) {
    return mesh.corner(point.getX(), point.getY(), 0);
  }
}
