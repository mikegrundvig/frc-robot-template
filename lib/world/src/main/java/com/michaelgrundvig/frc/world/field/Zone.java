package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Translation2d;

/**
 * A named area of the field, for questions like "is the robot in its alliance zone?" or "will it be
 * under the trench within 0.4 s?". Written as if the robot were on the blue alliance, like every
 * place, as one or more polygons: the manual's tape lines, or a rectangle ({@link
 * Polygon2d#rectangle(Translation2d, Translation2d)}).
 *
 * @param name what robot code and logs call it
 * @param side whose zone it is, from the robot's alliance's point of view
 * @param blueAreas where it is, written for the blue alliance
 */
public record Zone(String name, Side side, List<Polygon2d> blueAreas) {
  /**
   * Whose zone it is, from the robot's alliance's point of view: whose it is, not which half it's
   * on. A 2024 source zone is the alliance's own, by the opposing alliance's wall.
   */
  public enum Side {
    /** The robot's own alliance's: its alliance zone, its source zone, wherever they are. */
    OURS,
    /** The other alliance's. */
    THEIRS,
    /** Both alliances': an area that appears on each half, such as under a trench. */
    BOTH
  }

  public Zone {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a zone needs a name");
    }
    if (blueAreas.isEmpty()) {
      throw new IllegalArgumentException(name + " needs at least one area");
    }
    blueAreas = List.copyOf(blueAreas);
  }

  /** The robot's own alliance's zone, written as blue's. */
  public static Zone ours(String name, Polygon2d... blueAreas) {
    return new Zone(name, Side.OURS, List.of(blueAreas));
  }

  /** The other alliance's zone, written as if it were blue's. */
  public static Zone theirs(String name, Polygon2d... blueAreas) {
    return new Zone(name, Side.THEIRS, List.of(blueAreas));
  }

  /** A zone both alliances share: each area and its image under the field's symmetry. */
  public static Zone both(String name, Polygon2d... blueAreas) {
    return new Zone(name, Side.BOTH, List.of(blueAreas));
  }

  /** Where the zone is for a robot on an alliance, in field coordinates. */
  public List<Polygon2d> areas(FieldModel field, Alliance alliance) {
    List<Polygon2d> areas = new ArrayList<>();
    for (Polygon2d area : blueAreas) {
      switch (side) {
        case OURS -> areas.add(field.forAlliance(alliance, area));
        case THEIRS -> areas.add(field.forAlliance(other(alliance), area));
        case BOTH -> {
          areas.add(area);
          areas.add(field.forAlliance(Alliance.RED, area));
        }
      }
    }
    return areas;
  }

  /** Whether a point is inside the zone, edges included. */
  public boolean contains(FieldModel field, Alliance alliance, Translation2d point) {
    for (Polygon2d area : areas(field, alliance)) {
      if (area.contains(point)) {
        return true;
      }
    }
    return false;
  }

  /** How far a point is from the zone, in meters: zero inside it. */
  public double distance(FieldModel field, Alliance alliance, Translation2d point) {
    double nearest = Double.POSITIVE_INFINITY;
    for (Polygon2d area : areas(field, alliance)) {
      nearest = Math.min(nearest, area.distance(point));
    }
    return nearest;
  }

  /**
   * Whether a robot moving in a straight line at a steady velocity is inside the zone now or will
   * be within some time: the question to ask before arriving, so a mechanism has time to move.
   *
   * @param velocity field-relative, meters per second
   */
  public boolean reachedWithin(
      FieldModel field,
      Alliance alliance,
      Translation2d position,
      Translation2d velocity,
      double seconds) {
    Translation2d end = position.plus(velocity.times(seconds));
    for (Polygon2d area : areas(field, alliance)) {
      if (area.touches(position, end)) {
        return true;
      }
    }
    return false;
  }

  private static Alliance other(Alliance alliance) {
    return alliance == Alliance.BLUE ? Alliance.RED : Alliance.BLUE;
  }
}
