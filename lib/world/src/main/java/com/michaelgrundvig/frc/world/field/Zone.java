package com.michaelgrundvig.frc.world.field;

import java.util.ArrayList;
import java.util.List;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.shape.Rectangle2d;

/**
 * A named area of the field, for questions like "is the robot in its alliance zone?" or "will it be
 * under the trench within 0.4 s?". Written as if the robot were on the blue alliance, like every
 * place, as one or more rectangles.
 *
 * @param name what robot code and logs call it
 * @param side whose half of the field it is on, from the robot's alliance's point of view
 * @param blueAreas where it is, written for the blue alliance
 */
public record Zone(String name, Side side, List<Rectangle2d> blueAreas) {
  /** Whose half of the field a zone is on, from the robot's alliance's point of view. */
  public enum Side {
    /** The robot's own alliance's: its alliance zone, its loading station. */
    OURS,
    /** The other alliance's. */
    THEIRS,
    /** Both halves: an element that appears on each, such as a trench. */
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

  /** A zone on the robot's own alliance's half. */
  public static Zone ours(String name, Rectangle2d... blueAreas) {
    return new Zone(name, Side.OURS, List.of(blueAreas));
  }

  /** A zone on the other alliance's half. */
  public static Zone theirs(String name, Rectangle2d... blueAreas) {
    return new Zone(name, Side.THEIRS, List.of(blueAreas));
  }

  /** A zone on both halves: each area and its image under the field's symmetry. */
  public static Zone both(String name, Rectangle2d... blueAreas) {
    return new Zone(name, Side.BOTH, List.of(blueAreas));
  }

  /** Where the zone is for a robot on an alliance, in field coordinates. */
  public List<Rectangle2d> areas(FieldModel field, Alliance alliance) {
    List<Rectangle2d> areas = new ArrayList<>();
    for (Rectangle2d area : blueAreas) {
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
    for (Rectangle2d area : areas(field, alliance)) {
      if (area.contains(point)) {
        return true;
      }
    }
    return false;
  }

  /** How far a point is from the zone, in meters: zero inside it. */
  public double distance(FieldModel field, Alliance alliance, Translation2d point) {
    double nearest = Double.POSITIVE_INFINITY;
    for (Rectangle2d area : areas(field, alliance)) {
      nearest = Math.min(nearest, area.getDistance(point));
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
    for (Rectangle2d area : areas(field, alliance)) {
      if (crosses(area, position, end)) {
        return true;
      }
    }
    return false;
  }

  /** Whether the straight path from a to b touches the area. */
  static boolean crosses(Rectangle2d area, Translation2d a, Translation2d b) {
    // In the area's own frame it is centered and square to the axes: clip the path to it on each
    // axis in turn, keeping the part of the path (0 at a, 1 at b) that is inside on both.
    Pose2d center = area.getCenter();
    Translation2d from =
        a.minus(center.getTranslation()).rotateBy(center.getRotation().unaryMinus());
    Translation2d to = b.minus(center.getTranslation()).rotateBy(center.getRotation().unaryMinus());
    double[] inside = {0, 1};
    return clip(from.getX(), to.getX() - from.getX(), area.getXWidth() / 2, inside)
        && clip(from.getY(), to.getY() - from.getY(), area.getYWidth() / 2, inside);
  }

  /**
   * Narrows the range of t where start + t * step is within half of zero; false once it's empty.
   */
  private static boolean clip(double start, double step, double half, double[] range) {
    if (step == 0) {
      return Math.abs(start) <= half;
    }
    double enter = (-half - start) / step;
    double leave = (half - start) / step;
    range[0] = Math.max(range[0], Math.min(enter, leave));
    range[1] = Math.min(range[1], Math.max(enter, leave));
    return range[0] <= range[1];
  }

  private static Alliance other(Alliance alliance) {
    return alliance == Alliance.BLUE ? Alliance.RED : Alliance.BLUE;
  }
}
