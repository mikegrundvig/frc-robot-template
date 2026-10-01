package frc.robot.studies;

import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.Body;
import frc.robot.lib.field.FieldModel;
import java.util.ArrayList;
import java.util.List;
import java.util.SplittableRandom;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Other robots on the field, standing where they block cameras' views of tags: boxes the size of
 * the robot being designed (its bumpers' outline, as tall as it is), each placed and turned at
 * random near the robot. Nothing about any game: field elements block views too, but where they are
 * is the game's, so they're left out.
 */
final class OtherRobots {
  /** Robots farther than this block little a camera would use: the pipeline drops far tags. */
  static final double REACH_METERS = 6;

  /** How many tries to find a place for a robot clear of the others before leaving it out. */
  private static final int TRIES = 50;

  private OtherRobots() {}

  /**
   * One robot in the way: a box, on the floor.
   *
   * @param x its center on the field
   * @param y its center on the field
   * @param heading which way it faces, in radians
   */
  record Blocker(
      double x, double y, double heading, double halfLength, double halfWidth, double height) {
    /**
     * Whether the line between two points passes through this robot: the points in the robot's own
     * frame, then where the line enters and leaves each of the box's three slabs.
     */
    boolean blocks(Translation3d from, Translation3d to) {
      double cos = Math.cos(heading);
      double sin = Math.sin(heading);
      double fromX = (from.getX() - x) * cos + (from.getY() - y) * sin;
      double fromY = -(from.getX() - x) * sin + (from.getY() - y) * cos;
      double toX = (to.getX() - x) * cos + (to.getY() - y) * sin;
      double toY = -(to.getX() - x) * sin + (to.getY() - y) * cos;
      double[] enter = {0};
      double[] leave = {1};
      return slab(fromX, toX, -halfLength, halfLength, enter, leave)
          && slab(fromY, toY, -halfWidth, halfWidth, enter, leave)
          && slab(from.getZ(), to.getZ(), 0, height, enter, leave);
    }

    /** Narrows the part of the line inside the box to the part between two planes. */
    private static boolean slab(
        double from, double to, double low, double high, double[] enter, double[] leave) {
      double along = to - from;
      if (Math.abs(along) < 1e-12) {
        return from >= low && from <= high;
      }
      double first = (low - from) / along;
      double second = (high - from) / along;
      enter[0] = Math.max(enter[0], Math.min(first, second));
      leave[0] = Math.min(leave[0], Math.max(first, second));
      return enter[0] <= leave[0];
    }
  }

  /**
   * Robots placed near this one, clear of it, of each other, and of the walls, each at a random
   * place within {@link #REACH_METERS} and turned a random way.
   *
   * @param body the robot being designed: the others are its size
   * @param count how many to place; fewer if there's no room
   */
  static List<Blocker> around(
      Pose2d robot, FieldModel field, Body body, int count, SplittableRandom random) {
    double halfLength = body.bumperLength().in(Meters) / 2;
    double halfWidth = body.bumperWidth().in(Meters) / 2;
    double height = body.height().in(Meters);
    // Clear of another robot when their centers are farther apart than both robots' reach.
    double clear = 2 * Math.hypot(halfLength, halfWidth);
    List<Blocker> placed = new ArrayList<>();
    for (int n = 0; n < count; n++) {
      for (int attempt = 0; attempt < TRIES; attempt++) {
        // Evenly over the ring from just clear of this robot out to the reach.
        double radius =
            Math.sqrt(
                clear * clear
                    + random.nextDouble() * (REACH_METERS * REACH_METERS - clear * clear));
        double bearing = 2 * Math.PI * random.nextDouble();
        double x = robot.getX() + radius * Math.cos(bearing);
        double y = robot.getY() + radius * Math.sin(bearing);
        double heading = 2 * Math.PI * random.nextDouble();
        if (fits(x, y, clear / 2, field, placed, clear)) {
          placed.add(new Blocker(x, y, heading, halfLength, halfWidth, height));
          break;
        }
      }
    }
    return placed;
  }

  /** Whether a robot here is on the field and clear of the robots already placed. */
  private static boolean fits(
      double x, double y, double reach, FieldModel field, List<Blocker> placed, double clear) {
    if (x < reach || y < reach || x > field.length() - reach || y > field.width() - reach) {
      return false;
    }
    for (Blocker other : placed) {
      if (Math.hypot(other.x() - x, other.y() - y) < clear) {
        return false;
      }
    }
    return true;
  }
}
