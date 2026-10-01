package frc.robot.studies;

import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.Body;
import frc.robot.lib.field.FieldModel;
import java.util.ArrayList;
import java.util.List;
import java.util.SplittableRandom;
import java.util.function.Predicate;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Other robots on the field, standing where they block cameras' views of tags: solid boxes with the
 * outline of the robot being designed (its bumpers'), each placed and turned at random near the
 * robot. Nothing about any game: field elements block views too, but where they are is the game's,
 * so they're left out. A comparison under one crowded case, not a prediction (see
 * docs/camera-placement.md for what it leaves out).
 */
final class OtherRobots {
  /** An FRC field has five other robots on it. */
  static final int MOST = 5;

  /** How many tries to find a place for a robot clear of the others before leaving it out. */
  private static final int TRIES = 50;

  private OtherRobots() {}

  /**
   * One robot in the way: a box on the floor.
   *
   * @param x its center on the field
   * @param y its center on the field
   * @param heading which way it faces, in radians
   * @param halfLength half its length, front to back
   * @param halfWidth half its width
   * @param height how tall it is
   */
  record Blocker(
      double x, double y, double heading, double halfLength, double halfWidth, double height) {
    /**
     * Whether the line between two points passes through this robot: with the points in the robot's
     * own frame, the part of the line between each pair of the box's faces, narrowed axis by axis,
     * still has something in it.
     */
    boolean blocks(Translation3d from, Translation3d to) {
      double cos = Math.cos(heading);
      double sin = Math.sin(heading);
      double[] start = {
        (from.getX() - x) * cos + (from.getY() - y) * sin,
        -(from.getX() - x) * sin + (from.getY() - y) * cos,
        from.getZ()
      };
      double[] end = {
        (to.getX() - x) * cos + (to.getY() - y) * sin,
        -(to.getX() - x) * sin + (to.getY() - y) * cos,
        to.getZ()
      };
      double[] low = {-halfLength, -halfWidth, 0};
      double[] high = {halfLength, halfWidth, height};
      double enter = 0;
      double leave = 1;
      for (int axis = 0; axis < 3; axis++) {
        double along = end[axis] - start[axis];
        if (Math.abs(along) < 1e-12) {
          if (start[axis] < low[axis] || start[axis] > high[axis]) {
            return false;
          }
          continue;
        }
        double first = (low[axis] - start[axis]) / along;
        double second = (high[axis] - start[axis]) / along;
        enter = Math.max(enter, Math.min(first, second));
        leave = Math.min(leave, Math.max(first, second));
        if (enter > leave) {
          return false;
        }
      }
      return true;
    }

    /** The four corners of its outline on the floor. */
    List<Translation2d> corners() {
      double cos = Math.cos(heading);
      double sin = Math.sin(heading);
      List<Translation2d> corners = new ArrayList<>();
      for (int ahead = -1; ahead <= 1; ahead += 2) {
        for (int left = -1; left <= 1; left += 2) {
          double along = ahead * halfLength;
          double across = left * halfWidth;
          corners.add(
              new Translation2d(x + along * cos - across * sin, y + along * sin + across * cos));
        }
      }
      return corners;
    }

    /**
     * Whether the two outlines overlap on the floor, by separating axes: two rectangles are apart
     * exactly when, along one of their four edges' directions, their shadows don't meet. So a robot
     * can stand right up against another.
     */
    boolean overlaps(Blocker other) {
      List<Translation2d> mine = corners();
      List<Translation2d> theirs = other.corners();
      double[] edges = {heading, heading + Math.PI / 2, other.heading, other.heading + Math.PI / 2};
      for (double edge : edges) {
        Translation2d axis = new Translation2d(Math.cos(edge), Math.sin(edge));
        if (most(mine, axis) < least(theirs, axis) || most(theirs, axis) < least(mine, axis)) {
          return false;
        }
      }
      return true;
    }

    /** Whether its whole outline is on the field. */
    boolean onField(FieldModel field) {
      return corners().stream().allMatch(field::contains);
    }
  }

  /** The least of the corners' shadows along a direction. */
  private static double least(List<Translation2d> corners, Translation2d axis) {
    return corners.stream().mapToDouble(corner -> dot(corner, axis)).min().orElseThrow();
  }

  /** The most of the corners' shadows along a direction. */
  private static double most(List<Translation2d> corners, Translation2d axis) {
    return corners.stream().mapToDouble(corner -> dot(corner, axis)).max().orElseThrow();
  }

  private static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  /**
   * How far from the robot's center another robot can stand and still be in the way of a tag a
   * camera sees: the farthest a camera sees a tag, plus how far a camera can be from the robot's
   * center, plus how far the other robot's outline reaches from its own center (both robots the
   * same outline). The pipeline's limit on distance doesn't shorten it: that limit is on the
   * average of an image's tags, so a tag farther than it still counts beside nearer ones.
   */
  static double reachMeters(double sightMeters, Body body) {
    double circumradius =
        Math.hypot(body.bumperLength().in(Meters), body.bumperWidth().in(Meters)) / 2;
    return sightMeters + 2 * circumradius;
  }

  /**
   * Robots placed near this one, anywhere their outlines fit: clear of it, of each other, and of
   * the walls, each at a random place within the reach, evenly over the area, and turned a random
   * way.
   *
   * @param body the robot being designed: the others have its outline
   * @param heightMeters how tall the others are
   * @param count how many to place; fewer if there's no room
   * @param reachMeters how far from the robot's center to place them ({@link #reachMeters})
   * @param standable where a robot's center may be: not in a region the weights leave out, say
   */
  static List<Blocker> around(
      Pose2d robot,
      FieldModel field,
      Body body,
      double heightMeters,
      int count,
      double reachMeters,
      Predicate<Translation2d> standable,
      SplittableRandom random) {
    double halfLength = body.bumperLength().in(Meters) / 2;
    double halfWidth = body.bumperWidth().in(Meters) / 2;
    Blocker self =
        new Blocker(
            robot.getX(),
            robot.getY(),
            robot.getRotation().getRadians(),
            halfLength,
            halfWidth,
            heightMeters);
    List<Blocker> placed = new ArrayList<>();
    for (int n = 0; n < count; n++) {
      for (int attempt = 0; attempt < TRIES; attempt++) {
        double radius = reachMeters * Math.sqrt(random.nextDouble());
        double bearing = 2 * Math.PI * random.nextDouble();
        Blocker other =
            new Blocker(
                robot.getX() + radius * Math.cos(bearing),
                robot.getY() + radius * Math.sin(bearing),
                2 * Math.PI * random.nextDouble(),
                halfLength,
                halfWidth,
                heightMeters);
        if (fits(other, self, placed, field, standable)) {
          placed.add(other);
          break;
        }
      }
    }
    return placed;
  }

  /** Whether a robot here is on the field, may stand here, and is clear of every other robot. */
  private static boolean fits(
      Blocker other,
      Blocker self,
      List<Blocker> placed,
      FieldModel field,
      Predicate<Translation2d> standable) {
    if (!other.onField(field)
        || !standable.test(new Translation2d(other.x(), other.y()))
        || other.overlaps(self)) {
      return false;
    }
    for (Blocker already : placed) {
      if (other.overlaps(already)) {
        return false;
      }
    }
    return true;
  }
}
