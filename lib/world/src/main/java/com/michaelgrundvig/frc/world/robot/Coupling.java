package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;

/**
 * One joint that moves as a function of another: a cascade elevator's stages, a continuous
 * elevator's stages that wait for the carriage, a hopper a linkage moves as the intake turns. The
 * follower has no motor or sensor of its own to say where it is; its position comes from the
 * leader's, through the curve CAD gives. Robot code needs no linkage solver: the curve is solved
 * once, from CAD, into a table.
 *
 * <p>A joint follows at most one other, and none follows itself, however far round.
 */
public sealed interface Coupling permits Coupling.Linear, Coupling.Curve {
  /** The joint that moves on its own: driven, or sensed. */
  Joint leader();

  /** The joint that follows it. */
  Joint follower();

  /** Where the follower is, with the leader at a position. */
  double followerAt(double leaderPosition);

  /** How fast the follower moves for the leader's speed, at the leader's position. */
  double slopeAt(double leaderPosition);

  /**
   * The follower a fixed ratio of the leader, plus an offset: a cascade elevator's stage, moving a
   * third of the carriage's travel.
   *
   * @param leader the joint that moves on its own
   * @param follower the joint that follows it
   * @param ratio the follower's travel for each of the leader's, in their own units
   * @param offset where the follower is with the leader at 0
   */
  @ConfigRecord
  record Linear(Joint leader, Joint follower, double ratio, double offset) implements Coupling {
    public Linear {
      Checks.on(Linear.class, follower.label())
          .field("ratio", ratio, finite())
          .field("offset", offset, finite())
          .check("follower", () -> notItself(leader, follower))
          .orThrow();
    }

    /** Starts describing a linear coupling, one named value at a time. */
    public static CouplingLinearBuilder.LeaderStage builder() {
      return CouplingLinearBuilder.builder();
    }

    @Override
    public double followerAt(double leaderPosition) {
      return ratio * leaderPosition + offset;
    }

    @Override
    public double slopeAt(double leaderPosition) {
      return ratio;
    }
  }

  /**
   * The follower along a curve of the leader, straight between points from CAD: a continuous
   * elevator's stage, still until the carriage reaches it and then moving with it; a linkage. Past
   * the table's ends the follower stays where the end puts it.
   *
   * @param leader the joint that moves on its own
   * @param follower the joint that follows it
   * @param table the leader's position and the follower's there, in their own units, the leader's
   *     rising from point to point
   */
  @ConfigRecord
  record Curve(Joint leader, Joint follower, List<Translation2d> table) implements Coupling {
    public Curve {
      table = List.copyOf(table);
      Checks.on(Curve.class, follower.label())
          .field("table", table, Curve::notACurve)
          .check("follower", () -> notItself(leader, follower))
          .orThrow();
    }

    /** Starts describing a coupling by a table, one named value at a time. */
    public static CouplingCurveBuilder.LeaderStage builder() {
      return CouplingCurveBuilder.builder();
    }

    @Override
    public double followerAt(double leaderPosition) {
      if (leaderPosition <= table.get(0).getX()) {
        return table.get(0).getY();
      }
      for (int i = 1; i < table.size(); i++) {
        Translation2d from = table.get(i - 1);
        Translation2d to = table.get(i);
        if (leaderPosition <= to.getX()) {
          double along = (leaderPosition - from.getX()) / (to.getX() - from.getX());
          return from.getY() + along * (to.getY() - from.getY());
        }
      }
      return table.get(table.size() - 1).getY();
    }

    /** The slope of the segment the leader's position is on; zero past the ends. */
    @Override
    public double slopeAt(double leaderPosition) {
      for (int i = 1; i < table.size(); i++) {
        Translation2d from = table.get(i - 1);
        Translation2d to = table.get(i);
        if (leaderPosition >= from.getX() && leaderPosition < to.getX()) {
          return (to.getY() - from.getY()) / (to.getX() - from.getX());
        }
      }
      return 0;
    }

    private static @Nullable String notACurve(List<Translation2d> table) {
      if (table.size() < 2) {
        return "needs two points or more, not " + table.size();
      }
      for (int i = 0; i < table.size(); i++) {
        Translation2d point = table.get(i);
        if (!Double.isFinite(point.getX() + point.getY())) {
          return "point " + i + " isn't a number: " + point;
        }
        if (i > 0 && !(point.getX() > table.get(i - 1).getX())) {
          return "the leader's positions must rise from point to point, and point "
              + i
              + "'s doesn't";
        }
      }
      return null;
    }
  }

  private static @Nullable String notItself(Joint leader, Joint follower) {
    return leader.equals(follower) ? "a joint can't follow itself" : null;
  }
}
