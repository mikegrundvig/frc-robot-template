package com.michaelgrundvig.frc.world.field;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A line on raised ground that tells a robot crossing it where it is: a bump's crest, a ramp's top.
 * The IMU feels the robot go over it, which puts the robot's center on the line at that moment: a
 * landmark across the line, though not along it.
 *
 * @param name for the log: the surface's own, or the surface's and which of its sides
 * @param start one end of the line, in the field frame
 * @param end the other end
 * @param kind what the ground does there
 */
public record Crossing(String name, Translation2d start, Translation2d end, Kind kind) {
  /** What the ground does at a crossing, which is what the IMU feels there. */
  public enum Kind {
    /** It rises to the line and falls away past it: a bump's ridge, climbed either way. */
    CREST,
    /**
     * A slope meets level ground above it: the top of a ramp. The way {@link #across} it is the way
     * up.
     */
    TOP
  }

  public Crossing {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a crossing needs a name");
    }
    if (!(start.getDistance(end) > 0)) {
      throw new IllegalArgumentException(name + ": a crossing runs between two different points");
    }
  }

  /** How long the line is. */
  public double length() {
    return start.getDistance(end);
  }

  /** The way along the line, from its start to its end. */
  public Rotation2d along() {
    return end.minus(start).getAngle().orElseThrow();
  }

  /** The way across the line: a quarter turn left of along it. */
  public Rotation2d across() {
    return along().plus(Rotation2d.CCW_PI_2);
  }

  /** How far a point is from the line, across it: positive on the {@link #across} side. */
  public double acrossOffset(Translation2d point) {
    return point.minus(start).rotateBy(across().unaryMinus()).getX();
  }

  /** How far along the line a point is, from its start. */
  public double alongOffset(Translation2d point) {
    return point.minus(start).rotateBy(along().unaryMinus()).getX();
  }

  /** Whether a point is beside the line, not past either end of it, give or take a margin. */
  public boolean spans(Translation2d point, double marginMeters) {
    double along = alongOffset(point);
    return along >= -marginMeters && along <= length() + marginMeters;
  }

  /** The middle of the line. */
  public Translation2d middle() {
    return start.interpolate(end, 0.5);
  }
}
