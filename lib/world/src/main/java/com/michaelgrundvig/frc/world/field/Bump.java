package com.michaelgrundvig.frc.world.field;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A bump robots drive over: a ridge along a straight crest, with ramps down to the floor on both
 * sides (REBUILT's were 6.5 in tall, with 15° ramps). Crossing one puts the robot's center on the
 * crest line at the top: a landmark across the bump, though not along it.
 *
 * @param name for the log
 * @param crestStart one end of the crest line, in the field frame
 * @param crestEnd the other end
 * @param heightMeters how high the crest stands above the floor
 * @param depthMeters from the foot of one ramp to the foot of the other, across the crest
 */
public record Bump(
    String name,
    Translation2d crestStart,
    Translation2d crestEnd,
    double heightMeters,
    double depthMeters) {
  public Bump {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a bump needs a name");
    }
    if (!(crestStart.getDistance(crestEnd) > 0)) {
      throw new IllegalArgumentException(name + ": a crest runs between two different points");
    }
    if (!(heightMeters > 0) || !(depthMeters > 0) || Double.isInfinite(depthMeters)) {
      throw new IllegalArgumentException(name + ": a bump's height and depth must be positive");
    }
  }

  /** How long the crest is. */
  public double length() {
    return crestStart.getDistance(crestEnd);
  }

  /** The way along the crest, from its start to its end. */
  public Rotation2d along() {
    return crestEnd.minus(crestStart).getAngle().orElseThrow();
  }

  /** The way across the crest: its crossing axis, a quarter turn left of along it. */
  public Rotation2d across() {
    return along().plus(Rotation2d.CCW_PI_2);
  }

  /** How far a point is from the crest line, across it: positive on the {@link #across} side. */
  public double acrossOffset(Translation2d point) {
    return point.minus(crestStart).rotateBy(across().unaryMinus()).getX();
  }

  /** How far along the crest a point is, from its start. */
  public double alongOffset(Translation2d point) {
    return point.minus(crestStart).rotateBy(along().unaryMinus()).getX();
  }

  /** Whether a point is beside the crest, not past either end of it, give or take a margin. */
  public boolean spans(Translation2d point, double marginMeters) {
    double along = alongOffset(point);
    return along >= -marginMeters && along <= length() + marginMeters;
  }

  /** Whether a point is on the bump, or within a margin of its footprint. */
  public boolean covers(Translation2d point, double marginMeters) {
    return spans(point, marginMeters)
        && Math.abs(acrossOffset(point)) <= depthMeters / 2 + marginMeters;
  }

  /** How steep the ramps are. */
  public double slopeRadians() {
    return Math.atan2(heightMeters, depthMeters / 2);
  }

  /** How high the bump is at a point: the crest's height, down the ramps to the floor. */
  public double heightAt(Translation2d point) {
    if (!spans(point, 0)) {
      return 0;
    }
    return heightMeters * Math.max(0, 1 - Math.abs(acrossOffset(point)) / (depthMeters / 2));
  }

  /** The same bump under a symmetry: its partner on the other alliance's half. */
  Bump image(String imageName, Symmetry symmetry, double fieldLength, double fieldWidth) {
    return new Bump(
        imageName,
        symmetry.apply(crestStart, fieldLength, fieldWidth),
        symmetry.apply(crestEnd, fieldLength, fieldWidth),
        heightMeters,
        depthMeters);
  }
}
