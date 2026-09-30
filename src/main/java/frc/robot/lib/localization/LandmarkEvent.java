package frc.robot.lib.localization;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A line on the field the robot is known to have been on, and when: it says where the robot was
 * across the line, and nothing about where along it. Crossing a bump's crest puts the robot's
 * center on the crest line.
 *
 * @param timestampNanos when the robot was on the line; the estimator applies it then
 * @param name what the robot crossed, for the log
 * @param point a point on the line
 * @param across the direction across the line
 * @param stdDevMeters how sure it is, across the line
 */
public record LandmarkEvent(
    long timestampNanos, String name, Translation2d point, Rotation2d across, double stdDevMeters) {
  public LandmarkEvent {
    if (timestampNanos < 0) {
      throw new IllegalArgumentException("a time on the robot's clock: " + timestampNanos);
    }
    if (name.isBlank()) {
      throw new IllegalArgumentException("a landmark needs a name");
    }
    if (!(stdDevMeters > 0) || Double.isInfinite(stdDevMeters)) {
      throw new IllegalArgumentException(name + ": a landmark's spread must be positive");
    }
  }

  /** How far a point is from the line, across it: positive on the {@link #across} side. */
  public double offset(Translation2d position) {
    return position.minus(point).rotateBy(across.unaryMinus()).getX();
  }
}
