package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * How far a joint can move: its hard stops, in radians for a revolute joint and meters for a
 * prismatic one. Unwrapped, so a turret may span more than a turn: -390° to 40° is a range of its
 * own, not 330° to 40°. Soft limits are how the program drives it, and stay inside this. It knows
 * which it is, angles or lengths, as a position does, so a joint refuses one of the other kind.
 *
 * @param kind angles, for a joint that turns, or lengths, for one that slides
 * @param lower the stop at the low end: radians, or meters
 * @param upper the stop at the high end
 */
@ConfigRecord
public record Range(Position.Kind kind, double lower, double upper) {
  public Range {
    boolean angles = kind == Position.Kind.ANGLE;
    String stops = stop(angles, lower) + " to " + stop(angles, upper);
    if (!Double.isFinite(lower) || !Double.isFinite(upper)) {
      throw new IllegalArgumentException("a range's stops are numbers, not " + stops);
    }
    if (upper == lower) {
      throw new IllegalArgumentException(
          "a range's stops are apart, and " + stops + " is one place: give them both");
    }
    if (upper < lower) {
      throw new IllegalArgumentException(
          "a range goes from its lower stop to its upper, and "
              + stops
              + " is the other way round: give the lower first, "
              + (angles
                  ? "Range.degrees("
                      + shown(Math.toDegrees(upper))
                      + ", "
                      + shown(Math.toDegrees(lower))
                  : "Range.meters(" + shown(upper) + ", " + shown(lower))
              + ")");
    }
  }

  /** A stop as a message shows it: in degrees or meters, or as it is when it isn't a number. */
  private static String stop(boolean angles, double value) {
    return Double.isFinite(value) ? Places.shownAt(angles, value) : value + (angles ? "°" : " m");
  }

  /** Starts describing a range, one named value at a time. */
  public static RangeBuilder.KindStage builder() {
    return RangeBuilder.builder();
  }

  /** A revolute joint's stops, in degrees as CAD gives them. */
  public static Range degrees(double lower, double upper) {
    return builder()
        .kind(Position.Kind.ANGLE)
        .lower(Math.toRadians(lower))
        .upper(Math.toRadians(upper))
        .build();
  }

  /** A prismatic joint's stops, in meters. */
  public static Range meters(double lower, double upper) {
    return builder().kind(Position.Kind.LENGTH).lower(lower).upper(upper).build();
  }

  /** A revolute joint's stops: {@code Range.of(Rotations.of(-0.5), Rotations.of(0.5))}. */
  public static Range of(Angle lower, Angle upper) {
    return builder()
        .kind(Position.Kind.ANGLE)
        .lower(lower.in(Radians))
        .upper(upper.in(Radians))
        .build();
  }

  /** A prismatic joint's stops: {@code Range.of(Inches.of(0), Inches.of(48))}. */
  public static Range of(Distance lower, Distance upper) {
    return builder()
        .kind(Position.Kind.LENGTH)
        .lower(lower.in(Meters))
        .upper(upper.in(Meters))
        .build();
  }

  /** The same stops, in a joint's units: angles for a joint that turns, else lengths. */
  static Range in(Joint joint, double lower, double upper) {
    return builder().kind(Position.Kind.of(joint)).lower(lower).upper(upper).build();
  }

  /** How far it spans, from stop to stop. */
  public double width() {
    return upper - lower;
  }

  /**
   * How far past a stop a position may be and still count as at it: a hair, so a place given in
   * degrees one way ({@code Degrees.of(12)}) lands on a stop given in degrees another way ({@code
   * Range.degrees(12, 90)}) though the two convert a floating-point step apart.
   */
  static final double EDGE = 1e-9;

  /** Whether a position is at or between the stops. */
  public boolean contains(double position) {
    return position >= lower - EDGE && position <= upper + EDGE;
  }
}
