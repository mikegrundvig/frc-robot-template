package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.shown;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;

/**
 * How far a joint can move: its hard stops, in radians for a revolute joint and meters for a
 * prismatic one. Unwrapped, so a turret may span more than a turn: -390° to 40° is a range of its
 * own, not 330° to 40°. Soft limits are how the program drives it, and stay inside this.
 *
 * @param lower the stop at the low end: radians, or meters
 * @param upper the stop at the high end
 */
@ConfigRecord
public record Range(double lower, double upper) {
  public Range {
    Checks.on(Range.class)
        .field("lower", lower, finite())
        .field("upper", upper, finite())
        .check(
            "upper",
            () ->
                upper > lower
                    ? null
                    : shown(upper) + " must be above the lower stop (" + shown(lower) + ")")
        .orThrow();
  }

  /** Starts describing a range, one named value at a time. */
  public static RangeBuilder.LowerStage builder() {
    return RangeBuilder.builder();
  }

  /** A revolute joint's stops, in degrees as CAD gives them. */
  public static Range degrees(double lower, double upper) {
    return builder().lower(Math.toRadians(lower)).upper(Math.toRadians(upper)).build();
  }

  /** A prismatic joint's stops, in meters. */
  public static Range meters(double lower, double upper) {
    return builder().lower(lower).upper(upper).build();
  }

  /** How far it spans, from stop to stop. */
  public double width() {
    return upper - lower;
  }

  /** Whether a position is at or between the stops. */
  public boolean contains(double position) {
    return position >= lower && position <= upper;
  }
}
