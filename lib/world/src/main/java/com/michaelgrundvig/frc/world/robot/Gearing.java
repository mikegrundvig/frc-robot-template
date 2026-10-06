package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.atMost;
import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;

/**
 * What stands between a motor and what it turns: a reduction, and what the reduction loses. The
 * motor's speed divides by the ratio on the way through, and its torque multiplies by the ratio and
 * by the efficiency.
 *
 * @param ratio the motor's turns for each turn of what it drives: 6.75 for a 6.75:1 reduction, less
 *     than 1 for a step-up
 * @param efficiency the share of the motor's torque, times the ratio, that reaches what it drives:
 *     above 0, and at most 1 for a gearing that loses nothing. A gear stage loses a few percent; a
 *     belt or a chain, a little less. Until a mechanism's losses are measured, 1 says none are
 *     modeled.
 */
@ConfigRecord
public record Gearing(double ratio, double efficiency) {
  public Gearing {
    Checks.on(Gearing.class)
        .field("ratio", ratio, positive())
        .field("efficiency", efficiency, positive())
        .field("efficiency", efficiency, atMost(1.0))
        .orThrow();
  }

  /** Starts describing a gearing, one named value at a time. */
  public static GearingBuilder.RatioStage builder() {
    return GearingBuilder.builder();
  }

  /** A reduction whose losses aren't measured yet: {@code Gearing.of(25)}, losing nothing. */
  public static Gearing of(double ratio) {
    return builder().ratio(ratio).efficiency(1.0).build();
  }
}
