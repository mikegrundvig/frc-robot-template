package com.michaelgrundvig.frc.pitwall.simulator;

import com.michaelgrundvig.frc.pitwall.Limits;
import org.jspecify.annotations.Nullable;

/**
 * One number a fault takes, such as how far a camera is knocked: what it's called, its unit, the
 * range the page may set it in, and what it is unless the page or a scenario says otherwise.
 *
 * @param name what it's called on the page, such as {@code "Up"}
 * @param unit its unit, such as {@code "in"}; empty for a plain number, such as a scale
 * @param limits the range it may be set in, both ends included: a range the page can draw
 * @param initial what it is when not given: often the robot as built, so the page starts from what
 *     the robot has
 */
public record Parameter(String name, String unit, Limits limits, double initial) {
  public Parameter {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a fault's parameter needs a name");
    }
    if (!Double.isFinite(limits.min()) || !Double.isFinite(limits.max())) {
      throw new IllegalArgumentException(
          name + " needs a range with both ends, for the page to draw: " + limits);
    }
    String outside = limits.whyNot(initial);
    if (outside != null) {
      throw new IllegalArgumentException(name + " starts outside its range: " + outside);
    }
  }

  /**
   * Why a value can't be given to this parameter: outside its range, or not a number.
   *
   * @return null when it can
   */
  public @Nullable String whyNot(double value) {
    if (!Double.isFinite(value)) {
      return name + " must be a number";
    }
    String outside = limits.whyNot(value);
    return outside == null ? null : name + ": " + outside;
  }
}
