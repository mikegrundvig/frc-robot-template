package com.michaelgrundvig.frc.localization;

/**
 * What the estimator did with a landmark, and why: logged like a camera measurement's decision.
 *
 * @param landmark the landmark
 * @param verdict whether it moved the estimate, or why not: accepted, gated, or outside the history
 * @param innovationMeters how far across the line the estimate had the robot at the time; NaN if it
 *     was rejected before being compared
 * @param chiSquared the squared Mahalanobis distance; NaN if not compared
 * @param threshold the gate its χ² was held to; NaN if not compared
 */
public record LandmarkDecision(
    LandmarkEvent landmark,
    Verdict verdict,
    double innovationMeters,
    double chiSquared,
    double threshold) {
  /** Whether it moved the estimate. */
  public boolean accepted() {
    return verdict == Verdict.ACCEPTED;
  }

  /**
   * Why, in words, as the log has them: "accepted", "gate: 14.2 > 10.8", "too late: older than the
   * history kept".
   */
  public String reason() {
    return verdict.describe(chiSquared, threshold);
  }
}
