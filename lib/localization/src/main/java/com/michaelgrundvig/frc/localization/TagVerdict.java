package com.michaelgrundvig.frc.localization;

/**
 * What became of one tag in a camera's image: the verdict on the measurement it went into, with
 * that measurement's numbers. An image's tags are weighed together today, as one measurement, so
 * every tag of a frame carries the frame's verdict; each is its own record so a check of each tag
 * on its own has a place to say what it found.
 *
 * @param tag the tag's number
 * @param verdict whether the measurement moved the estimate, or why not
 * @param innovationMeters how far the measurement was from the estimate at its capture time; NaN if
 *     it was refused before being compared
 * @param chiSquared its squared Mahalanobis distance from the estimate; NaN if not compared
 * @param threshold the gate its χ² was held to; NaN if not compared
 */
public record TagVerdict(
    int tag, Verdict verdict, double innovationMeters, double chiSquared, double threshold) {
  /** Whether it moved the estimate. */
  public boolean accepted() {
    return verdict == Verdict.ACCEPTED;
  }

  /** Why, in words with its numbers, as the log has them: "gate: 45.2 > 13.8". */
  public String reason() {
    return verdict.describe(chiSquared, threshold);
  }
}
