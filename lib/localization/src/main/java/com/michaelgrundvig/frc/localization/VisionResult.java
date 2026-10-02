package com.michaelgrundvig.frc.localization;

import org.jspecify.annotations.Nullable;

/**
 * What the vision pipeline made of one image: a measurement for the estimator, or why not.
 *
 * @param detections the image's tags
 * @param measurement the measurement, or null if the image was thrown out
 * @param refused why the image was thrown out, or null if it became a measurement
 * @param outcome in words, for the log: "heading-constrained: 2 tags at 3.1 m", or the refusal's
 *     words with its numbers ("too far: 5.1 m")
 * @param distanceMeters the tags' average distance from the camera; NaN if not measured
 * @param heightErrorMeters how far the solve missed the tags' known heights, or the floor; NaN if
 *     not measured
 */
public record VisionResult(
    TagDetections detections,
    @Nullable VisionMeasurement measurement,
    @Nullable Verdict refused,
    String outcome,
    double distanceMeters,
    double heightErrorMeters) {
  public VisionResult {
    if ((measurement == null) == (refused == null)) {
      throw new IllegalArgumentException("an image becomes a measurement or is refused, not both");
    }
  }
}
