package frc.robot.lib.localization;

import org.jspecify.annotations.Nullable;

/**
 * What the vision pipeline made of one image: a measurement for the estimator, or why not.
 *
 * @param detections the image's tags
 * @param measurement the measurement, or null if the image was thrown out
 * @param outcome in words: "heading-constrained: 2 tags at 3.1 m", or why it was thrown out
 * @param distanceMeters the tags' average distance from the camera; NaN if not measured
 * @param heightErrorMeters how far the solve missed the tags' known heights, or the floor; NaN if
 *     not measured
 */
public record VisionResult(
    TagDetections detections,
    @Nullable VisionMeasurement measurement,
    String outcome,
    double distanceMeters,
    double heightErrorMeters) {}
