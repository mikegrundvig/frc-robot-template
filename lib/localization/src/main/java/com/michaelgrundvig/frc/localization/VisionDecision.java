package com.michaelgrundvig.frc.localization;

/**
 * What the estimator did with a camera measurement, and why: every one is logged, accepted or not,
 * and the innovations of the accepted ones are the data that tunes each camera's trust.
 *
 * @param measurement the measurement
 * @param accepted whether it moved the estimate
 * @param reason why, in words: "accepted", "gate: 45.2 > 13.8", "off the field"
 * @param innovationMeters how far it was from the estimate at its capture time; NaN if it was
 *     rejected before being compared
 * @param chiSquared its squared Mahalanobis distance from the estimate; NaN if not compared
 */
public record VisionDecision(
    VisionMeasurement measurement,
    boolean accepted,
    String reason,
    double innovationMeters,
    double chiSquared) {}
