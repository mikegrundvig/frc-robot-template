package com.michaelgrundvig.frc.localization;

/**
 * What the estimator did with a landmark, and why: logged like a camera measurement's decision.
 *
 * @param landmark the landmark
 * @param accepted whether it moved the estimate
 * @param reason why, in words: "accepted", "gate: 14.2 > 10.8", "too late: older than the history
 *     kept"
 * @param innovationMeters how far across the line the estimate had the robot at the time; NaN if it
 *     was rejected before being compared
 * @param chiSquared the squared Mahalanobis distance; NaN if not compared
 */
public record LandmarkDecision(
    LandmarkEvent landmark,
    boolean accepted,
    String reason,
    double innovationMeters,
    double chiSquared) {}
