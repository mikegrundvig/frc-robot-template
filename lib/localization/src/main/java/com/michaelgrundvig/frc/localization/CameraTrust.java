package com.michaelgrundvig.frc.localization;

/**
 * How far localization trusts one camera, beyond its configuration, as of an update.
 *
 * @param mountDoubtMeters how far the camera's measurements may be off from where its mount moved
 *     since it was calibrated ({@link MountCheck#doubtMeters}), as the robot last said: every image
 *     it takes is widened by this. Zero for a camera that sits where its configuration says
 * @param mountDistanceMeters how far its tags were, on average, when that was judged; NaN until
 *     then
 * @param keptApart whether the estimator keeps this camera's repeated errors (its turn and shift on
 *     its mount) apart from the others'; a camera beyond the estimator's places ({@value
 *     PoseFilter#MAX_CAMERAS}) counts each image's errors as new, so its images, averaged, grow
 *     surer than they should
 */
public record CameraTrust(double mountDoubtMeters, double mountDistanceMeters, boolean keptApart) {}
