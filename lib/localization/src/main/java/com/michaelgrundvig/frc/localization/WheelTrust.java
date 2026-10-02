package com.michaelgrundvig.frc.localization;

/**
 * How far localization trusts the wheels this loop, and why: what the IMU that turns odometry says
 * of them, through the {@link SlipCheck}. The slip check judges the wheels together, as they slide
 * together over the floor (a wheel slipping against the others shows in the kinematics' residual
 * instead, {@link BodyMotion#residualMeters}), so this is one record for the whole drivetrain.
 *
 * @param conditions what the estimator weighs each wheel step by: the robot's tilt, the ground's
 *     slope under it, a jolt's share of a collision, how fast the wheels may be sliding, and
 *     whether the robot is disabled
 * @param turnSlipMetersPerSecond how fast the wheels may be sliding, as the gyro sees them turning
 * @param speedSlipMetersPerSecond how fast they may be sliding, as the accelerometer sees their
 *     speed
 * @param turnApartRadians how far the wheels turned the robot beyond the gyro over the slip check's
 *     window, either way
 * @param slideMetersPerSecond how much faster or slower than the robot the wheels roll, as the
 *     accelerometer reckons it
 * @param unexplainedMetersPerSecSq what the IMU felt that the wheels' own change of velocity
 *     doesn't explain
 * @param accelerometerBiasMetersPerSecSq the accelerometer's bias, learned while the robot stands
 *     still
 * @param accelerometerLeaksGravity whether the accelerometer reads gravity's share standing still,
 *     past any bias a part has: the IMU's orientation in the robot's definition isn't how it's
 *     mounted
 */
public record WheelTrust(
    DriveConditions conditions,
    double turnSlipMetersPerSecond,
    double speedSlipMetersPerSecond,
    double turnApartRadians,
    double slideMetersPerSecond,
    double unexplainedMetersPerSecSq,
    double accelerometerBiasMetersPerSecSq,
    boolean accelerometerLeaksGravity) {}
