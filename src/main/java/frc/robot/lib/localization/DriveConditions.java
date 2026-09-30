package frc.robot.lib.localization;

/**
 * What the robot's other sensors and its state say about trusting its wheels right now. Give it to
 * the estimator once per loop, before that loop's odometry. The estimator weighs each odometry step
 * by the conditions it was taken under and keeps that weight with the step, so a replay of the
 * history weighs every step as it was weighed then.
 *
 * @param tiltRadians how far the robot is tilted from level, from the IMU. On a bump or a ramp its
 *     wheels lose contact, and their travel counts for less.
 * @param collision whether the IMU just felt a jolt beyond anything driving explains: the robot may
 *     have been moved without its wheels seeing it (6800 flagged 20 m/s²)
 * @param tractionLimited whether the wheels are at the edge of their grip: drive motors at their
 *     current limit, or well short of the speed they were commanded (581 and 1678 watched for that
 *     shortfall). Wheels slip here.
 * @param disabled whether the robot is disabled, when people pick it up and carry it where its
 *     wheels can't see: the position's uncertainty grows with time, so the cameras lead until it's
 *     enabled (254 trusted vision loosely while disabled, tightly once enabled)
 */
public record DriveConditions(
    double tiltRadians, boolean collision, boolean tractionLimited, boolean disabled) {
  /** Level, nothing hit, wheels gripping, enabled. */
  public static final DriveConditions NORMAL = new DriveConditions(0, false, false, false);

  public DriveConditions {
    if (!(tiltRadians >= 0) || Double.isInfinite(tiltRadians)) {
      throw new IllegalArgumentException("tilt is an angle from level: " + tiltRadians);
    }
  }
}
