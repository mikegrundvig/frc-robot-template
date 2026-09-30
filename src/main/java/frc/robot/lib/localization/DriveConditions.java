package frc.robot.lib.localization;

/**
 * What the robot's other sensors and its state say about trusting its wheels right now. Give it to
 * the estimator once per loop, before that loop's odometry. The estimator weighs each odometry step
 * by the conditions it was taken under and keeps that weight with the step, so a replay of the
 * history weighs every step as it was weighed then.
 *
 * @param tiltRadians how far the robot is tilted from level, from the IMU. Tilted beyond what the
 *     ground explains, its wheels may have lost contact, and their travel counts for less.
 * @param groundSlopeRadians how much of that tilt the ground under the robot explains: on a known
 *     bump's ramp the wheels still roll on the ground, and their travel counts in full, at the
 *     slope's cosine across the field. Zero on flat ground, or where the terrain isn't known.
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
    double tiltRadians,
    double groundSlopeRadians,
    boolean collision,
    boolean tractionLimited,
    boolean disabled) {
  /** Level, nothing hit, wheels gripping, enabled. */
  public static final DriveConditions NORMAL = new DriveConditions(0, 0, false, false, false);

  public DriveConditions {
    if (!(tiltRadians >= 0) || Double.isInfinite(tiltRadians)) {
      throw new IllegalArgumentException("tilt is an angle from level: " + tiltRadians);
    }
    if (!(groundSlopeRadians >= 0) || !(groundSlopeRadians <= tiltRadians)) {
      throw new IllegalArgumentException(
          "the ground explains some or all of the tilt, never more: " + groundSlopeRadians);
    }
  }

  /** How far the robot tilts beyond what the ground explains. */
  public double unexplainedTiltRadians() {
    return tiltRadians - groundSlopeRadians;
  }
}
