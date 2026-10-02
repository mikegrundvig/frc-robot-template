package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.config.ConfigRecord;

/**
 * The robot's IMUs, each where it's mounted.
 *
 * @param gyro the gyro's: the Pigeon 2, which turns odometry and owns the heading
 * @param backup the backup IMU's: Systemcore's own, which stands in while the gyro is lost, where
 *     the board sits and how it's turned
 */
@ConfigRecord
public record Imus(ImuMount gyro, ImuMount backup) {
  /** Where a new robot starts: both at the center, level, and neither accelerometer checked. */
  public static final Imus UNCHECKED =
      ImusBuilder.builder().gyro(ImuMount.UNCHECKED).backup(ImuMount.UNCHECKED).build();

  /** Starts describing the robot's IMUs, one named value at a time. */
  public static ImusBuilder.GyroStage builder() {
    return ImusBuilder.builder();
  }
}
