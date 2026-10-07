package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.Optional;

/**
 * The robot's IMUs, each where it's mounted.
 *
 * @param gyro the gyro's: the Pigeon 2, which turns odometry and owns the heading
 * @param backup the backup IMU's, if the robot has one: Systemcore's own, which stands in while the
 *     gyro is lost, where the board sits and how it's turned; or a second Pigeon. Empty for a robot
 *     with none, a roboRIO's: robot code then reads no backup, and the gyro lost, the heading rests
 *     on the wheels.
 */
@ConfigRecord
public record Imus(ImuMount gyro, Optional<ImuMount> backup) {
  /**
   * Where a new robot starts: the gyro and Systemcore's own IMU, both at the center, level, and
   * neither accelerometer checked.
   */
  public static final Imus UNCHECKED =
      ImusBuilder.builder().gyro(ImuMount.UNCHECKED).backup(ImuMount.UNCHECKED).build();

  /** Starts describing the robot's IMUs, one named value at a time. */
  public static ImusBuilder.GyroStage builder() {
    return ImusBuilder.builder();
  }
}
