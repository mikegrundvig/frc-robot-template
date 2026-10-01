package frc.robot.lib.localization;

import static frc.robot.lib.config.Rules.finite;
import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Distance;

/**
 * Where the robot's IMUs sit, and whether the gyro's accelerometer is known to read along the
 * robot's axes. Localization checks the wheels against what the IMU feels: an IMU off the robot's
 * center is carried around it as the robot turns, and feels that too, so the wheels' velocity is
 * compared where the IMU sits. The drivetrain's IO turns each IMU's own axes into the robot's (a
 * Pigeon 2 by the mount pose Phoenix Tuner X set, Systemcore by its configured yaw); whether that
 * came out right only driving the robot shows, so the accelerometer's part of the check waits for
 * the robot to pass the step that shows it.
 *
 * <p>Positions are from the robot's center, as a camera's are.
 *
 * @param gyroForward how far ahead of the center the gyro's IMU sits (negative: behind it)
 * @param gyroLeft how far left of the center it sits (negative: to the right)
 * @param backupForward the same for the backup IMU (Systemcore's), which stands in while the gyro
 *     is lost
 * @param backupLeft and how far left
 * @param accelerometerChecked whether this robot has passed the accelerometer check
 *     (docs/calibration.md): driving forward reads forward, strafing left reads left, and level and
 *     still reads near zero. Until it has, the slip check doesn't compare the wheels' speed with
 *     the accelerometer (an IMU turned on its mount would read every change of speed as a slide),
 *     and a jolt is judged by its size alone. Comparing the wheels' turn with the gyro's needs no
 *     check.
 */
@ConfigRecord
public record ImuPlacement(
    Distance gyroForward,
    Distance gyroLeft,
    Distance backupForward,
    Distance backupLeft,
    boolean accelerometerChecked) {
  /**
   * Where a new robot starts: both IMUs taken to sit at the center, and the accelerometer not yet
   * checked.
   */
  public static final ImuPlacement UNCHECKED =
      ImuPlacementBuilder.builder()
          .gyroForward(Meters.of(0))
          .gyroLeft(Meters.of(0))
          .backupForward(Meters.of(0))
          .backupLeft(Meters.of(0))
          .accelerometerChecked(false)
          .build();

  public ImuPlacement {
    Checks.on(ImuPlacement.class)
        .field("gyroForward", gyroForward, finite())
        .field("gyroLeft", gyroLeft, finite())
        .field("backupForward", backupForward, finite())
        .field("backupLeft", backupLeft, finite())
        .orThrow();
  }

  /** Starts describing where the IMUs sit, one named value at a time. */
  public static ImuPlacementBuilder.GyroForwardStage builder() {
    return ImuPlacementBuilder.builder();
  }

  /** Where the gyro's IMU sits, from the robot's center, in meters. */
  public Translation2d gyro() {
    return new Translation2d(gyroForward, gyroLeft);
  }

  /** Where the backup IMU sits, from the robot's center, in meters. */
  public Translation2d backup() {
    return new Translation2d(backupForward, backupLeft);
  }
}
