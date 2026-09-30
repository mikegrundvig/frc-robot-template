package frc.robot.lib.localization;

import frc.robot.lib.config.ConfigRecord;
import java.util.List;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * What the drivetrain read over one robot loop, for localization: every reading of its wheels since
 * the last loop, its IMUs, how it's moving, and whether its wheels are at the edge of their grip.
 * Any drivetrain that reports these can be localized, whatever its wheels: this is all the
 * localization needs from it. Built by name, so the gyro and the backup IMU can't trade places.
 *
 * @param wheels every reading of the wheels since the last loop, oldest first
 * @param gyro the main gyro, which owns the robot's heading
 * @param backupImu a second IMU (Systemcore's own) that stands in while the gyro is lost, or {@link
 *     ImuReadings#NONE}
 * @param velocities how the robot is moving, robot-relative, as its wheels measure it
 * @param tractionLimited whether the wheels are at the edge of their grip: motors at their current
 *     limit, or wheels well short of the speed they were sent. Wheels slip here.
 */
@ConfigRecord
public record DrivetrainReadings(
    List<OdometrySample> wheels,
    ImuReadings gyro,
    ImuReadings backupImu,
    ChassisVelocities velocities,
    boolean tractionLimited) {
  public static DrivetrainReadingsBuilder.WheelsStage builder() {
    return DrivetrainReadingsBuilder.builder();
  }
}
