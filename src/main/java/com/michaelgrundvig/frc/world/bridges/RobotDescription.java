package com.michaelgrundvig.frc.world.bridges;

import frc.robot.lib.config.Body;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.SwerveConfig;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Translation3d;

/**
 * A robot as the tool writers read it: the parts of its configuration that another tool can show.
 * Each part is the configuration record itself, already checked when it was built, so a writer
 * never sees a value the robot's checks would refuse.
 *
 * @param name the robot's name, as its configuration gives it: the writers name its folders and
 *     models for it
 * @param body its frame, bumpers, and height
 * @param drivetrain its modules, which (with the body) place the wheels
 * @param cameras each camera's mount and sensor
 * @param imu where the gyro's IMU sits, from the robot's origin, when its mount is known in three
 *     dimensions; empty when it isn't
 */
public record RobotDescription(
    String name,
    Body body,
    SwerveConfig drivetrain,
    List<CameraConfig> cameras,
    Optional<Translation3d> imu) {
  public RobotDescription {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a robot needs a name");
    }
    cameras = List.copyOf(cameras);
  }
}
