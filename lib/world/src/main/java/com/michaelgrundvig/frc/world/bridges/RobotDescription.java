package com.michaelgrundvig.frc.world.bridges;

import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
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
 * @param imu where the gyro's IMU sits, from the robot's origin, as its mount gives it; empty for a
 *     robot whose catalog doesn't give it
 */
public record RobotDescription(
    String name,
    Body body,
    DrivetrainGeometry drivetrain,
    List<CameraConfig> cameras,
    Optional<Translation3d> imu) {
  public RobotDescription {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a robot needs a name");
    }
    cameras = List.copyOf(cameras);
  }
}
