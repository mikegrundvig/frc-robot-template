package frc.robot.lib.robot;

import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;

/**
 * What one robot is: its body, its drivetrain, and where its cameras and IMUs are mounted. Every
 * reader of the robot reads this one description: localization (the wheels' positions, the mounts),
 * the simulator (all of it), and the views. How the robot is driven (gains, limits) and how it's
 * wired (CAN IDs) aren't here: they're the robot program's configuration around it.
 *
 * <p>Each part checks its own values when built; the checks here are those across parts, such as a
 * camera mounted inside the body. Mechanisms' joints, with what drives each, join the definition
 * with the first mechanism that describes one.
 *
 * @param body the frame, bumpers, and mass
 * @param drivetrain the swerve drivetrain, as built
 * @param cameras the AprilTag cameras
 * @param imus where each IMU is mounted, and whether its accelerometer has been checked to read
 *     along the robot's axes
 */
@ConfigRecord
public record RobotDefinition(
    Body body, DrivetrainGeometry drivetrain, List<CameraConfig> cameras, Imus imus) {
  public RobotDefinition {
    Checks.on(RobotDefinition.class)
        .field("cameras", cameras, RobotDefinition::sharedNames)
        .check("drivetrain.wheelInset", () -> drivetrain.whyWheelsDontFit(body))
        .check("cameras", () -> outside(cameras, body))
        .check("imus", () -> outside(imus, body))
        .orThrow();
  }

  /** Starts describing a robot, one named value at a time. */
  public static RobotDefinitionBuilder.BodyStage builder() {
    return RobotDefinitionBuilder.builder();
  }

  /** Where each wheel touches the floor: front left, front right, back left, back right. */
  public Translation2d[] moduleTranslations() {
    return drivetrain.moduleTranslations(body);
  }

  /** The names two cameras share, or null if none do. */
  private static @Nullable String sharedNames(List<CameraConfig> cameras) {
    Set<String> names = new HashSet<>();
    List<String> shared = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      if (!names.add(camera.name())) {
        shared.add("two cameras are named " + camera.name());
      }
    }
    return shared.isEmpty() ? null : String.join("; ", shared);
  }

  /** The cameras mounted outside the robot, or null if none are. */
  private static @Nullable String outside(List<CameraConfig> cameras, Body body) {
    List<String> outside = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      if (!body.contains(camera.position())) {
        outside.add("camera " + camera.name() + " is outside the robot, at " + camera.position());
      }
    }
    return outside.isEmpty() ? null : String.join("; ", outside);
  }

  /** The IMUs mounted outside the robot, or null if none are. */
  private static @Nullable String outside(Imus imus, Body body) {
    List<String> outside = new ArrayList<>();
    outside(outside, "the gyro", imus.gyro(), body);
    outside(outside, "the backup IMU", imus.backup(), body);
    return outside.isEmpty() ? null : String.join("; ", outside);
  }

  private static void outside(List<String> outside, String name, ImuMount imu, Body body) {
    if (!body.contains(imu.position())) {
      outside.add(name + " is outside the robot, at " + imu.position());
    }
  }
}
