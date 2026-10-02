package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
 * @param name which robot this is, as its logs and its checks' messages name it
 * @param body the frame, bumpers, and mass
 * @param drivetrain the swerve drivetrain, as built
 * @param cameras the AprilTag cameras
 * @param imus where each IMU is mounted, and whether its accelerometer has been checked to read
 *     along the robot's axes
 */
@ConfigRecord
public record RobotDefinition(
    String name, Body body, DrivetrainGeometry drivetrain, List<CameraConfig> cameras, Imus imus) {
  public RobotDefinition {
    Checks.on(RobotDefinition.class, name)
        .field("name", name, notBlank())
        .field("cameras", cameras, RobotDefinition::sharedNames)
        .check("drivetrain.wheelInset", () -> drivetrain.whyWheelsDontFit(body))
        .check("cameras", () -> outside(cameras, body))
        .check("imus", () -> outside(imus, body))
        .orThrow();
  }

  /** Starts describing a robot, one named value at a time. */
  public static RobotDefinitionBuilder.NameStage builder() {
    return RobotDefinitionBuilder.builder();
  }

  /** Where each wheel touches the floor: front left, front right, back left, back right. */
  public Translation2d[] moduleTranslations() {
    return drivetrain.moduleTranslations(body);
  }

  /**
   * The names two cameras share, or null if none do: as written, or as ids ({@link
   * CameraConfig#id()}), where "Front Left" and "front-left" are one.
   */
  private static @Nullable String sharedNames(List<CameraConfig> cameras) {
    Map<String, String> names = new HashMap<>();
    List<String> shared = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      String before = names.putIfAbsent(camera.id(), camera.name());
      if (before == null) {
        continue;
      }
      shared.add(
          before.equals(camera.name())
              ? "two cameras are named " + camera.name()
              : "cameras \""
                  + before
                  + "\" and \""
                  + camera.name()
                  + "\" differ only in case or punctuation, so they're one id, \""
                  + camera.id()
                  + "\": name them apart");
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
