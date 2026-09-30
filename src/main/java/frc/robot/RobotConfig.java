package frc.robot;

import frc.robot.lib.config.Body;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.ConfigRecord;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.config.SwerveWiring;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.wpilib.math.geometry.Translation2d;

/**
 * Everything the code needs to know about one physical robot. Each robot the team builds is one
 * instance in {@link Robots}. Every value is checked when the record is built, so a bad value fails
 * at startup, and before that in the tests, which build every robot.
 *
 * @param name which robot this is, as the logs name it
 * @param body frame, bumpers, and mass
 * @param drivetrain the swerve drive
 * @param drivetrainWiring where the drivetrain's devices are on the CAN bus, and how its encoders
 *     read: empty for a robot that only runs in simulation, whose drivetrain then stays still on
 *     real hardware
 * @param cameras the AprilTag cameras
 */
@ConfigRecord
public record RobotConfig(
    String name,
    Body body,
    SwerveConfig drivetrain,
    Optional<SwerveWiring> drivetrainWiring,
    List<CameraConfig> cameras) {
  public RobotConfig {
    if (name.isBlank()) {
      throw new IllegalArgumentException("robot name is required");
    }
    cameras = List.copyOf(cameras);
    drivetrain.moduleTranslations(body); // throws if the wheels don't fit the frame
    Set<String> names = new HashSet<>();
    for (CameraConfig camera : cameras) {
      if (!names.add(camera.name())) {
        throw new IllegalArgumentException(name + ": two cameras are named " + camera.name());
      }
      if (!body.contains(camera.position())) {
        throw new IllegalArgumentException(
            name + ": camera " + camera.name() + " is outside the robot, at " + camera.position());
      }
    }
  }

  /** Starts building a robot's configuration, one named value at a time. */
  public static RobotConfigBuilder.NameStage builder() {
    return RobotConfigBuilder.builder();
  }

  /** Where each wheel touches the floor: front left, front right, back left, back right. */
  public Translation2d[] moduleTranslations() {
    return drivetrain.moduleTranslations(body);
  }
}
