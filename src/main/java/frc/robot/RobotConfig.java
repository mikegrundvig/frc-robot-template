package frc.robot;

import static frc.robot.lib.config.Rules.notBlank;

import frc.robot.lib.config.Body;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.config.SwerveWiring;
import frc.robot.lib.localization.EstimatorConfig;
import frc.robot.lib.localization.ImuPlacement;
import frc.robot.lib.localization.VisionConfig;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
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
 * @param vision how far the cameras' images are trusted, and which are left out: the thresholds and
 *     uncertainties, some tunable live
 * @param estimator how the pose estimator weighs odometry, the gyro, and the cameras
 * @param imus where the IMUs sit, and whether the gyro's accelerometer has been checked to read
 *     along the robot's axes: localization compares the wheels with what they feel. Here rather
 *     than in the wiring, since a robot running in the simulator needs it too.
 */
@ConfigRecord
public record RobotConfig(
    String name,
    Body body,
    SwerveConfig drivetrain,
    Optional<SwerveWiring> drivetrainWiring,
    List<CameraConfig> cameras,
    VisionConfig vision,
    EstimatorConfig estimator,
    ImuPlacement imus) {
  public RobotConfig {
    Checks.on(RobotConfig.class, name)
        .field("name", name, notBlank())
        .field("cameras", cameras, RobotConfig::sharedNames)
        .check("drivetrain.wheelInset", () -> drivetrain.whyWheelsDontFit(body))
        .check("cameras", () -> outside(cameras, body))
        .orThrow();
  }

  /** Starts building a robot's configuration, one named value at a time. */
  public static RobotConfigBuilder.NameStage builder() {
    return RobotConfigBuilder.builder();
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
}
