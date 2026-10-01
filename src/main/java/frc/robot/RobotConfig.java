package frc.robot;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import frc.robot.lib.wiring.SwerveWiring;
import frc.robot.subsystems.drive.DrivetrainControl;
import java.util.Optional;

/**
 * Everything the code needs to know about one physical robot. Each robot the team builds is one
 * instance in {@link Robots}. Every value is checked when the record is built, so a bad value fails
 * at startup, and before that in the tests, which build every robot.
 *
 * <p>What the robot is, its definition, is read by everything that needs to know the robot:
 * localization, the simulator, the views. The rest is how this program drives it, wires it, and
 * judges what its sensors say.
 *
 * @param definition what the robot is: its body, drivetrain, cameras, and IMUs, and where each is
 *     mounted
 * @param drivetrain how the drivetrain is driven: its limits and gains, tunable live
 * @param drivetrainWiring where the drivetrain's devices are on the CAN bus, and how its encoders
 *     read: empty for a robot that only runs in simulation, whose drivetrain then stays still on
 *     real hardware
 * @param vision how far the cameras' images are trusted, and which are left out: the thresholds and
 *     uncertainties, some tunable live
 * @param estimator how the pose estimator weighs odometry, the gyro, and the cameras
 */
@ConfigRecord
public record RobotConfig(
    RobotDefinition definition,
    DrivetrainControl drivetrain,
    Optional<SwerveWiring> drivetrainWiring,
    VisionConfig vision,
    EstimatorConfig estimator) {
  public RobotConfig {
    Checks.on(RobotConfig.class, definition.name())
        .check(
            "drivetrain.limits.maxLinearSpeed",
            () -> drivetrain.whyTooFastFor(definition.drivetrain()))
        .orThrow();
  }

  /** Starts building a robot's configuration, one named value at a time. */
  public static RobotConfigBuilder.DefinitionStage builder() {
    return RobotConfigBuilder.builder();
  }

  /** Which robot this is, as its logs name it: its definition's name. */
  public String name() {
    return definition.name();
  }
}
