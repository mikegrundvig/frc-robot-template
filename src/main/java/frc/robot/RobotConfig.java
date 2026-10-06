package frc.robot;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import frc.robot.lib.wiring.CanIds;
import frc.robot.lib.wiring.MechanismWiring;
import frc.robot.lib.wiring.SensorWiring;
import frc.robot.lib.wiring.SwerveWiring;
import frc.robot.subsystems.drive.DrivetrainControl;
import frc.robot.subsystems.mechanisms.JointSetup;
import frc.robot.subsystems.mechanisms.Joints;
import frc.robot.subsystems.mechanisms.Sensors;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
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
 * @param joints each joint its motors drive, as this program drives it: its control, and its wiring
 *     on a robot. One for every such joint in the definition, from which the template builds its
 *     mechanism; none for the drivetrain's, which is driven above
 * @param sensors where each of the definition's switches and rangefinders is connected, on a robot:
 *     none for one that only runs in the simulator
 */
@ConfigRecord
public record RobotConfig(
    RobotDefinition definition,
    DrivetrainControl drivetrain,
    Optional<SwerveWiring> drivetrainWiring,
    VisionConfig vision,
    EstimatorConfig estimator,
    List<JointSetup> joints,
    List<SensorWiring> sensors) {
  public RobotConfig {
    joints = List.copyOf(joints);
    sensors = List.copyOf(sensors);
    List<JointSetup> setups = joints;
    List<SensorWiring> sensorWiring = sensors;
    Checks.on(RobotConfig.class, definition.name())
        .check(
            "drivetrain.limits.maxLinearSpeed",
            () -> drivetrain.whyTooFastFor(definition.drivetrain()))
        .check(
            "drivetrainWiring",
            () ->
                drivetrainWiring
                    .map(wiring -> DrivetrainControl.whyNotDriven(definition.drivetrain(), wiring))
                    .orElse(null))
        .check("joints", () -> JointSetup.problems(definition, setups))
        .check("joints", () -> CanIds.twice(drivetrainWiring, wired(setups)))
        .check("sensors", () -> CanIds.sensorsTwice(drivetrainWiring, wired(setups), sensorWiring))
        .check("sensors", () -> Sensors.wiringProblems(definition, wired(setups), sensorWiring))
        .check("definition", () -> LocalizationService.problems(definition))
        .check("definition.interlocks", () -> Joints.unenforceable(definition))
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

  /** Each wired joint's wiring, by its label. */
  private static Map<String, MechanismWiring> wired(List<JointSetup> setups) {
    Map<String, MechanismWiring> wired = new LinkedHashMap<>();
    for (JointSetup setup : setups) {
      setup.wiring().ifPresent(wiring -> wired.put(setup.joint().label(), wiring));
    }
    return wired;
  }
}
