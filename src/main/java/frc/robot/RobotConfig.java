package frc.robot;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import frc.robot.lib.wiring.CanIds;
import frc.robot.subsystems.mechanisms.JointSetup;
import frc.robot.subsystems.mechanisms.Joints;
import frc.robot.subsystems.mechanisms.Sensors;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * Everything the code needs to know about one physical robot. Each robot the team builds is one
 * instance in {@link Robots}. Every value is checked when the record is built, so a bad value fails
 * at startup, and before that in the tests, which build every robot.
 *
 * <p>What the robot is, its definition, is read by everything that needs to know the robot:
 * localization, the simulator, the views. The rest is how this program drives it, wires it, and
 * judges what its sensors say.
 *
 * @param definition what the robot is: its body, drivetrain, cameras, IMUs and joints, where each
 *     is mounted, and, in each one's block, how it's driven and wired
 * @param vision how far the cameras' images are trusted, and which are left out: the thresholds and
 *     uncertainties, some tunable live
 * @param estimator how the pose estimator weighs odometry, the gyro, and the cameras
 */
@ConfigRecord
public record RobotConfig(
    RobotDefinition definition, VisionConfig vision, EstimatorConfig estimator) {
  public RobotConfig {
    List<JointSetup> setups = setups(definition);
    List<SensorWiring> sensorWiring = sensorWiring(definition);
    Optional<DrivetrainControl> driving = definition.drivetrain().control();
    Optional<SwerveWiring> drivetrainWiring = definition.drivetrain().wiring();
    Checks.on(RobotConfig.class, definition.name())
        .check(
            "definition.drivetrain.control",
            () ->
                driving.isEmpty()
                    ? "the drivetrain has no control: give it its limits and gains,"
                        + " .control(DrivetrainControl.builder()...)"
                    : null)
        .check(
            "definition.drivetrain.control.limits.maxLinearSpeed",
            () -> driving.map(each -> each.whyTooFastFor(definition.drivetrain())).orElse(null))
        .check(
            "definition.drivetrain.wiring",
            () ->
                drivetrainWiring
                    .map(wiring -> DrivetrainControl.whyNotDriven(definition.drivetrain(), wiring))
                    .orElse(null))
        .check("definition.joints", () -> uncontrolled(definition))
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

  /** How the drivetrain is driven, from its block: its limits and gains, tunable live. */
  public DrivetrainControl drivetrain() {
    return definition.drivetrain().control().orElseThrow();
  }

  /**
   * Where the drivetrain's devices are on the CAN bus, and how its encoders read, from its block:
   * empty for a robot that only runs in simulation, whose drivetrain then stays still on real
   * hardware.
   */
  public Optional<SwerveWiring> drivetrainWiring() {
    return definition.drivetrain().wiring();
  }

  /**
   * Each joint its motors drive, as this program drives it: its control, and its wiring on a robot,
   * from its block in the definition. None for the drivetrain's, which is driven above.
   */
  public List<JointSetup> joints() {
    return setups(definition);
  }

  /**
   * Where each switch and rangefinder is connected, from the definition: each joint's own limit
   * switches, and the robot's sensors, that carry their wiring. None for one that only runs in the
   * simulator.
   */
  public List<SensorWiring> sensors() {
    return sensorWiring(definition);
  }

  /** Each joint's setup from its block: those with a control. */
  private static List<JointSetup> setups(RobotDefinition definition) {
    return JointSetup.all(definition);
  }

  /** Each sensor's wiring, from the sensors that carry it. */
  private static List<SensorWiring> sensorWiring(RobotDefinition definition) {
    List<SensorWiring> wiring = new ArrayList<>();
    for (Sensor sensor : definition.math().sensors()) {
      switch (sensor) {
        case Sensor.Switch onSwitch ->
            onSwitch
                .wiring()
                .ifPresent(
                    connection ->
                        wiring.add(
                            SensorWiring.OfSwitch.builder()
                                .sensor(onSwitch.label())
                                .input(connection.input())
                                .polarity(connection.polarity())
                                .build()));
        case Sensor.Rangefinder rangefinder ->
            rangefinder
                .wiring()
                .ifPresent(
                    canRange ->
                        wiring.add(
                            SensorWiring.OfRangefinder.builder()
                                .sensor(rangefinder.label())
                                .input(canRange)
                                .build()));
        case Sensor.Encoder encoder -> {}
      }
    }
    return wiring;
  }

  /** The joints with motors and no control, each named; or null. */
  private static @Nullable String uncontrolled(RobotDefinition definition) {
    List<String> missing = new ArrayList<>();
    for (Joint joint : definition.joints()) {
      if (joint.driven().motors().isPresent() && joint.driven().control().isEmpty()) {
        missing.add(
            joint.label()
                + " has motors and no control: give it .control(MechanismControl.position(kP)),"
                + " or .velocity(kP) for one that only runs");
      }
    }
    return missing.isEmpty() ? null : String.join("; ", missing);
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
