package frc.robot.sim;

import static frc.robot.lib.config.Rules.notBlank;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigRecord;
import frc.robot.lib.config.Rule;
import java.util.List;

/**
 * A named situation for the simulated robot: a fault in its sensors, or something that happens to
 * it, such as a collision. The simulator starts in one with {@code ./gradlew run
 * -Pscenario=<name>}, and a test builds its robot in one ({@code RobotSim}). {@link Scenarios}
 * lists them.
 *
 * <p>A scenario changes only the simulated world. The robot's mechanisms and localization are never
 * told: they have to find out from the sensors, as on a real robot.
 *
 * <p>A scenario has two parts, either of which may be empty. Its {@link #sensorFaults} change how
 * the robot's sensors err, and take effect when the world is built ({@link #errorsOn}), since the
 * simulated sensors are built with it. Its {@link #event} happens to the robot in a running world
 * ({@link #startIn}): {@code ./gradlew run} starts it once the robot is enabled, and a test when it
 * chooses.
 *
 * @param name how it's asked for: lowercase words joined by hyphens, such as {@code camera-low}
 * @param description what it does to the robot
 * @param sensorFaults how the robot's sensors err in it; {@link #NO_FAULTS} for none
 * @param event what happens to the robot when it starts; {@link #NOTHING} for nothing
 */
@ConfigRecord
public record Scenario(String name, String description, SensorFaults sensorFaults, Event event) {
  /** The sensors err as they otherwise would. */
  public static final SensorFaults NO_FAULTS = (errors, cameras) -> errors;

  /** Nothing happens when the scenario starts. */
  public static final Event NOTHING = world -> {};

  /** How a scenario changes the robot's sensors. */
  @FunctionalInterface
  public interface SensorFaults {
    /**
     * @param errors how the robot's sensors would otherwise err
     * @param cameras the robot's cameras, for a scenario that knocks one
     * @return how they err in the scenario
     */
    SensorErrors apply(SensorErrors errors, List<CameraConfig> cameras);
  }

  /** What happens to the robot when a scenario starts. */
  @FunctionalInterface
  public interface Event {
    /** Makes it happen, in the simulated world: a hit, a slip, a gyro unplugged. */
    void happen(SimulatedWorld world);
  }

  public Scenario {
    Checks.on(Scenario.class)
        .field("name", name, lowercaseWords())
        .field("description", description, notBlank())
        .orThrow();
  }

  /** Starts describing a scenario, one named value at a time. */
  public static ScenarioBuilder.NameStage builder() {
    return ScenarioBuilder.builder();
  }

  /**
   * How the robot's sensors err in this scenario: its faults, on a season-worn robot's sensors
   * ({@link SensorErrors#TYPICAL}), as the simulator's robot has them.
   *
   * @param cameras the robot's cameras
   */
  public SensorErrors errorsOn(List<CameraConfig> cameras) {
    return sensorFaults.apply(SensorErrors.TYPICAL, cameras);
  }

  /** Starts this scenario's event in the simulated world: now. */
  public void startIn(SimulatedWorld world) {
    event.happen(world);
  }

  /** Lowercase letters and digits, in words joined by single hyphens: typed on a command line. */
  private static Rule<String> lowercaseWords() {
    return value ->
        value.matches("[a-z0-9]+(-[a-z0-9]+)*")
            ? null
            : "\"" + value + "\" must be lowercase words joined by hyphens, like camera-low";
  }
}
