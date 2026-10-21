package frc.robot;

import com.michaelgrundvig.frc.runtime.RobotConfig;
import java.util.List;
import java.util.Optional;

/**
 * The team's robots, one {@link RobotConfig} each. A test builds every one, so an invalid value
 * fails the build rather than the robot.
 *
 * <p>This branch has one robot, the swerve test chassis ({@link ChassisRobot}): a drivetrain alone.
 * Read it from the top; its values to check on the real chassis are in one block at the start. Its
 * behavior is in {@link Controls}. Run it in the simulator with {@code ./gradlew run}, or drive the
 * real chassis from a laptop with {@code ./gradlew run -PhwSim} (runtime/docs/bench.md).
 *
 * <p>Every value is set by name through a builder, which asks in turn for each one the robot can't
 * do without: none can be left out or land in the wrong place. runtime/docs/values.md says how to
 * measure each value. Note a measured value where it's set: {@code
 * .weight(Body.Weight.weighed(Pounds.of(42.5))) // measured YYYY-MM-DD, battery on}.
 */
public final class Robots {
  private Robots() {}

  /** Every robot the team has: the swerve test chassis. Add each new robot here. */
  public static List<RobotConfig> all() {
    return List.of(ChassisRobot.CONFIG);
  }

  /**
   * The system property that names the robot to run. The build sets it from {@code -Probot=NAME}
   * ({@code ./gradlew run -Probot=chassis}), so the simulator can run any of the team's robots
   * without an edit here. Nothing sets it on a real robot.
   */
  static final String ROBOT_PROPERTY = "frc.robot";

  /**
   * The robot this program is running on: the one place that decides. It is the swerve test
   * chassis, unless the simulator was asked for another robot by name.
   */
  public static RobotConfig current() {
    return asked(Optional.ofNullable(System.getProperty(ROBOT_PROPERTY)));
  }

  /**
   * The robot that was asked for by name, or the swerve test chassis when none was.
   *
   * @throws IllegalArgumentException for a name that is none of the team's robots, saying which
   *     names there are
   */
  static RobotConfig asked(Optional<String> name) {
    if (name.isEmpty()) {
      return ChassisRobot.CONFIG;
    }
    for (RobotConfig robot : all()) {
      if (robot.name().equals(name.get())) {
        return robot;
      }
    }
    List<String> names = all().stream().map(RobotConfig::name).toList();
    throw new IllegalArgumentException(
        "No robot is named \"" + name.get() + "\". The robots are: " + String.join(", ", names));
  }
}
