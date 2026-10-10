package frc.robot;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import java.util.List;

/**
 * The team's robots, one {@link RobotConfig} each. A test builds every one, so an invalid value
 * fails the build rather than the robot.
 *
 * <p>Every value is set by name through a builder, which asks in turn for each one the robot can't
 * do without: none can be left out or land in the wrong place. What a robot may not have (a PDH,
 * its joints, a camera's joint, a center of gravity) comes after them, given as itself ({@code
 * .pdh(Pdh.can(1))}), or is left out.
 *
 * <p>The guide's robot ({@link ExampleRobot}, in the runtime) is a typical competition build, at
 * the 2026 rule maximums where the rules set one. Describe your robot here as you design and
 * measure it, starting from its blocks: runtime/docs/values.md says how to measure each value, and
 * where today's number came from. Note a measured value where it's set, as the calibration
 * procedures do: {@code .weight(Body.Weight.weighed(Pounds.of(131.2))) // measured YYYY-MM-DD: two
 * scales, bumpers and battery on}.
 *
 * <p>A robot's mechanisms are written here too, each joint in one block: where it is, its named
 * places and outputs, its motors and their wiring, its own sensors, how the program drives it, and
 * its interlocks, which hold beneath every command. The configurations the robot takes name the
 * joints and their places. The template runs the guide's robot, {@link ExampleRobot}, until you
 * replace it with your own ({@link #current}); its intake reads:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.ExampleRobot region=intake}
 *
 * <p>The definition refuses what can't be: a position outside its joint's range, a speed past what
 * its motors turn it at or a voltage past what they're rated for, a configuration that breaks an
 * interlock, interlocks that hold two joints by each other. {@code RobotDefinition.findings()}
 * lists what's doubtful, a speed so near its motors' free speed that its loop can't hold it, and
 * the program says it at startup: in the log ({@code Robot/Doubts}) and, on a robot, on the Driver
 * Station's console.
 *
 * <p>Behavior code names the constants themselves: {@code joints.of(PIVOT)}, {@code
 * pivot.goTo(PIVOT_OUT)}. A practice robot of the same design is the competition robot under its
 * own name, with its differences, each joint, sensor, camera or the drivetrain re-wired, re-tuned
 * or placed otherwise and the rest of it kept: a joint wired or tuned otherwise, or its named
 * places elsewhere, is still the same joint, so the same code runs on either, {@code
 * pivot.goTo(PIVOT_OUT)} to each robot's own out, and a revision of the competition robot's blocks
 * reaches it. It's listed beside the competition robot:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.ExampleRobot region=practice}
 *
 * <p>A team's 2026 robots are whole robots described this way ({@link BetaRobot}, with its behavior
 * in {@link BetaControls}, and {@link DeltaRobot}, with {@link DeltaControls}).
 * runtime/docs/mechanisms.md says how to describe a mechanism and use its commands, and how to
 * write a practice robot.
 */
public final class Robots {
  private Robots() {}

  /**
   * Every robot the team has: the guide's two and a team's 2026 two, until you add your own. Add
   * each real robot as it is built and measured.
   */
  public static List<RobotConfig> all() {
    return List.of(
        ExampleRobot.COMPETITION, ExampleRobot.PRACTICE, BetaRobot.CONFIG, DeltaRobot.CONFIG);
  }

  /**
   * The robot this program is running on: the one place that decides, so a new robot is one line
   * here. With several robots, choose by something the robot itself knows (a device's serial
   * number, a file on its storage) rather than by editing this before each deploy. The guide's
   * robot until you write yours, so the simulator and Pitwall show mechanisms from the start.
   */
  public static RobotConfig current() {
    return ExampleRobot.COMPETITION;
  }
}
