package frc.robot;

import com.michaelgrundvig.frc.runtime.ExampleControls;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.RecordingGamepad;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.List;
import java.util.Optional;
import org.wpilib.command3.Scheduler;

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
 * replace it with your own ({@link #current}), and with it that robot's buttons and autos, {@link
 * ExampleControls}; your own behavior goes in {@link Controls} ({@link #controls}). The guide's
 * robot's intake reads:
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
 * in {@link BetaControls}, and {@link DeltaRobot}, with {@link DeltaControls}). The simulator runs
 * either when asked: {@code ./gradlew run -Probot=beta}. runtime/docs/mechanisms.md says how to
 * describe a mechanism and use its commands, and how to write a practice robot.
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
   * The system property that names the robot to run. The build sets it from {@code -Probot=NAME}
   * ({@code ./gradlew run -Probot=beta}), so the simulator can run any of the team's robots without
   * an edit here. Nothing sets it on a real robot.
   */
  static final String ROBOT_PROPERTY = "frc.robot";

  /**
   * The robot this program is running on: the one place that decides, so a new robot is one line
   * here. With several real robots, choose by something the robot itself knows (a device's serial
   * number, a file on its storage) rather than by editing this before each deploy. It is the
   * guide's robot until you write yours, so the simulator and Pitwall show mechanisms from the
   * start, unless the simulator was asked for another by name.
   */
  public static RobotConfig current() {
    return asked(Optional.ofNullable(System.getProperty(ROBOT_PROPERTY)));
  }

  /**
   * The robot that was asked for by name, or the guide's robot when none was.
   *
   * @throws IllegalArgumentException for a name that is none of the team's robots, saying which
   *     names there are
   */
  static RobotConfig asked(Optional<String> name) {
    if (name.isEmpty()) {
      return ExampleRobot.COMPETITION;
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

  /**
   * What the robot does: the controls written for the robot this is.
   *
   * <ul>
   *   <li>The guide's robots ({@link ExampleRobot#all}) get the guide's controls, {@link
   *       ExampleControls}, from the runtime. That is where the example's buttons come from: the
   *       operator's right trigger intakes and the driver's right trigger shoots, in the simulator,
   *       before you have written anything (runtime/docs/guide.md).
   *   <li>Each of a team's 2026 robots has its own, since they are different designs.
   *   <li>Every other robot, which is each one you add to {@link #all}, gets {@link Controls}. Put
   *       your commands, bindings and autos there.
   * </ul>
   *
   * <p>It has the shape of {@code ControlsMaker.make}, so {@code Robots::controls} is what {@code
   * Main} hands the runtime. A team with one design passes {@code Controls::new} there and needs
   * none of this.
   */
  public static Object controls(
      Scheduler scheduler,
      RobotDefinition robot,
      Joints joints,
      Drive drive,
      RecordingGamepad driver,
      RecordingGamepad operator,
      Autos autos) {
    if (isTheGuides(robot)) {
      return new ExampleControls(scheduler, robot, joints, drive, driver, operator, autos);
    }
    if (robot.name().equals(BetaRobot.CONFIG.name())) {
      return new BetaControls(scheduler, robot, joints, drive, driver, operator, autos);
    }
    if (robot.name().equals(DeltaRobot.CONFIG.name())) {
      return new DeltaControls(scheduler, robot, joints, drive, driver, operator, autos);
    }
    return new Controls(scheduler, robot, joints, drive, driver, operator, autos);
  }

  /** Whether a robot is one of the guide's, by its name: the competition robot or its practice. */
  private static boolean isTheGuides(RobotDefinition robot) {
    for (RobotConfig guides : ExampleRobot.all()) {
      if (guides.name().equals(robot.name())) {
        return true;
      }
    }
    return false;
  }
}
