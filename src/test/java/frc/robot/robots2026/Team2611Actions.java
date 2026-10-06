package frc.robot.robots2026;

import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import frc.robot.subsystems.mechanisms.Gate;
import frc.robot.subsystems.mechanisms.JointMechanism;
import frc.robot.subsystems.mechanisms.Joints;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.math.filter.Debouncer;

/**
 * 2611's 2026 behavior, all of it, in the template's API: what ten files of their subsystems,
 * commands and bindings did (2611-Vectors/2026-Beta-Bot at 6a5da76), in one class. A test fixture
 * for the behavior design (docs/design/robot-behavior.md), not 2026 play: it shows the API carries
 * a real robot's behavior, and the simulator runs it ({@code Team2611RobotSimTest}).
 *
 * <p>Two pieces are the drive's, which the template doesn't have yet: aiming, and following a path
 * (the path tool is the team's open decision). So the shot takes how far off aim the robot is, and
 * the auto takes its paths' commands, from whatever drives them. The shooter's speed would come
 * from their shot law; here it's one speed.
 */
final class Team2611Actions {
  /** The shooter's speed for every shot, standing in for their shot law. */
  static final double SHOT_RPM = 3000;

  private final Joints joints;
  private final Team2611.Parts parts;
  private final JointMechanism pivot;
  private final JointMechanism roller;
  private final JointMechanism shooter;
  private final Gate readyToShoot;

  /**
   * @param scheduler the robot's command scheduler, which works out the gates
   * @param aimError how far off aim the robot is, in degrees: the aiming controller's own error
   */
  Team2611Actions(
      Joints joints, Team2611.Parts parts, Scheduler scheduler, DoubleSupplier aimError) {
    this.joints = joints;
    this.parts = parts;
    pivot = joints.of(parts.pivot());
    roller = joints.of(parts.intake());
    shooter = joints.of(parts.shooter());
    readyToShoot =
        Gate.all(
                "ready to shoot",
                shooter.isAtGoal(),
                Gate.within(scheduler, "aimed", aimError, 2.0, 4.0))
            .debounce(Milliseconds.of(80), Debouncer.DebounceType.FALLING); // a flywheel's dip
  }

  /** Whether the robot may shoot: the shooter at speed and the robot aimed, for 80 ms. */
  Gate readyToShoot() {
    return readyToShoot;
  }

  /** Deploy and roll. The definition's interlock holds the roller until the pivot passes "run". */
  Command intake() {
    return Command.requiring(pivot, roller)
        .executing(
            co -> {
              co.fork(pivot.goTo("out"));
              co.await(roller.run("intaking"));
            })
        .named("Intake");
  }

  /** Spin up and stay up; feed whenever ready to shoot, and pause when not. */
  Command shoot() {
    return Command.requiring(shooter, joints.of(parts.fullSend()), joints.of(parts.transition()))
        .executing(
            co -> {
              co.fork(shooter.follow(() -> RPM.of(SHOT_RPM)));
              while (true) {
                co.waitUntil(readyToShoot);
                co.await(joints.run(parts.feeding()).onlyWhile(readyToShoot));
              }
            })
        .named("Shoot");
  }

  /**
   * Left Double, step for step as their auto runs it (deploy/pathplanner/autos/Left Double.auto),
   * the intake running along each path as their event markers start it.
   *
   * @param path each path's command, by its name in their PathPlanner project
   */
  Command leftDouble(Function<String, Command> path) {
    return Command.noRequirements(
            co -> {
              co.await(pivot.goTo("out")); // ARRIVED, or TIMED_OUT and logged: never stalls
              co.awaitAny(path.apply("EXP Left 1"), intake());
              co.awaitAny(path.apply("EXP Left 2"), intake());
              co.await(shoot().withTimeout(Seconds.of(3)));
              co.awaitAny(path.apply("EXP Left 3"), intake());
              co.await(shoot()); // until auto ends
            })
        .named("Left Double");
  }

  /**
   * Their bindings (RobotContainer.java:216-242), with the retract they never had. Shooting is held
   * and feeds through the gate; the right trigger's feed is the manual override, as theirs was.
   */
  void bind(CommandGamepad driver, CommandGamepad operator) {
    operator.rightTrigger().whileTrue(intake());
    operator.leftTrigger().whileTrue(roller.run("outtaking")); // the interlock applies here too
    operator.faceDown().onTrue(pivot.goTo("out"));
    operator.faceRight().onTrue(pivot.goTo("in"));
    driver.leftBumper().whileTrue(shoot());
    driver.rightTrigger().whileTrue(joints.run(parts.feeding()));
    driver.rightBumper().whileTrue(joints.run(parts.unjamming()));
  }
}
