package frc.robot.robots2026;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import frc.robot.behavior.Gate;
import frc.robot.behavior.RecordingGamepad;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.JointMechanism;
import frc.robot.subsystems.mechanisms.Joints;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * 2611's 2026 behavior, all of it, in the template's API: what ten files of their subsystems,
 * commands and bindings did (2611-Vectors/2026-Beta-Bot at 6a5da76), in one class, their {@code
 * Controls}: in their fork of the template it's src/main/java/frc/robot/Controls.java, built by the
 * robot with its joints, drivetrain, sensors and both gamepads. A test fixture for the behavior
 * design (docs/design/robot-behavior.md), not 2026 play: it shows the API carries a real robot's
 * behavior, and the simulator runs it ({@code Team2611RobotSimTest}).
 *
 * <p>Two pieces are the drive's, which the template doesn't have yet: aiming, and following a path
 * (the path tool is the team's open decision). So the shot takes how far off aim the robot is,
 * worked out from where it is, and the auto takes its paths' commands, from whatever drives them.
 * The shooter's speed would come from their shot law; here it's one speed.
 */
final class Team2611Controls {
  /** The shooter's speed for every shot, standing in for their shot law. */
  static final double SHOT_RPM = 3000;

  /** The blue hub's center, as their code has it (Constants.java:178-182: 182.11 in, 158.84 in). */
  static final Translation2d HUB = new Translation2d(Inches.of(182.11), Inches.of(158.84));

  private final Joints joints;
  private final JointMechanism pivot;
  private final JointMechanism roller;
  private final JointMechanism shooter;
  private final Gate readyToShoot;

  /**
   * As the robot builds its {@code Controls}: aimed from where the drivetrain is, and bound to both
   * gamepads.
   */
  Team2611Controls(
      Scheduler scheduler,
      RobotDefinition robot,
      Joints joints,
      Drive drive,
      RecordingGamepad driver,
      RecordingGamepad operator) {
    this(scheduler, joints, () -> aimError(drive.getPose()));
    bind(driver, operator);
  }

  /**
   * Their behavior alone, bound to nothing: for their joints' own tests, and an auto aimed another
   * way.
   *
   * @param scheduler the robot's command scheduler, which works out the gates
   * @param aimError how far off aim the robot is, in degrees: the aiming controller's own error
   */
  Team2611Controls(Scheduler scheduler, Joints joints, DoubleSupplier aimError) {
    this.joints = joints;
    pivot = joints.of(Team2611.PIVOT);
    roller = joints.of(Team2611.INTAKE);
    shooter = joints.of(Team2611.SHOOTER);
    readyToShoot =
        Gate.all(
                "ready to shoot",
                shooter.isAtGoal(),
                Gate.within(scheduler, "aimed", aimError, 2.0, 4.0))
            .debounce(Milliseconds.of(80), Debouncer.DebounceType.FALLING); // a flywheel's dip
  }

  /**
   * How far off aim a robot is, in degrees, from where it is: their shooter faces the robot's back
   * (inferred: their paths end facing away from the hub to shoot, EXP Left 2 at 135°).
   */
  static double aimError(Pose2d pose) {
    Translation2d toHub = HUB.minus(pose.getTranslation());
    Rotation2d bearing = new Rotation2d(Math.atan2(toHub.getY(), toHub.getX()));
    Rotation2d shooting = pose.getRotation().plus(new Rotation2d(Math.PI));
    return shooting.minus(bearing).getDegrees();
  }

  /** Whether the robot may shoot: the shooter at speed and the robot aimed, for 80 ms. */
  Gate readyToShoot() {
    return readyToShoot;
  }

  /** Deploy and roll. The definition's interlock holds the roller until the pivot passes 110°. */
  Command intake() {
    return Command.requiring(pivot, roller)
        .executing(
            co -> {
              co.fork(pivot.goTo(Team2611.PIVOT_OUT));
              co.await(roller.run(Team2611.INTAKING));
            })
        .named("Intake");
  }

  /** Spin up and stay up; feed whenever ready to shoot, and pause when not. */
  Command shoot() {
    return Command.requiring(shooter, joints.of(Team2611.FULL_SEND), joints.of(Team2611.TRANSITION))
        .executing(
            co -> {
              co.fork(shooter.follow(() -> RPM.of(SHOT_RPM)));
              while (true) {
                co.waitUntil(readyToShoot);
                co.await(joints.run(Team2611.FEEDING).onlyWhile(readyToShoot));
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
              co.await(
                  pivot.goTo(Team2611.PIVOT_OUT)); // ARRIVED, or TIMED_OUT and logged: never stalls
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
  private void bind(RecordingGamepad driver, RecordingGamepad operator) {
    operator.rightTrigger().whileTrue(intake());
    operator
        .leftTrigger()
        .whileTrue(roller.run(Team2611.OUTTAKING)); // the interlock applies here too
    operator.faceDown().onTrue(pivot.goTo(Team2611.PIVOT_OUT));
    operator.faceRight().onTrue(pivot.goTo(Team2611.PIVOT_IN));
    driver.leftBumper().whileTrue(shoot());
    driver.rightTrigger().whileTrue(joints.run(Team2611.FEEDING));
    driver.rightBumper().whileTrue(joints.run(Team2611.UNJAMMING));
  }
}
