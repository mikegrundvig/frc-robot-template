package frc.robot.examples;

import static frc.robot.examples.ExampleRobot.INTAKING;
import static frc.robot.examples.ExampleRobot.PIVOT;
import static frc.robot.examples.ExampleRobot.PIVOT_IN;
import static frc.robot.examples.ExampleRobot.PIVOT_OUT;
import static frc.robot.examples.ExampleRobot.ROLLER;

import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import frc.robot.autos.Autos;
import frc.robot.behavior.RecordingGamepad;
import frc.robot.game.Rebuilt;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.JointMechanism;
import frc.robot.subsystems.mechanisms.Joints;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * What the robot of docs/mechanisms.md does, and the operator's buttons for it, in the class a team
 * writes its behavior in (src/main/java/frc/robot/Controls.java), as the guide's "Where behavior
 * goes" writes it (DocExamplesTest), and its one auto, as docs/autos.md does.
 */
public final class Controls {
  // @start region=constructor
  private static final Place PICKUP = Place.at("Pickup", new Pose2d(6, 3, Rotation2d.ZERO));

  private final JointMechanism pivot, roller;
  private final Drive drive;

  Controls(
      Scheduler scheduler,
      RobotDefinition robot,
      Joints joints,
      Drive drive,
      RecordingGamepad driver,
      RecordingGamepad operator,
      Autos autos) {
    this.drive = drive;
    pivot = joints.of(PIVOT);
    roller = joints.of(ROLLER);

    operator.rightTrigger().whileTrue(intake());
    operator.faceRight().onTrue(pivot.goTo(PIVOT_IN));

    autos.add("Grab one", Rebuilt.starts(robot.body()).center(), this::grabOne);
  }

  // @end

  /** Deploy and roll. The interlock holds the roller until the pivot passes 110°. */
  Command intake() {
    return Command.requiring(pivot, roller)
        .executing(
            co -> {
              co.fork(pivot.goTo(PIVOT_OUT));
              co.await(roller.run(INTAKING));
            })
        .named("Intake");
  }

  // @start region=auto
  /** Drives to the pickup with the intake running, then brings the intake in. */
  Command grabOne() {
    return Command.noRequirements(
            co -> {
              co.awaitAny(drive.goTo(PICKUP), intake());
              co.await(pivot.goTo(PIVOT_IN));
            })
        .named("Grab one");
  }

  // @end
}
