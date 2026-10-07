package frc.robot.examples;

import static frc.robot.examples.ExampleRobot.INTAKING;
import static frc.robot.examples.ExampleRobot.PIVOT;
import static frc.robot.examples.ExampleRobot.PIVOT_IN;
import static frc.robot.examples.ExampleRobot.PIVOT_OUT;
import static frc.robot.examples.ExampleRobot.ROLLER;

import frc.robot.behavior.RecordingGamepad;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.JointMechanism;
import frc.robot.subsystems.mechanisms.Joints;
import frc.robot.subsystems.mechanisms.Sensors;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

/**
 * What the robot of docs/mechanisms.md does, and the operator's buttons for it, in the class a team
 * writes its behavior in (src/main/java/frc/robot/Controls.java), as the guide's "Where behavior
 * goes" writes it (DocExamplesTest).
 */
public final class Controls {
  private final JointMechanism pivot, roller;

  Controls(
      Scheduler scheduler,
      Joints joints,
      Drive drive,
      Sensors sensors,
      RecordingGamepad driver,
      RecordingGamepad operator) {
    pivot = joints.of(PIVOT);
    roller = joints.of(ROLLER);

    operator.rightTrigger().whileTrue(intake());
    operator.faceRight().onTrue(pivot.goTo(PIVOT_IN));
  }

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
}
