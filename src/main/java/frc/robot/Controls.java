package frc.robot;

import frc.robot.behavior.RecordingGamepad;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mechanisms.Joints;
import frc.robot.subsystems.mechanisms.Sensors;
import org.wpilib.command3.Scheduler;

/**
 * What the robot does, and the drive team's buttons for it: your commands, your autos and your
 * bindings, in this one class. It's yours: the template never changes it, so its updates never
 * collide with your code. The robot builds it once, after its mechanisms, with everything behavior
 * reaches; write each command as a method, and bind it in the constructor, as on any v3 gamepad.
 * docs/mechanisms.md, "Where behavior goes", has a robot's:
 *
 * {@snippet :
 * private final JointMechanism pivot, roller;
 *
 * Controls(Scheduler scheduler, Joints joints, Drive drive, Sensors sensors,
 *     RecordingGamepad driver, RecordingGamepad operator) {
 *   pivot = joints.of(PIVOT);
 *   roller = joints.of(ROLLER);
 *
 *   operator.rightTrigger().whileTrue(intake());
 *   operator.faceRight().onTrue(pivot.goTo(PIVOT_IN));
 * }
 * }
 *
 * <p>The robot records each binding as it's made, with its button and its command's name, and says
 * at startup when two commands are bound to one button the same way. The driver's sticks drive the
 * robot and the driver's back button resets its heading: the template binds those itself ({@link
 * RobotContainer}).
 */
public final class Controls {
  /**
   * Builds the robot's behavior, and binds it to the gamepads.
   *
   * @param scheduler the command scheduler, for gates of your own ({@code Gate.within(scheduler,
   *     ...)})
   * @param joints every driven joint's mechanism, by its joint: {@code joints.of(PIVOT)}
   * @param drive the drivetrain, with its moves (docs/driving.md)
   * @param sensors the robot's switches and rangefinders, as gates: {@code sensors.gate("coral in
   *     claw", CLAW_BEAM.tripped())}
   * @param driver the driver's gamepad, port 0
   * @param operator the operator's gamepad, port 1
   */
  Controls(
      Scheduler scheduler,
      Joints joints,
      Drive drive,
      Sensors sensors,
      RecordingGamepad driver,
      RecordingGamepad operator) {
    // The template's robot has no mechanisms yet: build and bind yours here.
  }
}
