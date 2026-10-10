package frc.robot;

import com.michaelgrundvig.frc.runtime.RobotContainer;
import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.RecordingGamepad;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import org.wpilib.command3.Scheduler;

/**
 * What the robot does, and the drive team's buttons for it: your commands, your autos and your
 * bindings, in this one class. It's yours: the template never changes it, so its updates never
 * collide with your code. The robot builds it once, after its mechanisms, with everything behavior
 * reaches; write each command as a method, and bind it in the constructor, as on any v3 gamepad.
 * runtime/docs/mechanisms.md, "Where behavior goes", has a robot's:
 *
 * {@snippet class=com.michaelgrundvig.frc.runtime.examples.Controls region=constructor}
 *
 * <p>The robot records each binding as it's made, with its button and its command's name, and says
 * at startup when two commands are bound to one button the same way. The driver's sticks drive the
 * robot ({@code drive.driven()}, the drive's default command) and the driver's back button resets
 * its heading: the template binds those itself ({@link RobotContainer}). The drive's other commands
 * use the same sticks, so a button binds them as it binds anything: {@code drive.facing(heading)}
 * drives with the sticks while the robot holds a heading, and {@code drive.lockWheels()} turns the
 * wheels into an X (runtime/docs/driving.md).
 *
 * <p>Autos are declared here too: {@code autos.add("Left Double", start, this::leftDouble)} gives
 * the pit a choice, and the robot runs the chosen one's command, made fresh, when autonomous
 * starts. The robot adds "Do nothing" itself and starts with it chosen; a robot that declares
 * nothing has only that (runtime/docs/autos.md).
 */
public final class Controls {
  /**
   * Builds the robot's behavior, and binds it to the gamepads.
   *
   * @param scheduler the command scheduler, for gates of your own ({@code Gate.within(scheduler,
   *     ...)})
   * @param robot the robot this runs on, its definition: a subsystem of your own finds its joint
   *     there, {@code robot.own(HOOD)}, a practice robot's places its own
   * @param joints every driven joint's mechanism, by its joint, {@code joints.of(PIVOT)}, and a
   *     gate on anything the model says, a sensor's reading included: {@code joints.gate("coral in
   *     claw", CLAW_BEAM.tripped())}
   * @param drive the drivetrain, with its moves, assisted driving and wheel lock
   *     (runtime/docs/driving.md)
   * @param driver the driver's gamepad, port 0
   * @param operator the operator's gamepad, port 1
   * @param autos the robot's autos: declare each with {@code autos.add(name, start, factory)}, the
   *     factory making a fresh command every run
   */
  Controls(
      Scheduler scheduler,
      RobotDefinition robot,
      Joints joints,
      Drive drive,
      RecordingGamepad driver,
      RecordingGamepad operator,
      Autos autos) {
    // Nothing is bound: the guide's robot's joints idle, held where they rest, its rollers still,
    // until you bind your robot's commands here. runtime/docs/mechanisms.md shows the guide's.
  }
}
