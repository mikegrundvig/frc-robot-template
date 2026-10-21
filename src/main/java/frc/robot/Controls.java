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
 * bindings, in this one class. The robot builds it once, with everything behavior reaches; write
 * each command as a method, and bind it in the constructor, as on any v3 gamepad.
 *
 * <p>The test chassis only drives, so this starts empty: the driver's sticks drive the robot
 * ({@code drive.driven()}, the drive's default command) and the driver's back button resets its
 * heading, both bound by the template itself ({@link RobotContainer}). The drive's other commands
 * use the same sticks, so a button binds them as it binds anything: {@code drive.facing(heading)}
 * drives with the sticks while the robot holds a heading, and {@code drive.lockWheels()} turns the
 * wheels into an X (runtime/docs/driving.md). For example, to lock the wheels while the driver
 * holds the right trigger: {@code driver.rightTrigger().whileTrue(drive.lockWheels());}
 *
 * <p>The robot records each binding as it's made, with its button and its command's name, and says
 * at startup when two commands are bound to one button the same way.
 *
 * <p>Autos are declared here too: {@code autos.add("Drive out", start, this::driveOut)} gives the
 * pit a choice, and the robot runs the chosen one's command, made fresh, when autonomous starts.
 * The robot adds "Do nothing" itself and starts with it chosen; a robot that declares nothing has
 * only that (runtime/docs/autos.md).
 */
public final class Controls {
  /**
   * Builds the robot's behavior, and binds it to the gamepads.
   *
   * @param scheduler the command scheduler
   * @param robot the robot this runs on, its definition
   * @param joints every driven joint's mechanism; the test chassis has none
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
    // Nothing is bound yet: the driver's sticks already drive the chassis. Bind your own commands
    // here.
  }
}
