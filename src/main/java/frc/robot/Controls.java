package frc.robot;

import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.RecordingGamepad;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import org.wpilib.command3.Scheduler;

/**
 * What the sticks and buttons do. The driver's sticks already drive the chassis, and the back
 * button resets which way it thinks is forward. Bind your own commands in the constructor, for
 * example to lock the wheels in an X while the right trigger is held:
 *
 * <pre>{@code
 * driver.rightTrigger().whileTrue(drive.lockWheels());
 * }</pre>
 */
public final class Controls {
  /** The robot calls this once at startup, handing it everything there is to control. */
  Controls(
      Scheduler scheduler,
      RobotDefinition robot,
      Joints joints,
      Drive drive,
      RecordingGamepad driver,
      RecordingGamepad operator,
      Autos autos) {
    // Bind your commands here.
  }
}
