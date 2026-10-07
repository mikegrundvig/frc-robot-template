package frc.robot.robots2026;

import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.List;
import org.wpilib.math.geometry.Translation3d;

/**
 * What the 2026 robots' fixtures and their tests share: points in the units a team's code writes
 * them, and the changes a test makes to a robot to try a team's value the model refuses. Nothing
 * here is a fact about a robot; each record a fixture describes, it builds by name.
 */
final class Parts {
  private Parts() {}

  /** Meters in an inch: the teams' code measures in inches as often as in meters. */
  static final double IN = 0.0254;

  static Translation3d point(double x, double y, double z) {
    return new Translation3d(x, y, z);
  }

  static Translation3d inches(double x, double y, double z) {
    return new Translation3d(x * IN, y * IN, z * IN);
  }

  /** A joint's named outputs changed, the rest as it is: a team's value the model refuses. */
  static Joint.Revolute withOutputs(Joint.Revolute joint, Output... outputs) {
    return JointRevoluteBuilder.builder(joint).outputs(List.of(outputs)).build();
  }

  /**
   * A robot with one of its joints, and the drive on it, swapped for others, and none of its
   * configurations, groups or interlocks, which name the old joint: where a team's value the model
   * refuses is tried.
   */
  static RobotDefinition swapped(
      RobotDefinition robot, Joint old, Joint joint, Drive oldDrive, Drive drive) {
    return RobotDefinitionBuilder.builder(robot)
        .joints(robot.joints().stream().map(each -> each.equals(old) ? joint : each).toList())
        .drives(robot.drives().stream().map(each -> each.equals(oldDrive) ? drive : each).toList())
        .configurations(List.of())
        .interlocks(List.of())
        .build();
  }

  /** A CANcoder's resolution: 4096 counts a turn. */
  static final int CANCODER_COUNTS = 4096;
}
