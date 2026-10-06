package frc.robot.robots2026;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import org.wpilib.math.geometry.Transform3d;

/**
 * Comparing what the model derives with what a team's code configures, in the team's units:
 * Phoenix's rotations and volts per rotation per second, rpm, a camera's transform.
 */
final class Compare {
  private Compare() {}

  /** How far a derived ideal gain may be from a tuned one and still be the same mechanism. */
  static final double SANITY_LOW = 0.5;

  static final double SANITY_HIGH = 2.0;

  /**
   * The ideal kV as Phoenix holds it for a motor whose sensor reads in rotations of something
   * turning this many times for each of the joint's turns (or meters): volts per rotation per
   * second of the rotor for a ratio of the drive's own, of the mechanism for 1.
   */
  static double kV(RobotMath math, Joint joint, double turnsPerJointTurn) {
    double perUnit = math.idealGains(joint).kV();
    double perJointTurn = joint instanceof Joint.Revolute ? perUnit * 2 * Math.PI : perUnit;
    return perJointTurn / turnsPerJointTurn;
  }

  /** A joint's free speed in rotations per minute of the joint. */
  static double freeRpm(RobotMath math, Joint joint) {
    return math.freeSpeed(joint) * 60 / (2 * Math.PI);
  }

  /** A camera's mount is the transform a team's code gives, to a micron and a microradian. */
  static void sameMount(CameraConfig camera, Transform3d theirs) {
    sameTransform(camera.name(), camera.robotToCamera(), theirs);
  }

  static void sameTransform(String what, Transform3d ours, Transform3d theirs) {
    assertThat(ours.getTranslation().getDistance(theirs.getTranslation()))
        .as(what + "'s position, meters")
        .isLessThan(1e-6);
    assertThat(ours.getRotation().relativeTo(theirs.getRotation()).getAngle())
        .as(what + "'s rotation, radians")
        .isLessThan(1e-6);
  }
}
