package frc.robot.examples;

import static frc.robot.examples.ExampleRobot.PIVOT;
import static frc.robot.examples.ExampleRobot.PIVOT_OUT;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveMove;
import frc.robot.subsystems.drive.Keyframe;
import frc.robot.subsystems.mechanisms.JointMechanism;
import frc.robot.subsystems.mechanisms.Joints;
import org.wpilib.command3.Command;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * A path through three keyframes, the intake swung out as the path passes the one named "pickup",
 * as docs/driving.md writes it (DocExamplesTest).
 */
final class PickUp {
  private final Drive drive;
  private final JointMechanism pivot;

  PickUp(Drive drive, Joints joints) {
    this.drive = drive;
    pivot = joints.of(PIVOT);
  }

  /** Drives the path, swinging the intake out as it passes the pickup, and on to its end. */
  Command pickUp() {
    return Command.requiring(drive, pivot)
        .executing(
            co -> {
              DriveMove leg =
                  drive.follow(
                      Keyframe.at(new Pose2d(3, 1, Rotation2d.ZERO)),
                      Keyframe.at(new Pose2d(5, 2, Rotation2d.CCW_90DEG)).named("pickup"),
                      Keyframe.at(new Pose2d(7, 2, Rotation2d.PI)));
              co.fork(leg); // drive the path...
              co.waitUntil(leg.at("pickup")); // ...and as it passes pickup,
              co.await(pivot.goTo(PIVOT_OUT)); // swing the intake out
              co.await(leg); // then wait for the path's end
            })
        .named("Pick up");
  }
}
