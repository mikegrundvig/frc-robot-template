package frc.robot.examples;

import static frc.robot.examples.ExampleRobot.FEEDER;
import static frc.robot.examples.ExampleRobot.FEEDING;
import static frc.robot.examples.ExampleRobot.INDEXER;
import static frc.robot.examples.ExampleRobot.SHOOTER;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import com.michaelgrundvig.frc.world.field.Heading;
import frc.robot.behavior.Gate;
import frc.robot.behavior.RecordingGamepad;
import frc.robot.game.Rebuilt;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveAssist;
import frc.robot.subsystems.mechanisms.JointMechanism;
import frc.robot.subsystems.mechanisms.Joints;
import org.wpilib.command3.Command;

/**
 * Driving with the sticks while the robot faces something, as docs/driving.md writes it
 * (DocExamplesTest): the shooter's feed runs only while the robot is aimed.
 */
final class Aiming {
  private final Drive drive;
  private final Joints joints;
  private final JointMechanism shooter;
  private final JointMechanism feeder;
  private final JointMechanism indexer;

  Aiming(Drive drive, Joints joints) {
    this.drive = drive;
    this.joints = joints;
    shooter = joints.of(SHOOTER);
    feeder = joints.of(FEEDER);
    indexer = joints.of(INDEXER);
  }

  /** The driver's sticks drive; the robot faces the hub, and holds it while they let go. */
  DriveAssist facingTheHub() {
    // @start region=facing
    DriveAssist aim = drive.facing(Heading.toward(Rebuilt.HUB));
    // @end
    return aim;
  }

  /** The same, 20° to the driver's right of downfield: a snap to an angle, for either alliance. */
  DriveAssist snap() {
    // @start region=snap
    DriveAssist right = drive.facing(Heading.downfield(Degrees.of(-20)));
    // @end
    return right;
  }

  /** A robot that shoots from its back faces the hub with it. */
  DriveAssist backToTheHub() {
    // @start region=back
    DriveAssist aim = drive.facing(Heading.toward(Rebuilt.HUB).withBack());
    // @end
    return aim;
  }

  /** Snaps either side of downfield while a button is held, and locks the wheels on another. */
  void bind(RecordingGamepad driver) {
    // @start region=bind
    driver.faceDown().whileTrue(drive.facing(Heading.downfield(Degrees.of(-20))));
    driver.faceRight().whileTrue(drive.facing(Heading.downfield(Degrees.of(20))));
    driver.faceLeft().whileTrue(drive.lockWheels());
    // @end
  }

  /** Aims at the hub with the sticks driving, and feeds only while aimed and the shooter is up. */
  Command shoot() {
    // @start region=shoot
    DriveAssist aim = drive.facing(Heading.toward(Rebuilt.HUB));
    Gate ready = Gate.all("ready to feed", aim.aimed(), shooter.isAtGoal());
    return Command.requiring(drive, shooter, feeder, indexer)
        .executing(
            co -> {
              co.fork(shooter.follow(() -> RPM.of(3000)));
              co.fork(aim); // the sticks drive, and the robot holds its aim...
              while (true) {
                co.waitUntil(ready); // ...and the feed runs only while it is aimed
                co.await(joints.run(FEEDING).onlyWhile(ready));
              }
            })
        .named("Shoot at the hub");
    // @end
  }
}
