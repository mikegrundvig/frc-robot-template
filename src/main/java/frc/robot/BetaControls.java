package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.RecordingGamepad;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.drive.DriveAssist;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.field.Heading;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Start;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.path.Keyframe;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * A team's 2026 behavior, all of it, in the template's API: what ten files of their subsystems,
 * commands and bindings did (the Beta robot's original code), in one class, their {@code Controls}:
 * in their fork of the template it's src/main/java/frc/robot/Controls.java, built by the robot with
 * its joints, drivetrain, sensors and both gamepads. A test fixture for the behavior design
 * (docs/design/robot-behavior.md), not 2026 play: it shows the API carries a real robot's behavior,
 * and the simulator runs it ({@code BetaRobotSimTest}, and its autos {@code BetaAutosSimTest}).
 *
 * <p>Aiming is the drive's: the shooter faces the robot's back, so the shot holds the robot's back
 * toward the hub, whichever alliance, and is ready when the drive says it is aimed. The shooter's
 * speed would come from their shot law; here it's one speed.
 *
 * <p>Two of their autos are ported, Left Single and Left Double, declared in the constructor for
 * the pit to choose ({@code runtime/docs/autos.md}). Their paths are the drive's, {@code
 * drive.follow} through places written once for blue (the path files' anchor points, in meters; the
 * headings are their rotation targets at each anchor, to the nearest 5°), so each is right on both
 * alliances. A few anchors are moved from the files', where the files' paths ran through the
 * trench's column or out to the guardrail: each says so where it is written. Their Right autos are
 * not ported: the files' Right paths are hand-drawn, not mirrors of the Left ones.
 */
public final class BetaControls {
  /** The shooter's speed for every shot, standing in for their shot law. */
  public static final double SHOT_RPM = 3000;

  // Left Single's anchors (deploy/pathplanner/paths/Left Half 1.path, Left Half 2.path), after the
  // first, which is the robot's start. The Left start is the path's own, to 14 cm.
  //
  // The two anchors beyond the trench are moved from the files' (5.5, 7.5), so the robot runs clear
  // of the trench's column and the guardrail on the real field: 0.4 to 0.6 m farther out, where its
  // bumpers are past the column, and 0.3 m in from the guardrail, which the curve through them
  // otherwise swings out to. It goes back through the trench square, where the file turned it a
  // quarter turn inside: the opening is 1.28 m, and the bumpers are 1.15 m across a corner.
  private static final Place HALF_WALL = place("Half wall", 5.9, 7.2, 0);
  private static final Place HALF_FAR = place("Half far", 7.6, 4.5, -90);
  private static final Place HALF_BACK = place("Half back", 6.1, 7.2, 0);

  // Left Double's (EXP Left 1.path, EXP Left 2.path, EXP Left 3.path). Its first anchor, (4.5,
  // 7.419), is 0.9 m along the wall from the Left start, which the robot starts on: it passes the
  // anchor on its way.
  //
  // Moved to run clear on the real field: the anchor past the trench is 12 cm in from the file's
  // 7.419 and held square (the file turned 30° there, inside the trench); the third path goes back
  // out through the trench's mouth, squared up before it, where the file cut the corner from the
  // shot to the wall anchor through the trench's column; and the last anchor before the bump is
  // 0.5 m nearer it and 8 cm higher, which crosses the bump midway between the hub and the column.
  private static final Place WALL = place("Wall", 4.5, 7.419, 0);
  private static final Place TRENCH = place("Trench", 6.0, 7.3, 0);
  private static final Place MOUTH = place("Trench mouth", 3.3, 7.43, 0);
  private static final Place FAR = place("Far", 8.4, 4.869, -90);
  private static final Place BUMP = place("Bump", 4.63045, 5.4372, 140);
  private static final Place SHOT = place("Shot", 3.0, 6.069, 135);
  private static final Place SWEEP_OUT = place("Sweep out", 8.0, 6.269, -75);
  private static final Place SWEEP_IN = place("Sweep in", 6.92, 4.605, 170);
  private static final Place SWEEP_BACK = place("Sweep back", 5.0, 5.45, 140);

  /** A place on the field for blue, in meters and degrees as their path files give them. */
  private static Place place(String name, double x, double y, double degrees) {
    return Place.at(name, new Pose2d(x, y, Rotation2d.fromDegrees(degrees)));
  }

  private final Joints joints;
  private final JointMechanism pivot;
  private final JointMechanism roller;
  private final JointMechanism shooter;
  private final Gate readyToShoot;

  /** The drive facing the hub with the robot's back; null for behavior alone, with no drive. */
  private final @Nullable DriveAssist aim;

  /** The drivetrain their autos drive along their paths; null for behavior alone. */
  private final @Nullable Drive drive;

  /**
   * As the robot builds its {@code Controls}: aimed by the drive, bound to both gamepads, and with
   * their two Left autos declared. Their shooter faces the robot's back (inferred: their paths end
   * facing away from the hub to shoot, EXP Left 2 at 135°), so the drive holds the robot's back
   * toward the hub.
   */
  public BetaControls(
      Scheduler scheduler,
      RobotDefinition robot,
      Joints joints,
      Drive drive,
      RecordingGamepad driver,
      RecordingGamepad operator,
      Autos autos) {
    this(joints, drive, drive.facing(Heading.toward(Rebuilt.HUB).withBack()));
    bind(drive, driver, operator);
    Start left = Rebuilt.STARTS.left();
    // Left Single drives back onto its start: where this robot stands on it.
    Place backOnLeft = Place.at(left.name(), left.poseFor(robot.body().bumpers()));
    autos.add("Left Single", left, () -> leftSingle(backOnLeft));
    autos.add("Left Double", left, this::leftDouble);
  }

  private BetaControls(Joints joints, Drive drive, DriveAssist aim) {
    this(joints, drive, aim, aim.aimed());
  }

  /**
   * Their behavior alone, bound to nothing: for their joints' own tests, and an auto aimed another
   * way. It has no drive, so none of their autos.
   *
   * @param scheduler the robot's command scheduler, which works out the gates
   * @param aimError how far off aim the robot is, in degrees: the aiming controller's own error
   */
  public BetaControls(Scheduler scheduler, Joints joints, DoubleSupplier aimError) {
    this(joints, null, null, Gate.within(scheduler, "aimed", aimError, 2.0, 4.0));
  }

  private BetaControls(
      Joints joints, @Nullable Drive drive, @Nullable DriveAssist aim, Gate aimed) {
    this.joints = joints;
    this.drive = drive;
    this.aim = aim;
    pivot = joints.of(BetaRobot.PIVOT);
    roller = joints.of(BetaRobot.INTAKE);
    shooter = joints.of(BetaRobot.SHOOTER);
    readyToShoot =
        Gate.all("ready to shoot", shooter.isAtGoal(), aimed)
            .debounce(Milliseconds.of(80), Debouncer.DebounceType.FALLING); // a flywheel's dip
  }

  /** Whether the robot may shoot: the shooter at speed and the robot aimed, for 80 ms. */
  public Gate readyToShoot() {
    return readyToShoot;
  }

  /** Deploy and roll. The definition's interlock holds the roller until the pivot passes 110°. */
  public Command intake() {
    return Command.requiring(pivot, roller)
        .executing(
            co -> {
              co.fork(pivot.goTo(BetaRobot.PIVOT_OUT));
              co.await(roller.run(BetaRobot.INTAKING));
            })
        .named("Intake");
  }

  /** Spin up and stay up; feed whenever ready to shoot, and pause when not. */
  public Command shoot() {
    return Command.requiring(
            shooter, joints.of(BetaRobot.FULL_SEND), joints.of(BetaRobot.TRANSITION))
        .executing(
            co -> {
              co.fork(shooter.follow(() -> RPM.of(SHOT_RPM)));
              while (true) {
                co.waitUntil(readyToShoot);
                co.await(joints.run(BetaRobot.FEEDING).onlyWhile(readyToShoot));
              }
            })
        .named("Shoot");
  }

  /**
   * Shoot with the drive holding the robot's back toward the hub, the driver's left stick still
   * driving it, as their aimed shot did (AutoTargetDriverControl.java): aim and shoot start and end
   * together, on one button. With no drive, just shoot.
   */
  public Command aimAndShoot() {
    DriveAssist facing = aim;
    if (facing == null) {
      return shoot();
    }
    return Command.noRequirements(
            co -> {
              co.fork(facing);
              co.await(shoot());
            })
        .named("Aim and shoot");
  }

  /** The drivetrain, which their autos need: not there in their behavior alone. */
  private Drive drive() {
    return Objects.requireNonNull(drive, "their autos drive the robot: build with a drive");
  }

  /**
   * Left Single, step for step as their auto runs it (deploy/pathplanner/autos/Left Single.auto):
   * the intake out, along the first path with the intake running, back along the second, and the
   * aimed shot until autonomous ends, which the robot then cuts off. The path ends on the start.
   */
  private Command leftSingle(Place start) {
    Drive moves = drive();
    return Command.noRequirements(
            co -> {
              co.await(
                  pivot.goTo(
                      BetaRobot.PIVOT_OUT)); // ARRIVED, or TIMED_OUT and logged: never stalls
              co.awaitAny(moves.follow(Keyframe.at(HALF_WALL), Keyframe.at(HALF_FAR)), intake());
              co.await(moves.follow(Keyframe.at(HALF_BACK), Keyframe.at(start)));
              co.await(aimAndShoot()); // until autonomous ends
            })
        .named("Left Single");
  }

  /**
   * Left Double, step for step as their auto runs it (deploy/pathplanner/autos/Left Double.auto),
   * the intake running along each path as their event markers start it: the intake out, the first
   * two paths, a three-second aimed shot, the third path, and the aimed shot until autonomous ends,
   * which the robot then cuts off.
   */
  private Command leftDouble() {
    Drive moves = drive();
    return Command.noRequirements(
            co -> {
              co.await(
                  pivot.goTo(
                      BetaRobot.PIVOT_OUT)); // ARRIVED, or TIMED_OUT and logged: never stalls
              co.awaitAny(
                  moves.follow(Keyframe.at(WALL), Keyframe.at(TRENCH), Keyframe.at(FAR)), intake());
              co.awaitAny(moves.follow(Keyframe.at(BUMP), Keyframe.at(SHOT)), intake());
              co.await(aimAndShoot().withTimeout(Seconds.of(3)));
              co.awaitAny(
                  moves.follow(
                      Keyframe.at(MOUTH),
                      Keyframe.at(TRENCH),
                      Keyframe.at(SWEEP_OUT),
                      Keyframe.at(SWEEP_IN),
                      Keyframe.at(SWEEP_BACK),
                      Keyframe.at(SHOT)),
                  intake());
              co.await(aimAndShoot()); // until autonomous ends
            })
        .named("Left Double");
  }

  /**
   * Their bindings (RobotContainer.java:184-242), with the retract they never had. Shooting is held
   * and feeds through the gate, the drive aiming as it does; the right trigger's feed is the manual
   * override, as theirs was. The driver's A and B hold the robot's back 20° either side of
   * downfield while it drives (:184-202), and X turns the wheels into an X (:205). Their B also
   * toggled a shot by distance (:237-239), which isn't ported, so B holds the heading alone. On red
   * their code reflected the heading's angle, where it turned the position half a turn
   * (AutoMath.java: 78-102), which swaps A and B's sides; here each is 20° to the same side of the
   * driver's own forward, whichever alliance, as on blue.
   */
  private void bind(Drive drive, RecordingGamepad driver, RecordingGamepad operator) {
    operator.rightTrigger().whileTrue(intake());
    operator
        .leftTrigger()
        .whileTrue(roller.run(BetaRobot.OUTTAKING)); // the interlock applies here too
    operator.faceDown().onTrue(pivot.goTo(BetaRobot.PIVOT_OUT));
    operator.faceRight().onTrue(pivot.goTo(BetaRobot.PIVOT_IN));
    driver.leftBumper().whileTrue(aimAndShoot());
    driver.rightTrigger().whileTrue(joints.run(BetaRobot.FEEDING));
    driver.rightBumper().whileTrue(joints.run(BetaRobot.UNJAMMING));
    driver.faceDown().whileTrue(drive.facing(Heading.downfield(Degrees.of(-20)).withBack()));
    driver.faceRight().whileTrue(drive.facing(Heading.downfield(Degrees.of(20)).withBack()));
    driver.faceLeft().whileTrue(drive.lockWheels());
  }
}
