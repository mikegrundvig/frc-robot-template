package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.Gate;
import com.michaelgrundvig.frc.runtime.behavior.RecordingGamepad;
import com.michaelgrundvig.frc.runtime.subsystems.drive.Drive;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Joints;
import com.michaelgrundvig.frc.world.field.Heading;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.measure.Time;

/**
 * A team's 2026 Delta behavior, in the template's API: what its subsystems, commands and bindings
 * did (the Delta robot's original code), in one class, its {@code Controls}, as {@link
 * BetaControls} does Beta's: built by the robot with its joints, drivetrain, sensors and both
 * gamepads. A test fixture, not 2026 play: it shows the API carries a second real robot's behavior,
 * and the simulator runs it ({@code DeltaRobotSimTest}).
 *
 * <p>What Delta does and the template doesn't say is left out, not approximated: aiming's shot law
 * and the shots by distance, the pivot and shooter targets typed into a dashboard, live tuning of
 * the loops' gains, and every auto. The driver's left bumper, which starts the aimed shot
 * (RobotContainer.java:270-274), is therefore unbound, and B (:275-278), which also toggled the
 * shot by distance, holds the heading alone.
 */
public final class DeltaControls {
  /**
   * The shooter's speed on the driver's left trigger: the target a dashboard number holds, 2900 rpm
   * until someone types another (Shooter.java:25), and never above 4500 (Constants.java:68).
   */
  public static final double SHOT_RPM = 2900;

  /**
   * How long the pivot may be pushed at a fixed voltage, waiting to read out or in: 3 s, several
   * times what a swing takes. Their code had no limit. A run-until has none of its own either, so
   * this one is written here.
   */
  private static final Time SWING_DEADLINE = Seconds.of(3);

  private final Joints joints;
  private final JointMechanism pivot;
  private final JointMechanism roller;
  private final JointMechanism shooter;
  private final Gate intakeIsOut;
  private final Gate intakeIsIn;

  /**
   * As the robot builds its {@code Controls}: bound to both gamepads. It declares no autos (it has
   * none in this port), so the robot has "Do nothing" alone.
   */
  public DeltaControls(
      Scheduler scheduler,
      RobotDefinition robot,
      Joints joints,
      Drive drive,
      RecordingGamepad driver,
      RecordingGamepad operator,
      Autos autos) {
    this(joints);
    bind(drive, driver, operator);
  }

  /** Their behavior alone, bound to nothing: for their joints' own tests. */
  public DeltaControls(Joints joints) {
    this.joints = joints;
    pivot = joints.of(DeltaRobot.PIVOT);
    roller = joints.of(DeltaRobot.INTAKE);
    shooter = joints.of(DeltaRobot.SHOOTER);
    intakeIsOut =
        joints.gate("intake is out", DeltaRobot.PIVOT.atLeast(DeltaRobot.PIVOT_OUT_ANGLE));
    intakeIsIn = joints.gate("intake is in", DeltaRobot.PIVOT.atMost(DeltaRobot.PIVOT_IN_ANGLE));
  }

  /**
   * Swings the pivot out at 5 V until it reads out, then lets go of it (Pivot.java:51-65,
   * dumbIntakeOut): held on the operator's A button, it ends when it's out, button or not. A pivot
   * that never reads out isn't pushed at 5 V for longer than {@link #SWING_DEADLINE}.
   */
  public Command pivotOut() {
    return pivot.runUntil(DeltaRobot.PIVOT_SWINGING_OUT, intakeIsOut).within(SWING_DEADLINE);
  }

  /**
   * Swings the pivot in at -3 V until it reads in, then lets go of it (Pivot.java:66-79,
   * dumbIntakeIn): held on the operator's B button.
   */
  public Command pivotIn() {
    return pivot.runUntil(DeltaRobot.PIVOT_SWINGING_IN, intakeIsIn).within(SWING_DEADLINE);
  }

  /** Whether the pivot reads out, as Pivot.intakeIsOut does (Pivot.java:81-83). */
  public Gate intakeIsOut() {
    return intakeIsOut;
  }

  /** Whether the pivot reads in, as the in-drive ends on it (Pivot.java:70). */
  public Gate intakeIsIn() {
    return intakeIsIn;
  }

  /**
   * Their bindings (RobotContainer.java:213-281), the aimed shots left. The driver's A and B hold
   * the robot's back 20° either side of downfield while it drives (:213-234), and X turns the
   * wheels into an X (:237). On red their code reflected the heading's angle, where it turned the
   * position half a turn (AutoMath.java:84-109), which swaps A and B's sides; here each is 20° to
   * the same side of the driver's own forward, whichever alliance, as on blue.
   */
  private void bind(Drive drive, RecordingGamepad driver, RecordingGamepad operator) {
    operator.rightTrigger().whileTrue(roller.run(DeltaRobot.INTAKING));
    operator.leftTrigger().whileTrue(roller.run(DeltaRobot.OUTTAKING)); // the interlock applies
    driver.rightTrigger().whileTrue(joints.run(DeltaRobot.FEEDING));
    driver.rightBumper().whileTrue(joints.run(DeltaRobot.UNJAMMING));
    driver.leftTrigger().toggleOnTrue(shooter.follow(() -> RPM.of(SHOT_RPM)));
    operator.faceDown().whileTrue(pivotOut());
    operator.faceRight().whileTrue(pivotIn());
    driver.faceDown().whileTrue(drive.facing(Heading.downfield(Degrees.of(-20)).withBack()));
    driver.faceRight().whileTrue(drive.facing(Heading.downfield(Degrees.of(20)).withBack()));
    driver.faceLeft().whileTrue(drive.lockWheels());
  }
}
