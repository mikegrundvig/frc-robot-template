package frc.robot.robots2026;

import static com.michaelgrundvig.frc.runtime.Ended.ended;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.michaelgrundvig.frc.runtime.Ended;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.behavior.Binding;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.JointSimulation;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Held;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.record.MoveReason;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import frc.robot.BetaControls;
import frc.robot.BetaRobot;
import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.simulation.GamepadSim;

/**
 * A team's 2026 robot, Beta, end to end in the template's simulator: their definition, their
 * behavior ({@link BetaControls}) built as the robot builds its controls and bound to its two
 * gamepads, run by the robot program's own loop, the drive and localization included. The behavior
 * design's checkpoint C (docs/design/robot-behavior.md). Checked against the simulator's truth
 * throughout: the intake's roller is never driven up to speed while the pivot is short of 110°. A
 * test fixture, not 2026 play. Their autos are run in {@code BetaAutosSimTest}.
 */
class BetaRobotSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  private GamepadSim operator;
  private BetaControls controls;
  private JointMechanism pivot;
  private JointMechanism roller;
  private JointMechanism fullSend;
  private JointSimulation truth;

  /** The fastest the roller was turning last loop, and whether it ever sped up short of 110°. */
  private double rollerWas;

  private boolean ruleBroken;

  @BeforeEach
  void build() {
    sim.build(Parts.withoutCameras(BetaRobot.CONFIG), SensorErrors.NONE);
    operator = sim.operator();
    controls = sim.controls(BetaControls::new);
    pivot = sim.joints().of(BetaRobot.PIVOT);
    roller = sim.joints().of(BetaRobot.INTAKE);
    fullSend = sim.joints().of(BetaRobot.FULL_SEND);
    truth = Objects.requireNonNull(sim.world().joints());
  }

  /** The heading the robot really has, in degrees. */
  private double heading() {
    return sim.simulation().truePose().getRotation().getDegrees();
  }

  /** How far the robot really is from a heading, the nearer way round, in degrees. */
  private double offBy(double degrees) {
    return Math.IEEEremainder(heading() - degrees, 360);
  }

  /** Robot loops for a time, checking the interlock against the simulator's truth each loop. */
  private void run(double seconds) {
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      sim.loop();
      double speed = Math.abs(truth.velocity(BetaRobot.INTAKE));
      if (Math.toDegrees(truth.position(BetaRobot.PIVOT)) < 110 && speed > rollerWas + 0.5) {
        ruleBroken = true;
      }
      rollerWas = speed;
    }
  }

  private void press(Runnable change) {
    change.run();
    operator.notifyNewData();
    sim.driver().notifyNewData();
  }

  /** How fast a joint really turns, in rpm. */
  private double rpm(Joint joint) {
    return RPM.convertFrom(truth.velocity(joint), RadiansPerSecond);
  }

  /**
   * Their bindings are the robot's, beside the template's own, each with its button and command, as
   * a controller map reads them; none two to a button the same way.
   */
  @Test
  void theirBindingsAreRecorded() {
    assertThat(sim.bindings().all().stream().map(Binding::toString))
        .containsExactly(
            "driver back: on press: Reset heading",
            "operator right trigger: while held: Intake",
            "operator left trigger: while held: intake: outtaking",
            "operator face down: on press: pivot → out",
            "operator face right: on press: pivot → in",
            "driver left bumper: while held: Aim and shoot",
            "driver right trigger: while held: full send and transition: feeding",
            "driver right bumper: while held: full send and transition: unjamming",
            "driver face down: while held: Drive facing 20° right of downfield, back first",
            "driver face right: while held: Drive facing 20° left of downfield, back first",
            "driver face left: while held: Lock wheels");
    assertThat(sim.bindings().doubts()).isEmpty();
  }

  @Test
  void theOperatorsIntakeDeploysThePivotAndTheRollerWaitsForIt() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> operator.setRightTriggerAxis(1));
    run(0.1);
    List<Interlock> held = Held.interlocksHolding(roller);
    run(1.4);
    double intaking = rpm(BetaRobot.INTAKE);
    press(() -> operator.setRightTriggerAxis(0));
    run(1);

    assertThat(held)
        .containsExactly(
            Interlock.that(BetaRobot.INTAKE)
                .runsOnlyWhen(BetaRobot.PIVOT.atLeast(Degrees.of(110))));
    assertThat(intaking).as("near 2000 rpm, its free speed").isGreaterThan(1900);
    assertThat(Math.abs(rpm(BetaRobot.INTAKE))).as("let go: stopped").isLessThan(10);
    assertThat(Math.toDegrees(truth.position(BetaRobot.PIVOT)))
        .as("and held out")
        .isCloseTo(130, within(2.0));
    assertThat(ruleBroken).isFalse();
  }

  /** Their outtake button ignored the guard their auto had; the interlock holds it too. */
  @Test
  void outtakingWithThePivotStowedIsHeld() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> operator.setLeftTriggerAxis(1));
    run(1);

    assertThat(Held.interlocksHolding(roller))
        .containsExactly(
            Interlock.that(BetaRobot.INTAKE)
                .runsOnlyWhen(BetaRobot.PIVOT.atLeast(Degrees.of(110))));
    assertThat(Math.abs(rpm(BetaRobot.INTAKE))).isLessThan(10);
    assertThat(ruleBroken).isFalse();
  }

  /** Out on one button and back in on another: the retract they never had. */
  @Test
  void theOperatorDeploysAndRetractsThePivot() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> operator.setFaceDownButton(true));
    run(0.1);
    press(() -> operator.setFaceDownButton(false));
    run(1.5);
    double out = Math.toDegrees(truth.position(BetaRobot.PIVOT));
    press(() -> operator.setFaceRightButton(true));
    run(0.1);
    press(() -> operator.setFaceRightButton(false));
    run(1.5);

    assertThat(out).isCloseTo(130, within(2.0));
    assertThat(Math.toDegrees(truth.position(BetaRobot.PIVOT))).isCloseTo(1.5, within(2.0));
    ended(pivot).named("pivot → in").arrived();
  }

  /**
   * Holding the shot, the drive turns the robot's back to the hub (a half turn from its start) with
   * the shooter spinning up, and nothing feeds until it is aimed, the gate saying why. Aimed, it
   * feeds. The driver turns the robot away with the right stick and it pauses, saying why; let go,
   * and the drive aims again and it feeds on; the right trigger's feed is the manual override.
   */
  @Test
  void shootingAimsTheRobotAndFeedsOnlyWhenAimedAndSaysWhyNot() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> sim.driver().setLeftBumperButton(true));
    run(0.3);
    double unaimed = rpm(BetaRobot.FULL_SEND);
    var blocking = controls.readyToShoot().blocking();
    run(2.0);
    double aimed = rpm(BetaRobot.FULL_SEND);
    double shooter = rpm(BetaRobot.SHOOTER);
    double aimedAt = offBy(180); // the back to the hub at bearing 0 from the start: front to 180°
    press(() -> sim.driver().setRightX(0.5));
    run(0.4);
    double turnedAway = rpm(BetaRobot.FULL_SEND);
    Ended feedWhenAimedAway = ended(fullSend);
    press(() -> sim.driver().setRightX(0));
    run(2.0);
    double again = rpm(BetaRobot.FULL_SEND);
    press(() -> sim.driver().setRightX(0.5));
    run(0.4);
    press(() -> sim.driver().setRightTriggerAxis(1));
    run(1);
    double overridden = rpm(BetaRobot.FULL_SEND);

    assertThat(shooter).isCloseTo(BetaControls.SHOT_RPM, within(0.03 * 3000));
    assertThat(unaimed).isCloseTo(0, within(5.0));
    assertThat(blocking).containsExactly("aimed at the Hub, back first");
    assertThat(aimed).isCloseTo(1000, within(30.0));
    assertThat(aimedAt).as("turned from 0° to face away from the hub").isCloseTo(0, within(2.0));
    assertThat(turnedAway).isCloseTo(0, within(5.0));
    feedWhenAimedAway
        .named("full send and transition: feeding")
        .canceled()
        .because(MoveReason.GATE_FELL)
        .saying("ready to shoot is false: aimed at the Hub, back first");
    assertThat(overridden).as("the override feeds unaimed").isCloseTo(1000, within(30.0));
    assertThat(again).as("aimed again, it feeds on").isCloseTo(1000, within(30.0));
  }

  /** On red the back faces the red hub, at the other end of the field: a half turn from blue's. */
  @Test
  void aimingOnRedFacesTheRedHub() {
    RobotSim.station(AllianceStationID.RED_2);
    sim.loops(3); // disabled: the robot is put on red's start
    Pose2d start = sim.simulation().truePose();
    Translation2d hub = Rebuilt.HUB.on(RobotSim.FIELD, Alliance.RED).getTranslation();
    RobotSim.setEnabled(true);

    press(() -> sim.driver().setLeftBumperButton(true));
    run(3.0);
    Translation2d to = hub.minus(sim.simulation().truePose().getTranslation());
    double bearing = Math.toDegrees(Math.atan2(to.getY(), to.getX()));

    assertThat(start.getX()).as("red's start is at red's end of the field").isGreaterThan(8.27);
    assertThat(offBy(bearing + 180)).as("the back to red's hub").isCloseTo(0, within(2.0));
    assertThat(rpm(BetaRobot.FULL_SEND)).as("aimed, it feeds").isCloseTo(1000, within(30.0));
  }

  /**
   * The driver's A and B hold the robot's back 20° either side of downfield while the left stick
   * drives it, for either alliance: the original's lock, from each driver's own forward.
   */
  @Test
  void theDriversAAndBHoldTheHeadingEitherSideOfDownfield() {
    for (AllianceStationID station :
        new AllianceStationID[] {AllianceStationID.BLUE_2, AllianceStationID.RED_2}) {
      boolean red = station == AllianceStationID.RED_2;
      RobotSim.station(station);
      sim.loops(3);
      RobotSim.setEnabled(true);

      press(() -> sim.driver().setFaceDownButton(true));
      run(1.5);
      double a = heading();
      press(() -> sim.driver().setFaceDownButton(false));
      press(() -> sim.driver().setFaceRightButton(true));
      run(1.5);
      double b = heading();
      press(() -> sim.driver().setFaceRightButton(false));
      RobotSim.setEnabled(false);

      // The back 20° to the driver's right of downfield is the front 160° from it: 160° on blue
      // (downfield 0°), -20° on red (downfield 180°); B is the other side.
      assertThat(Math.IEEEremainder(a - (red ? -20 : 160), 360))
          .as("A on " + station)
          .isCloseTo(0, within(2.0));
      assertThat(Math.IEEEremainder(b - (red ? 20 : 200), 360))
          .as("B on " + station)
          .isCloseTo(0, within(2.0));
    }
  }

  /** A holds its heading while the left stick drives: the robot keeps both. */
  @Test
  void theHeadingHoldDrivesWithTheLeftStick() {
    RobotSim.setEnabled(true);
    run(0.1);
    double before = sim.simulation().truePose().getX();

    press(() -> sim.driver().setFaceDownButton(true));
    press(() -> sim.driver().setLeftY(-1)); // forward
    run(2.0);

    assertThat(sim.simulation().truePose().getX() - before)
        .as("driving downfield")
        .isGreaterThan(2);
    assertThat(offBy(160)).as("holding its heading").isCloseTo(0, within(2.0));
  }

  /** X turns the wheels into an X and the robot stays; let go, and the sticks drive again. */
  @Test
  void theDriversXLocksTheWheels() {
    RobotSim.setEnabled(true);
    run(0.1);
    Pose2d start = sim.simulation().truePose();

    press(() -> sim.driver().setFaceLeftButton(true));
    press(() -> sim.driver().setLeftY(-1));
    run(1.5);
    var wheels = sim.drive().moduleLocations();
    var angles = sim.drive().moduleVelocities();
    Pose2d locked = sim.simulation().truePose();
    press(() -> sim.driver().setFaceLeftButton(false));
    run(1.0);

    for (int i = 0; i < 4; i++) {
      double towardCenter = Math.toDegrees(Math.atan2(wheels[i].getY(), wheels[i].getX()));
      assertThat(Math.IEEEremainder(angles[i].angle.getDegrees() - towardCenter, 180))
          .as("wheel " + i)
          .isCloseTo(0, within(2.0));
    }
    assertThat(locked.getTranslation().getDistance(start.getTranslation())).isLessThan(0.02);
    assertThat(sim.simulation().truePose().getX() - start.getX())
        .as("then driving")
        .isGreaterThan(0.5);
  }
}
