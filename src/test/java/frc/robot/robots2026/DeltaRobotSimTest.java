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
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import frc.robot.DeltaControls;
import frc.robot.DeltaRobot;
import java.util.Objects;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.simulation.GamepadSim;

/**
 * A team's 2026 robot, Delta, end to end in the template's simulator: their definition, their
 * behavior ({@link DeltaControls}) built as the robot builds its controls and bound to its two
 * gamepads, run by the robot program's own loop, the drive and localization included. Checked
 * against the simulator's truth throughout: the intake's roller is never driven up to speed while
 * the pivot is short of 130°. A test fixture, not 2026 play.
 */
class DeltaRobotSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  private GamepadSim operator;
  private JointMechanism pivot;
  private JointMechanism roller;
  private JointSimulation truth;

  /** The fastest the roller was turning last loop, and whether it ever sped up short of 130°. */
  private double rollerWas;

  private boolean ruleBroken;

  @BeforeEach
  void build() {
    sim.build(Parts.withoutCameras(DeltaRobot.CONFIG), SensorErrors.NONE);
    operator = sim.operator();
    sim.controls(DeltaControls::new);
    pivot = sim.joints().of(DeltaRobot.PIVOT);
    roller = sim.joints().of(DeltaRobot.INTAKE);
    truth = Objects.requireNonNull(sim.world().joints());
  }

  /** Robot loops for a time, checking the interlock against the simulator's truth each loop. */
  private void run(double seconds) {
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      sim.loop();
      double speed = Math.abs(truth.velocity(DeltaRobot.INTAKE));
      if (Math.toDegrees(truth.position(DeltaRobot.PIVOT)) < 130 && speed > rollerWas + 0.5) {
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

  private double pivotDegrees() {
    return Math.toDegrees(truth.position(DeltaRobot.PIVOT));
  }

  /**
   * Their bindings are the robot's, beside the template's own, each with its button and command, as
   * a controller map reads them; none two to a button the same way. Their left bumper starts the
   * aimed shot, whose law the model doesn't carry, so it is unbound; their B also toggled the shot
   * by distance, so B holds the heading alone.
   */
  @Test
  void theirBindingsAreRecorded() {
    assertThat(sim.bindings().all().stream().map(Binding::toString))
        .containsExactly(
            "driver back: on press: Reset heading",
            "operator right trigger: while held: intake: intaking",
            "operator left trigger: while held: intake: outtaking",
            "driver right trigger: while held: full send and transition: feeding",
            "driver right bumper: while held: full send and transition: unjamming",
            "driver left trigger: toggled on press: shooter follows a target",
            "operator face down: while held: pivot: swinging out until intake is out",
            "operator face right: while held: pivot: swinging in until intake is in",
            "driver face down: while held: Drive facing 20° right of downfield, back first",
            "driver face right: while held: Drive facing 20° left of downfield, back first",
            "driver face left: while held: Lock wheels");
    assertThat(sim.bindings().doubts()).isEmpty();
  }

  /** Their intake button ignores the pivot's angle; the definition's interlock holds the roller. */
  @Test
  void theRollerWithThePivotStowedIsHeld() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> operator.setRightTriggerAxis(1));
    run(1);

    assertThat(Held.interlocksHolding(roller))
        .containsExactly(
            Interlock.that(DeltaRobot.INTAKE)
                .runsOnlyWhen(DeltaRobot.PIVOT.atLeast(Degrees.of(130))));
    assertThat(Math.abs(rpm(DeltaRobot.INTAKE))).isLessThan(10);
    assertThat(ruleBroken).isFalse();
  }

  /**
   * Their A button swings the pivot out at 5 V and ends once it reads out, the button still held;
   * the intake button, held after it, runs the roller at its free speed; let go, the roller stops
   * and the pivot stays out.
   */
  @Test
  void theOperatorSwingsThePivotOutAndTheRollerRuns() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> operator.setFaceDownButton(true));
    run(1);
    double out = pivotDegrees();
    Ended ended = ended(pivot);
    press(() -> operator.setRightTriggerAxis(1));
    run(1);
    double intaking = rpm(DeltaRobot.INTAKE);
    press(() -> operator.setRightTriggerAxis(0));
    press(() -> operator.setFaceDownButton(false));
    run(1);

    assertThat(out).as("past 130°, short of its guessed stop").isBetween(130.0, 135.0);
    ended.named("pivot: swinging out until intake is out").arrived();
    assertThat(intaking).as("near 3000 rpm, its free speed").isGreaterThan(2850);
    assertThat(Math.abs(rpm(DeltaRobot.INTAKE))).as("let go: stopped").isLessThan(10);
    assertThat(pivotDegrees()).as("and held out").isBetween(129.0, 135.0);
    assertThat(ruleBroken).isFalse();
  }

  /** Out on one button and back in on another, each ending where its angle says. */
  @Test
  void theOperatorSwingsThePivotOutAndBackIn() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> operator.setFaceDownButton(true));
    run(1);
    press(() -> operator.setFaceDownButton(false));
    press(() -> operator.setFaceRightButton(true));
    run(2);

    assertThat(pivotDegrees()).as("at or under 3°, short of its stop").isBetween(0.0, 3.0);
    ended(pivot).named("pivot: swinging in until intake is in").arrived();
  }

  /** The driver's right trigger feeds both rollers, the right bumper reverses them. */
  @Test
  void theDriverFeedsAndUnjams() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> sim.driver().setRightTriggerAxis(1));
    run(1);
    double fullSend = rpm(DeltaRobot.FULL_SEND);
    double transition = rpm(DeltaRobot.TRANSITION);
    press(() -> sim.driver().setRightTriggerAxis(0));
    press(() -> sim.driver().setRightBumperButton(true));
    run(1);

    assertThat(fullSend).isCloseTo(1000, within(30.0));
    assertThat(transition).isCloseTo(2000, within(60.0));
    assertThat(rpm(DeltaRobot.FULL_SEND)).isCloseTo(-1000, within(30.0));
    assertThat(rpm(DeltaRobot.TRANSITION)).isCloseTo(-2000, within(60.0));
  }

  /** The driver's left trigger toggles the shooter on at its one speed, and off again. */
  @Test
  void theDriversLeftTriggerTogglesTheShooter() {
    RobotSim.setEnabled(true);
    run(0.1);

    press(() -> sim.driver().setLeftTriggerAxis(1));
    run(0.1);
    press(() -> sim.driver().setLeftTriggerAxis(0));
    run(1.5);
    double on = rpm(DeltaRobot.SHOOTER);
    press(() -> sim.driver().setLeftTriggerAxis(1));
    run(0.1);
    press(() -> sim.driver().setLeftTriggerAxis(0));
    run(2);

    assertThat(on).isCloseTo(DeltaControls.SHOT_RPM, within(0.03 * DeltaControls.SHOT_RPM));
    assertThat(Math.abs(rpm(DeltaRobot.SHOOTER))).as("toggled off").isLessThan(50);
  }

  /** The heading the robot really has, in degrees. */
  private double heading() {
    return sim.simulation().truePose().getRotation().getDegrees();
  }

  /**
   * The driver's A and B hold the robot's back 20° either side of downfield while the left stick
   * drives it, for either alliance, and X turns the wheels into an X: the original's locks, from
   * each driver's own forward.
   */
  @Test
  void theDriversAAndBHoldTheHeadingEitherSideOfDownfieldAndXLocksTheWheels() {
    for (AllianceStationID station :
        new AllianceStationID[] {AllianceStationID.BLUE_2, AllianceStationID.RED_2}) {
      boolean red = station == AllianceStationID.RED_2;
      RobotSim.station(station);
      sim.loops(3);
      RobotSim.setEnabled(true);

      press(() -> sim.driver().setFaceDownButton(true));
      press(() -> sim.driver().setLeftY(-1)); // forward, while it holds
      run(1.5);
      double a = heading();
      press(() -> sim.driver().setFaceDownButton(false));
      press(() -> sim.driver().setLeftY(0));
      press(() -> sim.driver().setFaceRightButton(true));
      run(1.5);
      double b = heading();
      press(() -> sim.driver().setFaceRightButton(false));
      press(() -> sim.driver().setFaceLeftButton(true));
      run(1.5);
      var wheels = sim.drive().moduleLocations();
      var angles = sim.drive().moduleVelocities();
      press(() -> sim.driver().setFaceLeftButton(false));
      RobotSim.setEnabled(false);

      // The back 20° to the driver's right of downfield is the front 160° from it: 160° on blue
      // (downfield 0°), -20° on red (downfield 180°); B is the other side.
      assertThat(Math.IEEEremainder(a - (red ? -20 : 160), 360))
          .as("A on " + station)
          .isCloseTo(0, within(2.0));
      assertThat(Math.IEEEremainder(b - (red ? 20 : 200), 360))
          .as("B on " + station)
          .isCloseTo(0, within(2.0));
      for (int i = 0; i < 4; i++) {
        double towardCenter = Math.toDegrees(Math.atan2(wheels[i].getY(), wheels[i].getX()));
        assertThat(Math.IEEEremainder(angles[i].angle.getDegrees() - towardCenter, 180))
            .as("wheel " + i + " on " + station)
            .isCloseTo(0, within(2.0));
      }
    }
  }
}
