package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT_RUN;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.control;
import static frc.robot.subsystems.mechanisms.JointFixtures.profile;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import frc.robot.RobotSim;
import frc.robot.behavior.Outcome;
import frc.robot.sim.JointsSimHarness;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A joint whose loop closes on its absolute encoder on a Systemcore port: the robot works out its
 * motor's voltage each loop from where the encoder says it is, so a motor's count that slipped
 * doesn't send it to the wrong place, as it does a joint whose encoder only told its motor where it
 * was at startup.
 */
@ExtendWith(RobotSim.class)
class EncoderLoopSimTest {
  /** The pivot's through-bore on port 1, mounted reversed and turned a third of a turn. */
  private static final MechanismWiring.PortEncoder PORT_1 =
      MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0.3)).inverted();

  /** Its encoder tells its motor where it is, once. */
  private static final Joint.Revolute SEEDED =
      PIVOT.absoluteEncoder(1024, PORT_1).retuned(old -> control(30, 1, profile(8, 30)));

  /** Its loop closes on its encoder. */
  private static final Joint.Revolute CLOSED = SEEDED.encoderOn(PORT_1.closesLoop());

  private static JointsSimHarness sim(Joint.Revolute pivot) {
    RobotDefinition robot = ROBOT.with(pivot);
    return new JointsSimHarness(robot);
  }

  /** Runs a move to an end, at most 3 s. */
  private static Move run(JointsSimHarness sim, Move move) {
    sim.scheduler.schedule(move);
    sim.runUntil(() -> move.outcome().isPresent(), 3);
    return move;
  }

  /**
   * Its motor's count slips 20° behind the joint: told once where it was, the pivot is sent to its
   * run position by its motor's count, and arrives there by that count 20° past it; closed on its
   * encoder, it arrives at the run position itself, and knows it.
   */
  @Test
  void aSlippedMotorSendsASeededPivotToTheWrongPlaceAndNotOneClosedOnItsEncoder() {
    JointsSimHarness seeded = sim(SEEDED);
    JointsSimHarness closed = sim(CLOSED);
    for (JointsSimHarness each : new JointsSimHarness[] {seeded, closed}) {
      each.run(0.1);
      // The joint's at 21.5°, still, and its motor's count reads where it rests, 1.5°.
      each.simulation.powerOnAt(PIVOT, Math.toRadians(21.5));
      each.run(0.1);
    }

    Move wrong = run(seeded, seeded.joints.of(PIVOT).goTo(PIVOT_RUN));
    Move right = run(closed, closed.joints.of(PIVOT).goTo(PIVOT_RUN));
    closed.run(0.5);

    assertThat(wrong.arrived()).as("by its motor's count").isTrue();
    assertThat(Math.toDegrees(seeded.simulation.position(PIVOT))).isCloseTo(130, within(2.0));
    assertThat(right.arrived()).isTrue();
    assertThat(Math.toDegrees(closed.simulation.position(PIVOT))).isCloseTo(110, within(2.0));
    assertThat(closed.joints.of(PIVOT).position())
        .as("where it says it is, from its encoder")
        .isCloseTo(closed.simulation.position(PIVOT), within(Math.toRadians(0.5)));
  }

  /**
   * Closed on its encoder, the pivot holds its place against its weight, the robot's loop running
   * its feedforward, gravity included, as its motor controller's would.
   */
  @Test
  void itHoldsWhereItArrivesAgainstItsWeight() {
    JointsSimHarness sim = sim(CLOSED);
    sim.run(0.1);

    Move out = run(sim, sim.joints.of(PIVOT).goTo(PIVOT_RUN));
    sim.run(2);

    assertThat(out.arrived()).isTrue();
    assertThat(Math.toDegrees(sim.simulation.position(PIVOT))).isCloseTo(110, within(1.0));
    assertThat(Math.toDegrees(sim.simulation.velocity(PIVOT))).isCloseTo(0, within(1.0));
  }

  /**
   * The encoder its loop closes on comes unplugged as it moves: it's faulted, its move ends
   * refused, saying why, and the next is refused; it comes to a stop and holds there on its motor's
   * count rather than going limp. Plugged back in, it's ready at once, where the encoder says, and
   * moves again.
   */
  @Test
  void aLostEncoderFaultsItAndItHoldsOnItsMotorUntilTheEncoderAnswers() {
    JointsSimHarness sim = sim(CLOSED);
    JointMechanism pivot = sim.joints.of(PIVOT);
    sim.run(0.1);
    Move out = pivot.goTo(PIVOT_OUT);
    sim.scheduler.schedule(out);
    sim.run(0.3);

    sim.simulation.unplug(PIVOT, true);
    sim.loop();
    JointMechanism.State lost = pivot.state();
    Move refused = run(sim, pivot.goTo(PIVOT_RUN));
    sim.run(0.5);
    double stopped = sim.simulation.position(PIVOT);
    sim.run(1);
    double held = sim.simulation.position(PIVOT);
    sim.simulation.unplug(PIVOT, false);
    sim.loop();
    JointMechanism.State back = pivot.state();
    double knows = pivot.position();
    double truly = sim.simulation.position(PIVOT);
    Move again = run(sim, pivot.goTo(PIVOT_RUN));

    String why =
        "pivot's absolute encoder is lost, on port 1: its loop closes on it, so it holds where its"
            + " motor says until the encoder answers";
    assertThat(lost).isEqualTo(JointMechanism.State.FAULTED);
    assertThat(out.outcome()).contains(Outcome.REFUSED);
    assertThat(out.reason()).isEqualTo("pivot → out: REFUSED: can't go on: " + why);
    assertThat(refused.outcome()).contains(Outcome.REFUSED);
    assertThat(refused.reason()).isEqualTo("pivot → run: REFUSED: " + why);
    assertThat(Math.toDegrees(stopped)).as("stopped on its way out").isBetween(60.0, 130.0);
    assertThat(Math.toDegrees(held))
        .as("held where it stopped, not sagging")
        .isCloseTo(Math.toDegrees(stopped), within(0.5));
    assertThat(back).isEqualTo(JointMechanism.State.READY);
    assertThat(knows).isCloseTo(truly, within(Math.toRadians(0.5)));
    assertThat(again.arrived()).isTrue();
    assertThat(Math.toDegrees(sim.simulation.position(PIVOT))).isCloseTo(110, within(2.0));
  }
}
