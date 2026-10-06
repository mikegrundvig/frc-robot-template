package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.INTAKING;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROLLER;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;

import frc.robot.RobotSim;
import frc.robot.behavior.Gate;
import frc.robot.sim.JointsSimHarness;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/** A joint's gates, read from the simulator: each on what the joint measured this loop. */
@ExtendWith(RobotSim.class)
class JointGatesSimTest {
  private final JointsSimHarness sim = new JointsSimHarness(ROBOT, SETUPS);
  private final JointMechanism pivot = sim.joints.of(PIVOT);
  private final JointMechanism roller = sim.joints.of(ROLLER);

  /** Sends the pivot to an angle, waits for it to settle, and says whether a gate holds there. */
  private boolean at(Gate gate, double degrees) {
    sim.run(pivot.goTo(Degrees.of(degrees)), 1.2);
    return gate.getAsBoolean();
  }

  /** Into 2° of "out" to be at it, and out past 4° to leave: the enter and stay bands. */
  @Test
  void isAtArrivesInsideTheEnterBandAndLeavesPastTheStayBand() {
    Gate atOut = pivot.isAt("out");
    List<Boolean> seen = new ArrayList<>();

    for (double degrees : new double[] {127, 128.6, 127, 125.5, 128.6}) {
      seen.add(at(atOut, degrees));
    }

    assertThat(atOut.name()).isEqualTo("pivot at out");
    assertThat(seen).containsExactly(false, true, true, false, true);
  }

  @Test
  void isPastAndIsBetweenReadWhereTheJointIs() {
    Gate past = pivot.isPast("run");
    Gate between = pivot.isBetween("run", "out");
    sim.loop();
    boolean[] stowed = {past.getAsBoolean(), between.getAsBoolean()};

    boolean[] at120 = {at(past, 120), between.getAsBoolean()};
    boolean[] at60 = {at(past, 60), between.getAsBoolean()};

    assertThat(stowed).containsExactly(false, false);
    assertThat(at120).containsExactly(true, true);
    assertThat(at60).containsExactly(false, false);
    assertThat(between.name()).isEqualTo("pivot between run and out");
  }

  @Test
  void isAtGoalWhileMovingIsFalseAndTrueOnceThere() {
    Gate atGoal = pivot.isAtGoal();
    sim.run(0.1);
    boolean holding = atGoal.getAsBoolean();

    sim.run(pivot.goTo("out"), 0.2);
    boolean moving = atGoal.getAsBoolean();
    sim.run(1);

    assertThat(holding).as("idle, holding where it rests").isTrue();
    assertThat(moving).isFalse();
    assertThat(atGoal.getAsBoolean()).isTrue();
  }

  @Test
  void aRollerIsAtGoalAtItsSpeedOrStopped() {
    Gate atGoal = roller.isAtGoal();
    Gate atIntaking = roller.isAt("intaking");
    sim.run(0.1);
    boolean[] stopped = {atGoal.getAsBoolean(), atIntaking.getAsBoolean()};

    sim.run(roller.run("intaking"), 1);
    boolean[] intaking = {atGoal.getAsBoolean(), atIntaking.getAsBoolean()};
    sim.run(roller.run("outtaking"), 1);

    assertThat(stopped).containsExactly(true, false);
    assertThat(intaking).containsExactly(true, true);
    assertThat(atIntaking.getAsBoolean()).isFalse();
    assertThat(atGoal.getAsBoolean()).as("at its voltage").isTrue();
    assertThat(atIntaking.name()).isEqualTo("roller at intaking");
  }

  /**
   * Pushed up at 3 V, the pivot swings to its top stop and stops there: stalled a quarter of a
   * second later, and not while it was still swinging.
   */
  @Test
  void aJointPushedIntoItsStopIsStalled() {
    Gate stalled = pivot.isStalled();
    sim.run(pivot.run("push"), 0.2);
    boolean swinging = stalled.getAsBoolean();

    sim.run(1.5);

    assertThat(swinging).isFalse();
    assertThat(Math.toDegrees(sim.simulation.position(PIVOT))).isCloseTo(135, within(0.5));
    assertThat(stalled.getAsBoolean()).isTrue();
    assertThat(stalled.name()).isEqualTo("pivot stalled");
  }

  @Test
  void aConfigurationsGateIsEveryJointAtItsPlace() {
    Gate atIntaking = sim.joints.isAt(INTAKING);
    sim.loop();
    List<String> blocking = atIntaking.blocking();

    sim.run(sim.joints.run(INTAKING), 2);

    assertThat(atIntaking.name()).isEqualTo("at intaking");
    assertThat(blocking).containsExactly("pivot at out", "roller at intaking", "elevator at clear");
    assertThat(atIntaking.getAsBoolean()).isTrue();
  }

  @Test
  void aReadyJointIsReady() {
    Gate ready = pivot.isReady();
    sim.loop();

    assertThat(ready.getAsBoolean()).isTrue();
    assertThat(pivot.state()).isEqualTo(JointMechanism.State.READY);
  }
}
