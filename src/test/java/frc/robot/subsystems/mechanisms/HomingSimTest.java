package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROLLER;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static frc.robot.subsystems.mechanisms.JointFixtures.homed;
import static frc.robot.subsystems.mechanisms.JointFixtures.setup;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import frc.robot.RobotSim;
import frc.robot.behavior.Gate;
import frc.robot.behavior.Outcome;
import frc.robot.lib.phoenix.MechanismControl.Homing;
import frc.robot.sim.JointsSimHarness;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * Joints read by their motors alone, powered on away from where their definition rests them: they
 * home into a hard stop on the first enable, and moves wait for them.
 */
@ExtendWith(RobotSim.class)
class HomingSimTest {
  /** The fixtures' joints, with one homed by a recipe instead of stowed by hand. */
  private static List<JointSetup> homing(int index, Homing recipe) {
    List<JointSetup> setups = new ArrayList<>(SETUPS);
    JointSetup setup = setups.get(index);
    setups.set(index, setup(setup.joint(), homed(setup.control(), recipe)));
    return setups;
  }

  private static final int PIVOT_SETUP = 0;
  private static final int ELEVATOR_SETUP = 2;

  /**
   * Powered on 0.3 m up, the elevator reads 0 m, its reference. Homing pushes it down into its stop
   * and takes that as 0 m; the move to "clear" waits for it, then takes it to the real 0.4 m.
   */
  @Test
  void aJointHomesIntoItsStopOnTheFirstEnableAndMovesWaitForIt() {
    JointsSimHarness sim =
        new JointsSimHarness(ROBOT, homing(ELEVATOR_SETUP, Homing.toHardStop(Homing.Stop.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    Gate ready = elevator.isReady();
    sim.simulation.powerOnAt(ELEVATOR, 0.3);
    Move clear = elevator.goTo("clear");

    sim.scheduler.schedule(clear);
    sim.loop();
    JointMechanism.State first = elevator.state();
    boolean readyFirst = ready.getAsBoolean();
    double took = sim.runUntil(() -> clear.outcome().isPresent(), 4);

    assertThat(first).isEqualTo(JointMechanism.State.HOMING);
    assertThat(readyFirst).isFalse();
    assertThat(clear.arrived()).isTrue();
    assertThat(took).as("homing, then the move").isGreaterThan(0.6).isLessThan(3);
    assertThat(elevator.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(ready.getAsBoolean()).isTrue();
    assertThat(sim.simulation.position(ELEVATOR)).isCloseTo(0.4, within(0.01));
    assertThat(elevator.position()).isCloseTo(0.4, within(0.01));
  }

  /** Disabled, it waits to home, and so do moves on it; enabled, it homes. */
  @Test
  void itHomesOnlyOnceEnabled() {
    JointsSimHarness sim =
        new JointsSimHarness(ROBOT, homing(ELEVATOR_SETUP, Homing.toHardStop(Homing.Stop.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.setEnabled(false);
    Move clear = elevator.goTo("clear");
    sim.scheduler.schedule(clear);

    sim.run(1);
    JointMechanism.State disabled = elevator.state();
    sim.setEnabled(true);
    sim.runUntil(() -> clear.outcome().isPresent(), 3);

    assertThat(disabled).isEqualTo(JointMechanism.State.UNHOMED);
    assertThat(clear.arrived()).isTrue();
  }

  /**
   * Powered on near its top, with a homing too short to reach the bottom: it's faulted, says why,
   * and refuses its moves; homed again from its stop, it's ready.
   */
  @Test
  void aHomingThatFindsNoStopFaultsTheJointUntilItHomesAgain() {
    Homing quick =
        Homing.ToHardStop.builder()
            .stop(Homing.Stop.LOWER)
            .volts(2)
            .timeout(Seconds.of(0.3))
            .build();
    JointsSimHarness sim = new JointsSimHarness(ROBOT, homing(ELEVATOR_SETUP, quick));
    JointMechanism elevator = sim.joints.of(ELEVATOR);
    sim.simulation.powerOnAt(ELEVATOR, 1.1);

    sim.run(0.5);
    Move clear = elevator.goTo("clear");
    sim.scheduler.schedule(clear);
    sim.runUntil(() -> clear.outcome().isPresent(), 1);
    sim.simulation.place(ELEVATOR, 0); // lowered by hand
    Move home = elevator.home();
    sim.scheduler.schedule(home);
    double took = sim.runUntil(() -> home.outcome().isPresent(), 2);

    String why = "elevator's homing found no stop in 0.3 s, pushing toward its lower one";
    assertThat(clear.outcome()).contains(Outcome.REFUSED);
    assertThat(clear.reason()).isEqualTo("elevator → clear: REFUSED: " + why);
    assertThat(home.arrived()).isTrue();
    assertThat(took).isCloseTo(0.30, within(0.03));
    assertThat(elevator.state()).isEqualTo(JointMechanism.State.READY);
  }

  @Test
  void aJointThatNeedsNoHomingIsHomeAtOnce() {
    JointsSimHarness sim = new JointsSimHarness(ROBOT, SETUPS);
    Move home = sim.joints.of(PIVOT).home();

    sim.run(home, 0.1);

    assertThat(home.reason()).isEqualTo("pivot homes: ARRIVED in 0.00 s");
  }

  /**
   * The pivot is really past "run", but until it's homed nobody knows: the roller's interlock fails
   * safe and holds it. Homed into its top stop, the pivot is known past "run", and the roller runs.
   */
  @Test
  void anInterlockOnAJointNotYetHomedHoldsFailSafe() {
    RobotDefinition guarded =
        RobotDefinitionBuilder.builder(ROBOT)
            .interlocks(List.of(Interlock.that(ROLLER).runsOnlyWhen(PIVOT.past("run"))))
            .build();
    JointsSimHarness sim =
        new JointsSimHarness(guarded, homing(PIVOT_SETUP, Homing.toHardStop(Homing.Stop.UPPER)));
    JointMechanism pivot = sim.joints.of(PIVOT);
    JointMechanism roller = sim.joints.of(ROLLER);
    sim.simulation.powerOnAt(PIVOT, Math.toRadians(120));
    sim.scheduler.schedule(roller.run("intaking"));

    sim.run(0.1);
    JointMechanism.State homing = pivot.state();
    double whileHoming = sim.simulation.velocity(ROLLER);
    String heldBy = roller.heldBy();
    sim.run(1.5);

    assertThat(homing).isEqualTo(JointMechanism.State.HOMING);
    assertThat(whileHoming).isCloseTo(0, within(0.5));
    assertThat(heldBy).isEqualTo("roller runs only when pivot past run");
    assertThat(pivot.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(Math.toDegrees(pivot.position())).isCloseTo(135, within(1.0));
    assertThat(sim.simulation.velocity(ROLLER)).isGreaterThan(100);
  }

  @Test
  void aStillJointAtItsStopHomesInAQuarterSecond() {
    JointsSimHarness sim =
        new JointsSimHarness(ROBOT, homing(ELEVATOR_SETUP, Homing.toHardStop(Homing.Stop.LOWER)));
    JointMechanism elevator = sim.joints.of(ELEVATOR);

    double took = sim.runUntil(() -> elevator.state() == JointMechanism.State.READY, 1);

    assertThat(took)
        .as("still for a quarter second, read a loop late")
        .isCloseTo(0.30, within(0.02));
  }
}
