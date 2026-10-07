package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.ELEVATOR_CLEAR;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT_IN;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT_OUT;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT_RUN;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROLLER;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROLLER_INTAKING;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static frc.robot.subsystems.mechanisms.JointFixtures.TURRET;
import static frc.robot.subsystems.mechanisms.JointFixtures.control;
import static frc.robot.subsystems.mechanisms.JointFixtures.profile;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import frc.robot.RobotSim;
import frc.robot.behavior.Outcome;
import frc.robot.sim.JointsSimHarness;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Translation3d;

/**
 * The definition's interlocks, beneath every command: the roller runs only with the pivot past
 * "run", and the pivot keeps out of 40° to 80° while the elevator is below "clear".
 */
@ExtendWith(RobotSim.class)
class InterlocksSimTest {
  static final Interlock ROLLER_ONLY_WHEN_OUT =
      Interlock.that(ROLLER).runsOnlyWhen(PIVOT.past(PIVOT_RUN));

  static final Interlock PIVOT_CLEARS_ELEVATOR =
      Interlock.that(PIVOT).keepsOut(Range.degrees(40, 80)).when(ELEVATOR.before(ELEVATOR_CLEAR));

  /** The pivot out and the elevator clear: the order between them is the interlock's. */
  static final Configuration DEPLOYED =
      Configuration.of("deployed", PIVOT.at(PIVOT_OUT), ELEVATOR.at(ELEVATOR_CLEAR));

  static final RobotDefinition GUARDED =
      RobotDefinitionBuilder.builder(ROBOT)
          .name("guarded")
          .configurations(List.of(DEPLOYED))
          .interlocks(List.of(ROLLER_ONLY_WHEN_OUT, PIVOT_CLEARS_ELEVATOR))
          .build();

  private final JointsSimHarness sim = new JointsSimHarness(GUARDED, SETUPS);
  private final JointMechanism pivot = sim.joints.of(PIVOT);
  private final JointMechanism roller = sim.joints.of(ROLLER);

  private double pivotDegrees() {
    return Math.toDegrees(sim.simulation.position(PIVOT));
  }

  /**
   * Asked to run with the pivot stowed, the roller stays still, and says what holds it; once
   * another command deploys the pivot past "run", it runs, its command never having stopped.
   */
  @Test
  void theRollerWaitsForThePivotToPassRun() {
    Move intaking = roller.run(ROLLER_INTAKING);
    sim.run(intaking, 0.5);
    double waiting = sim.simulation.velocity(ROLLER);
    String heldBy = roller.heldBy();

    sim.run(sim.joints.goTo(DEPLOYED), 2);

    assertThat(waiting).isCloseTo(0, within(0.5));
    assertThat(heldBy).isEqualTo("roller runs only when pivot past run");
    assertThat(intaking.outcome()).as("still running").isEmpty();
    assertThat(roller.heldBy()).isEmpty();
    assertThat(sim.simulation.velocity(ROLLER))
        .isCloseTo(RPM.of(1500).baseUnitMagnitude(), within(0.03 * 157.08));
  }

  @Test
  void theRuleHoldsWhicheverCommandAsks() {
    sim.run(roller.follow(() -> RPM.of(1000)), 0.5);

    assertThat(sim.simulation.velocity(ROLLER)).isCloseTo(0, within(0.5));
    assertThat(roller.heldBy()).isEqualTo("roller runs only when pivot past run");
  }

  /**
   * Both together: the pivot swings out to 40° and waits there while the elevator rises, then goes
   * on once it's clear. No planner: the interlock orders them.
   */
  @Test
  void aConfigurationMovesInTheOrderTheInterlocksAllow() {
    Move deploy = sim.joints.goTo(DEPLOYED);
    sim.scheduler.schedule(deploy);
    double[] furthestWhileLow = {0};

    double took =
        sim.runUntil(
            () -> {
              if (sim.simulation.position(ELEVATOR) < 0.4) {
                furthestWhileLow[0] = Math.max(furthestWhileLow[0], pivotDegrees());
              }
              return deploy.outcome().isPresent();
            },
            3);

    assertThat(deploy.arrived()).isTrue();
    assertThat(took).isLessThan(1.6);
    assertThat(furthestWhileLow[0])
        .as("short of the region while the elevator was low")
        .isLessThan(41);
    assertThat(furthestWhileLow[0]).as("waiting at its edge").isGreaterThan(38);
    assertThat(pivotDegrees()).isCloseTo(130, within(2.0));
  }

  @Test
  void heldPastItsDeadlineAMoveTimesOutAndSaysWhatHeldIt() {
    Move out = pivot.goTo(PIVOT_OUT);
    sim.scheduler.schedule(out);

    sim.runUntil(() -> out.outcome().isPresent(), 3);

    assertThat(out.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(out.reason())
        .startsWith("pivot → out: TIMED_OUT after 1.6")
        .endsWith(", held: pivot keeps out of 40° to 80° while elevator before clear");
    assertThat(pivotDegrees()).isCloseTo(40, within(1.5));
  }

  /** Inside the region when it closes, the pivot may leave it, but goes no deeper. */
  @Test
  void insideTheClosedRegionItMayLeaveButNotGoDeeper() {
    sim.simulation.place(PIVOT, Math.toRadians(60));
    sim.loop();
    Move deeper = pivot.goTo(Degrees.of(75)).within(Seconds.of(0.5));
    sim.scheduler.schedule(deeper);
    sim.runUntil(() -> deeper.outcome().isPresent(), 2);
    double afterDeeper = pivotDegrees();

    Move in = pivot.goTo(PIVOT_IN);
    sim.scheduler.schedule(in);
    sim.runUntil(() -> in.outcome().isPresent(), 2);

    assertThat(deeper.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(afterDeeper).isCloseTo(60, within(1.5));
    assertThat(in.arrived()).isTrue();
  }

  /**
   * A joint that overshoots a region's edge never goes on across it. Its profile asking more
   * braking than its motor has, the pivot runs 19° past 40° into the region; the interlock had it
   * heading for the edge, and now inside, its goal across the region is refused: it may leave only
   * the way it came. Until 2026-10-05 a joint inside could head for any goal outside, and a team's
   * arm crossed the arc its interlock guards once it overshot the edge.
   */
  @Test
  void aJointThatOvershootsTheEdgeNeverGoesOnAcross() {
    JointsSimHarness swinging =
        new JointsSimHarness(
            GUARDED,
            List.of(
                JointFixtures.setup(PIVOT, JointFixtures.assumed(control(20, 0, profile(12, 200)))),
                SETUPS.get(1),
                SETUPS.get(2),
                SETUPS.get(3)));
    JointMechanism swung = swinging.joints.of(PIVOT);
    Move out = swung.goTo(PIVOT_OUT);
    swinging.scheduler.schedule(out);
    double furthest = 0;
    for (int i = 0; i < 100; i++) {
      swinging.loop();
      furthest = Math.max(furthest, Math.toDegrees(swinging.simulation.position(PIVOT)));
    }

    assertThat(furthest).as("it overshot the edge into the region").isGreaterThan(41);
    assertThat(furthest).as("and never crossed it").isLessThan(80);
    assertThat(out.outcome()).contains(Outcome.TIMED_OUT);
    assertThat(out.reason())
        .endsWith("held: pivot keeps out of 40° to 80° while elevator before clear");
  }

  /**
   * The same rules written in the restricted joints' blocks hold the same: the roller, asked to run
   * with the pivot stowed, stays still and says what holds it.
   */
  @Test
  void rulesInTheBlocksHoldAsTheRobotsDo() {
    RobotDefinition inBlocks =
        RobotDefinitionBuilder.builder(ROBOT)
            .name("guarded in blocks")
            .configurations(List.of(DEPLOYED))
            .build()
            .with(
                ROLLER.runsOnlyWhen(PIVOT.past(PIVOT_RUN)),
                PIVOT.keepsOut(Range.degrees(40, 80)).when(ELEVATOR.before(ELEVATOR_CLEAR)));
    JointsSimHarness blocks = new JointsSimHarness(inBlocks, SETUPS);
    JointMechanism held = blocks.joints.of(ROLLER);

    blocks.run(held.run(ROLLER_INTAKING), 0.5);

    assertThat(blocks.simulation.velocity(ROLLER)).isCloseTo(0, within(0.5));
    assertThat(held.heldBy()).isEqualTo("roller runs only when pivot past run");
  }

  /**
   * A joint held until another is up to speed: the pivot, asked out, waits where it is while the
   * roller is still, and goes once the roller runs at its 1500 rpm, past the 1000 it waits for.
   */
  @Test
  void aSpeedHoldsAJointUntilAnotherIsUpToIt() {
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT)
            .name("spun up")
            .configurations(List.of())
            .interlocks(List.of(Interlock.that(PIVOT).runsOnlyWhen(ROLLER.atLeast(RPM.of(1000)))))
            .build();
    JointsSimHarness spun = new JointsSimHarness(robot, SETUPS);
    JointMechanism held = spun.joints.of(PIVOT);
    Move out = held.goTo(PIVOT_OUT);
    spun.scheduler.schedule(out);

    spun.run(0.5);
    double waiting = Math.toDegrees(spun.simulation.position(PIVOT));
    String heldBy = held.heldBy();
    spun.scheduler.schedule(spun.joints.of(ROLLER).run(ROLLER_INTAKING));
    spun.runUntil(() -> out.outcome().isPresent(), 3);

    assertThat(waiting).isCloseTo(1.5, within(1.0));
    assertThat(heldBy).startsWith("pivot runs only when roller at least 1000");
    assertThat(out.arrived()).isTrue();
    assertThat(Math.toDegrees(spun.simulation.position(PIVOT))).isCloseTo(130, within(2.0));
  }

  /** An interlock about a joint the template doesn't drive can't be enforced, so it's refused. */
  @Test
  void anInterlockTheTemplateCantEnforceIsRefused() {
    var hatch =
        JointRevoluteBuilder.builder(PIVOT)
            .label("hatch")
            .link(JointFixtures.link("hatch", 1, new Translation3d()))
            .build();
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT)
            .joints(List.of(PIVOT, ROLLER, ELEVATOR, TURRET, hatch))
            .configurations(List.of())
            .interlocks(List.of(Interlock.that(ROLLER).runsOnlyWhen(hatch.past(PIVOT_RUN))))
            .build();

    assertThatThrownBy(
            () ->
                new Joints(
                    robot,
                    SETUPS,
                    setup -> new JointIO() {},
                    Scheduler.createIndependentScheduler()))
        .hasMessage(
            "\"roller runs only when hatch past run\" can't be enforced: hatch has no mechanism, as"
                + " nothing drives it");
  }
}
