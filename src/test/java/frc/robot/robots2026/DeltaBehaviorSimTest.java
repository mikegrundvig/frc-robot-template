package frc.robot.robots2026;

import static com.michaelgrundvig.frc.runtime.Ended.ended;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.sim.JointsSimHarness;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.Held;
import com.michaelgrundvig.frc.runtime.subsystems.mechanisms.JointMechanism;
import com.michaelgrundvig.frc.world.robot.Interlock;
import frc.robot.DeltaControls;
import frc.robot.DeltaRobot;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Trigger;

/**
 * A team's 2026 Delta pivot, roller, feeders and shooter ({@link DeltaControls}), their joints
 * alone in the simulator: the pivot swung out at 5 V and in at -3 V until it reads out or in, the
 * roller held by the definition's interlock until the pivot is out, the driver's feed, and the
 * shooter at its one speed. Each reaches its joints by the design's own constants. A test fixture,
 * not 2026 play.
 */
@ExtendWith(RobotSim.class)
class DeltaBehaviorSimTest {
  private final JointsSimHarness sim = new JointsSimHarness(DeltaRobot.ROBOT);
  private final JointMechanism pivot = sim.joints.of(DeltaRobot.PIVOT);
  private final JointMechanism roller = sim.joints.of(DeltaRobot.INTAKE);
  private final JointMechanism fullSend = sim.joints.of(DeltaRobot.FULL_SEND);
  private final DeltaControls actions = new DeltaControls(sim.joints);

  private final boolean[] swingOut = {false};
  private final boolean[] swingIn = {false};

  private double pivotDegrees() {
    return Math.toDegrees(sim.simulation.position(DeltaRobot.PIVOT));
  }

  /** Holds the pivot's out button for a time, as the operator's A button is. */
  private void holdOut(double seconds) {
    new Trigger(sim.scheduler, () -> swingOut[0]).whileTrue(actions.pivotOut());
    sim.loop();
    swingOut[0] = true;
    sim.run(seconds);
  }

  @Test
  void holdingOutSwingsThePivotPastOutAndItEndsThereWhileTheButtonIsHeld() {
    holdOut(1.5);

    assertThat(pivotDegrees()).as("past 130°, at or short of its stop").isBetween(130.0, 135.0);
    assertThat(sim.simulation.velocity(DeltaRobot.PIVOT)).isCloseTo(0, within(0.05));
    ended(pivot).named("pivot: swinging out until intake is out").arrived();
    assertThat(actions.intakeIsOut().getAsBoolean()).isTrue();
  }

  @Test
  void holdingInSwingsThePivotDownAndItEndsAtThree() {
    holdOut(1.5);
    swingOut[0] = false;
    sim.run(0.5);
    new Trigger(sim.scheduler, () -> swingIn[0]).whileTrue(actions.pivotIn());
    sim.loop();
    swingIn[0] = true;
    sim.run(2);

    assertThat(pivotDegrees()).as("at or under 3°, short of its stop").isBetween(0.0, 3.0);
    ended(pivot).named("pivot: swinging in until intake is in").arrived();
    assertThat(actions.intakeIsIn().getAsBoolean()).isTrue();
  }

  /** The roller waits for the pivot: held from the start, running once the pivot is out. */
  @Test
  void theInterlockHoldsTheRollerUntilThePivotIsOut() {
    sim.scheduler.schedule(roller.run(DeltaRobot.INTAKING));
    sim.run(0.2);
    List<Interlock> held = Held.interlocksHolding(roller);
    double stowed = sim.simulation.velocity(DeltaRobot.INTAKE);

    holdOut(1.5);
    double out = RPM.convertFrom(sim.simulation.velocity(DeltaRobot.INTAKE), RadiansPerSecond);

    assertThat(held)
        .containsExactly(
            Interlock.that(DeltaRobot.INTAKE)
                .runsOnlyWhen(DeltaRobot.PIVOT.atLeast(Degrees.of(130))));
    assertThat(stowed).isCloseTo(0, within(1e-6));
    assertThat(out).as("3000 rpm is the roller's free speed: it closes on it").isGreaterThan(2850);
    assertThat(roller.holders()).isEmpty();
  }

  @Test
  void theDriversFeedRunsBothRollersAndTheUnjamReversesThem() {
    sim.scheduler.schedule(sim.joints.run(DeltaRobot.FEEDING));
    sim.run(1);
    double feeding =
        RPM.convertFrom(sim.simulation.velocity(DeltaRobot.FULL_SEND), RadiansPerSecond);
    double transition =
        RPM.convertFrom(sim.simulation.velocity(DeltaRobot.TRANSITION), RadiansPerSecond);

    sim.scheduler.schedule(sim.joints.run(DeltaRobot.UNJAMMING));
    sim.run(1);

    assertThat(feeding).isCloseTo(1000, within(30.0));
    assertThat(transition).isCloseTo(2000, within(60.0));
    assertThat(RPM.convertFrom(sim.simulation.velocity(DeltaRobot.FULL_SEND), RadiansPerSecond))
        .isCloseTo(-1000, within(30.0));
    ended(fullSend).named("full send and transition: feeding").canceled();
  }

  @Test
  void theShooterRunsAtItsOneSpeed() {
    sim.scheduler.schedule(
        sim.joints.of(DeltaRobot.SHOOTER).follow(() -> RPM.of(DeltaControls.SHOT_RPM)));
    sim.run(1.5);

    assertThat(RPM.convertFrom(sim.simulation.velocity(DeltaRobot.SHOOTER), RadiansPerSecond))
        .isCloseTo(2900, within(0.03 * 2900));
  }
}
