package frc.robot.robots2026;

import static frc.robot.robots2026.Team2611.PARTS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.RPM;

import frc.robot.RobotSim;
import frc.robot.sim.JointsSimHarness;
import frc.robot.subsystems.mechanisms.JointMechanism;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Trigger;

/**
 * 2611's 2026 intake and shot ({@link Team2611Actions}), their joints alone in the simulator: the
 * behavior design's acceptance (docs/design/robot-behavior.md, checkpoints A and B). Their code at
 * 6a5da76 runs the roller once the pivot is past 110° only in auto (RobotContainer.java:143 vs
 * :216-218), and its readiness check gates nothing in teleop (:220-223). Here the guard is the
 * definition's interlock, beneath every command, and feeding goes through a gate. Each reaches its
 * joints through the design's parts. A test fixture, not 2026 play.
 */
@ExtendWith(RobotSim.class)
class Team2611BehaviorSimTest {
  private final JointsSimHarness sim = new JointsSimHarness(Team2611.ROBOT, Team2611.JOINTS);
  private final JointMechanism pivot = sim.joints.of(PARTS.pivot());
  private final JointMechanism roller = sim.joints.of(PARTS.intake());
  private final JointMechanism fullSend = sim.joints.of(PARTS.fullSend());

  /** The heading error the drive's aiming controller would report, in degrees. */
  private final double[] headingError = {0};

  private final Team2611Actions actions =
      new Team2611Actions(sim.joints, PARTS, sim.scheduler, () -> headingError[0]);

  private final boolean[] held = {false};

  /** When the pivot first passed "run", when the roller spun, and what held the roller first. */
  private record Intaking(double passedRun, double spun, String heldBy) {}

  /** Holds the intake button for a time. */
  private Intaking holdIntake(double seconds) {
    new Trigger(sim.scheduler, () -> held[0]).whileTrue(actions.intake());
    sim.loop();
    held[0] = true;
    double passedRun = Double.NaN;
    double spun = Double.NaN;
    String heldBy = "";
    for (int i = 1; i <= Math.round(seconds / 0.02); i++) {
      sim.loop();
      double t = i * 0.02;
      if (i == 2) {
        heldBy = roller.heldBy();
      }
      if (Double.isNaN(passedRun)
          && Math.toDegrees(sim.simulation.position(PARTS.pivot())) >= 110) {
        passedRun = t;
      }
      if (Double.isNaN(spun) && Math.abs(sim.simulation.velocity(PARTS.intake())) > 1) {
        spun = t;
      }
    }
    return new Intaking(passedRun, spun, heldBy);
  }

  @Test
  void holdingIntakeDeploysThePivotAndTheInterlockHoldsTheRollerUntilPastRun() {
    Intaking intaking = holdIntake(1.5);

    assertThat(intaking.passedRun()).as("the pivot passed 110°").isLessThan(0.6);
    assertThat(intaking.spun())
        .as("the roller spun only once the pivot was past it")
        .isGreaterThan(intaking.passedRun());
    assertThat(intaking.heldBy()).isEqualTo("intake runs only when pivot past run");
    assertThat(roller.heldBy()).isEmpty();
    assertThat(Math.toDegrees(sim.simulation.position(PARTS.pivot()))).isCloseTo(130, within(2.0));
    assertThat(pivot.lastOutcome()).matches("pivot → out: ARRIVED in 0\\.\\d\\d s");
    assertThat(sim.simulation.velocity(PARTS.intake()))
        .as("2000 rpm is the roller's free speed: it closes on it")
        .isCloseTo(
            RPM.of(2000).baseUnitMagnitude(), within(0.05 * RPM.of(2000).baseUnitMagnitude()));
  }

  /** Let go: the roller stops, and the pivot, idle, holds where it went, which theirs never did. */
  @Test
  void lettingGoStopsTheRollerAndThePivotStaysOut() {
    holdIntake(1.5);

    held[0] = false;
    sim.run(1);

    assertThat(roller.lastOutcome()).startsWith("intake: intaking: CANCELED after ");
    assertThat(sim.simulation.velocity(PARTS.intake())).isCloseTo(0, within(1.0));
    assertThat(Math.toDegrees(sim.simulation.position(PARTS.pivot()))).isCloseTo(130, within(2.0));
    assertThat(Math.toDegrees(pivot.goal())).isCloseTo(130, within(1e-9));
  }

  /**
   * Holding shoot: the shooter spins up and feeding starts once it's at speed and aimed; knocked
   * off aim, feeding pauses once the gate has been false for 80 ms, saying why; back on aim, it
   * feeds again.
   */
  @Test
  void shootingFeedsOnlyWhileReadyAndSaysWhyItPaused() {
    sim.scheduler.schedule(actions.shoot());
    sim.run(1.5);
    double feeding = sim.simulation.velocity(PARTS.fullSend());
    double shooterSpeed = sim.simulation.velocity(PARTS.shooter());

    headingError[0] = 10;
    sim.run(0.06);
    double ridingOut = sim.simulation.velocity(PARTS.fullSend());
    List<String> blocking = actions.readyToShoot().blocking();
    sim.run(0.5);
    double paused = sim.simulation.velocity(PARTS.fullSend());
    String why = fullSend.lastOutcome();

    headingError[0] = 1;
    sim.run(1);

    double feedSpeed = RPM.of(1000).baseUnitMagnitude();
    assertThat(shooterSpeed).isCloseTo(RPM.of(3000).baseUnitMagnitude(), within(0.03 * 314.16));
    assertThat(feeding).isCloseTo(feedSpeed, within(0.03 * feedSpeed));
    assertThat(ridingOut).as("the debounce rides out 60 ms").isGreaterThan(0.9 * feedSpeed);
    assertThat(blocking).isEmpty();
    assertThat(paused).isCloseTo(0, within(1.0));
    assertThat(why)
        .startsWith("at feeding: CANCELED after ")
        .endsWith(": ready to shoot is false: aimed");
    assertThat(sim.simulation.velocity(PARTS.fullSend()))
        .as("aimed again: feeding again")
        .isCloseTo(feedSpeed, within(0.03 * feedSpeed));
  }

  @Test
  void theHoodsServosArentDrivenByTheTemplateYet() {
    assertThatThrownBy(() -> sim.joints.of(PARTS.hood()))
        .hasMessage(
            "hood has no mechanism: it's moved by hood left and hood right, and the template drives"
                + " only motors turning a shaft so far");
  }
}
