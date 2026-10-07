package frc.robot.robots2026;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import frc.robot.RobotSim;
import frc.robot.sim.JointsSimHarness;
import frc.robot.subsystems.mechanisms.JointMechanism;
import frc.robot.subsystems.mechanisms.Move;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Trigger;

/**
 * 2611's 2026 intake and shot ({@link Team2611Controls}), their joints alone in the simulator: the
 * behavior design's acceptance (docs/design/robot-behavior.md, checkpoints A and B). Their code at
 * 6a5da76 runs the roller once the pivot is past 110° only in auto (RobotContainer.java:143 vs
 * :216-218), and its readiness check gates nothing in teleop (:220-223). Here the guard is the
 * definition's interlock, beneath every command, and feeding goes through a gate. Each reaches its
 * joints by the design's own constants. A test fixture, not 2026 play.
 */
@ExtendWith(RobotSim.class)
class Team2611BehaviorSimTest {
  private final JointsSimHarness sim = new JointsSimHarness(Team2611.ROBOT);
  private final JointMechanism pivot = sim.joints.of(Team2611.PIVOT);
  private final JointMechanism roller = sim.joints.of(Team2611.INTAKE);
  private final JointMechanism fullSend = sim.joints.of(Team2611.FULL_SEND);

  /** The heading error the drive's aiming controller would report, in degrees. */
  private final double[] headingError = {0};

  private final Team2611Controls actions =
      new Team2611Controls(sim.scheduler, sim.joints, () -> headingError[0]);

  private final boolean[] held = {false};

  /** When the pivot first passed 110°, when the roller spun, and what held the roller first. */
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
          && Math.toDegrees(sim.simulation.position(Team2611.PIVOT)) >= 110) {
        passedRun = t;
      }
      if (Double.isNaN(spun) && Math.abs(sim.simulation.velocity(Team2611.INTAKE)) > 1) {
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
    assertThat(intaking.heldBy()).isEqualTo("intake runs only when pivot at least 110°");
    assertThat(roller.heldBy()).isEmpty();
    assertThat(Math.toDegrees(sim.simulation.position(Team2611.PIVOT))).isCloseTo(130, within(2.0));
    assertThat(pivot.lastOutcome()).matches("pivot → out: ARRIVED in 0\\.\\d\\d s");
    assertThat(sim.simulation.velocity(Team2611.INTAKE))
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
    assertThat(sim.simulation.velocity(Team2611.INTAKE)).isCloseTo(0, within(1.0));
    assertThat(Math.toDegrees(sim.simulation.position(Team2611.PIVOT))).isCloseTo(130, within(2.0));
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
    double feeding = sim.simulation.velocity(Team2611.FULL_SEND);
    double shooterSpeed = sim.simulation.velocity(Team2611.SHOOTER);

    headingError[0] = 10;
    sim.run(0.06);
    double ridingOut = sim.simulation.velocity(Team2611.FULL_SEND);
    List<String> blocking = actions.readyToShoot().blocking();
    sim.run(0.5);
    double paused = sim.simulation.velocity(Team2611.FULL_SEND);
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
    assertThat(sim.simulation.velocity(Team2611.FULL_SEND))
        .as("aimed again: feeding again")
        .isCloseTo(feedSpeed, within(0.03 * feedSpeed));
  }

  /** Today the hood is undriven: the template drives motors, and its two servos aren't. */
  @Test
  void theHoodsServosArentDrivenByTheTemplateYet() {
    assertThatThrownBy(() -> sim.joints.of(Team2611.HOOD))
        .hasMessage(
            "hood has no mechanism: it's moved by hood left and hood right, and the template drives"
                + " only motors turning a shaft so far");
  }

  /**
   * What the hood does once the template drives a joint by its servos, a go or no-go the owner
   * decides. Asked for 20°, both servos are told the share the linkage puts there, and carry it
   * there at their own speed. Nothing reads back where it is, so it arrives by time: no sooner than
   * its servos, at 32 mm/s, bring it within its 2° band, and not long after they'd reach 20°. Below
   * its range, below 2611's floor of 0.65, it's refused.
   */
  @Test
  @Disabled("the template doesn't drive servos yet: built on a trigger, a design that needs it")
  void theHoodGoesToAnAngleOnItsServosOnceTheyreDriven() {
    JointMechanism hood = sim.joints.of(Team2611.HOOD);
    Move raise = hood.goTo(Degrees.of(20));

    sim.scheduler.schedule(raise);
    double arrived = sim.runUntil(raise::arrived, 2);
    Move below = hood.goTo(Degrees.of(-5));
    sim.run(below, 0.1);

    assertThat(arrived).isBetween(servoSeconds(18), servoSeconds(20) + 0.1);
    assertThat(Math.toDegrees(sim.simulation.position(Team2611.HOOD))).isCloseTo(20, within(1.0));
    assertThat(Math.toDegrees(hood.position())).as("where it's assumed").isCloseTo(20, within(1.0));
    assertThat(below.reason())
        .isEqualTo("hood → -5°: REFUSED: -5° is outside hood's limits, 0° to 40°");
  }

  /** How long the hood's servos take, at their free speed, to bring it from 0° to an angle. */
  private static double servoSeconds(double degrees) {
    RobotMath math = Team2611.ROBOT.math();
    Drive left = Team2611.HOOD.ownDrives().get(0);
    double from = math.coordinate(left, Map.of(Team2611.HOOD, 0.0));
    double to = math.coordinate(left, Map.of(Team2611.HOOD, Math.toRadians(degrees)));
    return (to - from) / Team2611.WCP_0408.speed();
  }
}
