package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static frc.robot.subsystems.mechanisms.JointFixtures.TURRET;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.RobotSim;
import frc.robot.sim.JointsSimHarness;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.command3.Command;

/**
 * A tripped limit switch stops its joint in the robot's code, beneath every command: a move past it
 * holds where it tripped, an output toward it goes neutral, and the joint is free to leave. The
 * only stop for a switch its controller can't read; beside the controller's own where it can.
 */
@ExtendWith(RobotSim.class)
class LimitStopSimTest {
  /** At 90° of the turret's swing, on its way to the back. */
  private static final Sensor.Switch TURRET_RIGHT =
      Sensor.Switch.limit(
          "turret right", TURRET, Sensor.Switch.Limit.End.UPPER, Math.toRadians(90));

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT).sensors(List.of(TURRET_RIGHT)).build();

  private final JointsSimHarness sim = new JointsSimHarness(ROBOT, SETUPS);
  private final JointMechanism turret = sim.joints.of(TURRET);

  private double turretDegrees() {
    return Math.toDegrees(sim.simulation.position(TURRET));
  }

  /**
   * A move to the back holds where the switch tripped, says so, and times out; back home is free.
   */
  @Test
  void aMovePastATrippedSwitchHoldsWhereItTripped() {
    Move back = turret.goTo("back");
    sim.scheduler.schedule(back);
    sim.runUntil(() -> back.outcome().isPresent(), 5);

    assertThat(back.outcome()).contains(Move.Outcome.TIMED_OUT);
    assertThat(back.reason()).contains("held: turret right tripped");
    assertThat(turretDegrees()).as("stopped past 90°, braking").isBetween(90.0, 100.0);

    Move front = turret.goTo("front");
    sim.scheduler.schedule(front);
    sim.runUntil(() -> front.outcome().isPresent(), 3);

    assertThat(front.arrived()).as("leaving it is free").isTrue();
    assertThat(turretDegrees()).isCloseTo(0, within(2.0));
  }

  /** A voltage toward the switch goes neutral once it trips; away from it, it's applied. */
  @Test
  void anOutputTowardATrippedSwitchGoesNeutral() {
    Command push =
        turret
            .run(
                co -> {
                  while (true) {
                    turret.setVoltage(2);
                    co.yield();
                  }
                })
            .named("Push right");
    sim.run(push, 3);

    assertThat(turretDegrees()).isBetween(90.0, 110.0);
    assertThat(turret.heldBy()).isEqualTo("turret right tripped");
  }
}
