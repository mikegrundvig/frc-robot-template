package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static frc.robot.subsystems.mechanisms.JointFixtures.TURRET;
import static frc.robot.subsystems.mechanisms.JointFixtures.TURRET_FRONT;
import static frc.robot.subsystems.mechanisms.JointFixtures.homed;
import static frc.robot.subsystems.mechanisms.JointFixtures.setup;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.robot.MechanismControl.Homing;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.RobotSim;
import frc.robot.sim.JointsSimHarness;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * A joint homed to its limit switch: driven gently toward it on the first enable, it's where the
 * switch says the loop it trips. One that starts on it backs off first, so it's home where the
 * switch trips, not wherever past it it rested.
 */
@ExtendWith(RobotSim.class)
class SwitchHomingSimTest {
  /** The pivot's bottom switch, tripping at 5°: above where it rests, at 1.5°. */
  private static final Sensor.Switch PIVOT_DOWN =
      Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, Math.toRadians(5));

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT).sensors(List.of(PIVOT_DOWN)).build();

  /** The turret's home switch, at -90°: it trips at or past it, turning left. */
  private static final Sensor.Switch TURRET_HOME =
      Sensor.Switch.limit(
          "turret home", TURRET, Sensor.Switch.Limit.End.LOWER, Math.toRadians(-90));

  private static final RobotDefinition TURRET_ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT).sensors(List.of(TURRET_HOME)).build();

  private static List<JointSetup> homingPivot(RobotDefinition robot) {
    List<JointSetup> setups = new ArrayList<>(SETUPS);
    setups.set(0, setup(PIVOT, homed(SETUPS.get(0).control(), Homing.toSwitch(Homing.Stop.LOWER))));
    return setups;
  }

  private final JointsSimHarness sim = new JointsSimHarness(ROBOT, homingPivot(ROBOT));
  private final JointMechanism pivot = sim.joints.of(PIVOT);

  private static double degrees(double radians) {
    return Math.toDegrees(radians);
  }

  /**
   * The turret, powered on 40° to the right of where it reads (0°), homes to its switch at -90° at
   * 1 V: driven left until the switch trips, it takes that as -90°, and the move to the front waits
   * for it, then lands at the real 0°, within a loop's travel at the homing's speed.
   */
  @Test
  void itHomesWhereItsSwitchTripsAndMovesWaitForIt() {
    List<JointSetup> setups = new ArrayList<>(SETUPS);
    setups.set(
        3,
        setup(
            TURRET,
            homed(
                SETUPS.get(3).control(),
                Homing.ToSwitch.builder()
                    .end(Homing.Stop.LOWER)
                    .volts(1)
                    .timeout(Seconds.of(5))
                    .build())));
    JointsSimHarness turretSim = new JointsSimHarness(TURRET_ROBOT, setups);
    JointMechanism turret = turretSim.joints.of(TURRET);
    turretSim.simulation.powerOnAt(TURRET, Math.toRadians(-40));
    Move front = turret.goTo(TURRET_FRONT).within(Seconds.of(6));
    turretSim.scheduler.schedule(front);

    turretSim.runUntil(() -> front.outcome().isPresent(), 8);

    assertThat(front.arrived()).isTrue();
    assertThat(degrees(turretSim.simulation.position(TURRET))).isCloseTo(0, within(1.5));
  }

  /**
   * Powered on resting at 1.5°, past where its switch trips, it backs off until the switch clears,
   * then comes back until it trips. Falling back under gravity and its 2 V, it travels 4° a loop,
   * so it reads within half of that: gentler homing is closer.
   */
  @Test
  void oneStartingOnItsSwitchBacksOffFirst() {
    double highest = 0;
    for (int i = 0; i < 100; i++) {
      sim.loop();
      highest = Math.max(highest, degrees(sim.simulation.position(PIVOT)));
    }

    assertThat(highest).as("off the switch first").isGreaterThan(5);
    assertThat(pivot.state()).isEqualTo(JointMechanism.State.READY);
    assertThat(degrees(pivot.position()))
        .isCloseTo(degrees(sim.simulation.position(PIVOT)), within(2.2));
  }

  /** With its switch lost, homing never ends well: the joint's faulted, saying why. */
  @Test
  void aLostSwitchFaultsItsHoming() {
    sim.simulation.powerOnAt(PIVOT, Math.toRadians(40));
    sim.sensors.silence(PIVOT_DOWN, true);

    sim.run(3.5);

    assertThat(pivot.state()).isEqualTo(JointMechanism.State.FAULTED);
    assertThat(pivot.fault())
        .isEqualTo("pivot's homing didn't trip pivot down in 3.0 s: pivot down is lost");
  }

  /** A joint homed to a switch it hasn't, or one whose position isn't known, is refused. */
  @Test
  void homingToASwitchNeedsOneWhosePositionIsKnown() {
    RobotDefinition unknown =
        RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
            .sensors(
                List.of(Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER)))
            .build();

    assertThat(JointSetup.problems(JointFixtures.ROBOT, homingPivot(JointFixtures.ROBOT)))
        .isEqualTo(
            "pivot homes to its lower limit switch, and its definition has none: add it"
                + " (Sensor.Switch.limit)");
    assertThat(JointSetup.problems(unknown, homingPivot(unknown)))
        .isEqualTo(
            "pivot homes to pivot down, whose position isn't known: measure where it trips, or"
                + " home another way");
  }
}
