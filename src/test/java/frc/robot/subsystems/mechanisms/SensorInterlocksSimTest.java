package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROLLER;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.RobotSim;
import frc.robot.sim.JointsSimHarness;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;

/**
 * Interlocks on what a sensor reads, enforced beneath every command as those on a joint's position
 * are: a roller that runs only while its beam is clear, a pivot kept out of a band while something
 * is near. A lost sensor restricts, as a joint that doesn't know where it is does.
 */
@ExtendWith(RobotSim.class)
class SensorInterlocksSimTest {
  private static final Sensor.Switch CLAW_BEAM = Sensor.Switch.beam("claw beam", PIVOT);

  private static final Sensor.Rangefinder REEF =
      Sensor.Rangefinder.builder()
          .label("reef range")
          .part(Part.CHASSIS)
          .pose(Optional.empty())
          .range(Meters.of(1))
          .fieldOfView(Degrees.of(27))
          .build();

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
          .sensors(List.of(CLAW_BEAM, REEF))
          .interlocks(
              List.of(
                  Interlock.that(ROLLER).runsOnlyWhen(CLAW_BEAM.clear()),
                  Interlock.that(PIVOT)
                      .keepsOut(Range.degrees(60, 90))
                      .when(REEF.nearerThan(Meters.of(0.3)))))
          .build();

  private final JointsSimHarness sim = new JointsSimHarness(ROBOT, SETUPS);
  private final JointMechanism pivot = sim.joints.of(PIVOT);
  private final JointMechanism roller = sim.joints.of(ROLLER);

  /** The roller runs while the beam is clear, stops as it trips, and stops with it lost. */
  @Test
  void aRollerRunsOnlyWhileItsBeamIsClear() {
    sim.scheduler.schedule(roller.run("intaking"));
    sim.run(1);
    double running = sim.simulation.velocity(ROLLER);

    sim.sensors.trip(CLAW_BEAM, true);
    sim.run(1);
    double tripped = sim.simulation.velocity(ROLLER);
    String heldBy = roller.heldBy();

    sim.sensors.trip(CLAW_BEAM, false);
    sim.sensors.silence(CLAW_BEAM, true);
    sim.run(1);

    assertThat(running).as("1500 rpm through 3:1, in rad/s").isGreaterThan(40);
    assertThat(tripped).isCloseTo(0, within(1.0));
    assertThat(heldBy).isEqualTo("roller runs only when claw beam clear");
    assertThat(sim.simulation.velocity(ROLLER)).as("lost: held").isCloseTo(0, within(1.0));
  }

  /**
   * The pivot on its way out waits at the band's edge while the reef is near, and goes on once it's
   * clear.
   */
  @Test
  void aPivotKeepsOutOfABandWhileSomethingIsNear() {
    sim.sensors.see(REEF, Meters.of(0.2));
    Move out = pivot.goTo("out").within(Seconds.of(5));
    sim.scheduler.schedule(out);
    sim.run(1.5);
    double waiting = Math.toDegrees(sim.simulation.position(PIVOT));
    String heldBy = pivot.heldBy();

    sim.sensors.see(REEF, Meters.of(0.8));
    sim.runUntil(() -> out.outcome().isPresent(), 3);

    assertThat(waiting).as("at the band's edge").isCloseTo(60, within(2.0));
    assertThat(heldBy)
        .isEqualTo("pivot keeps out of 60° to 90° while reef range nearer than 0.300 m");
    assertThat(out.arrived()).isTrue();
  }

  /** An interlock on a sensor the robot doesn't have is refused as the definition is built. */
  @Test
  void anInterlockOnAStrangersSensorIsRefused() {
    Sensor.Switch stray = Sensor.Switch.beam("stray beam", PIVOT);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(ROBOT)
                    .interlocks(List.of(Interlock.that(ROLLER).runsOnlyWhen(stray.clear())))
                    .build())
        .withMessageContaining(
            "interlock \"roller runs only when stray beam clear\"'s sensor, stray beam, isn't one"
                + " of the robot's sensors: list it");
  }
}
