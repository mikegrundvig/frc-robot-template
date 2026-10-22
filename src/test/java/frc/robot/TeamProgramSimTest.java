package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.Binding;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;

/**
 * The team's robot program, put together as its {@code Main} hands it to the runtime: its robot,
 * this season's field, and its controls. It starts, and drives its loop.
 */
@SimulationTest
class TeamProgramSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  /** {@code ./gradlew run} runs the test chassis, with no robot asked for by name. */
  @Test
  void theProgramRunsTheTestChassis() {
    assertThat(Robots.current()).isSameAs(ChassisRobot.CONFIG);
  }

  /**
   * The chassis runs with this season's field and the team's {@code Controls}, which binds nothing
   * and declares no auto until the team writes them: the runtime's own binding and its "Do nothing"
   * auto are all it has.
   */
  @Test
  void theTeamsRobotRunsWithItsSeasonAndItsControls() {
    sim.build(Robots.current(), Rebuilt.GAME, Controls::new, SensorErrors.TYPICAL);
    sim.run(1);

    assertThat(sim.bindings().all().stream().map(Binding::toString))
        .as("the runtime's own binding")
        .containsExactly("driver back: on press: Reset heading");
    assertThat(sim.autos().names()).containsExactly(Autos.DO_NOTHING);
  }
}
