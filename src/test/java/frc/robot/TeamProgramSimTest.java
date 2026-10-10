package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.autos.Autos;
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

  @Test
  void theTeamsRobotRunsWithItsSeasonAndItsControls() {
    sim.build(Robots.current(), Rebuilt::game, Controls::new, SensorErrors.TYPICAL);
    sim.run(1);

    // The robot starts with the choice it always has, plus whatever the team's Controls declares.
    assertThat(sim.autos().names()).contains(Autos.DO_NOTHING);
  }
}
