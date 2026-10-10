package frc.robot.robots2026;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.AutoCheck;
import com.michaelgrundvig.frc.runtime.RobotSim;
import frc.robot.DeltaControls;
import frc.robot.DeltaRobot;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;

/**
 * Delta declares no autos (its port has no shooting gate to build one on), so the robot has "Do
 * nothing" alone, and it passes the template's check on both alliances.
 */
class DeltaAutosSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  @Test
  void deltaHasOnlyDoNothingAndItPassesTheTemplatesCheck() {
    var results = AutoCheck.check(sim, Parts.withoutCameras(DeltaRobot.CONFIG), DeltaControls::new);

    assertThat(results).extracting(AutoCheck.Result::auto).containsOnly("Do nothing");
    assertThat(results).hasSize(2);
    assertThat(results).allMatch(result -> result.moves().isEmpty());
  }
}
