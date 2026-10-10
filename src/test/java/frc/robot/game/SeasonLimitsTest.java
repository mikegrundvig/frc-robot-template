package frc.robot.game;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import frc.robot.Robots;
import org.junit.jupiter.api.Test;

/** The season's limits on what a robot is: every robot's definition fits them. */
class SeasonLimitsTest {
  @Test
  void everyRobotFitsTheSeason() {
    for (RobotConfig robot : Robots.all()) {
      assertThat(Rebuilt.LIMITS.problems(robot.definition())).as(robot.name()).isEmpty();
    }
  }
}
