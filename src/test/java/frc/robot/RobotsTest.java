package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.runtime.RobotChecks;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/** The team's robots, in Robots: each a valid configuration, and each can be tuned live. */
class RobotsTest {
  @Test
  void everyRobotHasAValidConfiguration() {
    // Building a RobotConfig checks every value, so loading the robots is the check.
    assertThat(Robots.all()).isNotEmpty();
    assertThat(Robots.all()).extracting(RobotConfig::name).doesNotHaveDuplicates();
  }

  @Test
  void theRobotTheProgramRunsIsOneOfThem() {
    assertThat(Robots.all()).contains(Robots.current());
  }

  @Test
  void theRobotAskedForByNameIsTheOneThatRuns() {
    assertThat(Robots.asked(Optional.empty())).isSameAs(ChassisRobot.CONFIG);
    for (RobotConfig robot : Robots.all()) {
      assertThat(Robots.asked(Optional.of(robot.name()))).isSameAs(robot);
    }
    assertThatThrownBy(() -> Robots.asked(Optional.of("gamma")))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("No robot is named \"gamma\". The robots are: chassis");
  }

  @Test
  void everyRobotsCommittedValuesCanBeDialedIn() {
    for (RobotConfig robot : Robots.all()) {
      RobotChecks.committedValuesCanBeDialedIn(robot);
    }
  }
}
