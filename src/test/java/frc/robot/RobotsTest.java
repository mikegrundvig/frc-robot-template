package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotChecks;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.world.robot.Drivetrain;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * The team's robots, in Robots: each a valid configuration, each can be tuned live, and each
 * drivetrain modelled with the motors as the runtime runs them.
 */
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
    assertThat(Robots.asked(Optional.empty())).isSameAs(ExampleRobot.COMPETITION);
    for (RobotConfig robot : Robots.all()) {
      assertThat(Robots.asked(Optional.of(robot.name()))).isSameAs(robot);
    }
    assertThatThrownBy(() -> Robots.asked(Optional.of("gamma")))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage(
            "No robot is named \"gamma\". The robots are: example, practice, beta, delta, chassis");
  }

  @Test
  void everyRobotsCommittedValuesCanBeDialedIn() {
    for (RobotConfig robot : Robots.all()) {
      RobotChecks.committedValuesCanBeDialedIn(robot);
    }
  }

  /**
   * No robot's swerve says {@code .foc()}: the template's robots run without a Phoenix Pro license,
   * so their drives and steerings run, and are simulated, on the plain motors.
   */
  @Test
  void noRobotsSwerveIsDrivenWithFoc() {
    for (RobotConfig robot : Robots.all()) {
      Drivetrain drivetrain = robot.definition().drivetrain();
      assertThat(drivetrain.driveCommutation())
          .as("%s's drives", robot.name())
          .isEqualTo(MechanismWiring.Commutation.TRAPEZOIDAL);
      assertThat(drivetrain.steerCommutation())
          .as("%s's steerings", robot.name())
          .isEqualTo(MechanismWiring.Commutation.TRAPEZOIDAL);
    }
  }
}
