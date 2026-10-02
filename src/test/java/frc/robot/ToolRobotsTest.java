package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.bridges.RobotDescription;
import org.junit.jupiter.api.Test;

/** The robots the tool writers see are the robots in Robots, each as it's configured. */
class ToolRobotsTest {
  private final ToolRobots catalog = new ToolRobots();

  @Test
  void everyRobotIsThereByItsName() {
    assertThat(catalog.all())
        .extracting(RobotDescription::name)
        .containsExactlyElementsOf(Robots.all().stream().map(RobotConfig::name).toList());
  }

  @Test
  void eachRobotIsDescribedAsConfigured() {
    for (RobotConfig robot : Robots.all()) {
      RobotDescription described = catalog.named(robot.name());

      assertThat(described.body()).isEqualTo(robot.definition().body());
      assertThat(described.drivetrain()).isEqualTo(robot.definition().drivetrain());
      assertThat(described.cameras()).isEqualTo(robot.definition().cameras());
      assertThat(described.imu()).isEmpty();
    }
  }
}
