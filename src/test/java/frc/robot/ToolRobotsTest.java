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
    assertThat(catalog.named(Robots.current().name())).isEqualTo(catalog.current());
  }

  @Test
  void theCurrentRobotIsDescribedAsConfigured() {
    RobotDescription current = catalog.current();

    assertThat(current.name()).isEqualTo(Robots.current().name());
    assertThat(current.body()).isEqualTo(Robots.current().body());
    assertThat(current.drivetrain()).isEqualTo(Robots.current().drivetrain());
    assertThat(current.cameras()).isEqualTo(Robots.current().cameras());
    assertThat(current.imu()).isEmpty();
  }
}
