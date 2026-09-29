package frc.robot;

import static frc.robot.lib.config.ConfigFixtures.body;
import static frc.robot.lib.config.ConfigFixtures.camera;
import static frc.robot.lib.config.ConfigFixtures.gains;
import static frc.robot.lib.config.ConfigFixtures.limits;
import static frc.robot.lib.config.ConfigFixtures.modules;
import static frc.robot.lib.config.ConfigFixtures.swerve;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Inches;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.SwerveConfig;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

class RobotConfigTest {
  @Test
  void everyRobotHasAValidConfiguration() {
    // Building a RobotConfig checks every value, so loading the robots is the check.
    assertThat(Robots.all()).isNotEmpty();
    assertThat(Robots.all()).extracting(RobotConfig::name).doesNotHaveDuplicates();
  }

  @Test
  void wheelPositionsComeFromTheFrameAndDrivetrain() {
    RobotConfig robot = new RobotConfig("test", body(), swerve(), List.of());

    assertThat(robot.moduleTranslations()).containsExactly(swerve().moduleTranslations(body()));
  }

  @Test
  void aRobotNeedsAName() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new RobotConfig(" ", body(), swerve(), List.of()))
        .withMessageContaining("name");
  }

  @Test
  void theWheelsMustFitTheFrame() {
    var tooDeep = new SwerveConfig(modules(), Inches.of(14), limits(), gains());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> new RobotConfig("test", body(), tooDeep, List.of()))
        .withMessageContaining("wheelInset");
  }

  @Test
  void cameraNamesMustBeUnique() {
    var cameras = List.of(camera("front", inches(10, 0, 8)), camera("front", inches(-10, 0, 8)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> new RobotConfig("test", body(), swerve(), cameras))
        .withMessageContaining("two cameras are named front");
  }

  @Test
  void camerasMustBeOnTheRobot() {
    // 10 m ahead: the kind of mistake made by typing inches where meters belong.
    var cameras = List.of(camera("front", new Translation3d(10, 0, 0.2)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> new RobotConfig("test", body(), swerve(), cameras))
        .withMessageContaining("camera front is outside the robot");
  }

  @Test
  void theCameraListCannotChangeAfterwards() {
    List<CameraConfig> cameras = new ArrayList<>(List.of(camera("front", inches(10, 0, 8))));
    RobotConfig robot = new RobotConfig("test", body(), swerve(), cameras);

    cameras.clear();

    assertThat(robot.cameras()).hasSize(1);
  }

  private static Translation3d inches(double x, double y, double z) {
    return new Translation3d(Inches.of(x), Inches.of(y), Inches.of(z));
  }
}
