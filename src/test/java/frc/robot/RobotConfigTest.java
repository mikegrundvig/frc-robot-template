package frc.robot;

import static frc.robot.lib.config.ConfigFixtures.body;
import static frc.robot.lib.config.ConfigFixtures.camera;
import static frc.robot.lib.config.ConfigFixtures.swerve;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Inches;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.SwerveConfigBuilder;
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
    RobotConfig robot = robot(List.of());

    assertThat(robot.moduleTranslations()).containsExactly(swerve().moduleTranslations(body()));
  }

  @Test
  void aRobotNeedsAName() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> RobotConfigBuilder.builder(robot(List.of())).name(" ").build())
        .withMessageContaining("name");
  }

  @Test
  void theWheelsMustFitTheFrame() {
    var tooDeep = SwerveConfigBuilder.builder(swerve()).wheelInset(Inches.of(14)).build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> RobotConfigBuilder.builder(robot(List.of())).drivetrain(tooDeep).build())
        .withMessageContaining("wheelInset");
  }

  @Test
  void cameraNamesMustBeUnique() {
    var cameras = List.of(camera("front", inches(10, 0, 8)), camera("front", inches(-10, 0, 8)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(cameras))
        .withMessageContaining("two cameras are named front");
  }

  @Test
  void camerasMustBeOnTheRobot() {
    // 10 m ahead: the kind of mistake made by typing inches where meters belong.
    var cameras = List.of(camera("front", new Translation3d(10, 0, 0.2)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(cameras))
        .withMessageContaining("camera front is outside the robot");
  }

  @Test
  void theCameraListCannotChangeAfterwards() {
    List<CameraConfig> cameras = new ArrayList<>(List.of(camera("front", inches(10, 0, 8))));
    RobotConfig robot = robot(cameras);

    cameras.clear();

    assertThat(robot.cameras()).hasSize(1);
  }

  private static RobotConfig robot(List<CameraConfig> cameras) {
    return RobotConfig.builder()
        .name("test")
        .body(body())
        .drivetrain(swerve())
        .cameras(cameras)
        .build();
  }

  private static Translation3d inches(double x, double y, double z) {
    return new Translation3d(Inches.of(x), Inches.of(y), Inches.of(z));
  }
}
