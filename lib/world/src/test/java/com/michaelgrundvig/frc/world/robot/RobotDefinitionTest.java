package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.body;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.camera;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.definition;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.drivetrain;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Inches;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/** A robot's parts are checked against each other when the definition is written down. */
class RobotDefinitionTest {
  @Test
  void wheelPositionsComeFromTheFrameAndDrivetrain() {
    assertThat(definition(List.of()).moduleTranslations())
        .containsExactly(drivetrain().moduleTranslations(body()));
  }

  @Test
  void theWheelsMustFitTheFrame() {
    var tooDeep = DrivetrainGeometryBuilder.builder(drivetrain()).wheelInset(Inches.of(14)).build();

    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> RobotDefinitionBuilder.builder(definition(List.of())).drivetrain(tooDeep).build())
        .withMessageStartingWith("RobotDefinition[test].drivetrain.wheelInset");
  }

  /** A robot has a name, which every message about its definition starts with. */
  @Test
  void aRobotNeedsAName() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> RobotDefinitionBuilder.builder(definition(List.of())).name(" ").build())
        .withMessageStartingWith("RobotDefinition[ ].name");
  }

  @Test
  void cameraNamesMustBeUnique() {
    var cameras = List.of(camera("front", inches(10, 0, 8)), camera("front", inches(-10, 0, 8)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(cameras))
        .withMessageContaining("two cameras are named front");
  }

  @Test
  void camerasMustBeOnTheRobot() {
    // 10 m ahead: the kind of mistake made by typing inches where meters belong.
    var cameras = List.of(camera("front", new Translation3d(10, 0, 0.2)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(cameras))
        .withMessageContaining("camera front is outside the robot");
  }

  /**
   * An IMU is mounted inside the robot, as a camera is: the backup 2 m up, or the gyro 1 m behind,
   * is a mistake in units or sign.
   */
  @Test
  void imusMustBeOnTheRobot() {
    ImuMount high = ImuMountBuilder.builder(ImuMount.UNCHECKED).up(Inches.of(80)).build();
    ImuMount behind = ImuMountBuilder.builder(ImuMount.UNCHECKED).forward(Inches.of(-40)).build();
    RobotDefinition robot = definition(List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(robot)
                    .imus(ImusBuilder.builder(Imus.UNCHECKED).backup(high).build())
                    .build())
        .withMessageContaining("the backup IMU is outside the robot");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotDefinitionBuilder.builder(robot)
                    .imus(ImusBuilder.builder(Imus.UNCHECKED).gyro(behind).build())
                    .build())
        .withMessageContaining("the gyro is outside the robot");
  }

  @Test
  void anImusOrientationIsARotation() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                ImuMountBuilder.builder(ImuMount.UNCHECKED)
                    .orientation(new Rotation3d(Double.NaN, 0, 0))
                    .build())
        .withMessageContaining("ImuMount.orientation");
  }

  @Test
  void theCameraListCannotChangeAfterwards() {
    List<CameraConfig> cameras = new ArrayList<>(List.of(camera("front", inches(10, 0, 8))));
    RobotDefinition robot = definition(cameras);

    cameras.clear();

    assertThat(robot.cameras()).hasSize(1);
  }

  private static Translation3d inches(double x, double y, double z) {
    return new Translation3d(Inches.of(x), Inches.of(y), Inches.of(z));
  }
}
