package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.MetersPerSecond;

import frc.robot.subsystems.drive.DrivetrainControlBuilder;
import frc.robot.subsystems.drive.DrivetrainControlLimitsBuilder;
import org.junit.jupiter.api.Test;

class RobotConfigTest {
  @Test
  void everyRobotHasAValidConfiguration() {
    // Building a RobotConfig checks every value, so loading the robots is the check.
    assertThat(Robots.all()).isNotEmpty();
    assertThat(Robots.all()).extracting(RobotConfig::name).doesNotHaveDuplicates();
  }

  /**
   * The driver's top speed is how it's driven, the free speed what the robot is: the configuration
   * holds one to the other. The template's robot reaches 4.73 m/s free.
   */
  @Test
  void topSpeedCannotExceedTheFreeSpeed() {
    var tooFast =
        DrivetrainControlBuilder.builder(Robots.SIM.drivetrain())
            .limits(
                DrivetrainControlLimitsBuilder.builder(Robots.SIM.drivetrain().limits())
                    .maxLinearSpeed(MetersPerSecond.of(5))
                    .build())
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> RobotConfigBuilder.builder(Robots.SIM).drivetrain(tooFast).build())
        .withMessage(
            "RobotConfig[sim].drivetrain.limits.maxLinearSpeed: 5 m/s is faster than the"
                + " drivetrain's free speed of 4.729 m/s");
  }
}
