package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryCornerBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryModuleTypeBuilder;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import frc.robot.lib.wiring.CanBus;
import frc.robot.lib.wiring.MechanismWiring;
import frc.robot.lib.wiring.SwerveWiring;
import frc.robot.subsystems.drive.DrivetrainControlBuilder;
import frc.robot.subsystems.drive.DrivetrainControlLimitsBuilder;
import frc.robot.subsystems.mechanisms.JointFixtures;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.imu.OnboardIMU;

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

  /**
   * An interlock about a joint the template doesn't drive is refused as the configuration is built:
   * nothing could enforce it.
   */
  @Test
  void anInterlockNothingCanEnforceIsRefusedWithTheConfiguration() {
    RobotDefinition guarded =
        RobotDefinitionBuilder.builder(Robots.SIM.definition())
            .joints(List.of(JointFixtures.PIVOT, JointFixtures.ROLLER))
            .interlocks(
                List.of(
                    Interlock.that(JointFixtures.ROLLER)
                        .runsOnlyWhen(JointFixtures.PIVOT.past("run"))))
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> RobotConfigBuilder.builder(Robots.SIM).definition(guarded).build())
        .withMessage(
            "RobotConfig[sim].definition.interlocks: \"roller runs only when pivot past run\""
                + " can't be enforced: roller has no mechanism, as nothing drives it");
  }

  /**
   * Treads said to grip harder than the slip check can tell from a collision are refused as the
   * configuration is built: a path planner's friction, a tuning value, often is.
   */
  @Test
  void aPathPlannersFrictionIsRefusedAsTheTreadsGrip() {
    List<DrivetrainGeometry.Corner> corners =
        Robots.SIM.definition().drivetrain().corners().stream()
            .map(
                corner ->
                    DrivetrainGeometryCornerBuilder.builder(corner)
                        .module(
                            DrivetrainGeometryModuleTypeBuilder.builder(corner.module())
                                .wheelCoefficientOfFriction(1.6)
                                .build())
                        .build())
            .toList();
    RobotDefinition grippy =
        RobotDefinitionBuilder.builder(Robots.SIM.definition())
            .drivetrain(new DrivetrainGeometry(corners))
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> RobotConfigBuilder.builder(Robots.SIM).definition(grippy).build())
        .withMessage(
            "RobotConfig[sim].definition: the treads' grip, 1.60, is more than tread has: the slip"
                + " check tells a slide from a collision only under 1.53. Give the tread's own,"
                + " about 1.0 worn and 1.3 new; a path planner's friction is a tuning value, not"
                + " the tread's");
  }

  /**
   * A drivetrain wired for a robot whose controllers can't drive its motors is refused as the
   * configuration is built: its modules wouldn't move. In the simulator alone, any motor runs.
   */
  @Test
  void aWiredSwerveOfMotorsNoTalonFxDrivesIsRefused() {
    List<DrivetrainGeometry.Corner> neos =
        Robots.SIM.definition().drivetrain().corners().stream()
            .map(
                corner ->
                    DrivetrainGeometryCornerBuilder.builder(corner)
                        .module(
                            DrivetrainGeometryModuleTypeBuilder.builder(corner.module())
                                .driveMotor(Motor.NEO)
                                .build())
                        .build())
            .toList();
    RobotDefinition rev =
        RobotDefinitionBuilder.builder(Robots.SIM.definition())
            .drivetrain(new DrivetrainGeometry(neos))
            .build();
    SwerveWiring wiring =
        SwerveWiring.builder()
            .canBus(CanBus.port(CANPort.CAN_S0))
            .driveController(MechanismWiring.Controller.TALON_FX)
            .steerController(MechanismWiring.Controller.TALON_FX)
            .gyroId(1)
            .gyroYawScale(1.0)
            .modules(List.of(module(1, 2, 1), module(3, 4, 2), module(5, 6, 3), module(7, 8, 4)))
            .systemcoreMount(OnboardIMU.MountOrientation.FLAT)
            .build();

    assertThat(RobotConfigBuilder.builder(Robots.SIM).definition(rev).build().definition())
        .isEqualTo(rev);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotConfigBuilder.builder(Robots.SIM)
                    .definition(rev)
                    .drivetrainWiring(Optional.of(wiring))
                    .build())
        .withMessage(
            "RobotConfig[sim].drivetrainWiring: the drivetrain's wiring: its drive's NEO isn't"
                + " driven by a TalonFX");
  }

  private static SwerveWiring.ModuleWiring module(int drive, int steer, int encoder) {
    return SwerveWiring.ModuleWiring.builder()
        .driveId(drive)
        .steerId(steer)
        .encoderId(Optional.of(encoder))
        .encoderOffset(Rotations.of(0))
        .driveInverted(false)
        .steerInverted(true)
        .encoderInverted(false)
        .build();
  }
}
