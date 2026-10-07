package frc.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FXS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.DrivenBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainControlLimitsBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryCornerBuilder;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryModuleTypeBuilder;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import frc.robot.subsystems.mechanisms.JointFixtures;
import frc.robot.subsystems.mechanisms.JointSetup;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

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
        .isThrownBy(
            () ->
                RobotConfigBuilder.builder(Robots.SIM)
                    .definition(
                        RobotDefinitionBuilder.builder(Robots.SIM.definition())
                            .drivetrain(Robots.SIM.definition().drivetrain().control(tooFast))
                            .build())
                    .build())
        .withMessage(
            "RobotConfig[sim].definition.drivetrain.control.limits.maxLinearSpeed: 5 m/s is faster"
                + " than the"
                + " drivetrain's free speed of 4.729 m/s");
  }

  /**
   * Each joint's block is its setup: the configuration works out its control and its wiring, and
   * its limit switch's wiring, from the definition alone. A joint with motors and no control is
   * refused, naming what to give it.
   */
  @Test
  void eachJointsBlockIsItsSetup() {
    Joint.Revolute pivot =
        JointFixtures.PIVOT
            .motor(Motor.NEO, JointFixtures.gearing(25), SPARK_MAX.can(32).clockwise())
            .limitSwitch(
                Sensor.Switch.Limit.End.LOWER,
                Degrees.of(0),
                SensorWiring.Direction.REVERSE,
                SensorWiring.Polarity.NORMALLY_CLOSED);
    RobotDefinition wired =
        RobotDefinitionBuilder.builder(Robots.SIM.definition())
            .joints(List.of(pivot))
            .configurations(List.of())
            .build();

    RobotConfig robot = RobotConfigBuilder.builder(Robots.SIM).definition(wired).build();

    JointSetup setup = robot.joints().getFirst();
    assertThat(setup.joint()).isEqualTo(JointFixtures.PIVOT);
    assertThat(setup.control()).isEqualTo(JointFixtures.PIVOT.driven().control().orElseThrow());
    assertThat(setup.wiring().orElseThrow().motors())
        .containsExactly(new MechanismWiring.Motor(32, true));
    assertThat(robot.sensors())
        .containsExactly(
            SensorWiring.OfSwitch.builder()
                .sensor("pivot lower limit")
                .input(new SensorWiring.ControllerLimit("pivot", SensorWiring.Direction.REVERSE))
                .polarity(SensorWiring.Polarity.NORMALLY_CLOSED)
                .build());
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotConfigBuilder.builder(Robots.SIM)
                    .definition(
                        RobotDefinitionBuilder.builder(wired)
                            .joints(
                                List.of(
                                    pivot.driven(
                                        DrivenBuilder.builder(pivot.driven())
                                            .control(Optional.empty())
                                            .build())))
                            .build())
                    .build())
        .withMessageContaining(
            "pivot has motors and no control: give it .control(MechanismControl.position(kP))");
  }

  /**
   * A practice robot is the same definition with its differences: a joint re-wired replaces its
   * equal, and the configuration, and behavior that names the joint, find the practice one. A joint
   * the robot hasn't is refused.
   */
  @Test
  void aPracticeRobotIsTheSameRobotRewired() {
    Joint.Revolute pivot =
        JointFixtures.PIVOT.motor(
            Motor.NEO, JointFixtures.gearing(25), SPARK_MAX.can(32).clockwise());
    RobotDefinition competition =
        RobotDefinitionBuilder.builder(Robots.SIM.definition())
            .joints(List.of(pivot))
            .configurations(List.of())
            .build();

    RobotDefinition practice =
        competition.with(
            pivot.motor(Motor.NEO, JointFixtures.gearing(25), SPARK_MAX.can(33).clockwise()));
    RobotConfig robot = RobotConfigBuilder.builder(Robots.SIM).definition(practice).build();

    assertThat(practice.joints()).containsExactly(JointFixtures.PIVOT);
    assertThat(robot.joints().getFirst().wiring().orElseThrow().motors())
        .containsExactly(new MechanismWiring.Motor(33, true));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(JointFixtures.ROLLER))
        .withMessage("sim has no joint roller the same but for how it's driven");
  }

  /**
   * An interlock about a joint the template doesn't drive is refused as the configuration is built:
   * nothing could enforce it. The joints, driven by nothing here, are still the ones it names.
   */
  @Test
  void anInterlockNothingCanEnforceIsRefusedWithTheConfiguration() {
    RobotDefinition guarded =
        RobotDefinitionBuilder.builder(Robots.SIM.definition())
            .joints(
                List.of(
                    JointFixtures.PIVOT.driven(Driven.NONE),
                    JointFixtures.ROLLER.driven(Driven.NONE)))
            .interlocks(
                List.of(
                    Interlock.that(JointFixtures.ROLLER)
                        .runsOnlyWhen(JointFixtures.PIVOT.past(JointFixtures.PIVOT_RUN))))
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> RobotConfigBuilder.builder(Robots.SIM).definition(guarded).build())
        .withMessage(
            "RobotConfig[sim].definition.interlocks: \"roller runs only when pivot past run\""
                + " can't be enforced: roller has no mechanism, as nothing drives it");
  }

  /**
   * A control a joint's motor controllers can't take is refused as the configuration is built, in
   * the robot's own words, wherever the program runs: an arm's gravity angle a third of a turn from
   * its zero, past Phoenix's gravity offset, on the TalonFX the simulator stands in for when it's
   * wired to nothing, and on the TalonFXS it's wired to. A SPARK takes it, the robot working out
   * its gravity each loop; and it refuses what a TalonFX takes, a kI, only where the arm is wired
   * to one.
   */
  @Test
  void aControlItsMotorControllerWouldRefuseIsRefusedWithTheConfiguration() {
    MechanismControl held =
        MechanismControl.position(30).withHoming(MechanismControl.Homing.assumeAtReference());
    Joint.Revolute arm =
        Joint.pivot("arm", Part.CHASSIS, new Translation3d(0.2, 0, 0.4), Joint.PITCH_UP)
            .range(Range.degrees(0, 90), Degrees.of(0))
            .motor(Motor.NEO, JointFixtures.gearing(50))
            .control(held.withKg(0.4, Degrees.of(120)));
    String robots =
        "arm's gravity angle, 120.0°, is more than a quarter turn from its zero, past Phoenix's"
            + " gravity offset of ±0.25 rotations: its positive direction lowers it at its zero."
            + " Turn its axis around in the definition (and its motors' and encoder's directions"
            + " in the wiring), so that positive raises it; its gravity angle is then 60.0°";
    MechanismControl integral =
        MechanismControlBuilder.builder(held)
            .gains(MechanismControlGainsBuilder.builder(held.gains()).kI(1).build())
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> withJoint(arm))
        .withMessage("RobotConfig[sim].joints: " + robots);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> withJoint(arm.motor(Motor.NEO, JointFixtures.gearing(50), TALON_FXS.can(21))))
        .withMessage("RobotConfig[sim].joints: " + robots);
    assertThat(
            withJoint(arm.motor(Motor.NEO, JointFixtures.gearing(50), SPARK_MAX.can(21))).joints())
        .as("on a SPARK, the robot works out its gravity, any offset")
        .hasSize(1);
    assertThat(withJoint(arm.control(integral)).joints()).hasSize(1);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                withJoint(
                    arm.motor(Motor.NEO, JointFixtures.gearing(50), SPARK_MAX.can(21))
                        .control(integral)))
        .withMessage(
            "RobotConfig[sim].joints: arm's control has a kI, which isn't carried to a SPARK yet:"
                + " its integral's units aren't confirmed. Leave kI at 0");
  }

  /** The simulator's robot with one joint, its configuration built. */
  private static RobotConfig withJoint(Joint joint) {
    return RobotConfigBuilder.builder(Robots.SIM)
        .definition(
            RobotDefinitionBuilder.builder(Robots.SIM.definition())
                .joints(List.of(joint))
                .configurations(List.of())
                .build())
        .build();
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
                                .wheelCoefficientOfFriction(Optional.of(1.6))
                                .build())
                        .build())
            .toList();
    RobotDefinition grippy =
        RobotDefinitionBuilder.builder(Robots.SIM.definition())
            .drivetrain(
                DrivetrainGeometryBuilder.builder(Robots.SIM.definition().drivetrain())
                    .corners(corners)
                    .build())
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
            .drivetrain(
                DrivetrainGeometryBuilder.builder(Robots.SIM.definition().drivetrain())
                    .corners(neos)
                    .build())
            .build();
    SwerveWiring wiring =
        SwerveWiring.builder()
            .canBus(CanBus.port(CanBus.Systemcore.CAN_S0))
            .driveController(MechanismWiring.Controller.TALON_FX)
            .steerController(MechanismWiring.Controller.TALON_FX)
            .gyroId(1)
            .gyroYawScale(1.0)
            .modules(List.of(module(1, 2, 1), module(3, 4, 2), module(5, 6, 3), module(7, 8, 4)))
            .build();

    assertThat(RobotConfigBuilder.builder(Robots.SIM).definition(rev).build().definition())
        .isEqualTo(rev);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RobotConfigBuilder.builder(Robots.SIM)
                    .definition(
                        RobotDefinitionBuilder.builder(rev)
                            .drivetrain(rev.drivetrain().wired(wiring))
                            .build())
                    .build())
        .withMessage(
            "RobotConfig[sim].definition.drivetrain.wiring: the drivetrain's wiring: its drive's NEO isn't"
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
