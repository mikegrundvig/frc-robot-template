package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static frc.robot.subsystems.mechanisms.JointFixtures.control;
import static frc.robot.subsystems.mechanisms.JointFixtures.profile;
import static frc.robot.subsystems.mechanisms.JointFixtures.setup;
import static frc.robot.subsystems.mechanisms.JointFixtures.soft;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.config.ConfigException;
import com.michaelgrundvig.frc.world.robot.Coupling;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointPrismaticBuilder;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.lib.phoenix.MechanismControl;
import frc.robot.lib.wiring.CanBus;
import frc.robot.lib.wiring.MechanismWiring;
import frc.robot.lib.wiring.MechanismWiringBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.geometry.Translation3d;

/** A joint's setup against its definition: where it may be driven, and which joints need one. */
class JointSetupTest {
  @Test
  void softLimitsPastTheHardStopsAreRefused() {
    assertThatThrownBy(() -> setup(PIVOT, control(30, 1, soft(-0.1, 2.0), profile(8, 30), 40)))
        .isInstanceOf(ConfigException.class)
        .hasMessageContaining("-5.7° to 114.6° must be inside pivot's hard stops, 0.0° to 135.0°");
  }

  @Test
  void aNamedPositionOutsideTheSoftLimitsIsRefused() {
    assertThatThrownBy(() -> setup(PIVOT, control(30, 1, soft(0, 2.0), profile(8, 30), 40)))
        .isInstanceOf(ConfigException.class)
        .hasMessageContaining(
            "out, 130.0°, is outside its soft limits, 0.0° to 114.6°: it's never driven there")
        .message()
        .doesNotContain("run, ")
        .doesNotContain("in, ");
  }

  @Test
  void softLimitsAroundEveryNamedPositionAreFine() {
    JointSetup setup = setup(PIVOT, control(30, 1, soft(0, 2.3), profile(8, 30), 40));

    assertThat(setup.control().softLimits()).isPresent();
  }

  @Test
  void aSlidesLimitsReadInMetersAndInches() {
    assertThatThrownBy(() -> setup(ELEVATOR, control(60, 2, soft(0, 1.0), profile(1.2, 5), 40)))
        .hasMessageContaining("top, 1.100 m (43.31 in), is outside its soft limits");
  }

  @Test
  void theFixturesSetupsMatchTheirRobot() {
    assertThat(JointSetup.problems(ROBOT, SETUPS)).isNull();
  }

  @Test
  void aDrivenJointLeftOutIsNamed() {
    List<JointSetup> setups = new ArrayList<>(SETUPS);
    setups.removeLast();

    assertThat(JointSetup.problems(ROBOT, setups))
        .isEqualTo(
            "turret is driven by motors and has no setup: give it its control, and on a robot its"
                + " wiring");
  }

  @Test
  void aJointSetUpTwiceOrNotTheRobotsIsNamed() {
    Joint.Revolute stranger = JointRevoluteBuilder.builder(PIVOT).label("stranger").build();
    List<JointSetup> setups = new ArrayList<>(SETUPS);
    setups.add(setups.getFirst());
    setups.add(setup(stranger, SETUPS.getFirst().control()));

    assertThat(JointSetup.problems(ROBOT, setups))
        .isEqualTo("pivot is set up twice; stranger isn't one of joints's joints");
  }

  /**
   * A cascade's second stage follows the carriage, and a hatch nothing drives: neither has a
   * mechanism of its own, and a setup for one says why.
   */
  @Test
  void aJointThatFollowsOrThatNothingDrivesIsntSetUp() {
    Joint.Prismatic stage =
        JointPrismaticBuilder.builder(ELEVATOR)
            .label("second stage")
            .parent(ELEVATOR)
            .link(JointFixtures.link("stage", 1, new Translation3d()))
            .positions(List.of())
            .build();
    Joint.Revolute hatch =
        JointRevoluteBuilder.builder(PIVOT)
            .label("hatch")
            .link(JointFixtures.link("hatch", 1, new Translation3d()))
            .build();
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT)
            .joints(append(ROBOT.joints(), stage, hatch))
            .couplings(
                List.of(
                    Coupling.Linear.builder()
                        .leader(ELEVATOR)
                        .follower(stage)
                        .ratio(1)
                        .offset(0)
                        .build()))
            .build();

    assertThat(JointSetup.problems(robot, SETUPS)).isNull();
    assertThat(JointSetup.whyNotDriven(robot.math(), stage))
        .isEqualTo("it follows elevator, whose moves carry it");
    assertThat(JointSetup.whyNotDriven(robot.math(), hatch)).isEqualTo("nothing drives it");
    List<JointSetup> setups = new ArrayList<>(SETUPS);
    setups.add(setup(hatch, SETUPS.getFirst().control()));
    assertThat(JointSetup.problems(robot, setups))
        .isEqualTo("hatch is set up, but the template doesn't drive it: nothing drives it");
  }

  /**
   * Any motor is driven, in the simulator as it is and on a robot by the controller its wiring
   * names; a wiring whose controller can't drive its motor is refused, saying what can.
   */
  @Test
  void anyMotorIsDrivenAndItsWiringMustNameAControllerThatDrivesIt() {
    Joint.Revolute hatch =
        JointRevoluteBuilder.builder(PIVOT)
            .label("hatch")
            .link(JointFixtures.link("hatch", 1, new Translation3d()))
            .build();
    Joint.Revolute flap =
        JointRevoluteBuilder.builder(PIVOT)
            .label("flap")
            .link(JointFixtures.link("flap", 1, new Translation3d()))
            .build();
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(ROBOT)
            .joints(append(ROBOT.joints(), hatch, flap))
            .drives(
                appendDrives(
                    ROBOT.drives(),
                    hatch.motors(Motor.NEO, 1, JointFixtures.gearing(10)),
                    flap.motors(Motor.MINION, 1, JointFixtures.gearing(10))))
            .build();
    MechanismControl control = JointFixtures.assumed(SETUPS.getFirst().control());

    assertThat(JointSetup.whyNotDriven(robot.math(), hatch)).isNull();
    assertThat(JointSetup.whyNotDriven(robot.math(), flap)).isNull();
    List<JointSetup> setups = new ArrayList<>(SETUPS);
    setups.add(wired(hatch, control, MechanismWiring.Controller.SPARK_MAX, 21));
    setups.add(wired(flap, control, MechanismWiring.Controller.TALON_FXS, 22));
    assertThat(JointSetup.problems(robot, setups)).isNull();

    setups.set(setups.size() - 2, wired(hatch, control, MechanismWiring.Controller.TALON_FX, 21));
    assertThat(JointSetup.problems(robot, setups))
        .isEqualTo(
            "hatch's wiring puts its NEO on a TalonFX, which doesn't drive one: a TalonFXS or a"
                + " SPARK MAX or a SPARK Flex does");
  }

  /**
   * A joint held at positions by its absolute encoder needs no homing recipe when its wiring reads
   * the encoder, through a CANcoder or a SPARK's data port. A TalonFX has no data port.
   */
  @Test
  void anEncoderOnADataPortHomesAJointAndATalonFxHasNone() {
    RobotDefinition withEncoder =
        RobotDefinitionBuilder.builder(ROBOT)
            .drives(
                List.of(
                    PIVOT.motors(Motor.NEO, 1, JointFixtures.gearing(25)),
                    JointFixtures.ROLLER_MOTOR,
                    JointFixtures.ELEVATOR_MOTORS,
                    JointFixtures.TURRET_MOTOR))
            .sensors(List.of(Sensor.Encoder.absolute("pivot absolute", PIVOT, 8192)))
            .build();
    MechanismControl unhomed = MechanismControl.position(30);
    MechanismWiring.DataPortEncoder port =
        MechanismWiring.DataPortEncoder.builder().offset(Rotations.of(0)).reversed(false).build();
    JointSetup onSpark =
        JointSetupBuilder.builder(wired(PIVOT, unhomed, MechanismWiring.Controller.SPARK_MAX, 23))
            .wiring(
                Optional.of(
                    MechanismWiringBuilder.builder(
                            wired(PIVOT, unhomed, MechanismWiring.Controller.SPARK_MAX, 23)
                                .wiring()
                                .orElseThrow())
                        .dataPort(Optional.of(port))
                        .build()))
            .build();
    JointSetup onTalon =
        JointSetupBuilder.builder(onSpark)
            .wiring(
                Optional.of(
                    MechanismWiringBuilder.builder(onSpark.wiring().orElseThrow())
                        .controller(MechanismWiring.Controller.TALON_FX)
                        .build()))
            .build();
    List<JointSetup> setups = new ArrayList<>(SETUPS.subList(1, SETUPS.size()));

    setups.add(onSpark);
    assertThat(JointSetup.problems(withEncoder, setups)).isNull();
    setups.set(setups.size() - 1, onTalon);
    assertThat(JointSetup.problems(withEncoder, setups))
        .isEqualTo(
            "pivot's wiring names an encoder on its controller's data port, and a TalonFX has"
                + " none: wire it as a CANcoder");
  }

  private static JointSetup wired(
      Joint joint, MechanismControl control, MechanismWiring.Controller controller, int id) {
    return JointSetup.builder()
        .joint(joint)
        .control(control)
        .wiring(
            Optional.of(
                MechanismWiring.builder()
                    .canBus(CanBus.port(CANPort.CAN_S0))
                    .controller(controller)
                    .motors(
                        List.of(MechanismWiring.Motor.builder().id(id).clockwise(false).build()))
                    .cancoder(Optional.empty())
                    .dataPort(Optional.empty())
                    .build()))
        .build();
  }

  private static List<Drive> appendDrives(List<Drive> drives, Drive... more) {
    List<Drive> all = new ArrayList<>(drives);
    all.addAll(List.of(more));
    return all;
  }

  private static List<Joint> append(List<Joint> joints, Joint... more) {
    List<Joint> all = new ArrayList<>(joints);
    all.addAll(List.of(more));
    return all;
  }
}
