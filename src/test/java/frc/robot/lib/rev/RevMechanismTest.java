package frc.robot.lib.rev;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.subsystems.mechanisms.ControllerSettings;
import frc.robot.subsystems.mechanisms.JointFixtures;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The REV helper: a SPARK's settings, said from the program's plan for the mechanism in a SPARK's
 * units. The fixtures' pivot on two NEOs through 25:1.
 */
class RevMechanismTest {
  private static final RobotDefinition ROBOT =
      JointFixtures.ROBOT.with(
          JointFixtures.unmotored(PIVOT).motors(Motor.NEO, JointFixtures.gearing(25), 2));

  private static final RobotMath MATH = ROBOT.math();

  private static final Drive NEO = MATH.drivesOf(PIVOT).get(0);

  private static final MechanismControl CONTROL =
      MechanismControlBuilder.builder(ControllerSettings.control(PIVOT))
          .softLimits(MechanismControl.SoftLimits.builder().lower(0.1).upper(2.2).build())
          .build();

  /** Two motor controllers of a kind, at 41, turning clockwise, and 42, the other way. */
  private static List<MechanismWiring.Device> wiring(MechanismWiring.Controller controller) {
    return List.of(controller.can(41).clockwise(), controller.can(42));
  }

  private static final List<MechanismWiring.Device> SPARKS =
      wiring(MechanismWiring.Controller.SPARK_MAX);

  /**
   * Its loop's gain is the plan's kP per rotor turn as a share of 12 V, for positions, and per
   * rotor RPM for speeds; its soft limits are in rotor turns; and what the robot adds is the plan's
   * feedforward and kD, as a TalonFX's slot holds them per rotation.
   */
  @Test
  void itSaysThePlanInASparksUnits() {
    ControllerPlan plan = ControllerPlan.of(MATH, PIVOT, NEO, SPARKS, CONTROL);
    RevMechanism.Settings rev = RevMechanism.configure(plan);
    SlotConfigs slot = SlotConfigs.from(PhoenixMechanism.configure(plan).leader().Slot0);

    assertThat(rev.rotorRatio()).isCloseTo(25, within(1e-12));
    assertThat(rev.positionKp()).isCloseTo(slot.kP / (12 * 25), within(1e-12));
    assertThat(rev.velocityKp()).isCloseTo(slot.kP / (12 * 25 * 60), within(1e-15));
    assertThat(rev.softLimits().orElseThrow())
        .containsExactly(0.1 / (2 * Math.PI) * 25, 2.2 / (2 * Math.PI) * 25);
    assertThat(rev.plan()).isSameAs(plan);
    assertThat(rev.plan().kV() * 2 * Math.PI).isCloseTo(slot.kV, within(1e-12));
    assertThat(rev.plan().gravity().kG()).isEqualTo(slot.kG);
    assertThat(rev.plan().gravity().kind())
        .as("a pivot's gravity turns with it")
        .isEqualTo(ControllerPlan.Gravity.Kind.COSINE);
    assertThat(rev.plan().gravity().offset() / (2 * Math.PI))
        .isCloseTo(slot.GravityArmPositionOffset, within(1e-12));
    assertThat(rev.plan().kD() * 2 * Math.PI).isCloseTo(slot.kD, within(1e-12));
    assertThat(rev.followers()).extracting(RevMechanism.Follower::id).containsExactly(42);
  }

  /**
   * An unweighed pivot's tuned kG reaches a SPARK as it does a TalonFX: the robot adds it each
   * loop, along one cosine about where its weight pulls hardest, as given.
   */
  @Test
  void anUnweighedPivotsTunedKgPullsHardestAtItsGravityAngle() {
    Joint.Revolute bare = JointRevoluteBuilder.builder(PIVOT).link(Link.unseen("bare arm")).build();
    Drive neo = JointFixtures.motors(bare, Motor.NEO, 2, JointFixtures.gearing(25));
    RobotMath math = RobotMath.of(ROBOT.body(), List.of(bare), List.of(), List.of(neo), List.of());

    ControllerPlan plan =
        RevMechanism.configure(
                ControllerPlan.of(math, bare, neo, SPARKS, CONTROL.withKg(0.4, Degrees.of(-30))))
            .plan();

    assertThat(plan.gravity().kind()).isEqualTo(ControllerPlan.Gravity.Kind.COSINE);
    assertThat(plan.feedforward(Degrees.of(-30).in(Radians), 0, 0)).isCloseTo(0.4, within(1e-12));
    assertThat(plan.feedforward(Degrees.of(60).in(Radians), 0, 0)).isCloseTo(0, within(1e-12));
  }

  /**
   * An arm whose positive direction lowers it, its weight pulling hardest half a turn from its
   * zero, runs on a SPARK: the robot works out its gravity each loop, with no limit on the offset.
   * On a TalonFX, Phoenix's onboard gravity can't take it, and the robot says how to turn its axis.
   */
  @Test
  void anArmWhosePositiveLowersItRunsOnASpark() {
    Joint.Revolute bare = JointRevoluteBuilder.builder(PIVOT).link(Link.unseen("bare arm")).build();
    Drive neo = JointFixtures.motors(bare, Motor.NEO, 2, JointFixtures.gearing(25));
    RobotMath math = RobotMath.of(ROBOT.body(), List.of(bare), List.of(), List.of(neo), List.of());
    MechanismControl lowering = CONTROL.withKg(0.4, Degrees.of(180));

    ControllerPlan plan =
        RevMechanism.configure(ControllerPlan.of(math, bare, neo, SPARKS, lowering)).plan();

    assertThat(plan.feedforward(0, 0, 0))
        .as("pushing up, against positive")
        .isCloseTo(-0.4, within(1e-12));
    assertThat(plan.feedforward(Math.PI, 0, 0)).isCloseTo(0.4, within(1e-12));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                PhoenixMechanism.configure(
                    ControllerPlan.of(
                        math,
                        bare,
                        JointFixtures.motors(bare, Motor.KRAKEN_X60, 2, JointFixtures.gearing(25)),
                        wiring(MechanismWiring.Controller.TALON_FX),
                        lowering)))
        .withMessageContaining("past Phoenix's");
  }

  /**
   * An absolute encoder on the data port is carried in the settings, for reading at startup, with
   * where the SPARK's reading of it is zero: the low edge of the window its turn tells apart,
   * centered on the pivot's travel, 0° to 135°: -112.5°, so the reading wraps at 247.5°.
   */
  @Test
  void anEncoderOnTheDataPortIsReadAtStartup() {
    Sensor.Encoder absolute = Sensor.Encoder.absolute("pivot absolute", PIVOT, 8192);
    RobotMath math =
        RobotDefinitionBuilder.builder(ROBOT).sensors(List.of(absolute)).build().math();
    ControllerPlan plan = ControllerPlan.of(math, PIVOT, NEO, SPARKS, CONTROL);
    RevMechanism.Settings read =
        RevMechanism.configure(math, plan, absolute, DATA_PORT.offset(Rotations.of(0)));

    assertThat(read.absolute()).contains(absolute);
    assertThat(read.windowEdge()).as("in turns").isCloseTo(-0.3125, within(1e-12));
    assertThat(RevMechanism.configure(plan).absolute()).isEmpty();
  }

  /**
   * Closed on its data-port encoder, the SPARK's loop runs in the encoder's turns from the window's
   * edge, -112.5°: on the pivot's axle, a turn of the encoder is a turn of the pivot, so its kP is
   * the slot's per turn of the pivot, 25 times what it is per rotor turn, and its soft limits are
   * where the encoder reads them. Seeded once, the same encoder leaves the loop on the rotor. The
   * feedforward is the plan's either way.
   */
  @Test
  void aSparksLoopClosesOnItsDataPortEncoderInItsTurns() {
    Sensor.Encoder absolute = Sensor.Encoder.absolute("pivot absolute", PIVOT, 8192);
    RobotMath math =
        RobotDefinitionBuilder.builder(ROBOT).sensors(List.of(absolute)).build().math();
    MechanismWiring.DataPortEncoder port = DATA_PORT.offset(Rotations.of(0));

    ControllerPlan plan = ControllerPlan.of(math, PIVOT, NEO, SPARKS, CONTROL);
    RevMechanism.Settings once = RevMechanism.configure(math, plan, absolute, port);
    RevMechanism.Settings closed = RevMechanism.configure(math, plan, absolute, port.closesLoop());

    assertThat(once.closedLoop()).isFalse();
    assertThat(once.loopRatio()).isCloseTo(25, within(1e-12));
    assertThat(once.loopZero()).isZero();
    assertThat(closed.closedLoop()).isTrue();
    assertThat(closed.loopRatio()).isCloseTo(1, within(1e-12));
    assertThat(closed.loopZero()).isCloseTo(0.3125, within(1e-12));
    assertThat(closed.positionKp()).isCloseTo(once.positionKp() * 25, within(1e-12));
    assertThat(closed.velocityKp()).isCloseTo(once.velocityKp() * 25, within(1e-12));
    assertThat(closed.softLimits().orElseThrow())
        .containsExactly(0.1 / (2 * Math.PI) + 0.3125, 2.2 / (2 * Math.PI) + 0.3125);
    assertThat(closed.plan()).isSameAs(once.plan());
  }

  @Test
  void itRefusesWhatASparkCantTakeYet() {
    MechanismControl integral =
        MechanismControlBuilder.builder(CONTROL)
            .gains(MechanismControlGainsBuilder.builder(CONTROL.gains()).kI(1).build())
            .build();
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RevMechanism.configure(
                    ControllerPlan.of(
                        MATH, PIVOT, NEO, wiring(MechanismWiring.Controller.TALON_FXS), CONTROL)))
        .withMessage("pivot's wiring is a TalonFXS's, not a SPARK's");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> RevMechanism.configure(ControllerPlan.of(MATH, PIVOT, NEO, SPARKS, integral)))
        .withMessageStartingWith("pivot's control has a kI");
  }
}
