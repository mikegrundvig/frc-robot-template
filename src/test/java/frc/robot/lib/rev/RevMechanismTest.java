package frc.robot.lib.rev;

import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.SETUPS;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControlGainsBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.MechanismWiringBuilder;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.subsystems.mechanisms.JointFixtures;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * The REV helper: a SPARK's settings carried from the Phoenix helper's for the same mechanism, in a
 * SPARK's units. The fixtures' pivot on a NEO through 25:1.
 */
class RevMechanismTest {
  private static final Drive NEO =
      JointFixtures.motors(PIVOT, Motor.NEO, 2, JointFixtures.gearing(25));

  private static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(JointFixtures.ROBOT)
          .drives(
              List.of(
                  NEO,
                  JointFixtures.ROLLER_MOTOR,
                  JointFixtures.ELEVATOR_MOTORS,
                  JointFixtures.TURRET_MOTOR))
          .build();

  private static final RobotMath MATH = ROBOT.math();

  private static final MechanismControl CONTROL =
      MechanismControlBuilder.builder(SETUPS.get(0).control())
          .softLimits(
              Optional.of(MechanismControl.SoftLimits.builder().lower(0.1).upper(2.2).build()))
          .build();

  private static MechanismWiring wiring(
      MechanismWiring.Controller controller, Optional<MechanismWiring.Cancoder> cancoder) {
    return MechanismWiring.builder()
        .canBus(CanBus.port(CanBus.Systemcore.CAN_S0))
        .controller(controller)
        .motors(
            List.of(
                MechanismWiring.Motor.builder().id(41).clockwise(true).build(),
                MechanismWiring.Motor.builder().id(42).clockwise(false).build()))
        .cancoder(cancoder)
        .dataPort(Optional.empty())
        .build();
  }

  private static final MechanismWiring SPARKS =
      wiring(MechanismWiring.Controller.SPARK_MAX, Optional.empty());

  /**
   * Its loop's gain is the Phoenix slot's kP per rotor turn as a share of 12 V, for positions, and
   * per rotor RPM for speeds; its soft limits are in rotor turns; and what the robot adds is the
   * slot's feedforward and kD, as Phoenix holds them.
   */
  @Test
  void itCarriesThePhoenixSlotInASparksUnits() {
    RevMechanism.Settings rev = RevMechanism.configure(MATH, PIVOT, NEO, SPARKS, CONTROL);
    SlotConfigs slot =
        SlotConfigs.from(
            PhoenixMechanism.configure(MATH, PIVOT, NEO, SPARKS, CONTROL).leader().Slot0);

    assertThat(rev.rotorRatio()).isCloseTo(25, within(1e-12));
    assertThat(rev.positionKp()).isCloseTo(slot.kP / (12 * 25), within(1e-12));
    assertThat(rev.velocityKp()).isCloseTo(slot.kP / (12 * 25 * 60), within(1e-15));
    assertThat(rev.softLimits().orElseThrow())
        .containsExactly(0.1 / (2 * Math.PI) * 25, 2.2 / (2 * Math.PI) * 25);
    PhoenixMechanism.Feedforward feedforward = rev.feedforward();
    assertThat(feedforward.kV()).isEqualTo(slot.kV);
    assertThat(feedforward.kG()).isEqualTo(slot.kG);
    assertThat(feedforward.arm()).as("a pivot's gravity turns with it").isTrue();
    assertThat(feedforward.armOffset()).isEqualTo(slot.GravityArmPositionOffset);
    assertThat(feedforward.kD()).isEqualTo(slot.kD);
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

    PhoenixMechanism.Feedforward feedforward =
        RevMechanism.configure(math, bare, neo, SPARKS, CONTROL.withKg(0.4, Degrees.of(-30)))
            .feedforward();

    assertThat(feedforward.arm()).isTrue();
    assertThat(feedforward.atPosition(Degrees.of(-30).in(Rotations), 0, 0))
        .isCloseTo(0.4, within(1e-12));
    assertThat(feedforward.atPosition(Degrees.of(60).in(Rotations), 0, 0))
        .isCloseTo(0, within(1e-12));
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

    PhoenixMechanism.Feedforward feedforward =
        RevMechanism.configure(math, bare, neo, SPARKS, lowering).feedforward();

    assertThat(feedforward.atPosition(0, 0, 0))
        .as("pushing up, against positive")
        .isCloseTo(-0.4, within(1e-12));
    assertThat(feedforward.atPosition(0.5, 0, 0)).isCloseTo(0.4, within(1e-12));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                PhoenixMechanism.configure(
                    math,
                    bare,
                    JointFixtures.motors(bare, Motor.KRAKEN_X60, 2, JointFixtures.gearing(25)),
                    wiring(MechanismWiring.Controller.TALON_FX, Optional.empty()),
                    lowering))
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
    MechanismWiring withPort =
        MechanismWiringBuilder.builder(SPARKS)
            .dataPort(
                Optional.of(
                    MechanismWiring.DataPortEncoder.builder()
                        .offset(Rotations.of(0))
                        .reversed(false)
                        .closedLoop(false)
                        .build()))
            .build();

    RevMechanism.Settings read =
        RevMechanism.configure(math, PIVOT, NEO, absolute, withPort, CONTROL);

    assertThat(read.absolute()).contains(absolute);
    assertThat(read.windowEdge()).as("in turns").isCloseTo(-0.3125, within(1e-12));
    assertThat(RevMechanism.configure(math, PIVOT, NEO, SPARKS, CONTROL).absolute()).isEmpty();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> RevMechanism.configure(math, PIVOT, NEO, withPort, CONTROL))
        .withMessageStartingWith("pivot's wiring names a data-port encoder");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> RevMechanism.configure(math, PIVOT, NEO, absolute, SPARKS, CONTROL))
        .withMessage("pivot's wiring names no data-port encoder for pivot absolute: wire it");
  }

  /**
   * Closed on its data-port encoder, the SPARK's loop runs in the encoder's turns from the window's
   * edge, -112.5°: on the pivot's axle, a turn of the encoder is a turn of the pivot, so its kP is
   * the slot's per turn of the pivot, 25 times what it is per rotor turn, and its soft limits are
   * where the encoder reads them. Seeded once, the same encoder leaves the loop on the rotor. The
   * feedforward is the same either way.
   */
  @Test
  void aSparksLoopClosesOnItsDataPortEncoderInItsTurns() {
    Sensor.Encoder absolute = Sensor.Encoder.absolute("pivot absolute", PIVOT, 8192);
    RobotMath math =
        RobotDefinitionBuilder.builder(ROBOT).sensors(List.of(absolute)).build().math();
    MechanismWiring.DataPortEncoder port =
        MechanismWiring.DataPortEncoder.builder()
            .offset(Rotations.of(0))
            .reversed(false)
            .closedLoop(false)
            .build();

    RevMechanism.Settings once =
        RevMechanism.configure(
            math,
            PIVOT,
            NEO,
            absolute,
            MechanismWiringBuilder.builder(SPARKS).dataPort(Optional.of(port)).build(),
            CONTROL);
    RevMechanism.Settings closed =
        RevMechanism.configure(
            math,
            PIVOT,
            NEO,
            absolute,
            MechanismWiringBuilder.builder(SPARKS).dataPort(Optional.of(port.closesLoop())).build(),
            CONTROL);

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
    assertThat(closed.feedforward()).isEqualTo(once.feedforward());
  }

  @Test
  void itRefusesWhatASparkCantTakeYet() {
    MechanismControl integral =
        MechanismControlBuilder.builder(CONTROL)
            .gains(MechanismControlGainsBuilder.builder(CONTROL.gains()).kI(1).build())
            .build();
    MechanismWiring.Cancoder cancoder =
        MechanismWiring.Cancoder.builder()
            .id(43)
            .offset(Rotations.of(0))
            .clockwise(false)
            .feedback(MechanismWiring.Feedback.ROTOR)
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RevMechanism.configure(
                    MATH,
                    PIVOT,
                    NEO,
                    wiring(MechanismWiring.Controller.TALON_FXS, Optional.empty()),
                    CONTROL))
        .withMessage("pivot's wiring is a TalonFXS's, not a SPARK's");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RevMechanism.configure(
                    MATH,
                    PIVOT,
                    NEO,
                    MechanismWiringBuilder.builder(SPARKS).canBus(CanBus.canivore("upper")).build(),
                    CONTROL))
        .withMessage("pivot's SPARKs are on a CANivore: a SPARK is on Systemcore's own CAN");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                RevMechanism.configure(
                    MATH,
                    PIVOT,
                    NEO,
                    wiring(MechanismWiring.Controller.SPARK_MAX, Optional.of(cancoder)),
                    CONTROL))
        .withMessageStartingWith("pivot's wiring names a CANcoder, which a SPARK doesn't read");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> RevMechanism.configure(MATH, PIVOT, NEO, SPARKS, integral))
        .withMessageStartingWith("pivot's control has a kI");
  }
}
