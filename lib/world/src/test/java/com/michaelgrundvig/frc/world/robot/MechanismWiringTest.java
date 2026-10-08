package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/** A mechanism's wiring is checked when it's written down, not when the mechanism won't move. */
class MechanismWiringTest {
  /** A joint's motors on TalonFXs at these IDs, the first leading. */
  private static Joint.Revolute onTalons(MechanismWiring.Device... controllers) {
    return Joint.pivot("arm", Part.CHASSIS, new Translation3d(), Joint.PITCH_UP)
        .motors(Motor.KRAKEN_X60, Gearing.of(60), controllers);
  }

  @Test
  void theFirstMotorLeads() {
    Joint.Revolute arm =
        onTalons(
            MechanismWiring.Controller.TALON_FX.can(11),
            MechanismWiring.Controller.TALON_FX.can(12));

    assertThat(arm.driven().motors().orElseThrow().controllers().getFirst().id()).isEqualTo(11);
    assertThat(arm.driven().motors().orElseThrow().count()).isEqualTo(2);
  }

  /**
   * A motor its block wires to nothing runs on the controller it plugs into: a NEO's a SPARK MAX, a
   * NEO Vortex's a SPARK Flex, a Minion's a TalonFXS, any other's a TalonFX.
   */
  @Test
  void anUnwiredMotorStandsInOnItsOwnController() {
    assertThat(MechanismWiring.Controller.standIn(Motor.NEO))
        .isEqualTo(MechanismWiring.Controller.SPARK_MAX);
    assertThat(MechanismWiring.Controller.standIn(Motor.NEO_550))
        .isEqualTo(MechanismWiring.Controller.SPARK_MAX);
    assertThat(MechanismWiring.Controller.standIn(Motor.NEO_VORTEX))
        .isEqualTo(MechanismWiring.Controller.SPARK_FLEX);
    assertThat(MechanismWiring.Controller.standIn(Motor.MINION))
        .isEqualTo(MechanismWiring.Controller.TALON_FXS);
    assertThat(MechanismWiring.Controller.standIn(Motor.KRAKEN_X60))
        .isEqualTo(MechanismWiring.Controller.TALON_FX);
  }

  /**
   * A joint's motors are at least one, and no two of its controllers share an ID: the definition
   * says so as it lists every device on the bus once.
   */
  @Test
  void itTakesAMotorAndNoIdTwice() {
    assertThatThrownBy(() -> onTalons()).hasMessage("Motors.count: 0 must be positive");
    assertThatThrownBy(
            () ->
                RobotDefinitionBuilder.builder(ConfigFixtures.definition(List.of()))
                    .joints(
                        List.of(
                            onTalons(
                                MechanismWiring.Controller.TALON_FX.can(11),
                                MechanismWiring.Controller.TALON_FX.can(11))))
                    .build())
        .hasMessageContaining(
            "two TalonFXs on Systemcore's CAN_S0 port share CAN ID 11, arm motor 11 and arm motor"
                + " 11: give each its own");
  }

  @Test
  void idsAreOnTheBusAndAnOffsetWithinATurn() {
    assertThatThrownBy(() -> MechanismWiring.Controller.TALON_FX.can(63))
        .hasMessage("Device.id: 63 must be from 0 to 62");
    assertThatThrownBy(() -> CANCODER.can(5).offset(Rotations.of(1.5)))
        .hasMessage("Cancoder.offset: 1.5 R must be from -1 R to 1 R");
    assertThatThrownBy(() -> CANCODER.can(63).offset(Rotations.of(0)))
        .hasMessage("Cancoder.id: 63 must be from 0 to 62");
  }

  /**
   * A data-port encoder says what its controller reads raw where it reads a number of turns, its
   * offset and direction undone, from 0 to 1, a turn on or back the same: what a SPARK is told to
   * read as zero. The joint has one absolute encoder, so the wiring reads it one way.
   */
  @Test
  void aDataPortEncoderGivesItsRawReadingAndIsTheOnlyReader() {
    MechanismWiring.DataPortEncoder port = DATA_PORT.offset(Rotations.of(-0.25)).inverted();
    MechanismWiring.Cancoder cancoder = CANCODER.can(5).offset(Rotations.of(0));

    // Reversed, offset a quarter turn back: reading -0.35 turns, it reads 0.1 raw.
    assertThat(port.raw(-0.35)).isCloseTo(0.1, within(1e-12));
    assertThat(port.raw(0.65)).isCloseTo(0.1, within(1e-12));
    assertThatThrownBy(() -> Driven.AbsoluteEncoder.of().on(cancoder).on(port))
        .hasMessage(
            "AbsoluteEncoder.port: names more than one of a CANcoder, a data port and a Systemcore"
                + " port: it's read one way");
  }

  /**
   * A port encoder, or a data-port one, tells its motor where the joint is, once, unless its wiring
   * says the joint's loop closes on it; nothing else about it changes.
   */
  @Test
  void anEncodersLoopClosesOnItOnlyWhenItsWiringSays() {
    MechanismWiring.PortEncoder once =
        MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0.82)).inverted();
    MechanismWiring.PortEncoder closing = once.closesLoop();

    assertThat(once.closedLoop()).isFalse();
    assertThat(MechanismWiring.PortEncoder.analog(2, 10).closedLoop()).isFalse();
    assertThat(closing.closedLoop()).isTrue();
    assertThat(closing.read(0.25)).isEqualTo(once.read(0.25));
    assertThat(Driven.AbsoluteEncoder.of().on(once).closedLoop()).isFalse();
    assertThat(Driven.AbsoluteEncoder.of().on(closing).closedLoop()).isTrue();
    MechanismWiring.DataPortEncoder dataPort = DATA_PORT.offset(Rotations.of(-0.25)).inverted();
    assertThat(dataPort.closesLoop().closedLoop()).isTrue();
    assertThat(dataPort.closesLoop().raw(0.4)).isEqualTo(dataPort.raw(0.4));
    assertThat(Driven.AbsoluteEncoder.of().on(dataPort).closedLoop()).isFalse();
    assertThat(Driven.AbsoluteEncoder.of().on(dataPort.closesLoop()).closedLoop()).isTrue();
  }

  /**
   * A CANcoder is named as a motor's controller is, by its bus and ID, with its offset: read once
   * at startup unless it says the TalonFX's loop runs on it, counterclockwise unless it says
   * otherwise.
   */
  @Test
  void aCancoderIsNamedByItsBusAndIdWithItsOffset() {
    CanBus canivore = CanBus.canivore("canivore");
    MechanismWiring.Cancoder seeded = CANCODER.can(canivore, 33).offset(Rotations.of(0.82));

    assertThat(seeded)
        .isEqualTo(
            MechanismWiring.Cancoder.builder()
                .bus(canivore)
                .id(33)
                .offset(Rotations.of(0.82))
                .turnsClockwise(false)
                .feedback(MechanismWiring.Feedback.ROTOR)
                .build());
    assertThat(CANCODER.can(33).offset(Rotations.of(0)).bus()).isEqualTo(CanBus.SYSTEMCORE);
    assertThat(CANCODER.can(canivore, 33).offset(Rotations.of(0.82)).clockwise().turnsClockwise())
        .isTrue();
    assertThat(seeded.remote().feedback()).isEqualTo(MechanismWiring.Feedback.REMOTE);
    assertThat(seeded.fused().feedback()).isEqualTo(MechanismWiring.Feedback.FUSED);
    assertThat(seeded.sync().feedback()).isEqualTo(MechanismWiring.Feedback.SYNC);
    assertThat(seeded.fused().offset()).isEqualTo(seeded.offset());
  }

  /** A data-port encoder is named by its offset: read once, its reading growing with the joint. */
  @Test
  void aDataPortEncoderIsNamedByItsOffset() {
    MechanismWiring.DataPortEncoder once = DATA_PORT.offset(Rotations.of(0.3));

    assertThat(once)
        .isEqualTo(
            MechanismWiring.DataPortEncoder.builder()
                .offset(Rotations.of(0.3))
                .reversed(false)
                .closedLoop(false)
                .build());
    assertThat(once.inverted().reversed()).isTrue();
    assertThat(once.inverted().closesLoop())
        .isEqualTo(
            MechanismWiring.DataPortEncoder.builder()
                .offset(Rotations.of(0.3))
                .reversed(true)
                .closedLoop(true)
                .build());
  }
}
