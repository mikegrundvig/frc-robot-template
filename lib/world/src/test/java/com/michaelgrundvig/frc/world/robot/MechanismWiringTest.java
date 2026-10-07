package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Rotations;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/** A mechanism's wiring is checked when it's written down, not when the mechanism won't move. */
class MechanismWiringTest {
  private static MechanismWiring.Motor motor(int id) {
    return MechanismWiring.Motor.builder().id(id).clockwise(false).build();
  }

  private static MechanismWiring wiring(List<MechanismWiring.Motor> motors) {
    return MechanismWiring.builder()
        .canBus(CanBus.port(CanBus.Systemcore.CAN_S0))
        .controller(MechanismWiring.Controller.TALON_FX)
        .motors(motors)
        .cancoder(Optional.empty())
        .dataPort(Optional.empty())
        .build();
  }

  @Test
  void theFirstMotorLeads() {
    MechanismWiring wiring = wiring(List.of(motor(11), motor(12)));

    assertThat(wiring.leader().id()).isEqualTo(11);
    assertThat(wiring.motors()).hasSize(2);
  }

  @Test
  void itTakesAMotorAndNoIdTwice() {
    assertThatThrownBy(() -> wiring(List.of())).hasMessage("MechanismWiring.motors: needs a motor");
    assertThatThrownBy(() -> wiring(List.of(motor(11), motor(11))))
        .hasMessage("MechanismWiring.motors: CAN ID 11 is used by two motor controllers");
  }

  @Test
  void idsAreOnTheBusAndAnOffsetWithinATurn() {
    assertThatThrownBy(() -> motor(63)).hasMessage("Motor.id: 63 must be from 0 to 62");
    assertThatThrownBy(
            () ->
                MechanismWiring.Cancoder.builder()
                    .id(5)
                    .offset(Rotations.of(1.5))
                    .clockwise(false)
                    .feedback(MechanismWiring.Feedback.REMOTE)
                    .build())
        .hasMessage("Cancoder.offset: 1.5 R must be from -1 R to 1 R");
  }

  /**
   * A data-port encoder says what its controller reads raw where it reads a number of turns, its
   * offset and direction undone, from 0 to 1, a turn on or back the same: what a SPARK is told to
   * read as zero. The joint has one absolute encoder, so the wiring reads it one way.
   */
  @Test
  void aDataPortEncoderGivesItsRawReadingAndIsTheOnlyReader() {
    MechanismWiring.DataPortEncoder port =
        MechanismWiring.DataPortEncoder.builder()
            .offset(Rotations.of(-0.25))
            .reversed(true)
            .closedLoop(false)
            .build();
    MechanismWiring.Cancoder cancoder =
        MechanismWiring.Cancoder.builder()
            .id(5)
            .offset(Rotations.of(0))
            .clockwise(false)
            .feedback(MechanismWiring.Feedback.ROTOR)
            .build();

    // Reversed, offset a quarter turn back: reading -0.35 turns, it reads 0.1 raw.
    assertThat(port.raw(-0.35)).isCloseTo(0.1, within(1e-12));
    assertThat(port.raw(0.65)).isCloseTo(0.1, within(1e-12));
    assertThatThrownBy(
            () ->
                MechanismWiringBuilder.builder(wiring(List.of(motor(11))))
                    .cancoder(Optional.of(cancoder))
                    .dataPort(Optional.of(port))
                    .build())
        .hasMessageStartingWith("MechanismWiring.dataPort: names a CANcoder and a data-port");
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
    MechanismWiring.DataPortEncoder dataPort =
        MechanismWiring.DataPortEncoder.builder()
            .offset(Rotations.of(-0.25))
            .reversed(true)
            .closedLoop(false)
            .build();
    assertThat(dataPort.closesLoop().closedLoop()).isTrue();
    assertThat(dataPort.closesLoop().raw(0.4)).isEqualTo(dataPort.raw(0.4));
    assertThat(Driven.AbsoluteEncoder.of().on(dataPort).closedLoop()).isFalse();
    assertThat(Driven.AbsoluteEncoder.of().on(dataPort.closesLoop()).closedLoop()).isTrue();
  }
}
