package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Rotations;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/** A drivetrain's wiring is checked when it's written down, not when the robot won't drive. */
class SwerveWiringTest {
  private static SwerveWiring.ModuleWiring module(int drive, int steer, int encoder) {
    return SwerveWiring.ModuleWiring.builder()
        .driveId(drive)
        .steerId(steer)
        .encoderOffset(Rotations.of(-0.125))
        .driveInverted(false)
        .steerInverted(true)
        .encoderInverted(false)
        .encoderId(encoder)
        .build();
  }

  private static List<SwerveWiring.ModuleWiring> modules() {
    return List.of(module(1, 2, 1), module(3, 4, 2), module(5, 6, 3), module(7, 8, 4));
  }

  private static SwerveWiring wiring(List<SwerveWiring.ModuleWiring> modules) {
    return SwerveWiring.builder()
        .canBus(CanBus.port(CanBus.Systemcore.CAN_S0))
        .driveController(MechanismWiring.Controller.TALON_FX)
        .steerController(MechanismWiring.Controller.TALON_FX)
        .gyroId(1)
        .gyroYawScale(1.0)
        .modules(modules)
        .build();
  }

  @Test
  void devicesOfDifferentKindsMayShareAnId() {
    SwerveWiring wiring = wiring(modules()); // encoder 1 and drive 1, gyro 1

    assertThat(wiring.modules()).hasSize(4);
    assertThat(wiring.canBus()).isEqualTo(new CanBus.Port(CanBus.Systemcore.CAN_S0));
  }

  @Test
  void twoMotorControllersMayNot() {
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>(modules());
    modules.set(3, module(7, 2, 4));

    assertThatThrownBy(() -> wiring(modules))
        .hasMessage("SwerveWiring.modules: CAN ID 2 is used by two TalonFXs");
  }

  @Test
  void idsAreOnTheBus() {
    assertThatThrownBy(() -> module(63, 2, 1))
        .hasMessage("ModuleWiring.driveId: 63 must be from 0 to 62");
  }

  @Test
  void itTakesFourModules() {
    assertThatThrownBy(() -> wiring(modules().subList(0, 3)))
        .hasMessage("SwerveWiring.modules: must have 4, has 3");
  }

  @Test
  void anOffsetIsWithinATurnAndTheCouplingNeverNegative() {
    assertThatThrownBy(
            () ->
                SwerveWiring.ModuleWiring.builder()
                    .driveId(1)
                    .steerId(2)
                    .encoderOffset(Rotations.of(1.5))
                    .driveInverted(false)
                    .steerInverted(false)
                    .encoderInverted(false)
                    .encoderId(3)
                    .build())
        .hasMessage("ModuleWiring.encoderOffset: 1.5 R must be from -1 R to 1 R");
  }

  @Test
  void aCanivoreHasAName() {
    assertThatThrownBy(() -> CanBus.canivore(" ")).hasMessage("Canivore.name: must not be blank");
  }

  /**
   * A TalonFX steers by a CANcoder, a SPARK by the encoder on its data port, and a TalonFXS doesn't
   * run a module yet.
   */
  @Test
  void eachSteeringReadsItsOwnKindOfEncoder() {
    assertThatThrownBy(
            () ->
                SwerveWiringBuilder.builder(
                        wiring(
                            List.of(
                                module(1, 2, 3),
                                module(4, 5, 6),
                                module(7, 8, 9),
                                module(10, 11, 12))))
                    .steerController(MechanismWiring.Controller.SPARK_MAX)
                    .driveController(MechanismWiring.Controller.SPARK_MAX)
                    .build())
        .hasMessageStartingWith("SwerveWiring.modules: a SPARK steers by the absolute encoder");
    assertThatThrownBy(
            () ->
                SwerveWiringBuilder.builder(
                        wiring(
                            List.of(
                                module(1, 2, 3),
                                module(4, 5, 6),
                                module(7, 8, 9),
                                module(10, 11, 12))))
                    .driveController(MechanismWiring.Controller.TALON_FXS)
                    .build())
        .hasMessageStartingWith("SwerveWiring.driveController: a TalonFXS doesn't run a swerve");
  }
}
