package frc.robot.lib.config;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Rotations;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.imu.OnboardIMU;

/** A drivetrain's wiring is checked when it's written down, not when the robot won't drive. */
class SwerveWiringTest {
  private static SwerveWiring.ModuleWiring module(int drive, int steer, int encoder) {
    return SwerveWiring.ModuleWiring.builder()
        .driveId(drive)
        .steerId(steer)
        .encoderId(encoder)
        .encoderOffset(Rotations.of(-0.125))
        .driveInverted(false)
        .steerInverted(true)
        .encoderInverted(false)
        .build();
  }

  private static List<SwerveWiring.ModuleWiring> modules() {
    return List.of(module(1, 2, 1), module(3, 4, 2), module(5, 6, 3), module(7, 8, 4));
  }

  private static SwerveWiring wiring(List<SwerveWiring.ModuleWiring> modules) {
    return SwerveWiring.builder()
        .canBus(CanBus.port(CANPort.CAN_S0))
        .gyroId(1)
        .couplingRatio(3.5)
        .modules(modules)
        .systemcoreMount(OnboardIMU.MountOrientation.FLAT)
        .build();
  }

  @Test
  void devicesOfDifferentKindsMayShareAnId() {
    SwerveWiring wiring = wiring(modules()); // encoder 1 and drive 1, gyro 1

    assertThat(wiring.modules()).hasSize(4);
    assertThat(wiring.canBus()).isEqualTo(new CanBus.Port(CANPort.CAN_S0));
  }

  @Test
  void twoMotorControllersMayNot() {
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>(modules());
    modules.set(3, module(7, 2, 4));

    assertThatThrownBy(() -> wiring(modules)).hasMessageContaining("CAN ID 2 is used twice");
  }

  @Test
  void idsAreOnTheBus() {
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>(modules());
    modules.set(0, module(63, 2, 1));

    assertThatThrownBy(() -> wiring(modules)).hasMessageContaining("from 0 to 62");
  }

  @Test
  void itTakesFourModules() {
    assertThatThrownBy(() -> wiring(modules().subList(0, 3))).hasMessageContaining("four modules");
  }

  @Test
  void anOffsetIsWithinATurnAndTheCouplingNeverNegative() {
    assertThatThrownBy(
            () ->
                SwerveWiring.ModuleWiring.builder()
                    .driveId(1)
                    .steerId(2)
                    .encoderId(3)
                    .encoderOffset(Rotations.of(1.5))
                    .driveInverted(false)
                    .steerInverted(false)
                    .encoderInverted(false)
                    .build())
        .hasMessageContaining("within a rotation");
    assertThatThrownBy(
            () ->
                SwerveWiring.builder()
                    .canBus(CanBus.canivore("drivetrain"))
                    .gyroId(1)
                    .couplingRatio(-1)
                    .modules(modules())
                    .systemcoreMount(OnboardIMU.MountOrientation.FLAT)
                    .build())
        .hasMessageContaining("couplingRatio");
  }

  @Test
  void aCanivoreHasAName() {
    assertThatThrownBy(() -> CanBus.canivore(" ")).hasMessageContaining("named");
  }
}
