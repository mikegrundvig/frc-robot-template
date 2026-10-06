package frc.robot.lib.wiring;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * Two parts of the program on one motor: the survey found an intake and a roller each building CAN
 * ID 3 and both driving it every loop, which no command's requirements could sort out.
 */
class CanIdsTest {
  private static final CanBus BUS = CanBus.port(CanBus.Systemcore.CAN_S0);

  private static MechanismWiring wiring(CanBus bus, int... ids) {
    return MechanismWiring.builder()
        .canBus(bus)
        .controller(MechanismWiring.Controller.TALON_FX)
        .motors(
            java.util.Arrays.stream(ids)
                .mapToObj(id -> MechanismWiring.Motor.builder().id(id).clockwise(false).build())
                .toList())
        .cancoder(Optional.empty())
        .dataPort(Optional.empty())
        .build();
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

  private static final SwerveWiring DRIVETRAIN =
      SwerveWiring.builder()
          .canBus(BUS)
          .driveController(MechanismWiring.Controller.TALON_FX)
          .steerController(MechanismWiring.Controller.TALON_FX)
          .gyroId(1)
          .gyroYawScale(1.0)
          .modules(List.of(module(1, 2, 1), module(3, 4, 2), module(5, 6, 3), module(7, 8, 4)))
          .build();

  @Test
  void twoMechanismsOnOneMotorAreNamed() {
    Map<String, MechanismWiring> mechanisms = new LinkedHashMap<>();
    mechanisms.put("coral intake", wiring(BUS, 13));
    mechanisms.put("algae intake", wiring(BUS, 13));

    assertThat(CanIds.twice(Optional.empty(), mechanisms))
        .isEqualTo(
            "two TalonFXs on Systemcore's CAN_S0 port share CAN ID 13, coral intake and algae"
                + " intake: give each its own");
  }

  @Test
  void aMechanismOnADrivetrainsIdIsNamed() {
    assertThat(CanIds.twice(Optional.of(DRIVETRAIN), Map.of("pivot", wiring(BUS, 5))))
        .isEqualTo(
            "two TalonFXs on Systemcore's CAN_S0 port share CAN ID 5, the back left module's"
                + " drive and pivot: give each its own");
  }

  @Test
  void devicesOfOtherKindsOrOnOtherBusesMayShareAnId() {
    Map<String, MechanismWiring> mechanisms = new LinkedHashMap<>();
    mechanisms.put("pivot", wiring(BUS, 11));
    mechanisms.put("roller", wiring(CanBus.canivore("drive"), 11));

    // The drivetrain's own CANcoder 1, drive motor 1 and gyro 1 share an ID too.
    assertThat(CanIds.twice(Optional.of(DRIVETRAIN), mechanisms)).isNull();
  }
}
