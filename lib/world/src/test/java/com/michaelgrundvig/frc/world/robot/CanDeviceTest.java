package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Rotations;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * Two parts of the program on one motor: the survey found an intake and a roller each building CAN
 * ID 3 and both driving it every loop, which no command's requirements could sort out. The
 * definition refuses two devices of a kind on one bus at one ID, over its one list of them.
 */
class CanDeviceTest {
  private static final Translation3d AXLE = new Translation3d(0.25, 0, 0.32);
  private static final Output RUNNING = Output.volts("running", 6);
  private static final CanBus CANIVORE = CanBus.canivore("drive");

  /** A roller on one controller: a Kraken on a TalonFX, a NEO Vortex on a SPARK. */
  private static Joint.Revolute roller(String label, MechanismWiring.Device controller) {
    Motor motor = controller.controller() == TALON_FX ? Motor.KRAKEN_X60 : Motor.NEO_VORTEX;
    return Joint.roller(label, Part.CHASSIS, AXLE)
        .outputs(RUNNING)
        .motor(motor, Gearing.of(3), controller);
  }

  private static SwerveWiring.ModuleWiring module(int drive, int steer, int encoder) {
    return SwerveWiring.ModuleWiring.builder()
        .driveId(drive)
        .steerId(steer)
        .encoderOffset(Rotations.of(0))
        .driveInverted(false)
        .steerInverted(true)
        .encoderInverted(false)
        .encoderId(encoder)
        .build();
  }

  /** The test robot's drivetrain on Systemcore's CAN: TalonFXs 1 to 8, CANcoders and gyro 1. */
  private static RobotDefinition robot(Joint... joints) {
    return RobotDefinitionBuilder.builder(ConfigFixtures.definition(List.of()))
        .drivetrain(
            ConfigFixtures.drivetrain()
                .wired(
                    SwerveWiring.builder()
                        .canBus(CanBus.SYSTEMCORE)
                        .driveController(TALON_FX)
                        .steerController(TALON_FX)
                        .gyroId(1)
                        .gyroYawScale(1.0)
                        .modules(
                            List.of(
                                module(1, 2, 1), module(3, 4, 2), module(5, 6, 3), module(7, 8, 4)))
                        .build()))
        .joints(List.of(joints))
        .build();
  }

  @Test
  void twoMechanismsOnOneMotorAreNamed() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    roller("coral intake", SPARK_MAX.can(13)),
                    roller("algae intake", SPARK_FLEX.can(13))))
        .withMessage(
            "RobotDefinition[test].devices: two SPARKs on Systemcore's CAN_S0 port share CAN ID 13,"
                + " coral intake motor 13 and algae intake motor 13: give each its own");
  }

  @Test
  void aMechanismOnADrivetrainsIdIsNamed() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(roller("pivot", TALON_FX.can(5))))
        .withMessage(
            "RobotDefinition[test].devices: two TalonFXs on Systemcore's CAN_S0 port share CAN ID"
                + " 5, back-left drive and pivot motor 5: give each its own");
  }

  /**
   * Devices of other kinds, or on other buses, may share an ID: the drivetrain's own CANcoder 1,
   * drive motor 1 and gyro 1 do, and a SPARK at 1 beside them.
   */
  @Test
  void devicesOfOtherKindsOrOnOtherBusesMayShareAnId() {
    RobotDefinition robot =
        robot(roller("pivot", SPARK_MAX.can(1)), roller("roller", TALON_FX.can(CANIVORE, 1)));

    assertThat(robot.devices())
        .filteredOn(device -> device.id() == 1)
        .extracting(CanDevice::name)
        .containsExactly(
            "the gyro",
            "front-left drive",
            "front-left CANcoder",
            "pivot motor 1",
            "roller motor 1");
  }
}
