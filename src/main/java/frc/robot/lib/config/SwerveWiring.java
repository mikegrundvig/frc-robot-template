package frc.robot.lib.config;

import static frc.robot.lib.config.Rules.between;
import static frc.robot.lib.config.Rules.nonNegative;
import static frc.robot.lib.config.Rules.size;
import static org.wpilib.units.Units.Rotations;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.units.measure.Angle;

/**
 * Where a swerve drivetrain's devices are on the CAN bus, which way each motor turns, and how each
 * module's absolute encoder reads: what the robot code needs to drive real hardware. It differs
 * robot to robot even with identical drivetrains, and the encoder offsets change whenever a module
 * is rebuilt, so measure them again then.
 *
 * <p>Phoenix Tuner X's swerve generator reports every value here for a CTRE drivetrain; copy them
 * across as it gives them.
 *
 * @param canBus the bus every drivetrain device is on
 * @param gyroId the Pigeon 2's CAN ID
 * @param gyroYawScale the gyro's readings times this are the true rotation: 1 until the gyro scale
 *     procedure measures it (docs/calibration.md)
 * @param couplingRatio how many rotations the drive motor turns for each full turn of the module,
 *     with the wheel held still: turning an MK4-style module turns its wheel through the bevel
 *     gear, which the drive encoder would otherwise count as travel. Zero for modules without
 *     coupling.
 * @param modules front left, front right, back left, back right
 * @param systemcoreMount how Systemcore itself is mounted, for its onboard IMU, the backup to the
 *     gyro: flat, or on edge along its long side (landscape) or short side (portrait). WPILib takes
 *     its yaw about the axis this says is up. How the board is turned on the robot, which turns its
 *     accelerometer's axes into the robot's, is its mount in the robot's definition ({@code
 *     Imus.backup}).
 */
@ConfigRecord
public record SwerveWiring(
    CanBus canBus,
    int gyroId,
    double gyroYawScale,
    double couplingRatio,
    List<ModuleWiring> modules,
    OnboardIMU.MountOrientation systemcoreMount) {
  public SwerveWiring {
    Checks.on(SwerveWiring.class)
        .field("gyroId", gyroId, between(0, 62)) // a CAN ID
        .field("gyroYawScale", gyroYawScale, between(0.9, 1.1)) // off by more is a broken gyro
        .field("couplingRatio", couplingRatio, nonNegative())
        .field("modules", modules, size(4))
        .field("modules", modules, SwerveWiring::sharedIds)
        .orThrow();
  }

  /** Starts describing a drivetrain's wiring, one named value at a time. */
  public static SwerveWiringBuilder.CanBusStage builder() {
    return SwerveWiringBuilder.builder();
  }

  /**
   * One module's devices. A CAN ID may be shared by devices of different types, but never by two
   * motor controllers or two encoders on one bus.
   *
   * @param driveId the drive motor controller's CAN ID
   * @param steerId the steering motor controller's CAN ID
   * @param encoderId the CANcoder's CAN ID
   * @param encoderOffset added to the CANcoder's reading so that it reads zero with the wheel
   *     pointing straight ahead: Phoenix Tuner X's swerve generator reports it for each module
   * @param driveInverted whether the drive motor turns clockwise, seen from its shaft, to drive
   *     forward
   * @param steerInverted whether the steering motor turns clockwise to turn the module
   *     counterclockwise
   * @param encoderInverted whether the CANcoder counts clockwise
   */
  @ConfigRecord
  public record ModuleWiring(
      int driveId,
      int steerId,
      int encoderId,
      Angle encoderOffset,
      boolean driveInverted,
      boolean steerInverted,
      boolean encoderInverted) {
    public ModuleWiring {
      Checks.on(ModuleWiring.class)
          .field("driveId", driveId, between(0, 62))
          .field("steerId", steerId, between(0, 62))
          .field("encoderId", encoderId, between(0, 62))
          .field("encoderOffset", encoderOffset, between(Rotations.of(-1), Rotations.of(1)))
          .orThrow();
    }

    /** Starts describing a module's wiring, one named value at a time. */
    public static SwerveWiringModuleWiringBuilder.DriveIdStage builder() {
      return SwerveWiringModuleWiringBuilder.builder();
    }
  }

  /**
   * Which CAN IDs two motor controllers, or two encoders, share, or null if none do. Devices of
   * different kinds may share an ID; two of one kind on one bus may not.
   */
  private static @Nullable String sharedIds(List<ModuleWiring> modules) {
    Set<Integer> motorControllers = new HashSet<>();
    Set<Integer> encoders = new HashSet<>();
    List<String> shared = new ArrayList<>();
    for (ModuleWiring module : modules) {
      for (int id : List.of(module.driveId(), module.steerId())) {
        if (!motorControllers.add(id)) {
          shared.add("CAN ID " + id + " is used by two motor controllers");
        }
      }
      if (!encoders.add(module.encoderId())) {
        shared.add("CAN ID " + module.encoderId() + " is used by two encoders");
      }
    }
    return shared.isEmpty() ? null : String.join("; ", shared);
  }
}
