package frc.robot.lib.config;

import static org.wpilib.units.Units.Rotations;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
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
 *     gyro: flat, or on edge along its long side (landscape) or short side (portrait)
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
    modules = List.copyOf(modules);
    if (modules.size() != 4) {
      throw new IllegalArgumentException(
          "four modules: front left, front right, back left, back right");
    }
    if (!(gyroYawScale >= 0.9 && gyroYawScale <= 1.1)) {
      throw new IllegalArgumentException(
          "a gyro's scale is within 10% of true, or it's broken: " + gyroYawScale);
    }
    if (!(couplingRatio >= 0) || Double.isInfinite(couplingRatio)) {
      throw new IllegalArgumentException("couplingRatio must be zero or more: " + couplingRatio);
    }
    requireId("gyroId", gyroId, new HashSet<>());
    // Devices of different types may share an ID; two of one type may not.
    Set<Integer> motorControllers = new HashSet<>();
    Set<Integer> encoders = new HashSet<>();
    for (ModuleWiring module : modules) {
      requireId("driveId", module.driveId(), motorControllers);
      requireId("steerId", module.steerId(), motorControllers);
      requireId("encoderId", module.encoderId(), encoders);
    }
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
      double rotations = encoderOffset.in(Rotations);
      if (!(Math.abs(rotations) <= 1)) {
        throw new IllegalArgumentException(
            "an encoder offset is within a rotation either way: " + rotations + " rotations");
      }
    }

    /** Starts describing a module's wiring, one named value at a time. */
    public static SwerveWiringModuleWiringBuilder.DriveIdStage builder() {
      return SwerveWiringModuleWiringBuilder.builder();
    }
  }

  private static void requireId(String name, int id, Set<Integer> seen) {
    if (id < 0 || id > 62) {
      throw new IllegalArgumentException(name + " must be a CAN ID from 0 to 62: " + id);
    }
    if (!seen.add(id)) {
      throw new IllegalArgumentException(
          "CAN ID " + id + " is used twice by one kind of device on the drivetrain's bus");
    }
  }
}
