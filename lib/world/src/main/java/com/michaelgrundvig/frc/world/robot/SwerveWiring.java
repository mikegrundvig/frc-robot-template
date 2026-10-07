package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.size;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.Angle;

/**
 * Where a swerve drivetrain's devices are on the CAN bus, which way each motor turns, and how each
 * module's absolute encoder reads: what the robot code needs to drive real hardware. It differs
 * robot to robot even with identical drivetrains, and the encoder offsets change whenever a module
 * is rebuilt, so measure them again then.
 *
 * <p>Phoenix Tuner X's swerve generator reports every value here for a CTRE drivetrain; copy them
 * across as it gives them. Its coupling ratio is the module's gearing, so it goes in the robot's
 * definition ({@code DrivetrainGeometry.ModuleType.couplingRatio}). A REV drivetrain (MAXSwerve,
 * say) has SPARKs and each module's absolute encoder on its steering SPARK's data port.
 *
 * @param canBus the bus every drivetrain device is on
 * @param driveController what the drive motors are on: a TalonFX, or a SPARK MAX or SPARK Flex
 * @param steerController what the steering motors are on, as the drive's; on a SPARK, each module's
 *     absolute encoder is on its data port
 * @param gyroId the Pigeon 2's CAN ID
 * @param gyroYawScale the gyro's readings times this are the true rotation: 1 until the gyro scale
 *     procedure measures it (docs/calibration.md)
 * @param modules front left, front right, back left, back right
 */
@ConfigRecord
public record SwerveWiring(
    CanBus canBus,
    MechanismWiring.Controller driveController,
    MechanismWiring.Controller steerController,
    int gyroId,
    double gyroYawScale,
    List<ModuleWiring> modules) {
  public SwerveWiring {
    modules = List.copyOf(modules);
    List<ModuleWiring> wired = modules;
    Checks.on(SwerveWiring.class)
        .field("gyroId", gyroId, between(0, 62)) // a CAN ID
        .field("gyroYawScale", gyroYawScale, between(0.9, 1.1)) // off by more is a broken gyro
        .field("modules", modules, size(4))
        .field("driveController", driveController, SwerveWiring::notSwerve)
        .field("steerController", steerController, SwerveWiring::notSwerve)
        .check("modules", () -> encoders(steerController, wired))
        .check("modules", () -> sharedIds(driveController, steerController, wired))
        .orThrow();
  }

  /** Starts describing a drivetrain's wiring, one named value at a time. */
  public static SwerveWiringBuilder.CanBusStage builder() {
    return SwerveWiringBuilder.builder();
  }

  /**
   * Its devices on the CAN bus: the gyro, then each module's drive, steering and CANcoder, in the
   * modules' order, "front-left drive", "front-left steering", "front-left CANcoder", each motor
   * controller with the PDH channel that feeds it where its module's wiring says.
   */
  List<CanDevice> devices() {
    List<CanDevice> devices = new ArrayList<>();
    devices.add(
        new CanDevice("the gyro", CanDevice.Kind.PIGEON_2, canBus, gyroId, Optional.empty()));
    for (int i = 0; i < modules.size(); i++) {
      String module = DrivetrainGeometry.LABELS.get(i);
      ModuleWiring wiring = modules.get(i);
      devices.add(
          new CanDevice(
              module + " drive",
              driveController.kind(),
              canBus,
              wiring.driveId(),
              wiring.driveFeed()));
      devices.add(
          new CanDevice(
              module + " steering",
              steerController.kind(),
              canBus,
              wiring.steerId(),
              wiring.steerFeed()));
      wiring
          .encoderId()
          .ifPresent(
              id ->
                  devices.add(
                      new CanDevice(
                          module + " CANcoder",
                          CanDevice.Kind.CANCODER,
                          canBus,
                          id,
                          Optional.empty())));
    }
    return devices;
  }

  /**
   * One module's devices. A CAN ID may be shared by devices of different types, but never by two
   * motor controllers or two encoders on one bus.
   *
   * @param driveId the drive motor controller's CAN ID
   * @param steerId the steering motor controller's CAN ID
   * @param encoderId the CANcoder's CAN ID; empty for an absolute encoder on the steering SPARK's
   *     data port
   * @param encoderOffset added to the absolute encoder's reading so that it reads zero with the
   *     wheel pointing straight ahead: Phoenix Tuner X's swerve generator reports it for each
   *     module
   * @param driveInverted whether the drive motor turns clockwise, seen from its shaft, to drive
   *     forward
   * @param steerInverted whether the steering motor turns clockwise to turn the module
   *     counterclockwise
   * @param encoderInverted whether the absolute encoder counts clockwise
   * @param driveFeed the PDH channel the drive motor controller is powered from, and its breaker:
   *     {@code .driveFeed(Pdh.feed(0, Amps.of(40)))}; empty where it isn't said
   * @param steerFeed the same, the steering motor controller's
   */
  @ConfigRecord
  public record ModuleWiring(
      int driveId,
      int steerId,
      Optional<Integer> encoderId,
      Angle encoderOffset,
      boolean driveInverted,
      boolean steerInverted,
      boolean encoderInverted,
      Optional<Pdh.Feed> driveFeed,
      Optional<Pdh.Feed> steerFeed) {
    public ModuleWiring {
      Checks.on(ModuleWiring.class)
          .field("driveId", driveId, between(0, 62))
          .field("steerId", steerId, between(0, 62))
          .field(
              "encoderId",
              encoderId,
              id -> id.isPresent() ? between(0, 62).whyNot((double) id.get()) : null)
          .field("encoderOffset", encoderOffset, between(Rotations.of(-1), Rotations.of(1)))
          .orThrow();
    }

    /** Starts describing a module's wiring, one named value at a time. */
    public static SwerveWiringModuleWiringBuilder.DriveIdStage builder() {
      return SwerveWiringModuleWiringBuilder.builder();
    }
  }

  /** Why a controller can't run a swerve module yet, or null when it can. */
  private static @Nullable String notSwerve(MechanismWiring.Controller controller) {
    return controller == MechanismWiring.Controller.TALON_FXS
        ? "a TalonFXS doesn't run a swerve module in the template yet"
        : null;
  }

  /**
   * Why the modules' encoders don't fit their steering, or null when they do: a TalonFX steers by a
   * CANcoder; a SPARK by the encoder on its data port, with no CANcoder.
   */
  private static @Nullable String encoders(
      MechanismWiring.Controller steer, List<ModuleWiring> modules) {
    boolean spark = steer != MechanismWiring.Controller.TALON_FX;
    for (ModuleWiring module : modules) {
      if (spark && module.encoderId().isPresent()) {
        return "a SPARK steers by the absolute encoder on its data port, so its module names no"
            + " CANcoder";
      }
      if (!spark && module.encoderId().isEmpty()) {
        return "a TalonFX steers by a CANcoder: give each module its CANcoder's ID";
      }
    }
    return null;
  }

  /**
   * Which CAN IDs two devices of one kind share, or null if none do: two TalonFXs, two SPARKs, or
   * two CANcoders. Devices of different kinds may share an ID.
   */
  private static @Nullable String sharedIds(
      MechanismWiring.Controller drive,
      MechanismWiring.Controller steer,
      List<ModuleWiring> modules) {
    Set<String> seen = new HashSet<>();
    List<String> shared = new ArrayList<>();
    for (ModuleWiring module : modules) {
      for (var device :
          List.of(
              Map.entry(drive.kind(), module.driveId()),
              Map.entry(steer.kind(), module.steerId()))) {
        if (!seen.add(device.getKey() + " " + device.getValue())) {
          shared.add("CAN ID " + device.getValue() + " is used by two " + device.getKey() + "s");
        }
      }
      module
          .encoderId()
          .ifPresent(
              id -> {
                if (!seen.add("CANcoder " + id)) {
                  shared.add("CAN ID " + id + " is used by two CANcoders");
                }
              });
    }
    return shared.isEmpty() ? null : String.join("; ", shared);
  }
}
