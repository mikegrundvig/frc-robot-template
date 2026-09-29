package frc.robot;

import frc.robot.lib.config.Body;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.swerve.SwerveSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;

/**
 * The composition root: builds every mechanism with the right hardware for the current mode, and
 * binds controls to commands. This is the one file that shows how the whole robot fits together, so
 * keep it readable.
 */
public final class RobotContainer implements AutoCloseable {
  private static final int DRIVER_PORT = 0;
  private static final double LOOP_PERIOD_S = 0.02;

  /** Physics for the whole robot in simulation; null on a real robot or in replay. */
  private final @Nullable SwerveSim swerveSim;

  /**
   * The simulator's field view: the robot where odometry thinks it is, plus a "TruePose" ghost
   * where the physics put it. Null unless simulating.
   */
  private final @Nullable Field2d field;

  private final Drive drive;
  private final CommandGamepad driver;
  private final Alert noHardware =
      new Alert(
          "Drive",
          "The drivetrain has no motor-controller code yet: it runs only in simulation. Add a"
              + " ModuleIO for your hardware (subsystems/drive).",
          Alert.Level.HIGH);

  /**
   * @param body the robot's frame and mass
   * @param drivetrain the drivetrain's configuration, with any values tuned while running
   */
  public RobotContainer(
      Constants.Mode mode, Scheduler scheduler, Body body, Supplier<SwerveConfig> drivetrain) {
    driver = new CommandGamepad(scheduler, DRIVER_PORT);
    swerveSim =
        mode == Constants.Mode.SIM
            ? new SwerveSim(
                body,
                drivetrain.get(),
                SwerveSim.Battery.TYPICAL,
                new Pose2d(3, 3, Rotation2d.ZERO))
            : null;
    GyroIO gyro;
    ModuleIO[] modules;
    if (swerveSim != null) {
      gyro = new GyroIOSim(swerveSim);
      modules = ModuleIOSim.of(swerveSim);
    } else {
      // In replay the inputs come from the log. On a robot there is no hardware code yet, so the
      // drivetrain stays still rather than guessing at motors, and says so.
      gyro = new GyroIO() {};
      modules =
          new ModuleIO[] {
            new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}
          };
      noHardware.set(mode == Constants.Mode.REAL);
    }
    drive = new Drive("Drive", gyro, modules, body, drivetrain, scheduler);
    field = swerveSim != null ? new Field2d() : null;
    if (swerveSim != null) {
      drive.resetPose(swerveSim.truePose());
    }

    // Driver: left stick translates (field-relative), right stick turns. Gamepad Y is negative
    // when pushed forward, so negate to get +x away from the driver.
    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
  }

  /** Mechanism periodic methods, called once per loop before the command scheduler runs. */
  public void periodic() {
    boolean enabled = RobotState.isEnabled();
    if (swerveSim != null) {
      swerveSim.update(LOOP_PERIOD_S, enabled);
      // Report the simulated battery through WPILib, so everything that reads battery voltage
      // (dashboards, AdvantageKit's logs, brownout detection) sees the sag, not a steady 12 V.
      RoboRioSim.setVInVoltage(swerveSim.batteryVolts());
      Logger.recordOutput("Sim/TruePose", swerveSim.truePose());
      Logger.recordOutput("Sim/BatteryVolts", swerveSim.batteryVolts());
      Logger.recordOutput("Sim/TotalSupplyAmps", swerveSim.totalSupplyAmps());
    }
    drive.periodic(enabled);
    if (field != null && swerveSim != null) {
      field.setRobotPose(drive.getPose());
      field.getObject("TruePose").setPose(swerveSim.truePose());
      Telemetry.log("Field", field);
    }
  }

  /** Simulated physics, for tests. Null unless simulating. */
  @Nullable SwerveSim swerveSim() {
    return swerveSim;
  }

  Drive drive() {
    return drive;
  }

  /** Whether the alert saying the drivetrain has no hardware code is active. */
  boolean isNoHardwareAlertActive() {
    return noHardware.get();
  }

  /** Releases what holds global resources (alerts), so a test can build another robot. */
  @Override
  public void close() {
    drive.close();
    noHardware.close();
    if (field != null) {
      field.close();
    }
  }
}
