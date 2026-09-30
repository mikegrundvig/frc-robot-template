package frc.robot;

import frc.robot.lib.config.Body;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
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

  /**
   * Where odometry starts, in every mode: on the field, clear of the walls. The same in every mode
   * so a replay starts where the run it replays did; an auto routine or vision sets the real pose.
   */
  static final Pose2d START_POSE = new Pose2d(3, 3, Rotation2d.ZERO);

  /** The simulator's field view, showing where odometry puts the robot. Null unless simulating. */
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
    ModuleIO[] modules;
    if (mode == Constants.Mode.SIM) {
      modules = ModuleIOSim.of(drivetrain.get().modules());
    } else {
      // In replay the inputs come from the log. On a robot there is no hardware code yet, so the
      // drivetrain stays still rather than guessing at motors, and says so.
      modules =
          new ModuleIO[] {
            new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}
          };
      noHardware.set(mode == Constants.Mode.REAL);
    }
    // No gyro is simulated: there the heading comes from the wheels, as in AdvantageKit's template.
    drive = new Drive("Drive", new GyroIO() {}, modules, body, drivetrain, scheduler);
    field = mode == Constants.Mode.SIM ? new Field2d() : null;
    drive.resetPose(START_POSE);

    // Driver: left stick translates (field-relative), right stick turns. Gamepad Y is negative
    // when pushed forward, so negate to get +x away from the driver.
    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
  }

  /** Mechanism periodic methods, called once per loop before the command scheduler runs. */
  public void periodic() {
    drive.periodic(RobotState.isEnabled());
    if (field != null) {
      field.setRobotPose(drive.getPose());
      Telemetry.log("Field", field);
    }
  }

  /** A pit check for the Pitwall page: drive forward slowly for two seconds. */
  Command driveCheck() {
    return drive.driveForward(0.5, 2.0);
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
