package frc.robot.subsystems.drive;

import frc.robot.RobotConfig;
import frc.robot.SimTestSupport;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;

/**
 * Drives the drivetrain in simulation the way the robot runs it: the simulated Driver Station, the
 * command scheduler, and the joystick drive command, on simulated modules.
 */
final class DriveSimHarness implements AutoCloseable {
  final Scheduler scheduler = Scheduler.createIndependentScheduler();
  final Drive drive;

  DriveSimHarness(RobotConfig robot) {
    drive =
        new Drive(
            "Drive",
            new GyroIO() {},
            ModuleIOSim.of(robot.drivetrain().modules()),
            robot.body(),
            robot::drivetrain,
            scheduler);
  }

  /** Hold the sticks (as {@link Drive#driveWithSticks} takes them) for {@code seconds}, enabled. */
  void drive(double seconds, double x, double y, double omega) {
    SimTestSupport.setEnabled(true);
    Command driving = drive.joystickDrive(() -> x, () -> y, () -> omega);
    scheduler.schedule(driving);
    for (int i = 0; i < SimTestSupport.loops(seconds); i++) {
      SimTestSupport.stepLoop();
      drive.periodic(RobotState.isEnabled());
      scheduler.run();
    }
    scheduler.cancel(driving);
  }

  /** Run a command for {@code seconds}, enabled, the way the robot loop runs it. */
  void run(Command command, double seconds) {
    SimTestSupport.setEnabled(true);
    scheduler.schedule(command);
    for (int i = 0; i < SimTestSupport.loops(seconds); i++) {
      SimTestSupport.stepLoop();
      drive.periodic(RobotState.isEnabled());
      scheduler.run();
    }
  }

  @Override
  public void close() {
    drive.close();
  }
}
