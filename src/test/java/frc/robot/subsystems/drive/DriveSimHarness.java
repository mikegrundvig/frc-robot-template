package frc.robot.subsystems.drive;

import frc.robot.RobotConfig;
import frc.robot.SimTestSupport;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;

/**
 * Drives the drivetrain in simulation the way the robot runs it: the simulated Driver Station, the
 * command scheduler, and the joystick drive command, on simulated modules and a simulated gyro,
 * with the simulator's physics stepped before each loop reads the sensors.
 */
final class DriveSimHarness implements AutoCloseable {
  final Scheduler scheduler = Scheduler.createIndependentScheduler();
  final DriveSimulation simulation;
  final Drive drive;

  /** Wheels and sensors exactly as configured: odometry is the truth. */
  DriveSimHarness(RobotConfig robot) {
    this(robot, SensorErrors.NONE);
  }

  DriveSimHarness(RobotConfig robot, SensorErrors errors) {
    simulation =
        new DriveSimulation(
            robot.drivetrain().modules(),
            robot.drivetrain().moduleTranslations(robot.body()),
            errors);
    drive =
        new Drive(
            "Drive",
            simulation.gyro(),
            simulation.modules(),
            robot.body(),
            robot::drivetrain,
            scheduler);
  }

  /** One robot loop: the physics moves, then the drivetrain reads its sensors, then commands. */
  void loop() {
    SimTestSupport.stepLoop();
    simulation.step(SimTestSupport.LOOP_PERIOD_S);
    drive.periodic(RobotState.isEnabled());
    scheduler.run();
  }

  /** Hold the sticks (as {@link Drive#driveWithSticks} takes them) for {@code seconds}, enabled. */
  void drive(double seconds, double x, double y, double omega) {
    SimTestSupport.setEnabled(true);
    Command driving = drive.joystickDrive(() -> x, () -> y, () -> omega);
    scheduler.schedule(driving);
    for (int i = 0; i < SimTestSupport.loops(seconds); i++) {
      loop();
    }
    scheduler.cancel(driving);
  }

  /** Run a command for {@code seconds}, enabled, the way the robot loop runs it. */
  void run(Command command, double seconds) {
    SimTestSupport.setEnabled(true);
    scheduler.schedule(command);
    for (int i = 0; i < SimTestSupport.loops(seconds); i++) {
      loop();
    }
  }

  /** Sit disabled for {@code seconds}. */
  void idle(double seconds) {
    SimTestSupport.setEnabled(false);
    for (int i = 0; i < SimTestSupport.loops(seconds); i++) {
      loop();
    }
  }

  /** How far the drivetrain's estimate is from where the robot really is, in meters. */
  double positionError() {
    return drive.getPose().getTranslation().getDistance(simulation.truePose().getTranslation());
  }

  /** How far the estimate's heading is turned from the truth, in degrees. */
  double headingErrorDegrees() {
    Pose2d truth = simulation.truePose();
    return drive.getPose().getRotation().minus(truth.getRotation()).getDegrees();
  }

  @Override
  public void close() {
    drive.close();
  }
}
