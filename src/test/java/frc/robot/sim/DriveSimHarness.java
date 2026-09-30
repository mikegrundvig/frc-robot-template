package frc.robot.sim;

import frc.robot.RobotConfig;
import frc.robot.SimTestSupport;
import frc.robot.lib.localization.EstimatorConfig;
import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.PoseEstimator;
import frc.robot.lib.localization.PoseReset;
import frc.robot.lib.localization.VisionDecision;
import frc.robot.lib.localization.VisionMeasurement;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.RobotController;

/**
 * Drives the drivetrain in simulation the way the robot runs it: the simulated Driver Station, the
 * command scheduler, and the joystick drive command, on simulated modules and a simulated gyro,
 * with the simulator's physics stepped before each loop reads the sensors.
 */
public final class DriveSimHarness implements AutoCloseable {
  public final Scheduler scheduler = Scheduler.createIndependentScheduler();
  public final DriveSimulation simulation;
  public final PoseEstimator estimator;
  public final Drive drive;

  /** Every camera measurement's fate, in order. */
  public final List<VisionDecision> decisions = new ArrayList<>();

  private @Nullable FakeCamera camera;
  private @Nullable Runnable eachLoop;

  /**
   * An estimator that starts with the robot exactly here, and accepts any point as on the field.
   */
  public static PoseEstimator estimatorAt(Pose2d start) {
    return estimatorAt(start, EstimatorConfig.DEFAULTS);
  }

  public static PoseEstimator estimatorAt(Pose2d start, EstimatorConfig config) {
    return new PoseEstimator(
        config,
        point -> true,
        new PoseReset(0, start, PoseCovariance.fromStdDevs(0.001, 0.001, 0.001), "test start"));
  }

  /** Wheels and sensors exactly as configured: odometry is the truth. */
  public DriveSimHarness(RobotConfig robot) {
    this(robot, SensorErrors.NONE);
  }

  public DriveSimHarness(RobotConfig robot, SensorErrors errors) {
    this(robot, errors, EstimatorConfig.DEFAULTS);
  }

  public DriveSimHarness(RobotConfig robot, SensorErrors errors, EstimatorConfig config) {
    estimator = estimatorAt(Pose2d.ZERO, config);
    simulation =
        new DriveSimulation(
            robot.drivetrain().modules(),
            robot.drivetrain().moduleTranslations(robot.body()),
            errors);
    drive =
        new Drive(
            "Drive",
            simulation.gyro(),
            simulation.backupGyro(),
            simulation.modules(),
            robot.body(),
            robot::drivetrain,
            scheduler,
            estimator,
            new ReentrantLock());
  }

  /** From now on, a camera's results reach the estimator every loop. */
  public void watchWith(FakeCamera camera) {
    this.camera = camera;
  }

  /** Something to do at the end of every loop, such as feeding another estimator to compare. */
  public void eachLoop(Runnable action) {
    eachLoop = action;
  }

  /**
   * One robot loop: the physics moves, the drivetrain reads its sensors, the camera's results
   * arrive, the estimator takes them, then commands run.
   */
  public void loop() {
    SimTestSupport.stepLoop();
    long now = RobotController.getTime();
    simulation.step(SimTestSupport.LOOP_PERIOD_S, now);
    drive.periodic(RobotState.isEnabled());
    FakeCamera watching = camera;
    if (watching != null) {
      for (VisionMeasurement measurement : watching.poll(now)) {
        estimator.addVision(measurement);
      }
    }
    decisions.addAll(estimator.update());
    scheduler.run();
    Runnable action = eachLoop;
    if (action != null) {
      action.run();
    }
  }

  /** Hold the sticks (as {@link Drive#driveWithSticks} takes them) for {@code seconds}, enabled. */
  public void drive(double seconds, double x, double y, double omega) {
    SimTestSupport.setEnabled(true);
    Command driving = drive.joystickDrive(() -> x, () -> y, () -> omega);
    scheduler.schedule(driving);
    for (int i = 0; i < SimTestSupport.loops(seconds); i++) {
      loop();
    }
    scheduler.cancel(driving);
  }

  /** Run a command for {@code seconds}, enabled, the way the robot loop runs it. */
  public void run(Command command, double seconds) {
    SimTestSupport.setEnabled(true);
    scheduler.schedule(command);
    for (int i = 0; i < SimTestSupport.loops(seconds); i++) {
      loop();
    }
  }

  /** Sit disabled for {@code seconds}. */
  public void idle(double seconds) {
    SimTestSupport.setEnabled(false);
    for (int i = 0; i < SimTestSupport.loops(seconds); i++) {
      loop();
    }
  }

  /** How far the drivetrain's estimate is from where the robot really is, in meters. */
  public double positionError() {
    return drive.getPose().getTranslation().getDistance(simulation.truePose().getTranslation());
  }

  /** How far the estimate's heading is turned from the truth, in degrees. */
  public double headingErrorDegrees() {
    Pose2d truth = simulation.truePose();
    return drive.getPose().getRotation().minus(truth.getRotation()).getDegrees();
  }

  @Override
  public void close() {
    drive.close();
  }
}
