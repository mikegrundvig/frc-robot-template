package frc.robot.sim;

import frc.robot.RobotConfig;
import frc.robot.RobotSim;
import frc.robot.lib.field.Terrain;
import frc.robot.lib.localization.EstimatorConfig;
import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.PoseEstimator;
import frc.robot.lib.localization.PoseReset;
import frc.robot.lib.localization.SwerveKinematicsModel;
import frc.robot.lib.localization.VisionDecision;
import frc.robot.lib.localization.VisionMeasurement;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveHardware;
import frc.robot.subsystems.localization.Localization;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.RobotController;

/**
 * Drives the drivetrain in simulation the way the robot runs it: the simulated Driver Station, the
 * command scheduler, and the joystick drive command, on simulated modules and IMUs, with
 * localization taking what they read, and the simulator's physics stepped before each loop reads
 * the sensors.
 */
public final class DriveSimHarness implements AutoCloseable {
  public final Scheduler scheduler = Scheduler.createIndependentScheduler();
  public final DriveSimulation simulation;
  public final PoseEstimator estimator;
  public final Localization localization;
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

  /** Localization for a robot, on flat ground, starting with its estimator. */
  public static Localization localizationFor(RobotConfig robot, PoseEstimator estimator) {
    return localizationFor(robot, estimator, Terrain.FLAT);
  }

  /** Localization for a robot on this terrain, starting with its estimator. */
  public static Localization localizationFor(
      RobotConfig robot, PoseEstimator estimator, Terrain terrain) {
    Translation2d[] wheels = robot.drivetrain().moduleTranslations(robot.body());
    double reach = 0;
    for (Translation2d wheel : wheels) {
      reach = Math.max(reach, wheel.getNorm());
    }
    return new Localization(
        estimator, new SwerveKinematicsModel(wheels), terrain, reach, robot.imus());
  }

  /** Wheels and sensors exactly as configured: odometry is the truth. */
  public DriveSimHarness(RobotConfig robot) {
    this(robot, SensorErrors.NONE);
  }

  public DriveSimHarness(RobotConfig robot, SensorErrors errors) {
    this(robot, errors, EstimatorConfig.DEFAULTS);
  }

  public DriveSimHarness(RobotConfig robot, SensorErrors errors, EstimatorConfig config) {
    this(robot, errors, config, Terrain.FLAT);
  }

  /** On this terrain: the simulator rides over it, and localization knows it. */
  public DriveSimHarness(
      RobotConfig robot, SensorErrors errors, EstimatorConfig config, Terrain terrain) {
    estimator = estimatorAt(Pose2d.ZERO, config);
    localization = localizationFor(robot, estimator, terrain);
    simulation = new DriveSimulation(robot.body(), robot.drivetrain(), errors, robot.imus());
    simulation.setTerrain(terrain);
    drive =
        new Drive(
            "Drive",
            DriveHardware.builder()
                .modules(List.of(simulation.modules()))
                .gyro(simulation.gyro())
                .backupImu(simulation.backupGyro())
                .odometryLock(new ReentrantLock())
                .build(),
            robot.body(),
            robot::drivetrain,
            scheduler,
            localization::pose);
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
   * One robot loop: the physics moves, the drivetrain reads its sensors, localization takes them,
   * the camera's results arrive, the estimate takes those, then commands run.
   */
  public void loop() {
    RobotSim.stepClock();
    long now = RobotController.getTime();
    simulation.step(RobotLoop.PERIOD_SECONDS, now);
    boolean enabled = RobotState.isEnabled();
    drive.periodic(enabled);
    localization.periodic(drive.readings(), enabled);
    FakeCamera watching = camera;
    if (watching != null) {
      for (VisionMeasurement measurement : watching.poll(now)) {
        localization.forCameras().addVision(measurement);
      }
    }
    decisions.addAll(localization.update());
    scheduler.run();
    Runnable action = eachLoop;
    if (action != null) {
      action.run();
    }
  }

  /** Hold the sticks (as {@link Drive#driveWithSticks} takes them) for {@code seconds}, enabled. */
  public void drive(double seconds, double x, double y, double omega) {
    RobotSim.setEnabled(true);
    Command driving = drive.joystickDrive(() -> x, () -> y, () -> omega);
    scheduler.schedule(driving);
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      loop();
    }
    scheduler.cancel(driving);
  }

  /** Run a command for {@code seconds}, enabled, the way the robot loop runs it. */
  public void run(Command command, double seconds) {
    RobotSim.setEnabled(true);
    scheduler.schedule(command);
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      loop();
    }
  }

  /** Sit disabled for {@code seconds}. */
  public void idle(double seconds) {
    RobotSim.setEnabled(false);
    for (int i = 0; i < RobotLoop.loops(seconds); i++) {
      loop();
    }
  }

  /** How far the estimate is from where the robot really is, in meters. */
  public double positionError() {
    return localization.pose().getTranslation().getDistance(simulation.truePose().getTranslation());
  }

  /** How far the estimate's heading is turned from the truth, in degrees. */
  public double headingErrorDegrees() {
    Pose2d truth = simulation.truePose();
    return localization.pose().getRotation().minus(truth.getRotation()).getDegrees();
  }

  @Override
  public void close() {
    drive.close();
    localization.close();
  }
}
