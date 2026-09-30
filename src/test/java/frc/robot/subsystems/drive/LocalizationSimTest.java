package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.Robots;
import frc.robot.SimTestSupport;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.localization.VisionDecision;
import frc.robot.lib.localization.VisionMeasurement;
import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.system.RobotController;

/**
 * The pose estimator on the simulated robot, with a season's wear on its wheels and gyro, and a
 * camera that sees the robot's true pose late and noisily. Does the estimate hold to the truth
 * where odometry alone drifts, and does it survive a shove, a slipping wheel, and a misread tag?
 */
class LocalizationSimTest {
  /** A match-like route: sprints, strafes, turns while driving, and a stop. */
  private static final double[][] ROUTE = {
    {3, 1, 0, 0},
    {2, 0, 1, 0.5},
    {3, -1, 0, 0},
    {2, 0.5, -0.5, -0.8},
    {3, -0.7, 0.4, 0.3},
    {2, 0, 0, 0}
  };

  private @Nullable DriveSimHarness harness;

  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
  }

  @BeforeEach
  void setUp() {
    SimTestSupport.pauseTimeAndDisable();
  }

  @AfterEach
  void tearDown() {
    if (harness != null) {
      harness.close();
    }
    SimTestSupport.resumeTime();
  }

  private DriveSimHarness robot(SensorErrors errors) {
    if (harness != null) {
      harness.close();
    }
    SimTestSupport.pauseTimeAndDisable();
    harness = new DriveSimHarness(Robots.SIM, errors);
    return harness;
  }

  /** A good camera: 3 cm of noise, results 40 ms late, 30 frames a second. */
  private static FakeCamera camera(DriveSimHarness robot) {
    return new FakeCamera(robot.simulation, 0.03, 0.04, 30, 3);
  }

  /** Drives the route and returns the worst position error seen at the end of each leg. */
  private static double driveTheRoute(DriveSimHarness robot) {
    double worst = 0;
    for (double[] leg : ROUTE) {
      robot.drive(leg[0], leg[1], leg[2], leg[3]);
      worst = Math.max(worst, robot.positionError());
    }
    return worst;
  }

  /** 15 s of driving: odometry alone ends 12 cm off; with the camera, within a few centimeters. */
  @Test
  void odometryAloneDriftsAndTheCameraHoldsTheEstimateToTheTruth() {
    double blind = driveTheRoute(robot(SensorErrors.TYPICAL));

    DriveSimHarness seeing = robot(SensorErrors.TYPICAL);
    seeing.watchWith(camera(seeing));
    double withCamera = driveTheRoute(seeing);

    assertThat(blind).as("odometry alone, worst").isGreaterThan(0.15);
    assertThat(withCamera).as("with the camera, worst").isLessThan(0.06);
    assertThat(seeing.positionError()).as("with the camera, at the end").isLessThan(0.02);
    LocalizationEstimate estimate = seeing.estimator.estimate();
    assertThat(estimate.mode()).isEqualTo(LocalizationEstimate.Mode.TRACKING);
    assertThat(estimate.health().accepted()).isGreaterThan(300);
  }

  /** Knocked 60 cm sideways: the camera disagrees, then keeps disagreeing, and wins. */
  @Test
  void aShoveIsCaughtAndTheEstimateRecovers() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    robot.watchWith(camera(robot));
    robot.drive(1.0, 0.3, 0, 0);

    robot.simulation.shove(new Transform2d(0, 0.6, Rotation2d.ZERO));
    List<LocalizationEstimate.Mode> modes = new ArrayList<>();
    for (int i = 0; i < 15; i++) {
      robot.drive(0.02, 0.3, 0, 0);
      modes.add(robot.estimator.estimate().mode());
    }

    assertThat(modes).as("it knew it was lost").contains(LocalizationEstimate.Mode.LOST);
    assertThat(robot.estimator.estimate().health().recoveries()).isEqualTo(1);
    assertThat(robot.positionError()).as("0.3 s after the shove").isLessThan(0.08);
    robot.drive(0.2, 0.3, 0, 0);
    assertThat(robot.positionError()).as("0.5 s after").isLessThan(0.04);
  }

  /**
   * A wheel spins out: the wheels disagree, so the estimate trusts them less, and the camera wins.
   */
  @Test
  void aSlippingWheelWidensTheUncertaintyAndTheCameraCorrectsIt() {
    DriveSimHarness robot = robot(SensorErrors.NONE);
    robot.watchWith(camera(robot));
    robot.idle(1.0);

    robot.simulation.spin(0, 3.0); // 15 cm of wheel travel the floor never saw
    robot.idle(0.02);
    double falseMove = robot.positionError();
    double uncertainty = robot.estimator.estimate().covariance().xStdDev();
    robot.idle(0.5);

    assertThat(falseMove).as("odometry was fooled").isGreaterThan(0.02);
    assertThat(uncertainty).as("but knew to doubt itself").isGreaterThan(falseMove);
    assertThat(robot.positionError()).as("half a second later").isLessThan(0.02);
  }

  /**
   * One misread tag, 1.5 m off and claiming 3 cm of certainty. WPILib's estimator has no gate (its
   * documentation asks teams to filter measurements themselves), so it jumps; this one rejects it.
   */
  @Test
  void aMisreadTagIsGatedWhereWpilibsEstimatorJumps() {
    DriveSimHarness robot = robot(SensorErrors.NONE);
    FakeCamera camera = camera(robot);
    SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(Robots.SIM.drivetrain().moduleTranslations(Robots.SIM.body()));
    SwerveDrivePoseEstimator wpilib =
        new SwerveDrivePoseEstimator(
            kinematics, robot.drive.gyroYaw(), robot.drive.modulePositions(), Pose2d.ZERO);
    double[] worst = new double[2];
    robot.eachLoop(
        () -> {
          long now = RobotController.getTime();
          wpilib.updateWithTime(now / 1e9, robot.drive.gyroYaw(), robot.drive.modulePositions());
          for (VisionMeasurement seen : camera.poll(now)) {
            robot.estimator.addVision(seen);
            wpilib.addVisionMeasurement(
                seen.pose(), seen.timestampNanos() / 1e9, VecBuilder.fill(0.03, 0.03, 1e6));
          }
          Pose2d truth = robot.simulation.truePose();
          worst[0] = Math.max(worst[0], robot.positionError());
          worst[1] =
              Math.max(
                  worst[1],
                  wpilib
                      .getEstimatedPosition()
                      .getTranslation()
                      .getDistance(truth.getTranslation()));
        });
    robot.idle(1.0);
    worst[0] = 0;
    worst[1] = 0;

    camera.lieOnce(new Transform2d(1.5, 0, Rotation2d.ZERO));
    robot.idle(0.3);

    assertThat(worst[0]).as("ours, worst after the misread").isLessThan(0.05);
    assertThat(worst[1]).as("WPILib's, worst after the misread").isGreaterThan(0.5);
    assertThat(robot.decisions)
        .filteredOn(decision -> decision.innovationMeters() > 1)
        .extracting(VisionDecision::accepted)
        .containsExactly(false);
  }
}
