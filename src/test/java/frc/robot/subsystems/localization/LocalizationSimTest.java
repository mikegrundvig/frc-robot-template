package frc.robot.subsystems.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.SimulationTest;
import frc.robot.lib.localization.DriveConditions;
import frc.robot.lib.localization.EstimatorConfig;
import frc.robot.lib.localization.EstimatorConfigBuilder;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.localization.TerrainCrossing;
import frc.robot.lib.localization.VisionDecision;
import frc.robot.lib.localization.VisionMeasurement;
import frc.robot.lib.localization.YawSource;
import frc.robot.sim.DriveSimHarness;
import frc.robot.sim.FakeCamera;
import frc.robot.sim.SensorErrors;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.system.RobotController;

/**
 * The pose estimator on the simulated robot, with a season's wear on its wheels and gyro, and a
 * camera that sees the robot's true pose late and noisily. Does the estimate hold to the truth
 * where odometry alone drifts, and does it survive a shove, a slipping wheel, and a misread tag?
 */
@SimulationTest
@ExtendWith(RobotSim.class)
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

  @AfterEach
  void tearDown() {
    if (harness != null) {
      harness.close();
    }
  }

  private DriveSimHarness robot(SensorErrors errors) {
    return robot(errors, EstimatorConfig.DEFAULTS);
  }

  private DriveSimHarness robot(SensorErrors errors, EstimatorConfig config) {
    if (harness != null) {
      harness.close();
    }
    harness = new DriveSimHarness(Robots.SIM, errors, config);
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

  /**
   * 15 s of driving: odometry alone drifts past 15 cm; with the camera, within the design's 8 cm
   * while moving. At the ends of the legs that turn while driving, the heavy robot is still
   * changing course, its wheels disagreeing with each other as they speed up and slow down to their
   * new speeds, so the estimate doubts them and leans on the camera's noisy fixes: 5.9 cm off at
   * worst, about the 6.2 cm it claims.
   */
  @Test
  void odometryAloneDriftsAndTheCameraHoldsTheEstimateToTheTruth() {
    double blind = driveTheRoute(robot(SensorErrors.TYPICAL));

    DriveSimHarness seeing = robot(SensorErrors.TYPICAL);
    seeing.watchWith(camera(seeing));
    double withCamera = driveTheRoute(seeing);

    assertThat(blind).as("odometry alone, worst").isGreaterThan(0.15);
    assertThat(withCamera).as("with the camera, worst").isLessThan(0.08);
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

  /** Knocked 60 cm sideways hard enough to feel: the camera is believed at once, no recovery. */
  @Test
  void aHitIsFeltSoTheCameraIsBelievedAtOnce() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    robot.watchWith(camera(robot));
    robot.drive(1.0, 0.3, 0, 0);

    robot.simulation.hit(new Transform2d(0, 0.6, Rotation2d.ZERO));
    List<LocalizationEstimate.Mode> modes = new ArrayList<>();
    for (int i = 0; i < 8; i++) {
      robot.drive(0.02, 0.3, 0, 0);
      modes.add(robot.estimator.estimate().mode());
    }

    assertThat(modes).doesNotContain(LocalizationEstimate.Mode.LOST);
    assertThat(robot.estimator.estimate().health().recoveries()).isZero();
    assertThat(robot.positionError()).as("0.16 s after the hit").isLessThan(0.05);
  }

  /** Half a second on a bump, front wheels in the air: the tilt says their travel is suspect. */
  private static double crossABump(DriveSimHarness robot) {
    robot.drive(1.0, 0.5, 0, 0); // 1.1 m/s
    robot.simulation.tilt(Math.toRadians(-20), 0); // nose up
    robot.simulation.grip(0.4);
    robot.drive(0.5, 0.5, 0, 0);
    robot.simulation.tilt(0, 0);
    robot.simulation.grip(1);
    robot.drive(0.5, 0.5, 0, 0);
    return robot.positionError();
  }

  @Test
  void onABumpTheTiltDiscountsWheelsThatLostTheFloor() {
    EstimatorConfig ignoringTilt =
        EstimatorConfigBuilder.builder(EstimatorConfig.DEFAULTS)
            .tiltStartDegrees(80)
            .tiltZeroDegrees(89)
            .build();
    double trusting = crossABump(robot(SensorErrors.NONE, ignoringTilt));
    DriveSimHarness robot = robot(SensorErrors.NONE);
    double discounting = crossABump(robot);

    assertThat(trusting).as("trusting every wheel turn").isGreaterThan(0.2);
    assertThat(discounting).as("discounting by the tilt").isLessThan(trusting / 3);
    assertThat(robot.estimator.estimate().covariance().xStdDev())
        .as("and it knows it may be off")
        .isGreaterThan(discounting);
  }

  @Test
  void theDrivetrainReadsTiltAndJoltsFromItsSensors() {
    DriveSimHarness robot = robot(SensorErrors.NONE);
    robot.idle(0.1);
    boolean disabled = robot.localization.conditions().disabled();

    robot.drive(0.04, 1.0, 0, 0); // full stick from a standstill
    double starting = robot.localization.conditions().slipMetersPerSecond();
    robot.simulation.tilt(0.3, 0.1);
    robot.simulation.hit(new Transform2d(0.1, 0, Rotation2d.ZERO));
    robot.drive(0.02, 1.0, 0, 0);
    DriveConditions hit = robot.localization.conditions();
    robot.drive(0.02, 1.0, 0, 0);

    assertThat(disabled).isTrue();
    assertThat(starting).as("motors at their current limit, wheels gripping").isZero();
    assertThat(hit.tiltRadians()).isCloseTo(Math.acos(Math.cos(0.3) * Math.cos(0.1)), within(1e-9));
    assertThat(hit.collision()).isTrue();
    assertThat(hit.disabled()).isFalse();
    assertThat(robot.localization.conditions().collision()).as("a loop later").isFalse();
    assertThat(robot.localization.pitchRadians()).isEqualTo(0.3);
    assertThat(robot.localization.rollRadians()).isEqualTo(0.1);
  }

  /** Over a bump, nose up then nose down: the drivetrain follows the crossing from its IMU. */
  @Test
  void theDrivetrainFollowsTheRobotOverABump() {
    DriveSimHarness robot = robot(SensorErrors.NONE);
    robot.drive(0.6, 0.5, 0, 0); // 1.1 m/s ahead
    List<TerrainCrossing.Phase> phases =
        new ArrayList<>(List.of(robot.localization.terrainPhase()));
    robot.eachLoop(
        () -> {
          TerrainCrossing.Phase now = robot.localization.terrainPhase();
          if (phases.isEmpty() || phases.get(phases.size() - 1) != now) {
            phases.add(now);
          }
        });

    robot.simulation.tilt(Math.toRadians(-15), 0); // nose up
    robot.drive(0.3, 0.5, 0, 0);
    robot.simulation.tilt(Math.toRadians(15), 0); // over the crest, nose down
    robot.drive(0.3, 0.5, 0, 0);
    robot.simulation.tilt(0, 0);
    robot.drive(0.3, 0.5, 0, 0);

    assertThat(phases)
        .containsExactly(
            TerrainCrossing.Phase.LEVEL,
            TerrainCrossing.Phase.CLIMBING,
            TerrainCrossing.Phase.DESCENDING,
            TerrainCrossing.Phase.LEVEL);
    assertThat(robot.localization.terrainCrossings()).isEqualTo(1);
  }

  /** Drives the route with the gyro unplugged a second in; returns the heading error at the end. */
  private static double loseTheGyro(DriveSimHarness robot, boolean backupToo) {
    robot.drive(1.0, 0.5, 0, 0.3);
    robot.simulation.disconnectGyro(true);
    robot.simulation.disconnectBackupGyro(backupToo);
    driveTheRoute(robot);
    return Math.abs(robot.headingErrorDegrees());
  }

  /** The gyro unplugged mid-match: the backup IMU keeps the heading, where the wheels drift. */
  @Test
  void whenTheGyroIsLostTheBackupImuKeepsTheHeading() {
    double wheels = loseTheGyro(robot(SensorErrors.TYPICAL), true);
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    double backup = loseTheGyro(robot, false);

    assertThat(backup).as("heading error on the backup IMU, degrees").isLessThan(1.5);
    assertThat(wheels).as("on the wheels alone").isGreaterThan(3 * backup);
    assertThat(robot.localization.yawSource()).isEqualTo(YawSource.BACKUP);
    assertThat(robot.localization.isUsingBackupAlertActive()).isTrue();
    assertThat(robot.localization.isGyroDisconnectedAlertActive()).isFalse();
    assertThat(robot.estimator.estimate().mode()).isNotEqualTo(LocalizationEstimate.Mode.IMU_FAULT);
  }

  /** Switching between IMUs never shows up as a turn, though their zeros differ. */
  @Test
  void switchingImusDoesNotTurnTheEstimate() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    robot.drive(1.0, 0.3, 0, 0.5);
    double before = robot.headingErrorDegrees();

    robot.simulation.disconnectGyro(true);
    robot.drive(0.1, 0.3, 0, 0.5);
    double onBackup = robot.headingErrorDegrees();
    robot.simulation.disconnectGyro(false);
    robot.drive(0.1, 0.3, 0, 0.5);

    assertThat(onBackup).isCloseTo(before, within(0.2));
    assertThat(robot.headingErrorDegrees()).isCloseTo(before, within(0.2));
    assertThat(robot.localization.yawSource()).isEqualTo(YawSource.GYRO);
    assertThat(robot.localization.isUsingBackupAlertActive()).isFalse();
  }

  /** A gyro that freezes still reports: only the backup IMU's disagreement catches it. */
  @Test
  void aFrozenGyroIsCaughtByTheBackupImu() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    robot.drive(1.0, 0, 0, 0.4);
    boolean beforeFreezing = robot.localization.isImuDisagreementAlertActive();

    robot.simulation.freezeGyro(true);
    robot.drive(0.3, 0, 0, 0.4);

    assertThat(beforeFreezing).isFalse();
    assertThat(robot.localization.isImuDisagreementAlertActive()).isTrue();
  }

  @Test
  void drivingHardIsNeverMistakenForAHit() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    boolean[] felt = {false};
    robot.eachLoop(() -> felt[0] |= robot.localization.conditions().collision());

    driveTheRoute(robot);

    assertThat(felt[0]).isFalse();
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
   * Full stick from a standstill: the drive motors sit at their current limit for most of a second
   * while the robot speeds up, its wheels gripping. That's no slip, so the estimate stays as sure
   * as odometry over that distance is.
   */
  @Test
  void aHardLaunchAtTheCurrentLimitIsNoSlip() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    robot.idle(0.2);
    Translation2d start = robot.simulation.truePose().getTranslation();
    double[] most = new double[2]; // the most slip any loop saw, and the most suspect travel
    boolean[] hit = {false};
    robot.eachLoop(
        () -> {
          most[0] = Math.max(most[0], robot.localization.conditions().slipMetersPerSecond());
          most[1] = Math.max(most[1], robot.estimator.estimate().health().suspectTravelMeters());
          hit[0] |= robot.localization.conditions().collision();
        });

    robot.drive(0.5, 1.0, 0, 0);
    double speed = robot.drive.getRobotRelativeVelocities().vx;
    robot.drive(1.0, 1.0, 0, 0);

    double traveled = robot.simulation.truePose().getTranslation().getDistance(start);
    double odometry = EstimatorConfig.DEFAULTS.translationStdDevPerMeter() * Math.sqrt(traveled);
    assertThat(speed).as("speeding up at its current limit, about 4.4 m/s²").isBetween(1.9, 2.3);
    assertThat(most[0]).as("no slip").isZero();
    assertThat(most[1]).as("nothing suspect").isLessThan(0.005);
    assertThat(hit[0]).as("no collision").isFalse();
    assertThat(robot.estimator.estimate().covariance().largestPositionStdDev())
        .as("as sure as odometry over %.1f m", traveled)
        .isBetween(0.9 * odometry, 1.2 * odometry);
  }

  /** The uncertainty, the error, and the most slip seen, at the end of a run. */
  private record Outcome(double stdDev, double error, double mostSlip) {}

  /** Runs a script on a fresh robot, and says how it ended. */
  private Outcome outcome(SensorErrors errors, Consumer<DriveSimHarness> script) {
    DriveSimHarness robot = robot(errors);
    double[] most = {0};
    robot.eachLoop(
        () -> most[0] = Math.max(most[0], robot.localization.conditions().slipMetersPerSecond()));
    script.accept(robot);
    return new Outcome(
        robot.estimator.estimate().covariance().largestPositionStdDev(),
        robot.positionError(),
        most[0]);
  }

  /** A second at half stick, then a second with the front left wheel spinning past the floor. */
  private Outcome spinningAWheel(double metersPerSecond) {
    return outcome(
        SensorErrors.NONE,
        robot -> {
          robot.drive(1.0, 0.5, 0, 0);
          robot.simulation.spinAt(0, metersPerSecond);
          robot.drive(1.0, 0.5, 0, 0);
          robot.simulation.spinAt(0, 0);
        });
  }

  /**
   * One wheel spinning faster than the floor moves it: the wheels disagree with each other and with
   * the gyro, and the estimate doubts itself by about as far as the wheel put it off.
   */
  @Test
  void aWheelSpinningPastTheFloorIsDoubtedAsFarAsItPutTheEstimateOff() {
    Outcome gripping = spinningAWheel(0);
    Outcome slow = spinningAWheel(0.5);
    Outcome fast = spinningAWheel(1.0);

    assertThat(gripping.mostSlip()).isZero();
    assertThat(slow.mostSlip()).as("the gyro sees it").isPositive();
    assertThat(slow.stdDev()).isGreaterThan(3 * gripping.stdDev());
    assertThat(fast.stdDev()).as("the doubt grows with it").isGreaterThan(1.6 * slow.stdDev());
    for (Outcome spun : List.of(slow, fast)) {
      assertThat(spun.error()).as("off by").isBetween(0.5 * spun.stdDev(), 1.5 * spun.stdDev());
    }
  }

  /** A launch at full stick with only this share of the wheels' rolling moving the robot. */
  private Outcome launchingWithGrip(double grip) {
    return outcome(
        SensorErrors.TYPICAL,
        robot -> {
          robot.idle(0.2);
          robot.simulation.grip(grip);
          robot.drive(1.5, 1.0, 0, 0);
        });
  }

  /**
   * Every wheel spinning in a launch: they agree with each other, but the accelerometer feels the
   * robot speed up slower than they do. The doubt grows the less they grip. It covers only what the
   * accelerometer felt: the robot falling ever further behind wheels at a steady speed is the
   * cameras' to find.
   */
  @Test
  void wheelsSpinningInALaunchAreDoubtedTheMoreTheLessTheyGrip() {
    Outcome gripping = launchingWithGrip(1);
    Outcome threeQuarters = launchingWithGrip(0.75);
    Outcome half = launchingWithGrip(0.5);

    assertThat(gripping.mostSlip()).isZero();
    assertThat(threeQuarters.mostSlip()).isPositive();
    assertThat(threeQuarters.stdDev()).isGreaterThan(2 * gripping.stdDev());
    assertThat(half.stdDev()).isGreaterThan(1.5 * threeQuarters.stdDev());
    assertThat(half.error()).as("the slide").isGreaterThan(threeQuarters.error());
  }

  /** Standing, then pushed sideways at this speed for a second, its wheels skidding. */
  private Outcome pushedSideways(double metersPerSecond) {
    return outcome(
        SensorErrors.TYPICAL,
        robot -> {
          robot.drive(0.5, 0, 0, 0);
          robot.simulation.push(new Translation2d(0, metersPerSecond));
          robot.drive(1.0, 0, 0, 0);
          robot.simulation.push(Translation2d.ZERO);
          robot.drive(0.5, 0, 0, 0);
        });
  }

  /**
   * Pushed sideways by another robot, its wheels never turning: only the accelerometer feels it, as
   * the push gets going. The doubt grows with the push. The steady slide after that fades from what
   * the speed comparison remembers, and the stop cancels against the faded start, so neither is
   * seen: how far the robot slid, the cameras find.
   */
  @Test
  void aPushSidewaysIsDoubtedTheMoreTheFasterItSlides() {
    Outcome standing = pushedSideways(0);
    Outcome pushed = pushedSideways(1.0);
    Outcome shoved = pushedSideways(2.0);

    assertThat(standing.mostSlip()).isZero();
    assertThat(pushed.mostSlip()).isPositive();
    assertThat(pushed.stdDev()).isGreaterThan(5 * standing.stdDev());
    assertThat(shoved.stdDev()).isGreaterThan(2 * pushed.stdDev());
    assertThat(shoved.error()).as("slid").isGreaterThan(1.5);
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
            robot.localization.forCameras().addVision(seen);
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
