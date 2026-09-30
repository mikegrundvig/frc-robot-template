package frc.robot.lib.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;

/**
 * The estimator, fed by hand: odometry steps at 250 Hz, and camera measurements with the capture
 * times and uncertainties a camera would give.
 */
class PoseEstimatorTest {
  private static final long STEP = 4_000_000L; // 250 Hz
  private static final long T0 = 1_000_000_000L;
  private static final Pose2d START = new Pose2d(1, 1, Rotation2d.ZERO);
  private static final PoseCovariance PLACED = PoseCovariance.fromStdDevs(0.05, 0.05, 0.02);

  /** A robot driven by hand: a clock, a gyro, and wheels that report steady motion. */
  private static final class Robot {
    final PoseEstimator estimator;
    long time = T0;
    double gyroYaw = 0.7; // the gyro started somewhere arbitrary

    Robot(EstimatorConfig config, PoseReset start) {
      estimator = new PoseEstimator(config, point -> field(point), start);
      estimator.addOdometry(time, new BodyMotion(0, 0, 0, 0), gyroYaw, true);
    }

    Robot() {
      this(EstimatorConfig.DEFAULTS, new PoseReset(0, START, PLACED, "start"));
      estimator.reset(new PoseReset(T0, START, PLACED, "placed"));
    }

    /** From now on, the robot's other sensors say this about its wheels. */
    Robot under(DriveConditions conditions) {
      estimator.setConditions(conditions);
      return this;
    }

    /** Drives at robot-relative velocities for some steps. */
    Robot drive(int steps, double vx, double vy, double omega) {
      return drive(steps, vx, vy, omega, 0, true);
    }

    Robot drive(int steps, double vx, double vy, double omega, double residual, boolean gyro) {
      double dt = STEP / 1e9;
      for (int i = 0; i < steps; i++) {
        time += STEP;
        gyroYaw += omega * dt;
        estimator.addOdometry(
            time, new BodyMotion(vx * dt, vy * dt, omega * dt, residual), gyroYaw, gyro);
      }
      return this;
    }

    List<VisionDecision> see(Pose2d pose, long when, PoseCovariance noise) {
      estimator.addVision(
          new VisionMeasurement(
              when,
              pose,
              noise,
              "front",
              List.of(26),
              VisionMeasurement.Solver.HEADING_CONSTRAINED));
      return estimator.update();
    }

    /** A camera measurement with heading, from several tags, taken now. */
    VisionDecision seeWithHeading(Pose2d pose) {
      estimator.addVision(
          new VisionMeasurement(
              time,
              pose,
              PoseCovariance.fromStdDevs(0.05, 0.05, Math.toRadians(2)),
              "front",
              List.of(25, 26),
              VisionMeasurement.Solver.MULTI_TAG));
      return estimator.update().get(0);
    }

    Pose2d pose() {
      return estimator.pose();
    }

    LocalizationEstimate estimate() {
      return estimator.estimate();
    }

    /** A landmark on the line x = lineX, crossed at some time: it says where x was, not y. */
    List<LandmarkDecision> cross(double lineX, long when, double stdDev) {
      estimator.addLandmark(
          new LandmarkEvent(when, "bump", new Translation2d(lineX, 4), Rotation2d.ZERO, stdDev));
      estimator.update();
      return estimator.landmarkDecisions();
    }
  }

  private static DriveConditions tilted(double degrees) {
    return new DriveConditions(Math.toRadians(degrees), 0, false, false, false);
  }

  private static final DriveConditions HIT = new DriveConditions(0, 0, true, false, false);
  private static final DriveConditions AT_THE_GRIP = new DriveConditions(0, 0, false, true, false);
  private static final DriveConditions DISABLED = new DriveConditions(0, 0, false, false, true);

  private static boolean field(Translation2d point) {
    return point.getX() >= 0 && point.getX() <= 16 && point.getY() >= 0 && point.getY() <= 8;
  }

  private static PoseCovariance position(double stdDev) {
    return PoseCovariance.fromStdDevs(stdDev, stdDev, Double.POSITIVE_INFINITY);
  }

  private static void assertSame(Pose2d actual, Pose2d expected, double tolerance) {
    assertThat(actual.getTranslation().getDistance(expected.getTranslation()))
        .as(actual + " vs " + expected)
        .isLessThan(tolerance);
    assertThat(actual.getRotation().minus(expected.getRotation()).getRadians())
        .isCloseTo(0.0, within(tolerance));
  }

  @Test
  void odometryFollowsTheWheelsAlongTheGyrosArc() {
    Robot robot = new Robot().drive(500, 1.0, 0.2, 0.5);

    Pose2d expected = START;
    double dt = STEP / 1e9;
    for (int i = 0; i < 500; i++) {
      expected = expected.plus(new Twist2d(1.0 * dt, 0.2 * dt, 0.5 * dt).exp());
    }
    assertSame(robot.pose(), expected, 1e-9);
  }

  @Test
  void uncertaintyGrowsWithDistanceAndHeadingUncertaintySwingsItSideways() {
    Robot robot = new Robot().drive(1000, 1.0, 0, 0); // 4 m straight ahead, along x

    PoseCovariance covariance = robot.estimate().covariance();
    // Placed to 5 cm, plus 3 cm per square root of meter: sqrt(0.05^2 + 0.03^2 * 4).
    assertThat(covariance.xStdDev()).isCloseTo(Math.sqrt(0.0025 + 0.0036), within(0.001));
    assertThat(covariance.yStdDev()).as("a 0.02 rad heading doubt over 4 m").isGreaterThan(0.09);
  }

  @Test
  void theGyroDriftsSoHeadingGrowsUnsureOverTime() {
    Robot robot = new Robot().drive(15_000, 0, 0, 0); // a minute, sitting still

    double headingStdDev = robot.estimate().covariance().headingStdDev();
    // 0.02 rad when placed, plus 0.5 degrees a minute of drift.
    assertThat(headingStdDev).isCloseTo(Math.hypot(0.02, Math.toRadians(0.5)), within(1e-4));
  }

  @Test
  void aMeasurementPullsTheEstimateByTheKalmanGain() {
    Robot robot = new Robot(EstimatorConfig.DEFAULTS, new PoseReset(0, START, position(0.1), "x"));
    robot.estimator.reset(
        new PoseReset(T0, START, PoseCovariance.fromStdDevs(0.1, 0.1, 0.02), "x"));
    robot.drive(1, 0, 0, 0);

    List<VisionDecision> decisions =
        robot.see(new Pose2d(1.2, 1, Rotation2d.ZERO), robot.time, position(0.1));

    assertThat(decisions).singleElement().extracting(VisionDecision::accepted).isEqualTo(true);
    assertThat(robot.pose().getX()).as("equal trust: halfway").isCloseTo(1.1, within(1e-6));
    assertThat(robot.estimate().covariance().xStdDev()).isCloseTo(Math.sqrt(0.005), within(1e-5));
    assertThat(robot.pose().getRotation().getRadians())
        .as("a position measurement leaves heading alone")
        .isCloseTo(0.0, within(1e-9));
  }

  /** A camera's result arrives tens of milliseconds after its image: it counts at the image. */
  @Test
  void aLateMeasurementLandsWhereAnOnTimeOneWould() {
    Robot onTime = new Robot();
    Robot late = new Robot();
    long captured = T0 + 25 * STEP + STEP / 3; // between two odometry steps
    Pose2d seen = new Pose2d(1.2, 1.05, Rotation2d.ZERO);

    onTime.drive(26, 2.0, 0, 0.8);
    onTime.see(seen, captured, position(0.05));
    onTime.drive(125, 2.0, 0, 0.8);
    late.drive(151, 2.0, 0, 0.8);
    late.see(seen, captured, position(0.05));

    assertSame(late.pose(), onTime.pose(), 1e-9);
    assertThat(late.estimate().covariance().xx())
        .isCloseTo(onTime.estimate().covariance().xx(), within(1e-12));
  }

  @Test
  void measurementsGiveTheSameResultInAnyOrderOfArrival() {
    Robot inOrder = new Robot().drive(100, 1.0, 0.5, 0.3);
    Robot reversed = new Robot().drive(100, 1.0, 0.5, 0.3);
    long first = T0 + 40 * STEP + 1_000_000;
    long second = T0 + 70 * STEP + 2_500_000;
    // A few centimeters from where the robot was then: both pass the gate.
    Pose2d one = new Pose2d(1.18, 1.10, Rotation2d.ZERO);
    Pose2d two = new Pose2d(1.29, 1.13, Rotation2d.ZERO);

    inOrder.see(one, first, position(0.05));
    inOrder.see(two, second, position(0.05));
    reversed.see(two, second, position(0.05));
    reversed.see(one, first, position(0.05));

    assertSame(reversed.pose(), inOrder.pose(), 1e-9);
    assertThat(inOrder.estimate().health().accepted()).isEqualTo(2);
  }

  @Test
  void aMeasurementFarOutsideTheUncertaintyIsGatedOut() {
    Robot robot = new Robot().drive(10, 0, 0, 0);

    List<VisionDecision> decisions =
        robot.see(new Pose2d(3, 1, Rotation2d.ZERO), robot.time, position(0.05));

    VisionDecision decision = decisions.get(0);
    assertThat(decision.accepted()).isFalse();
    assertThat(decision.reason()).startsWith("gate: ").endsWith("> 13.8");
    assertThat(decision.innovationMeters()).isCloseTo(2.0, within(1e-9));
    assertSame(robot.pose(), START, 1e-9);
  }

  @Test
  void measurementsTheHistoryCannotPlaceAreRejected() {
    Robot robot = new Robot().drive(500, 0, 0, 0); // 2 s: the history keeps 1.5 s
    long now = robot.time;

    assertThat(robot.see(START, T0 + STEP, position(0.05)).get(0).reason())
        .isEqualTo("too late: older than the history kept");
    assertThat(robot.see(START, now + 100_000_000L, position(0.05)).get(0).reason())
        .isEqualTo("from the future: after the newest odometry");
    assertThat(robot.see(new Pose2d(-1, 1, Rotation2d.ZERO), now, position(0.05)).get(0).reason())
        .isEqualTo("off the field");
    robot.estimator.reset(new PoseReset(now, START, PLACED, "placed again"));
    robot.drive(5, 0, 0, 0);
    assertThat(robot.see(START, now, position(0.05)).get(0).reason())
        .isEqualTo("before the robot was last placed or recovered");
    assertThat(robot.estimate().health().rejected()).isEqualTo(4);
  }

  @Test
  void aMeasurementBeforeOdometryStartsIsRejected() {
    PoseEstimator estimator =
        new PoseEstimator(
            EstimatorConfig.DEFAULTS, point -> true, new PoseReset(0, START, PLACED, "start"));

    estimator.addVision(
        new VisionMeasurement(
            5, START, position(0.05), "front", List.of(1), VisionMeasurement.Solver.MULTI_TAG));

    assertThat(estimator.update().get(0).reason()).isEqualTo("before odometry started");
    assertThat(estimator.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.INITIALIZING);
  }

  /** Knocked a meter sideways, the wheels saw nothing; the cameras all agree it moved. */
  @Test
  void whenRejectedMeasurementsAgreeOdometryWasWrongAndTheEstimateRecovers() {
    Robot robot = new Robot().drive(10, 0, 0, 0);
    Pose2d truth = new Pose2d(1, 2, Rotation2d.ZERO);

    robot.drive(10, 0, 0, 0).see(truth, robot.time, position(0.05));
    robot.drive(10, 0, 0, 0).see(truth, robot.time, position(0.05));
    assertThat(robot.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.LOST);
    robot.drive(10, 0, 0, 0).see(truth, robot.time, position(0.05));

    assertThat(robot.pose().getTranslation().getDistance(truth.getTranslation())).isLessThan(1e-9);
    assertThat(robot.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.TRACKING);
    assertThat(robot.estimate().health().recoveries()).isEqualTo(1);
    assertThat(robot.estimate().covariance().xStdDev()).isCloseTo(0.25, within(1e-9));
  }

  @Test
  void rejectedMeasurementsThatDisagreeWithEachOtherChangeNothing() {
    Robot robot = new Robot().drive(10, 0, 0, 0);

    robot.drive(10, 0, 0, 0).see(new Pose2d(1, 2, Rotation2d.ZERO), robot.time, position(0.05));
    robot.drive(10, 0, 0, 0).see(new Pose2d(3, 1, Rotation2d.ZERO), robot.time, position(0.05));
    robot.drive(10, 0, 0, 0).see(new Pose2d(1, 4, Rotation2d.ZERO), robot.time, position(0.05));

    assertSame(robot.pose(), START, 1e-9);
    assertThat(robot.estimate().health().recoveries()).isZero();
  }

  @Test
  void aMeasurementWithHeadingCorrectsTheGyroOffset() {
    Robot robot = new Robot(EstimatorConfig.DEFAULTS, new PoseReset(0, START, PLACED, "x"));
    robot.estimator.reset(
        new PoseReset(T0, START, PoseCovariance.fromStdDevs(0.05, 0.05, 0.1), "x"));
    robot.drive(1, 0, 0, 0);

    robot.see(
        new Pose2d(1, 1, new Rotation2d(0.1)),
        robot.time,
        PoseCovariance.fromStdDevs(0.05, 0.05, 0.1));

    assertThat(robot.pose().getRotation().getRadians()).isCloseTo(0.05, within(1e-4));
    double before = robot.pose().getRotation().getRadians();
    robot.drive(250, 0, 0, 1.0); // a radian of turning, by the gyro
    assertThat(robot.pose().getRotation().getRadians() - before).isCloseTo(1.0, within(1e-9));
  }

  @Test
  void modesFollowWhatTheEstimateRestsOn() {
    PoseEstimator estimator =
        new PoseEstimator(
            EstimatorConfig.DEFAULTS,
            point -> true,
            new PoseReset(0, START, PoseCovariance.fromStdDevs(10, 10, 3), "unknown"));
    estimator.addOdometry(T0, new BodyMotion(0, 0, 0, 0), 0, true);
    assertThat(estimator.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.INITIALIZING);

    estimator.reset(new PoseReset(T0, START, PLACED, "placed"));
    assertThat(estimator.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.DEAD_RECKONING);

    estimator.addOdometry(T0 + STEP, new BodyMotion(0, 0, 0, 0), 0, true);
    estimator.addVision(
        new VisionMeasurement(
            T0 + STEP,
            START,
            position(0.05),
            "front",
            List.of(26),
            VisionMeasurement.Solver.MULTI_TAG));
    estimator.update();
    assertThat(estimator.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.TRACKING);
    assertThat(estimator.estimate().health().visionAgeSeconds()).isZero();

    estimator.addOdometry(T0 + 300 * STEP, new BodyMotion(0, 0, 0, 0), 0, true);
    assertThat(estimator.estimate().mode())
        .as("1.2 s without vision")
        .isEqualTo(LocalizationEstimate.Mode.DEAD_RECKONING);

    estimator.addOdometry(T0 + 301 * STEP, new BodyMotion(0, 0, 0, 0), 0, false);
    assertThat(estimator.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.IMU_FAULT);
  }

  @Test
  void withTheGyroLostTheWheelsTurnTheRobotAndHeadingGrowsUnsureFaster() {
    Robot withGyro = new Robot().drive(250, 0, 0, 1.0);
    Robot without = new Robot().drive(250, 0, 0, 1.0, 0, false);

    assertThat(without.pose().getRotation().getRadians()).isCloseTo(1.0, within(1e-9));
    assertThat(without.estimate().covariance().headingStdDev())
        .isGreaterThan(3 * withGyro.estimate().covariance().headingStdDev());
    assertThat(without.estimate().health().gyroConnected()).isFalse();
  }

  /** Back from a brownout, a gyro may read anything: the heading carries on without a jump. */
  @Test
  void aGyroThatComesBackReadingSomethingElseDoesNotJumpTheHeading() {
    Robot robot = new Robot().drive(10, 0, 0, 0, 0, false);
    double heading = robot.pose().getRotation().getRadians();

    robot.gyroYaw += 2.5; // it rebooted
    robot.drive(1, 0, 0, 0);
    robot.drive(10, 0, 0, 0);

    assertThat(robot.pose().getRotation().getRadians()).isCloseTo(heading, within(1e-9));
  }

  @Test
  void slippingWheelsMakeOdometryLessTrusted() {
    Robot gripping = new Robot().drive(250, 2.0, 0, 0);
    Robot slipping = new Robot().drive(250, 2.0, 0, 0, 0.004, true);

    assertThat(slipping.estimate().covariance().xStdDev())
        .isGreaterThan(gripping.estimate().covariance().xStdDev() * 1.2);
    assertThat(slipping.estimate().health().residualMeters()).isEqualTo(0.004);
  }

  @Test
  void aResetPlacesTheRobotAndAResetWithoutHeadingKeepsItsHeading() {
    Robot robot = new Robot().drive(250, 0, 0, 0.5);
    double heading = robot.pose().getRotation().getRadians();

    robot.estimator.reset(
        new PoseReset(robot.time, new Pose2d(4, 3, Rotation2d.ZERO), position(0.1), "x"));

    assertThat(robot.pose().getTranslation()).isEqualTo(new Translation2d(4, 3));
    assertThat(robot.pose().getRotation().getRadians()).isCloseTo(heading, within(1e-12));
  }

  @Test
  void itPredictsWhereTheRobotWillBe() {
    Robot robot = new Robot().drive(100, 2.0, 0, 0);

    LocalizationEstimate estimate = robot.estimate();
    Pose2d ahead = estimate.predict(0.5);

    assertThat(estimate.fieldVelocity().getX()).isCloseTo(2.0, within(1e-9));
    assertThat(ahead.getX() - estimate.pose().getX()).isCloseTo(1.0, within(1e-9));
  }

  @Test
  void itReportsItsHealth() {
    Robot robot = new Robot().drive(10, 0, 0, 0);

    robot.see(new Pose2d(1.02, 1, Rotation2d.ZERO), robot.time, position(0.05));
    robot.see(new Pose2d(5, 1, Rotation2d.ZERO), robot.time, position(0.05));
    robot.drive(50, 0, 0, 0);

    LocalizationEstimate.Health health = robot.estimate().health();
    assertThat(health.accepted()).isEqualTo(1);
    assertThat(health.rejected()).isEqualTo(1);
    assertThat(health.lastInnovationMeters()).isGreaterThan(3.9);
    assertThat(health.visionAgeSeconds()).isCloseTo(0.2, within(1e-9));
  }

  @Test
  void aTiltedRobotsWheelsCountForLessAndPastTheLimitForNothing() {
    Robot halfway = new Robot().under(tilted(15)).drive(250, 1.0, 0, 0); // a meter of wheel travel
    Robot steep = new Robot().under(tilted(30)).drive(250, 1.0, 0, 0);
    Robot level = new Robot().drive(250, 0.5, 0, 0); // half a meter, level

    assertThat(halfway.pose().getX() - START.getX()).isCloseTo(0.5, within(1e-9));
    assertThat(steep.pose().getX()).isCloseTo(START.getX(), within(1e-12));
    // The half meter left out may have been real: the doubt grows by a good share of it.
    double added =
        Math.sqrt(halfway.estimate().covariance().xx() - level.estimate().covariance().xx());
    assertThat(added).isBetween(0.3, 0.5);
    assertThat(halfway.estimate().health().tiltRadians()).isEqualTo(Math.toRadians(15));
  }

  @Test
  void slipAtTheEdgeOfGripAddsUpAsOneErrorNotManySmallOnes() {
    Robot gripping = new Robot().drive(62, 2.0, 0, 0); // half a meter in a quarter second
    Robot slipping = new Robot().under(AT_THE_GRIP).drive(62, 2.0, 0, 0);

    double added =
        Math.sqrt(slipping.estimate().covariance().xx() - gripping.estimate().covariance().xx());
    // A quarter of the half meter is suspect. Squared step by step it would add only 1.6 cm; as one
    // error building in one direction it adds nearly all of it.
    assertThat(added).isBetween(0.09, 0.125);
    assertThat(slipping.estimate().health().suspectTravelMeters()).isGreaterThan(0.09);
  }

  @Test
  void aCollisionAddsItsDoubtAtOnceAndOnceForItsWindow() {
    Robot robot = new Robot().drive(10, 0, 0, 0);
    double before = robot.estimate().covariance().xx();

    robot.under(HIT).drive(1, 0, 0, 0);
    double hit = robot.estimate().covariance().xx();
    robot.under(HIT).drive(5, 0, 0, 0); // the IMU still jolting, a loop later
    boolean duringWindow = robot.estimate().health().collisionActive();
    robot.under(DriveConditions.NORMAL).drive(100, 0, 0, 0);

    assertThat(hit - before).isCloseTo(0.0625, within(1e-9)); // 25 cm
    assertThat(robot.estimate().covariance().xx()).isCloseTo(hit, within(1e-12));
    assertThat(duringWindow).isTrue();
    assertThat(robot.estimate().health().collisionActive()).isFalse();
  }

  @Test
  void aDisabledRobotMayBeCarriedSoItsPositionGrowsUnsure() {
    Robot robot = new Robot().under(DISABLED).drive(15_000, 0, 0, 0); // a minute

    // Placed to 5 cm, plus 2 cm per square root of second.
    assertThat(robot.estimate().covariance().xStdDev())
        .isCloseTo(Math.sqrt(0.0025 + 0.0004 * 60), within(1e-3));
  }

  @Test
  void aLateMeasurementReplaysEachStepUnderItsOwnConditions() {
    Robot inTime = new Robot().drive(50, 1.0, 0, 0);
    Robot late = new Robot().drive(50, 1.0, 0, 0);
    long captured = inTime.time;
    Pose2d seen = new Pose2d(1.22, 1.03, Rotation2d.ZERO);

    inTime.see(seen, captured, position(0.05));
    for (Robot robot : List.of(inTime, late)) {
      robot.under(tilted(12)).drive(40, 1.0, 0.2, 0.3);
      robot.under(HIT).drive(10, 0.5, 0, 0);
      robot.under(AT_THE_GRIP).drive(40, 2.0, -0.5, 0);
      robot.under(DriveConditions.NORMAL).drive(10, 1.0, 0, 0);
    }
    late.see(seen, captured, position(0.05));

    assertSame(late.pose(), inTime.pose(), 1e-9);
    PoseCovariance expected = inTime.estimate().covariance();
    PoseCovariance actual = late.estimate().covariance();
    assertThat(actual.xx()).isCloseTo(expected.xx(), within(1e-12));
    assertThat(actual.yy()).isCloseTo(expected.yy(), within(1e-12));
    assertThat(actual.heading()).isCloseTo(expected.heading(), within(1e-12));
  }

  @Test
  void theHealthSaysHowFarTheCamerasDisagreedLately() {
    Robot robot = new Robot().drive(10, 0, 0, 0);
    double before = robot.estimate().health().recentDisagreementMeters();

    robot.see(new Pose2d(1.05, 1, Rotation2d.ZERO), robot.time, position(0.05));
    double after = robot.estimate().health().recentDisagreementMeters();
    robot.drive(150, 0, 0, 0); // 0.6 s, no camera

    assertThat(before).isNaN();
    assertThat(after).isCloseTo(0.05, within(1e-9));
    assertThat(robot.estimate().health().recentDisagreementMeters()).isNaN();
  }

  @Test
  void aHeadingResetTurnsTheRobotWhereItStands() {
    Robot robot = new Robot().drive(100, 1.0, 0, 0);
    Translation2d here = robot.pose().getTranslation();

    robot.estimator.resetHeading(Rotation2d.fromDegrees(90), Math.toRadians(5));
    Pose2d reset = robot.pose();
    robot.drive(250, 1.0, 0, 0);

    assertThat(reset.getTranslation()).isEqualTo(here);
    assertThat(reset.getRotation().getDegrees()).isCloseTo(90, within(1e-9));
    assertThat(robot.estimate().covariance().headingStdDev())
        .isCloseTo(Math.toRadians(5), within(1e-4));
    assertThat(robot.pose().getY() - here.getY())
        .as("forward is now +y")
        .isCloseTo(1, within(1e-6));
  }

  @Test
  void aHeadingResetBeforeOdometryStartsIsKept() {
    PoseEstimator estimator =
        new PoseEstimator(
            EstimatorConfig.DEFAULTS, point -> true, new PoseReset(0, START, PLACED, "start"));

    estimator.resetHeading(Rotation2d.PI, 0.1);
    estimator.addOdometry(T0, new BodyMotion(0, 0, 0, 0), 0.4, true);

    assertThat(estimator.pose().getRotation().getDegrees()).isCloseTo(180, within(1e-9));
    assertThat(estimator.pose().getTranslation()).isEqualTo(START.getTranslation());
    assertThatThrownBy(() -> estimator.resetHeading(Rotation2d.ZERO, 0))
        .isInstanceOf(IllegalArgumentException.class);
  }

  @Test
  void camerasThatAgreeOnAnotherHeadingTurnTheEstimateToIt() {
    // The robot really faces +y, but was told it faces +x: a driver's reset facing the wrong way.
    Robot robot = new Robot().drive(10, 0, 0, 0);
    Pose2d truth = new Pose2d(1, 1, Rotation2d.fromDegrees(90));

    List<Boolean> accepted = new ArrayList<>();
    for (int i = 0; i < 3; i++) {
      accepted.add(robot.seeWithHeading(truth).accepted());
      robot.drive(8, 0, 0, 0);
    }
    Pose2d recovered = robot.pose();
    boolean afterwards = robot.seeWithHeading(truth).accepted();

    assertThat(accepted).containsExactly(false, false, false);
    assertThat(recovered.getRotation().getDegrees()).isCloseTo(90, within(1e-6));
    assertThat(robot.estimate().health().recoveries()).isEqualTo(1);
    assertThat(afterwards).as("the next one, taken").isTrue();
  }

  @Test
  void headingsEitherSideOfHalfATurnAgree() {
    Robot robot = new Robot().drive(10, 0, 0, 0);

    for (double degrees : new double[] {179, -179, 179}) {
      robot.seeWithHeading(new Pose2d(1, 1, Rotation2d.fromDegrees(degrees)));
      robot.drive(8, 0, 0, 0);
    }

    assertThat(Math.abs(robot.pose().getRotation().getDegrees())).isGreaterThan(179);
  }

  @Test
  void camerasThatDisagreeWithEachOtherDontTurnIt() {
    Robot robot = new Robot().drive(10, 0, 0, 0);

    for (double degrees : new double[] {60, 90, 120}) {
      robot.seeWithHeading(new Pose2d(1, 1, Rotation2d.fromDegrees(degrees)));
      robot.drive(8, 0, 0, 0);
    }

    assertThat(robot.pose().getRotation().getDegrees()).isCloseTo(0, within(1e-6));
    assertThat(robot.estimate().health().recoveries()).isZero();
  }

  @Test
  void aLandmarkMovesTheEstimateAcrossItsLineAndNotAlongIt() {
    Robot robot = new Robot().drive(1, 0, 0, 0);
    PoseCovariance before = robot.estimate().covariance();

    List<LandmarkDecision> decisions = robot.cross(1.2, robot.time, 0.05);

    LandmarkDecision decision = decisions.get(0);
    assertThat(decision.accepted()).isTrue();
    assertThat(decision.innovationMeters()).isCloseTo(0.2, within(1e-9));
    assertThat(robot.pose().getX()).as("equal trust: halfway").isCloseTo(1.1, within(1e-6));
    assertThat(robot.pose().getY()).isCloseTo(1.0, within(1e-9));
    assertThat(robot.estimate().covariance().xx()).isCloseTo(before.xx() / 2, within(1e-9));
    assertThat(robot.estimate().covariance().yy()).isCloseTo(before.yy(), within(1e-12));
  }

  @Test
  void aLandmarkFarFromTheEstimateIsGatedOut() {
    Robot robot = new Robot().drive(1, 0, 0, 0);

    LandmarkDecision decision = robot.cross(2.0, robot.time, 0.05).get(0);

    assertThat(decision.accepted()).isFalse();
    assertThat(decision.reason()).startsWith("gate: ").endsWith("> 10.8");
    assertSame(robot.pose(), START, 1e-9);
  }

  /** A crest is known only once the robot is down the far ramp: it counts when it was crossed. */
  @Test
  void aLateLandmarkLandsWhereAnOnTimeOneWould() {
    Robot onTime = new Robot();
    Robot late = new Robot();
    long crossed = T0 + 25 * STEP + STEP / 3;

    onTime.drive(26, 1.0, 0.2, 0.3);
    onTime.cross(1.15, crossed, 0.05);
    onTime.drive(125, 1.0, 0.2, 0.3);
    late.drive(151, 1.0, 0.2, 0.3);
    late.cross(1.15, crossed, 0.05);

    assertSame(late.pose(), onTime.pose(), 1e-9);
    assertThat(late.estimate().covariance().xx())
        .isCloseTo(onTime.estimate().covariance().xx(), within(1e-12));
  }

  @Test
  void aLandmarkTheHistoryCannotPlaceIsRejected() {
    Robot robot = new Robot().drive(500, 0, 0, 0); // 2 s: the history keeps 1.5 s

    assertThat(robot.cross(1.0, T0 + STEP, 0.05).get(0).reason())
        .isEqualTo("too late: older than the history kept");
  }

  @Test
  void aLandmarkIsNoCamera() {
    Robot robot = new Robot().drive(1, 0, 0, 0);

    robot.cross(1.05, robot.time, 0.05);

    LocalizationEstimate.Health health = robot.estimate().health();
    assertThat(health.accepted()).isZero();
    assertThat(health.visionAgeSeconds()).isInfinite();
    assertThat(robot.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.DEAD_RECKONING);
  }

  @Test
  void onAKnownRampTheWheelsTravelCountsAtTheSlopesCosine() {
    double slope = Math.toRadians(15);
    Robot ramp = new Robot().under(new DriveConditions(slope, slope, false, false, false));
    Robot unknown = new Robot().under(tilted(15));

    ramp.drive(250, 1.0, 0, 0); // a meter along the ramp
    unknown.drive(250, 1.0, 0, 0);

    assertThat(ramp.pose().getX() - START.getX()).isCloseTo(Math.cos(slope), within(1e-9));
    assertThat(unknown.pose().getX() - START.getX()).as("halved").isCloseTo(0.5, within(1e-9));
    assertThat(ramp.estimate().health().suspectTravelMeters()).isZero();
  }

  @Test
  void theGroundExplainsSomeOrAllOfATiltNeverMore() {
    assertThatThrownBy(() -> new DriveConditions(0.1, 0.2, false, false, false))
        .hasMessageContaining("never more");
    assertThat(new DriveConditions(0.3, 0.2, false, false, false).unexplainedTiltRadians())
        .isCloseTo(0.1, within(1e-12));
  }

  @Test
  void settingsMustMakeSense() {
    assertThatThrownBy(() -> with().translationStdDevPerMeter(-1).build())
        .hasMessage("EstimatorConfig.translationStdDevPerMeter: -1 must be zero or more");
    assertThatThrownBy(() -> with().historySeconds(0).build())
        .hasMessage("EstimatorConfig.historySeconds: 0 must be positive");
    assertThatThrownBy(() -> with().recoveryMeasurements(1).build())
        .hasMessage("EstimatorConfig.recoveryMeasurements: 1 must be at least 2");
    assertThatThrownBy(() -> with().tractionLimitedSlipFraction(1.5).build())
        .hasMessage("EstimatorConfig.tractionLimitedSlipFraction: 1.5 must be from 0 to 1");
    assertThatThrownBy(() -> with().tiltStartDegrees(30).build())
        .hasMessageStartingWith(
            "EstimatorConfig.tiltZeroDegrees: 25 must be above tiltStartDegrees (30)");
    assertThatThrownBy(() -> with().gateChiSquaredPose(0).recoveryStdDevMeters(-1).build())
        .as("each value is named")
        .hasMessage(
            "EstimatorConfig.gateChiSquaredPose: 0 must be positive\n"
                + "EstimatorConfig.recoveryStdDevMeters: -1 must be positive");
  }

  /** A gyro that sends NaN would poison the heading for good: it counts as lost instead. */
  @Test
  void aGyroYawThatIsntANumberCountsAsLost() {
    Robot robot = new Robot().drive(25, 1, 0, 0);
    robot.time += STEP;
    robot.estimator.addOdometry(
        robot.time, new BodyMotion(0.004, 0, 0, 0), Double.NaN, /* reporting */ true);

    assertThat(robot.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.IMU_FAULT);
    robot.drive(25, 1, 0, 0);
    assertThat(robot.estimate().mode()).isNotEqualTo(LocalizationEstimate.Mode.IMU_FAULT);
    assertThat(robot.pose().getRotation().getRadians()).isCloseTo(0, within(1e-9));
    assertThat(robot.pose().getX()).isCloseTo(START.getX() + 0.204, within(1e-6));
    assertThat(robot.estimate().covariance().largestPositionStdDev()).isFinite();
  }

  /** A measurement that isn't a number fails the gate; it doesn't pass it. */
  @Test
  void aMeasurementThatIsntANumberIsRejected() {
    Robot robot = new Robot().drive(25, 0, 0, 0);

    List<VisionDecision> decisions =
        robot.see(
            new Pose2d(Double.NaN, Double.NaN, Rotation2d.ZERO),
            robot.time,
            PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY));

    assertThat(decisions).singleElement().satisfies(d -> assertThat(d.accepted()).isFalse());
    assertThat(robot.pose().getTranslation()).isEqualTo(START.getTranslation());
  }

  @Test
  void aLandmarkThatIsntANumberIsRejected() {
    Robot robot = new Robot().drive(25, 0, 0, 0);

    List<LandmarkDecision> decisions = robot.cross(Double.NaN, robot.time, 0.05);

    assertThat(decisions).singleElement().satisfies(d -> assertThat(d.accepted()).isFalse());
    assertThat(robot.pose().getTranslation()).isEqualTo(START.getTranslation());
  }

  private static EstimatorConfigBuilder with() {
    return EstimatorConfigBuilder.builder(EstimatorConfig.DEFAULTS);
  }

  /** What the estimate rests on, in order: each state overrides the ones after it. */
  @Test
  void theModeIsTheFirstThatApplies() {
    assertThat(PoseEstimator.mode(false, false, true, true))
        .isEqualTo(LocalizationEstimate.Mode.IMU_FAULT);
    assertThat(PoseEstimator.mode(true, false, true, true))
        .isEqualTo(LocalizationEstimate.Mode.INITIALIZING);
    assertThat(PoseEstimator.mode(true, true, true, true))
        .isEqualTo(LocalizationEstimate.Mode.LOST);
    assertThat(PoseEstimator.mode(true, true, false, true))
        .isEqualTo(LocalizationEstimate.Mode.TRACKING);
    assertThat(PoseEstimator.mode(true, true, false, false))
        .isEqualTo(LocalizationEstimate.Mode.DEAD_RECKONING);
  }
}
