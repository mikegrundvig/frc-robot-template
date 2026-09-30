package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import frc.robot.Robots;
import frc.robot.SimTestSupport;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * The simulator keeps where the robot really is, and its wheels and gyro report that with the
 * errors real ones have. Each test injects one error and checks that odometry drifts from the truth
 * the way a real robot's does: the gap localization has to close.
 */
class DriveSimulationTest {
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
    harness = new DriveSimHarness(Robots.SIM, errors);
    return harness;
  }

  private static SensorErrors gyro(double scale, double driftDegreesPerMinute) {
    return new SensorErrors(
        List.of(1.0, 1.0, 1.0, 1.0),
        List.of(0.0, 0.0, 0.0, 0.0),
        scale,
        driftDegreesPerMinute,
        0,
        1);
  }

  @Test
  void withoutErrorsOdometryIsTheTruth() {
    DriveSimHarness robot = robot(SensorErrors.NONE);

    robot.drive(2.0, 0.6, 0.3, 0.4); // curving and spinning at once

    assertThat(robot.simulation.truePose().getTranslation().getNorm()).isGreaterThan(1);
    assertThat(robot.positionError()).isLessThan(1e-6);
    assertThat(robot.headingErrorDegrees()).isCloseTo(0.0, within(1e-6));
  }

  /** Worn tread rolls less far per turn than the code thinks, so odometry overshoots. */
  @Test
  void wornTreadMakesOdometryOverestimateDistance() {
    DriveSimHarness robot = robot(SensorErrors.NONE.withWheelRadiusScale(0.98));

    robot.drive(2.0, 1.0, 0, 0);

    double traveled = robot.simulation.truePose().getX();
    assertThat(traveled).isGreaterThan(3);
    assertThat(robot.drive.getPose().getX() / traveled).isCloseTo(1 / 0.98, within(1e-3));
  }

  @Test
  void aDriftingGyroTurnsTheEstimateWhileTheRobotSitsStill() {
    DriveSimHarness robot = robot(gyro(1.0, 1.0));

    robot.idle(60);

    assertThat(robot.headingErrorDegrees()).isCloseTo(1.0, within(0.01));
    assertThat(robot.positionError()).as("its wheels never turned").isLessThan(1e-9);
  }

  @Test
  void aGyroReadingHighMisjudgesEveryTurn() {
    DriveSimHarness robot = robot(gyro(1.01, 0));

    robot.drive(1.0, 0, 0, 1.0); // spin in place

    double turnedDegrees = Math.toDegrees(robot.simulation.yawTurnedRadians());
    assertThat(turnedDegrees).isGreaterThan(200);
    assertThat(robot.headingErrorDegrees()).isCloseTo(turnedDegrees * 0.01, within(0.05));
  }

  /** An absolute encoder zeroed 5° off steers its module 5° off, and odometry can't tell. */
  @Test
  void anEncoderZeroedOffPointsItsModuleWrongAndOdometryDrifts() {
    SensorErrors oneOff =
        new SensorErrors(List.of(1.0, 1.0, 1.0, 1.0), List.of(5.0, 0.0, 0.0, 0.0), 1.0, 0, 0, 1);
    DriveSimHarness robot = robot(oneOff);

    robot.drive(2.0, 1.0, 0, 0);

    assertThat(robot.positionError()).isGreaterThan(0.05);
  }

  /** Another robot's push moves it without turning its wheels: only cameras can catch that. */
  @Test
  void aShoveMovesTheRobotWhereOdometryCannotSee() {
    DriveSimHarness robot = robot(SensorErrors.NONE);

    robot.simulation.shove(new Transform2d(0, 0.3, Rotation2d.ZERO));
    robot.idle(0.1);

    assertThat(robot.positionError()).isCloseTo(0.3, within(1e-9));
  }

  @Test
  void theGyroFeelsTheRobotTurnedByHand() {
    DriveSimHarness robot = robot(SensorErrors.NONE);

    robot.simulation.shove(new Transform2d(0, 0, Rotation2d.fromDegrees(30)));
    robot.idle(0.1);

    assertThat(robot.headingErrorDegrees()).as("the gyro followed").isCloseTo(0.0, within(1e-9));
    assertThat(robot.drive.getPose().getRotation().getDegrees()).isCloseTo(30.0, within(1e-9));
  }

  @Test
  void placedAndToldTheRobotKnowsWhereItIs() {
    DriveSimHarness robot = robot(SensorErrors.TYPICAL);
    Pose2d start = new Pose2d(3, 4, Rotation2d.fromDegrees(90));

    robot.simulation.place(start);
    robot.loop();
    robot.drive.resetPose(start);

    assertThat(robot.positionError()).isLessThan(1e-9);
    assertThat(robot.headingErrorDegrees()).isCloseTo(0.0, within(1e-9));
  }

  /** Noise comes from a seed, so a simulated run, like a test, gives the same answer every time. */
  @Test
  void theSameSeedRepeatsARunExactly() {
    Pose2d first = drivenWith(SensorErrors.TYPICAL);
    Pose2d second = drivenWith(SensorErrors.TYPICAL);

    assertThat(second).isEqualTo(first);
  }

  private static Pose2d drivenWith(SensorErrors errors) {
    SimTestSupport.pauseTimeAndDisable();
    try (DriveSimHarness robot = new DriveSimHarness(Robots.SIM, errors)) {
      robot.drive(1.0, 0.8, 0.2, 0.5);
      return robot.drive.getPose();
    }
  }

  @Test
  void errorsMustBeRealistic() {
    assertThatThrownBy(() -> SensorErrors.NONE.withWheelRadiusScale(0))
        .hasMessageContaining("positive");
    assertThatThrownBy(
            () ->
                new SensorErrors(List.of(1.0, 1.0, 1.0), List.of(0.0, 0.0, 0.0, 0.0), 1.0, 0, 0, 1))
        .hasMessageContaining("per module");
    assertThatThrownBy(
            () ->
                new SensorErrors(
                    List.of(1.0, 1.0, 1.0, 1.0), List.of(0.0, 0.0, 0.0, Double.NaN), 1.0, 0, 0, 1))
        .hasMessageContaining("steer offset");
    assertThatThrownBy(() -> gyro(1.0, Double.NaN)).hasMessageContaining("gyro");
    assertThatThrownBy(() -> robot(SensorErrors.NONE).simulation.step(0))
        .hasMessageContaining("a step takes time");
  }
}
