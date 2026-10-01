package frc.robot.sim;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.SimulationTest;
import frc.robot.lib.field.Bump;
import frc.robot.lib.field.Terrain;
import frc.robot.lib.localization.EstimatorConfig;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The simulator keeps where the robot really is, and its wheels and gyro report that with the
 * errors real ones have. Each test injects one error and checks that odometry drifts from the truth
 * the way a real robot's does: the gap localization has to close.
 */
@SimulationTest
@ExtendWith(RobotSim.class)
class DriveSimulationTest {
  private @Nullable DriveSimHarness harness;

  @AfterEach
  void tearDown() {
    if (harness != null) {
      harness.close();
    }
  }

  private DriveSimHarness robot(SensorErrors errors) {
    harness = new DriveSimHarness(Robots.SIM, errors);
    return harness;
  }

  /** Both IMUs reading with this scale and drift, and no noise. */
  private static SensorErrors gyro(double scale, double driftDegreesPerMinute) {
    return SensorErrors.NONE.withGyros(
        SensorErrors.GyroErrors.builder()
            .scale(scale)
            .driftDegreesPerMinute(driftDegreesPerMinute)
            .noiseDegrees(0)
            .build());
  }

  /** Without errors, and with the wheels holding their angle, odometry is the truth. */
  @Test
  void withoutErrorsOdometryIsTheTruth() {
    DriveSimHarness robot = robot(SensorErrors.NONE);

    robot.drive(2.0, 1.0, 0, 0);

    assertThat(robot.simulation.truePose().getTranslation().getNorm()).isGreaterThan(1);
    assertThat(robot.positionError()).isLessThan(1e-6);
    assertThat(robot.headingErrorDegrees()).isCloseTo(0.0, within(1e-6));
  }

  /**
   * Even without errors, turning wheels cost odometry a little: each wheel's angle comes from its
   * CANcoder, which sends it every 10 ms, so odometry sees a turning wheel up to 10 ms late. Over
   * two seconds of curving while spinning, about a centimeter. The gyro still has the heading
   * exactly.
   */
  @Test
  void turningWheelsReachOdometryAsLateAsTheirCanCodersSendThem() {
    DriveSimHarness robot = robot(SensorErrors.NONE);

    robot.drive(2.0, 0.6, 0.3, 0.4); // curving and spinning at once

    assertThat(robot.simulation.truePose().getTranslation().getNorm()).isGreaterThan(1);
    assertThat(robot.positionError()).isBetween(0.005, 0.02);
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
    DriveSimHarness robot = robot(gyro(1.0, 10.0));

    robot.idle(6); // a tenth of a minute

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
        SensorErrorsBuilder.builder(SensorErrors.NONE)
            .steerOffsetDegrees(List.of(5.0, 0.0, 0.0, 0.0))
            .build();
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
    robot.idle(0.1);

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
    robot.localization.resetPose(start);

    assertThat(robot.positionError()).isLessThan(1e-9);
    assertThat(robot.headingErrorDegrees()).isCloseTo(0.0, within(1e-9));
  }

  /**
   * A long ramp up to a crest at x = 5, rising a quarter meter per meter: every wheel fits on it.
   */
  private static final Terrain RAMP =
      new Terrain(
          List.of(new Bump("Ramp", new Translation2d(5, 0), new Translation2d(5, 8), 0.5, 4)));

  @Test
  void theGroundUnderItsWheelsTiltsAndRaisesTheRobot() {
    DriveSimHarness robot = robot(SensorErrors.NONE);
    robot.simulation.setTerrain(RAMP);

    robot.simulation.place(new Pose2d(4, 4, Rotation2d.ZERO)); // facing up the ramp
    Pose3d facingUp = robot.simulation.truePose3d();
    robot.simulation.place(new Pose2d(4, 4, Rotation2d.CCW_PI_2)); // across it
    Pose3d across = robot.simulation.truePose3d();

    assertThat(facingUp.getZ()).isCloseTo(0.25, within(1e-9));
    assertThat(facingUp.getRotation().getY())
        .as("nose up: a negative pitch")
        .isCloseTo(-Math.atan(0.25), within(1e-9));
    assertThat(facingUp.getRotation().getX()).isCloseTo(0.0, within(1e-9));
    assertThat(across.getRotation().getX())
        .as("left side downhill: a negative roll")
        .isCloseTo(-Math.atan(0.25), within(1e-9));
    assertThat(across.getRotation().getY()).isCloseTo(0.0, within(1e-9));
  }

  /** Up a ramp the wheels roll farther than the robot moves across the field; odometry knows. */
  @Test
  void onAKnownRampOdometryCountsTheWheelsTravelAtTheSlope() {
    DriveSimHarness robot =
        new DriveSimHarness(Robots.SIM, SensorErrors.NONE, EstimatorConfig.DEFAULTS, RAMP);
    harness = robot;
    Pose2d start = new Pose2d(3.4, 4, Rotation2d.ZERO);
    robot.simulation.place(start);
    robot.loop();
    robot.localization.resetPose(start);
    double wheelBefore = robot.drive.wheelRotationsRadians()[0];

    robot.run(robot.drive.driveForward(1.0, 0.8), 1.0);

    double moved = robot.simulation.truePose().getX() - start.getX();
    double radius = Robots.SIM.drivetrain().modules().wheelRadius().in(Meters);
    double rolled = (robot.drive.wheelRotationsRadians()[0] - wheelBefore) * radius;
    assertThat(moved).isGreaterThan(0.5);
    assertThat(rolled).isCloseTo(moved * Math.hypot(1, 0.25), within(0.01));
    assertThat(robot.positionError()).isLessThan(0.005);
  }

  /** Noise comes from a seed, so a simulated run, like a test, gives the same answer every time. */
  @Test
  void theSameSeedRepeatsARunExactly() {
    Pose2d first = drivenWith(SensorErrors.TYPICAL);
    Pose2d second = drivenWith(SensorErrors.TYPICAL);

    assertThat(second).isEqualTo(first);
  }

  private static Pose2d drivenWith(SensorErrors errors) {
    RobotSim.setEnabled(false);
    try (DriveSimHarness robot = new DriveSimHarness(Robots.SIM, errors)) {
      robot.drive(1.0, 0.8, 0.2, 0.5);
      return robot.drive.getPose();
    }
  }

  @Test
  void errorsMustBeRealistic() {
    assertThatThrownBy(() -> SensorErrors.NONE.withWheelRadiusScale(0))
        .hasMessage(
            "SensorErrors.wheelRadiusScale: [0] 0 must be positive; [1] 0 must be positive;"
                + " [2] 0 must be positive; [3] 0 must be positive");
    assertThatThrownBy(
            () ->
                SensorErrorsBuilder.builder(SensorErrors.NONE)
                    .wheelRadiusScale(List.of(1.0, 1.0, 1.0))
                    .build())
        .hasMessage("SensorErrors.wheelRadiusScale: must have 4, has 3");
    assertThatThrownBy(
            () ->
                SensorErrorsBuilder.builder(SensorErrors.NONE)
                    .steerOffsetDegrees(List.of(0.0, 0.0, 0.0, Double.NaN))
                    .build())
        .hasMessage("SensorErrors.steerOffsetDegrees: [3] NaN must be a finite number");
    assertThatThrownBy(() -> gyro(1.0, Double.NaN))
        .hasMessage("GyroErrors.driftDegreesPerMinute: NaN must be a finite number");
    assertThatThrownBy(() -> robot(SensorErrors.NONE).simulation.step(0, 0))
        .hasMessageContaining("a step takes time");
  }
}
