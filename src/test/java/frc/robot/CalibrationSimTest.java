package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import frc.robot.game.Rebuilt;
import frc.robot.lib.field.FieldModel;
import frc.robot.sim.DriveSimulation;
import frc.robot.sim.SensorErrors;
import java.util.List;
import java.util.Objects;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * The calibration procedures on the simulated robot, whose true wheels, gyro, and camera mounts are
 * known: each should find the number the simulator hid.
 */
class CalibrationSimTest {
  private static final FieldModel FIELD = Rebuilt.field(Robots.SIM.body());
  private static final Pattern NUMBER = Pattern.compile("-?\\d+\\.\\d+");

  private @Nullable RobotContainer container;
  private Scheduler scheduler = Scheduler.createIndependentScheduler();

  /** Made after the Driver Station's data is reset, which clears a gamepad's axes. */
  private GamepadSim driver = new GamepadSim(0);

  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
  }

  @BeforeEach
  void setUp() {
    SimTestSupport.pauseTimeAndDisable();
    DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_2);
    DriverStationSim.notifyNewData();
    TelemetryRegistry.registerBackend("", new MockTelemetryBackend());
    TunableRegistry.registerBackend("", new MockTunableBackend());
  }

  @AfterEach
  void tearDown() {
    driver.setLeftX(0);
    driver.setLeftY(0);
    driver.setRightX(0);
    driver.notifyNewData();
    if (container != null) {
      container.close();
    }
    TelemetryRegistry.reset();
    TunableRegistry.reset();
    SimTestSupport.resumeTime();
  }

  private RobotContainer build(SensorErrors errors, boolean cameras) {
    scheduler = Scheduler.createIndependentScheduler();
    driver = new GamepadSim(0);
    RobotConfig robot =
        cameras ? Robots.SIM : RobotConfigBuilder.builder(Robots.SIM).cameras(List.of()).build();
    container =
        new RobotContainer(
            Constants.Mode.SIM, scheduler, robot, Robots.SIM::drivetrain, FIELD, errors);
    return container;
  }

  private RobotContainer robot() {
    return Objects.requireNonNull(container, "build the robot first");
  }

  private void tick(int loops) {
    for (int i = 0; i < loops; i++) {
      SimTestSupport.stepLoop();
      robot().periodic();
      scheduler.run();
    }
  }

  /** Runs a procedure to its end, enabled, the way the Pitwall page runs it. */
  private String run(Command procedure, double atMostSeconds) {
    SimTestSupport.setEnabled(true);
    scheduler.schedule(procedure);
    for (int i = 0; i < SimTestSupport.loops(atMostSeconds); i++) {
      tick(1);
      if (!scheduler.isScheduledOrRunning(procedure)) {
        break;
      }
    }
    assertThat(scheduler.isScheduledOrRunning(procedure)).as("finished in time").isFalse();
    return robot().calibration().lastResult();
  }

  /** The numbers in a report, in order. */
  private static double[] numbers(String text) {
    Matcher matcher = NUMBER.matcher(text);
    return matcher.results().mapToDouble(match -> Double.parseDouble(match.group())).toArray();
  }

  @Test
  void theWheelRadiusIsFoundBySpinningInPlace() {
    build(SensorErrors.NONE.withWheelRadiusScale(0.98), false); // tread worn 2%
    tick(SimTestSupport.loops(0.5));

    String report = run(robot().calibration().wheelRadius(), 60);

    assertThat(report).startsWith("Wheel radius ").contains("In Robots: .wheelRadius(Inches.of(");
    assertThat(numbers(report)[0]).as("inches").isCloseTo(2 * 0.98, within(0.005));
  }

  @Test
  void theGyrosScaleIsFoundAgainstTheCameras() {
    SensorErrors gyroHigh =
        new SensorErrors(List.of(1.0, 1.0, 1.0, 1.0), List.of(0.0, 0.0, 0.0, 0.0), 1.003, 0, 0, 1);
    build(gyroHigh, true);
    tick(SimTestSupport.loops(1.0)); // placed, and the cameras settle

    String report = run(robot().calibration().gyroScale(), 60);

    assertThat(report).startsWith("Gyro scale ").contains("reads 0.3");
    assertThat(numbers(report)[0]).isCloseTo(1 / 1.003, within(0.0002));
  }

  @Test
  void theLevelCheckReadsTheImu() {
    build(SensorErrors.NONE, false);
    tick(SimTestSupport.loops(0.5));
    String level = run(robot().calibration().level(), 5);

    DriveSimulation sim = Objects.requireNonNull(robot().simulation());
    sim.tilt(Math.toRadians(1.5), 0);
    String tilted = run(robot().calibration().level(), 5);

    assertThat(level).startsWith("Level: pitch 0.00°, roll 0.00°");
    assertThat(tilted).startsWith("Not level: pitch 1.50°");
  }

  /**
   * The front-left camera knocked 4 in low: driving past tags while the other three cameras hold
   * the robot's pose, its mount is found again.
   */
  @Test
  void aCameraKnockedLowIsPlacedAgainByTheOthers() {
    build(
        SensorErrors.TYPICAL.withCameraMountError(
            "front-left", new Transform3d(0, 0, -0.1016, Rotation3d.ZERO)),
        true);
    tick(SimTestSupport.loops(1.0));
    Command procedure = robot().calibration().cameraMount("front-left");
    SimTestSupport.setEnabled(true);
    scheduler.schedule(procedure);

    // Stop at spots in view of the hub's tags, nearer and farther, turned a little each time.
    double[][] moves = {
      {1.0, -0.5, 0, 0.1},
      {1.0, -0.5, 0.3, 0},
      {1.0, 0, -0.5, -0.15},
      {1.0, 0.5, 0, 0.1},
      {1.0, -0.5, -0.3, 0},
      {1.0, 0.4, 0.4, -0.1}
    };
    for (double[] move : moves) {
      driver.setLeftY(-move[1]);
      driver.setLeftX(-move[2]);
      driver.setRightX(-move[3]);
      driver.notifyNewData();
      tick(SimTestSupport.loops(move[0]));
      driver.setLeftY(0);
      driver.setLeftX(0);
      driver.setRightX(0);
      driver.notifyNewData();
      tick(SimTestSupport.loops(2.0)); // standing still
    }
    driver.setLeftY(0);
    driver.setLeftX(0);
    driver.setRightX(0);
    driver.notifyNewData();
    tick(SimTestSupport.loops(0.5));

    String report = robot().calibration().lastResult();
    assertThat(report).startsWith("front-left: tags landed").contains("camera(\"front-left\"");
    String line = report.substring(report.indexOf("camera(\"front-left\""));
    double[] mount = numbers(line);
    // Configured at (-7.75, 12.25, 21) in, roll 0, tilted up 10, turned 23.162: really 4 in lower.
    assertThat(mount[0]).as("forward").isCloseTo(-7.75, within(0.4));
    assertThat(mount[1]).as("left").isCloseTo(12.25, within(0.4));
    assertThat(mount[2]).as("up").isCloseTo(17.0, within(0.4));
    assertThat(mount[4]).as("tilted up").isCloseTo(10, within(0.5));
    assertThat(mount[5]).as("turned").isCloseTo(23.162, within(0.5));
    assertThat(numbers(report)[0]).as("tags landed off, cm").isGreaterThan(5);
    assertThat(numbers(report)[1]).as("and with the mount found").isLessThan(3);
  }

  /**
   * A canceled command never resumes, so its cleanup can't be code after its loop: disabling the
   * robot mid-calibration must give the camera back to localization at once, not at a restart.
   */
  @Test
  void aCanceledCalibrationGivesTheCamerasBack() {
    build(SensorErrors.NONE, true);
    tick(SimTestSupport.loops(1.0));
    SimTestSupport.setEnabled(true);
    for (Command procedure :
        List.of(
            robot().calibration().cameraMount("front-left"), robot().calibration().gyroScale())) {
      scheduler.schedule(procedure);
      tick(5);
      assertThat(robot().vision().isCalibrating()).as(procedure.name()).isTrue();

      scheduler.cancel(procedure);
      tick(1);

      assertThat(robot().vision().isCalibrating()).as(procedure.name()).isFalse();
      assertThat(robot().calibration().lastResult()).endsWith(": canceled before it finished");
    }
  }
}
