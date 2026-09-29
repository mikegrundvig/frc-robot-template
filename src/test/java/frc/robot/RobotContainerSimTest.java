package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.lib.swerve.SwerveSim;
import java.util.Objects;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;

/**
 * The whole robot, end to end, as the simulator runs it: the real composition root, the real
 * controller bindings, a simulated gamepad, and the physics. If this passes, a driver in the
 * simulator GUI gets the same behavior.
 */
class RobotContainerSimTest {
  private static final int DRIVER_PORT = 0;

  private Scheduler scheduler;
  private RobotContainer container;
  private GamepadSim driver;
  private MockTelemetryBackend telemetry;

  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
  }

  @BeforeEach
  void setUp() {
    SimTestSupport.pauseTimeAndDisable();
    telemetry = new MockTelemetryBackend();
    TelemetryRegistry.registerBackend("", telemetry);
    scheduler = Scheduler.createIndependentScheduler();
    container = containerFor(Constants.Mode.SIM);
    driver = new GamepadSim(DRIVER_PORT);
  }

  @AfterEach
  void tearDown() {
    driver.setLeftY(0.0);
    driver.notifyNewData();
    container.close();
    TelemetryRegistry.reset();
    SimTestSupport.resumeTime();
  }

  private RobotContainer containerFor(Constants.Mode mode) {
    return new RobotContainer(mode, scheduler, Robots.SIM.body(), Robots.SIM::drivetrain);
  }

  /** One robot loop, exactly as Robot.robotPeriodic runs it. */
  private void tick(int loops) {
    for (int i = 0; i < loops; i++) {
      SimTestSupport.stepLoop();
      container.periodic();
      scheduler.run();
    }
  }

  private SwerveSim sim() {
    return Objects.requireNonNull(container.swerveSim(), "physics while simulating");
  }

  @Test
  void idlesWhileDisabledOnAFullBattery() {
    tick(SimTestSupport.loops(1.0));

    assertEquals(3.0, sim().truePose().getX(), 1e-6, "robot did not move");
    assertEquals(sim().batteryVolts(), RobotController.getBatteryVoltage(), 1e-6);
    assertTrue(RobotController.getBatteryVoltage() > 12.4, "only the base load on the battery");
    assertFalse(container.isNoHardwareAlertActive(), "the simulator is the drivetrain's hardware");
  }

  @Test
  void driverLeftStickDrivesForwardAndWpilibSeesTheBatterySag() {
    SimTestSupport.setEnabled(true);
    driver.setLeftY(-1.0); // gamepad Y is negative when pushed forward
    driver.notifyNewData();

    double lowestReported = Double.POSITIVE_INFINITY;
    for (int i = 0; i < SimTestSupport.loops(1.5); i++) {
      tick(1);
      lowestReported = Math.min(lowestReported, RobotController.getBatteryVoltage());
    }

    assertTrue(sim().truePose().getX() > 5.0, "drove forward: now at x=" + sim().truePose().getX());
    assertEquals(3.0, sim().truePose().getY(), 0.05, "straight ahead");
    assertEquals(sim().batteryVolts(), RobotController.getBatteryVoltage(), 1e-6);
    assertTrue(lowestReported < 11.5, "WPILib saw the sag while accelerating: " + lowestReported);
  }

  @Test
  void fieldViewShowsWhereOdometryAndPhysicsPutTheRobot() {
    SimTestSupport.setEnabled(true);
    driver.setLeftY(-1.0);
    driver.notifyNewData();
    tick(SimTestSupport.loops(1.0));

    Pose2d[] robot = fieldPoses("Robot");
    Pose2d[] truePose = fieldPoses("TruePose");
    assertEquals(container.drive().getPose(), robot[0]);
    assertEquals(sim().truePose(), truePose[0]);
    assertTrue(robot[0].getX() > 3.5, "the field shows the robot moving: x=" + robot[0].getX());
  }

  /** The poses of one object in the field view, as the simulator GUI receives them. */
  private Pose2d[] fieldPoses(String object) {
    return (Pose2d[])
        telemetry
            .getLastValue("/Field/" + object, MockTelemetryBackend.LogStructArrayValue.class)
            .value();
  }

  @Test
  void replayRunsWithoutTheSimulatorOnlyParts() {
    container.close();
    container = containerFor(Constants.Mode.REPLAY);

    tick(5);

    assertNull(container.swerveSim(), "no physics in replay");
    assertFalse(container.isNoHardwareAlertActive(), "replay's hardware is the log");
  }

  /**
   * Until a team writes the motor-controller code for its modules, a real robot holds still and
   * says why, rather than guessing at motors.
   */
  @Test
  void aRealRobotWithoutHardwareCodeHoldsStillAndSaysSo() {
    container.close();
    container = containerFor(Constants.Mode.REAL);
    SimTestSupport.setEnabled(true);
    driver.setLeftY(-1.0);
    driver.notifyNewData();

    tick(SimTestSupport.loops(0.5));

    assertNull(container.swerveSim(), "no physics on a robot");
    assertTrue(container.isNoHardwareAlertActive());
    assertEquals(new Pose2d(), container.drive().getPose(), "nothing moved");
  }
}
