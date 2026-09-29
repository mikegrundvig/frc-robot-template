package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;

/**
 * The whole robot as the simulator runs it: the real composition root, the real controller
 * bindings, a simulated gamepad, and the simulated drivetrain. If this passes, a driver in the
 * simulator GUI gets the same behavior.
 */
class RobotContainerSimTest {
  private static final int DRIVER_PORT = 0;
  private static final Pose2d START = new Pose2d(3, 3, Rotation2d.ZERO);

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

  /** Robot loops, exactly as Robot.robotPeriodic runs them. */
  private void tick(int loops) {
    for (int i = 0; i < loops; i++) {
      SimTestSupport.stepLoop();
      container.periodic();
      scheduler.run();
    }
  }

  private void pushLeftStickForward() {
    SimTestSupport.setEnabled(true);
    driver.setLeftY(-1.0); // gamepad Y is negative when pushed forward
    driver.notifyNewData();
  }

  @Test
  void holdsStillWhileDisabled() {
    tick(SimTestSupport.loops(1.0));

    assertEquals(START, container.drive().getPose());
    assertFalse(container.isNoHardwareAlertActive(), "the simulator is the drivetrain's hardware");
  }

  @Test
  void driverLeftStickDrivesForward() {
    pushLeftStickForward();

    tick(SimTestSupport.loops(1.5));

    Pose2d pose = container.drive().getPose();
    assertTrue(pose.getX() > 5.0, "drove forward: now at x=" + pose.getX());
    assertEquals(3.0, pose.getY(), 0.05, "straight ahead");
  }

  @Test
  void fieldViewShowsTheRobotWhereOdometryPutsIt() {
    pushLeftStickForward();

    tick(SimTestSupport.loops(1.0));

    Pose2d[] shown =
        (Pose2d[])
            telemetry
                .getLastValue("/Field/Robot", MockTelemetryBackend.LogStructArrayValue.class)
                .value();
    assertEquals(container.drive().getPose(), shown[0]);
  }

  @Test
  void replayRunsWithoutTheSimulator() {
    container.close();
    container = containerFor(Constants.Mode.REPLAY);

    tick(5);

    assertEquals(new Pose2d(), container.drive().getPose(), "nothing to replay, nothing moves");
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
    pushLeftStickForward();

    tick(SimTestSupport.loops(0.5));

    assertTrue(container.isNoHardwareAlertActive());
    assertEquals(new Pose2d(), container.drive().getPose(), "nothing moved");
  }
}
