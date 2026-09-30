package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.lib.field.FieldModel;
import frc.robot.lib.field.Placement;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;

/**
 * The whole robot as the simulator runs it: the real composition root, the real controller
 * bindings, the real field, a simulated gamepad, and the simulated drivetrain. If this passes, a
 * driver in the simulator GUI gets the same behavior.
 */
class RobotContainerSimTest {
  private static final int DRIVER_PORT = 0;
  private static final FieldModel FIELD = Rebuilt.field(Robots.SIM.body());

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
    setStation(AllianceStationID.BLUE_2);
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
    return new RobotContainer(mode, scheduler, Robots.SIM.body(), Robots.SIM::drivetrain, FIELD);
  }

  private static void setStation(AllianceStationID station) {
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.notifyNewData();
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

  private void releaseStickAndDisable() {
    driver.setLeftY(0.0);
    driver.notifyNewData();
    SimTestSupport.setEnabled(false);
  }

  private static Pose2d start(String name, Alliance alliance) {
    return FIELD.starts().stream()
        .filter(start -> start.name().equals(name))
        .findFirst()
        .orElseThrow()
        .on(FIELD, alliance);
  }

  @Test
  void waitsOnItsStartWhileDisabled() {
    tick(SimTestSupport.loops(1.0));

    assertEquals(start("Center", Alliance.BLUE), container.drive().getPose());
    assertThat(container.allianceStation()).isEqualTo("Blue 2");
    assertThat(container.placement().status()).isEqualTo(Placement.Status.READY);
    assertFalse(container.isNoHardwareAlertActive(), "the simulator is the drivetrain's hardware");
  }

  @Test
  void theDriverStationsLocationChoosesTheStart() {
    setStation(AllianceStationID.RED_1);

    tick(1);

    assertEquals(start("Left", Alliance.RED), container.drive().getPose());
    assertThat(container.placement().describe()).isEqualTo("Ready on Left: 0 cm, 0° off");
  }

  @Test
  void theLeftStickDrivesAwayFromTheBlueDrivers() {
    Pose2d start = start("Center", Alliance.BLUE);
    pushLeftStickForward();

    tick(SimTestSupport.loops(1.5));

    Pose2d pose = container.drive().getPose();
    assertTrue(pose.getX() > start.getX() + 2.0, "toward red's wall: now at x=" + pose.getX());
    assertEquals(start.getY(), pose.getY(), 0.05, "straight ahead");
  }

  /** The red drivers stand at the other end, so their forward is the field's -x. */
  @Test
  void theLeftStickDrivesAwayFromTheRedDrivers() {
    setStation(AllianceStationID.RED_2);
    Pose2d start = start("Center", Alliance.RED);
    pushLeftStickForward();

    tick(SimTestSupport.loops(1.5));

    Pose2d pose = container.drive().getPose();
    assertTrue(pose.getX() < start.getX() - 2.0, "toward blue's wall: now at x=" + pose.getX());
    assertEquals(start.getY(), pose.getY(), 0.05, "straight ahead");
  }

  @Test
  void afterDrivingThePlacementCheckSaysHowFarAwayTheStartIs() {
    pushLeftStickForward();
    tick(SimTestSupport.loops(1.0));
    releaseStickAndDisable();

    tick(1);

    assertThat(container.placement().status()).isEqualTo(Placement.Status.NOT_ON_A_START);
    assertThat(container.placement().describe()).startsWith("Not on a start: Center is ");
  }

  /** In the pit or the simulator, the robot goes on the start chosen while it's disabled. */
  @Test
  void changingTheStationWhileDisabledMovesTheRobotButNeverWhileItDrives() {
    pushLeftStickForward();
    tick(5);
    setStation(AllianceStationID.BLUE_3);
    tick(5);
    assertThat(container.drive().getPose().getY())
        .as("still driving from the center start")
        .isCloseTo(start("Center", Alliance.BLUE).getY(), within(0.05));

    releaseStickAndDisable();
    tick(SimTestSupport.loops(1.0)); // coasts to a stop, then the drive team moves it
    setStation(AllianceStationID.BLUE_1);
    tick(1);

    assertThat(
            container
                .drive()
                .getPose()
                .minus(start("Left", Alliance.BLUE))
                .getTranslation()
                .getNorm())
        .isLessThan(0.001);
  }

  @Test
  void zonesFollowTheRobot() {
    tick(1);

    assertThat(container.inZone(Rebuilt.ALLIANCE_ZONE)).as("on a start, in its zone").isTrue();
    assertThat(container.inZone(Rebuilt.TRENCH)).isFalse();
  }

  /** From the left start, straight ahead passes under the left trench. */
  @Test
  void theTrenchTriggerTurnsOnBeforeTheRobotReachesIt() {
    setStation(AllianceStationID.BLUE_1);
    tick(1);
    pushLeftStickForward();

    boolean onBeforeArriving = false;
    for (int i = 0; i < SimTestSupport.loops(2.0); i++) {
      tick(1);
      boolean under =
          Rebuilt.TRENCH.contains(
              FIELD, Alliance.BLUE, container.drive().getPose().getTranslation());
      if (container.inZone(Rebuilt.TRENCH) && !under) {
        onBeforeArriving = true;
      }
      if (under) {
        break;
      }
    }

    assertThat(onBeforeArriving).isTrue();
    assertThat(container.inZone(Rebuilt.TRENCH)).as("and on while under it").isTrue();
  }

  @Test
  void untilTheAllianceIsKnownTheRobotWaitsMidfieldAndSaysSo() {
    setStation(AllianceStationID.UNKNOWN);

    tick(1);

    assertThat(container.drive().getPose().getX()).isCloseTo(FIELD.length() / 2, within(1e-9));
    assertThat(container.allianceStation()).isEqualTo("Unknown");
    assertThat(container.placement().describe()).isEqualTo("Alliance unknown");
  }

  @Test
  void theFieldViewShowsTheRobotItsStartAndTheTargets() {
    pushLeftStickForward();

    tick(SimTestSupport.loops(1.0));

    assertEquals(container.drive().getPose(), shown("/Field/Robot")[0]);
    assertEquals(start("Center", Alliance.BLUE), shown("/Field/Start")[0]);
    assertEquals(Rebuilt.HUB.on(FIELD, Alliance.BLUE), shown("/Field/Hub")[0]);
  }

  @Test
  void replayRunsWithoutTheSimulator() {
    container.close();
    container = containerFor(Constants.Mode.REPLAY);

    tick(5);

    assertEquals(
        start("Center", Alliance.BLUE),
        container.drive().getPose(),
        "on its start, as the log's Driver Station says; nothing moves");
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
    tick(1);
    pushLeftStickForward();

    tick(SimTestSupport.loops(0.5));

    assertTrue(container.isNoHardwareAlertActive());
    assertEquals(start("Center", Alliance.BLUE), container.drive().getPose(), "nothing moved");
  }

  private Pose2d[] shown(String path) {
    return (Pose2d[])
        telemetry.getLastValue(path, MockTelemetryBackend.LogStructArrayValue.class).value();
  }
}
