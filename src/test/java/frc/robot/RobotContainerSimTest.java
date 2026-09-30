package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Rotations;

import frc.robot.lib.config.CanBus;
import frc.robot.lib.config.SwerveWiring;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.field.Placement;
import frc.robot.subsystems.drive.DriveSimulation;
import frc.robot.subsystems.drive.SensorErrors;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.Alliance;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

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
  private MockTunableBackend tunables;

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
    tunables = new MockTunableBackend();
    TunableRegistry.registerBackend("", tunables);
    scheduler = Scheduler.createIndependentScheduler();
    container = containerFor(Constants.Mode.SIM);
    driver = new GamepadSim(DRIVER_PORT);
  }

  @AfterEach
  void tearDown() {
    driver.setLeftY(0.0);
    driver.setBackButton(false);
    driver.notifyNewData();
    container.close();
    TelemetryRegistry.reset();
    TunableRegistry.reset();
    SimTestSupport.resumeTime();
  }

  /** A robot whose wheels and sensors are exactly as configured, so poses come out exact. */
  private RobotContainer containerFor(Constants.Mode mode) {
    return containerFor(mode, SensorErrors.NONE);
  }

  private RobotContainer containerFor(Constants.Mode mode, SensorErrors errors) {
    return containerFor(mode, errors, Robots.SIM);
  }

  private RobotContainer containerFor(Constants.Mode mode, SensorErrors errors, RobotConfig robot) {
    return new RobotContainer(mode, scheduler, robot, Robots.SIM::drivetrain, FIELD, errors);
  }

  /** Turns the simulated robot by hand, where it stands: its gyro feels it. */
  private void turnByHand(double degrees) {
    DriveSimulation sim = Objects.requireNonNull(container.simulation());
    Pose2d truth = sim.truePose();
    sim.place(
        new Pose2d(
            truth.getTranslation(), truth.getRotation().plus(Rotation2d.fromDegrees(degrees))));
  }

  private void pressBack() {
    driver.setBackButton(true);
    driver.notifyNewData();
    tick(1);
    driver.setBackButton(false);
    driver.notifyNewData();
  }

  private double headingDegrees() {
    return container.drive().getPose().getRotation().getDegrees();
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

    // Placed on its start; the cameras then refine that to within their noise.
    Pose2d pose = container.drive().getPose();
    Pose2d center = start("Center", Alliance.BLUE);
    assertThat(pose.getTranslation().getDistance(center.getTranslation())).isLessThan(0.03);
    assertThat(pose.getRotation().minus(center.getRotation()).getDegrees())
        .isCloseTo(0.0, within(0.5));
    assertThat(container.allianceStation()).isEqualTo("Blue 2");
    assertThat(container.placement().status()).isEqualTo(Placement.Status.READY);
    assertFalse(container.isNoHardwareAlertActive(), "the simulator is the drivetrain's hardware");
  }

  @Test
  void theBackButtonSaysTheRobotFacesDownfieldForTheDriversAlliance() {
    container.close();
    container =
        containerFor(
            Constants.Mode.SIM,
            SensorErrors.NONE,
            RobotConfigBuilder.builder(Robots.SIM).cameras(List.of()).build());
    setStation(AllianceStationID.RED_2);
    tick(2); // placed on red's center start, facing blue's wall
    turnByHand(30);
    tick(1);
    double turned = headingDegrees();

    pressBack();

    assertThat(turned).isCloseTo(-150, within(0.5));
    assertThat(Math.abs(headingDegrees())).as("red faces -x").isCloseTo(180, within(1e-6));
  }

  /**
   * The driver pressed Back with the robot turned 30 degrees: the cameras turn the estimate back.
   */
  @Test
  void theCamerasTurnBackAWrongHeadingReset() {
    setStation(AllianceStationID.RED_2);
    tick(SimTestSupport.loops(1.0));
    turnByHand(30);
    tick(SimTestSupport.loops(0.5));

    pressBack();
    double wrong = headingDegrees();
    tick(SimTestSupport.loops(1.0));

    assertThat(Math.abs(wrong)).isCloseTo(180, within(0.5));
    assertThat(headingDegrees()).isCloseTo(-150, within(1.0));
    DriveSimulation sim = Objects.requireNonNull(container.simulation());
    assertThat(
            container
                .drive()
                .getPose()
                .getTranslation()
                .getDistance(sim.truePose().getTranslation()))
        .isLessThan(0.05);
  }

  @Test
  void itSaysWhenItIsReadyToScoreAndWhyNot() {
    String before = container.scoring();
    tick(SimTestSupport.loops(1.0));
    String settled = container.scoring();
    boolean ready = container.readyToScore();
    Objects.requireNonNull(container.simulation()).tilt(Math.toRadians(10), 0);
    tick(2);

    assertThat(before).isEqualTo("Not ready: not started");
    assertThat(settled).isEqualTo("Ready to score");
    assertThat(ready).isTrue();
    assertThat(container.scoring()).isEqualTo("Not ready: the robot is tilted 10 degrees");
  }

  @Test
  void autonomousStartingHoldsTheCamerasOnlyWhenAsked() {
    tunables.setDouble("Vision/autoStartHoldSeconds", 2.0);
    tunables.update();
    tick(SimTestSupport.loops(1.0));
    int before = container.estimate().health().accepted();

    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    tick(SimTestSupport.loops(1.0));
    int held = container.estimate().health().accepted() - before;
    tick(SimTestSupport.loops(1.5));

    assertThat(held).as("held for 2 s").isZero();
    assertThat(container.estimate().health().accepted() - before).as("then used").isPositive();
  }

  @Test
  void eachLoopIsTimedAndOverrunsRaiseAnAlert() {
    long first = System.nanoTime();
    container.loop().start(first);
    tick(1);
    container.finishLoop(first + 5_000_000L); // as if the loop took 5 ms
    String quick = container.loopStatus();
    boolean alertAfterQuick = container.isOverrunAlertActive();

    long start = System.nanoTime();
    container.loop().start(start);
    tick(1);
    container.finishLoop(start + 35_000_000L); // as if the loop took 35 ms

    assertThat(quick).isEqualTo("5.0 ms, worst 5.0 ms lately");
    assertThat(alertAfterQuick).isFalse();
    assertThat(container.isOverrunAlertActive()).isTrue();
    assertThat(container.overrunAlertText())
        .startsWith("The loop overran 20 ms 1 time in the last 10 s; the last took 35 ms");
    assertThat(container.loopStatus()).isEqualTo("Overran 1 time lately, worst 35.0 ms");
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

  /**
   * The simulator's robot has a season's wear on its wheels and gyro, as {@code ./gradlew run}
   * builds it, and four cameras whose views PhotonVision renders from where the robot really is:
   * the estimate stays with the truth, and the field view shows both.
   */
  @Test
  void theCamerasKeepTheEstimateWithTheTruth() {
    container.close();
    container = containerFor(Constants.Mode.SIM, SensorErrors.TYPICAL);
    tick(1);
    pushLeftStickForward();

    tick(SimTestSupport.loops(1.5));

    DriveSimulation simulation =
        Objects.requireNonNull(container.simulation(), "the simulator has physics");
    Pose2d truth = simulation.truePose();
    Pose2d believed = container.drive().getPose();
    assertThat(believed.getTranslation().getDistance(truth.getTranslation()))
        .as("6 m driven on worn wheels")
        .isLessThan(0.10);
    assertThat(container.estimate().health().accepted()).as("camera measurements").isPositive();
    assertEquals(truth, shown("/Field/Truth")[0]);
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
    assertThat(container.simulation()).as("no physics in a replay").isNull();
  }

  /**
   * Until a team writes the motor-controller code for its modules, a real robot holds still and
   * says why, rather than guessing at motors.
   */
  @Test
  void aRealRobotWithoutWiringHoldsStillAndSaysSo() {
    container.close();
    container = containerFor(Constants.Mode.REAL);
    tick(1);
    pushLeftStickForward();

    tick(SimTestSupport.loops(0.5));

    assertTrue(container.isNoHardwareAlertActive());
    assertEquals(start("Center", Alliance.BLUE), container.drive().getPose(), "nothing moved");
  }

  /**
   * A robot whose configuration has its wiring drives CTRE hardware: here Phoenix's simulated
   * devices, which answer like real ones.
   */
  @Test
  void aRealRobotWithWiringReadsItsCtreHardware() {
    container.close();
    SwerveWiring.ModuleWiring[] modules = new SwerveWiring.ModuleWiring[4];
    for (int i = 0; i < 4; i++) {
      modules[i] =
          SwerveWiring.ModuleWiring.builder()
              .driveId(40 + 3 * i)
              .steerId(41 + 3 * i)
              .encoderId(42 + 3 * i)
              .encoderOffset(Rotations.of(0))
              .driveInverted(false)
              .steerInverted(true)
              .encoderInverted(false)
              .build();
    }
    RobotConfig wired =
        RobotConfigBuilder.builder(Robots.SIM)
            .drivetrainWiring(
                Optional.of(
                    SwerveWiring.builder()
                        .canBus(CanBus.canivore("drivetrain"))
                        .gyroId(52)
                        .gyroYawScale(1.0)
                        .couplingRatio(3.5)
                        .modules(List.of(modules))
                        .systemcoreMount(OnboardIMU.MountOrientation.FLAT)
                        .build()))
            .build();
    container = containerFor(Constants.Mode.REAL, SensorErrors.NONE, wired);

    tick(SimTestSupport.loops(0.5));

    assertFalse(container.isNoHardwareAlertActive());
    assertThat(container.estimate().health().gyroConnected()).as("the Pigeon answers").isTrue();
  }

  private Pose2d[] shown(String path) {
    return (Pose2d[])
        telemetry.getLastValue(path, MockTelemetryBackend.LogStructArrayValue.class).value();
  }
}
