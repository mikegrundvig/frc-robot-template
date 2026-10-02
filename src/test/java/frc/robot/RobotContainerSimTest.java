package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Outcome;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Placement;
import frc.robot.game.Rebuilt;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.lib.wiring.CanBus;
import frc.robot.lib.wiring.SwerveWiring;
import frc.robot.sim.DriveSimulation;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.telemetry.MockTelemetryBackend;

/**
 * The whole robot as the simulator runs it: the real composition root, the real controller
 * bindings, the real field, a simulated gamepad, and the simulated drivetrain. If this passes, a
 * driver in the simulator GUI gets the same behavior.
 */
@SimulationTest
class RobotContainerSimTest {
  private static final FieldModel FIELD = RobotSim.FIELD;

  @RegisterExtension final RobotSim sim = new RobotSim();

  /** Turns the simulated robot by hand, where it stands: its gyro feels it. */
  private void turnByHand(double degrees) {
    DriveSimulation simulation = sim.simulation();
    Pose2d truth = simulation.truePose();
    simulation.place(
        new Pose2d(
            truth.getTranslation(), truth.getRotation().plus(Rotation2d.fromDegrees(degrees))));
  }

  private void pressBack() {
    sim.driver().setBackButton(true);
    sim.driver().notifyNewData();
    sim.loops(1);
    sim.driver().setBackButton(false);
    sim.driver().notifyNewData();
  }

  private double headingDegrees() {
    return sim.robot().drive().getPose().getRotation().getDegrees();
  }

  private void pushLeftStickForward() {
    RobotSim.setEnabled(true);
    sim.driver().setLeftY(-1.0); // gamepad Y is negative when pushed forward
    sim.driver().notifyNewData();
  }

  private void releaseStickAndDisable() {
    sim.driver().setLeftY(0.0);
    sim.driver().notifyNewData();
    RobotSim.setEnabled(false);
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
    sim.run(1.0);

    // Placed on its start; the cameras then refine that to within their noise.
    Pose2d pose = sim.robot().drive().getPose();
    Pose2d center = start("Center", Alliance.BLUE);
    assertThat(pose.getTranslation().getDistance(center.getTranslation())).isLessThan(0.03);
    assertThat(pose.getRotation().minus(center.getRotation()).getDegrees())
        .isCloseTo(0.0, within(0.5));
    assertThat(sim.robot().allianceStation()).isEqualTo("Blue 2");
    assertThat(sim.robot().placement().status()).isEqualTo(Placement.Status.READY);
    assertFalse(
        sim.robot().isNoHardwareAlertActive(), "the simulator is the drivetrain's hardware");
  }

  @Test
  void theBackButtonSaysTheRobotFacesDownfieldForTheDriversAlliance() {
    sim.build(RobotSim.WITHOUT_CAMERAS, SensorErrors.NONE);
    RobotSim.station(AllianceStationID.RED_2);
    sim.loops(2); // placed on red's center start, facing blue's wall
    turnByHand(30);
    sim.loops(1);
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
    RobotSim.station(AllianceStationID.RED_2);
    sim.run(1.0);
    turnByHand(30);
    sim.run(0.5);

    pressBack();
    double wrong = headingDegrees();
    sim.run(1.0);

    assertThat(Math.abs(wrong)).isCloseTo(180, within(0.5));
    assertThat(headingDegrees()).isCloseTo(-150, within(1.0));
    assertThat(
            sim.robot()
                .drive()
                .getPose()
                .getTranslation()
                .getDistance(sim.simulation().truePose().getTranslation()))
        .isLessThan(0.05);
  }

  @Test
  void itSaysWhenItIsReadyToScoreAndWhyNot() {
    String before = sim.robot().scoring();
    sim.run(1.0);
    String settled = sim.robot().scoring();
    boolean ready = sim.robot().readyToScore();
    sim.simulation().tilt(Math.toRadians(10), 0);
    sim.loops(2);

    assertThat(before).isEqualTo("Not ready: not started");
    assertThat(settled).isEqualTo("Ready to score");
    assertThat(ready).isTrue();
    assertThat(sim.robot().scoring()).isEqualTo("Not ready: the robot is tilted 10 degrees");
  }

  @Test
  void autonomousStartingHoldsTheCamerasOnlyWhenAsked() {
    var hold = sim.pitwall().tunables().requestChanges(Map.of("Vision/autoStartHoldSeconds", 2.0));
    sim.run(1.0);
    assertThat(hold.getNow(null)).isEqualTo(new Outcome(Outcome.Result.APPLIED, ""));
    int before = sim.robot().estimate().health().accepted();

    RobotSim.enable(RobotMode.AUTONOMOUS, true);
    sim.run(1.0);
    int held = sim.robot().estimate().health().accepted() - before;
    sim.run(1.5);

    assertThat(held).as("held for 2 s").isZero();
    assertThat(sim.robot().estimate().health().accepted() - before).as("then used").isPositive();
  }

  @Test
  void theDriverStationsLocationChoosesTheStart() {
    RobotSim.station(AllianceStationID.RED_1);

    sim.loops(1);

    assertEquals(start("Left", Alliance.RED), sim.robot().drive().getPose());
    assertThat(sim.robot().placement().describe()).isEqualTo("Ready on Left: 0 cm, 0° off");
  }

  @Test
  void theLeftStickDrivesAwayFromTheBlueDrivers() {
    Pose2d start = start("Center", Alliance.BLUE);
    pushLeftStickForward();

    sim.run(1.5);

    Pose2d pose = sim.robot().drive().getPose();
    assertTrue(pose.getX() > start.getX() + 2.0, "toward red's wall: now at x=" + pose.getX());
    assertEquals(start.getY(), pose.getY(), 0.05, "straight ahead");
  }

  /** The red drivers stand at the other end, so their forward is the field's -x. */
  @Test
  void theLeftStickDrivesAwayFromTheRedDrivers() {
    RobotSim.station(AllianceStationID.RED_2);
    Pose2d start = start("Center", Alliance.RED);
    pushLeftStickForward();

    sim.run(1.5);

    Pose2d pose = sim.robot().drive().getPose();
    assertTrue(pose.getX() < start.getX() - 2.0, "toward blue's wall: now at x=" + pose.getX());
    assertEquals(start.getY(), pose.getY(), 0.05, "straight ahead");
  }

  @Test
  void afterDrivingThePlacementCheckSaysHowFarAwayTheStartIs() {
    pushLeftStickForward();
    sim.run(1.0);
    releaseStickAndDisable();

    sim.loops(1);

    assertThat(sim.robot().placement().status()).isEqualTo(Placement.Status.NOT_ON_A_START);
    assertThat(sim.robot().placement().describe()).startsWith("Not on a start: Center is ");
  }

  /** In the pit or the simulator, the robot goes on the start chosen while it's disabled. */
  @Test
  void changingTheStationWhileDisabledMovesTheRobotButNeverWhileItDrives() {
    pushLeftStickForward();
    sim.loops(5);
    RobotSim.station(AllianceStationID.BLUE_3);
    sim.loops(5);
    assertThat(sim.robot().drive().getPose().getY())
        .as("still driving from the center start")
        .isCloseTo(start("Center", Alliance.BLUE).getY(), within(0.05));

    releaseStickAndDisable();
    sim.run(1.0); // coasts to a stop, then the drive team moves it
    RobotSim.station(AllianceStationID.BLUE_1);
    sim.loops(1);

    assertThat(
            sim.robot()
                .drive()
                .getPose()
                .minus(start("Left", Alliance.BLUE))
                .getTranslation()
                .getNorm())
        .isLessThan(0.001);
  }

  @Test
  void zonesFollowTheRobot() {
    sim.loops(1);

    assertThat(sim.robot().inZone(Rebuilt.ALLIANCE_ZONE)).as("on a start, in its zone").isTrue();
    assertThat(sim.robot().inZone(Rebuilt.TRENCH)).isFalse();
  }

  /** From the left start, straight ahead passes under the left trench. */
  @Test
  void theTrenchTriggerTurnsOnBeforeTheRobotReachesIt() {
    RobotSim.station(AllianceStationID.BLUE_1);
    sim.loops(1);
    pushLeftStickForward();

    boolean onBeforeArriving = false;
    for (int i = 0; i < RobotLoop.loops(2.0); i++) {
      sim.loops(1);
      boolean under =
          Rebuilt.TRENCH.contains(
              FIELD, Alliance.BLUE, sim.robot().drive().getPose().getTranslation());
      if (sim.robot().inZone(Rebuilt.TRENCH) && !under) {
        onBeforeArriving = true;
      }
      if (under) {
        break;
      }
    }

    assertThat(onBeforeArriving).isTrue();
    assertThat(sim.robot().inZone(Rebuilt.TRENCH)).as("and on while under it").isTrue();
  }

  @Test
  void untilTheAllianceIsKnownTheRobotWaitsMidfieldAndSaysSo() {
    RobotSim.station(AllianceStationID.UNKNOWN);

    sim.loops(1);

    assertThat(sim.robot().drive().getPose().getX()).isCloseTo(FIELD.length() / 2, within(1e-9));
    assertThat(sim.robot().allianceStation()).isEqualTo("Unknown");
    assertThat(sim.robot().placement().describe()).isEqualTo("Alliance unknown");
  }

  @Test
  void theFieldViewShowsTheRobotItsStartAndTheTargets() {
    pushLeftStickForward();

    sim.run(1.0);

    assertEquals(sim.robot().drive().getPose(), shown("/Field/Robot")[0]);
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
    sim.build(SensorErrors.TYPICAL);
    sim.loops(1);
    pushLeftStickForward();

    sim.run(1.5);

    DriveSimulation simulation = sim.simulation();
    Pose2d truth = simulation.truePose();
    Pose2d believed = sim.robot().drive().getPose();
    assertThat(believed.getTranslation().getDistance(truth.getTranslation()))
        .as("6 m driven on worn wheels")
        .isLessThan(0.10);
    assertThat(sim.robot().estimate().health().accepted()).as("camera measurements").isPositive();
    assertEquals(truth, shown("/Field/Truth")[0]);
  }

  @Test
  void replayRunsWithoutTheSimulator() {
    sim.build(Constants.Mode.REPLAY, Robots.SIM, SensorErrors.NONE);

    sim.loops(5);

    assertEquals(
        start("Center", Alliance.BLUE),
        sim.robot().drive().getPose(),
        "on its start, as the log's Driver Station says; nothing moves");
    assertFalse(sim.robot().isNoHardwareAlertActive(), "replay's hardware is the log");
    assertThat(sim.robot().simulation()).as("no physics in a replay").isNull();
  }

  /**
   * Until a team writes the motor-controller code for its modules, a real robot holds still and
   * says why, rather than guessing at motors.
   */
  @Test
  void aRealRobotWithoutWiringHoldsStillAndSaysSo() {
    sim.build(Constants.Mode.REAL, Robots.SIM, SensorErrors.NONE);
    sim.loops(1);
    pushLeftStickForward();

    sim.run(0.5);

    assertTrue(sim.robot().isNoHardwareAlertActive());
    assertEquals(start("Center", Alliance.BLUE), sim.robot().drive().getPose(), "nothing moved");
  }

  /**
   * A robot whose configuration has its wiring drives CTRE hardware: here Phoenix's simulated
   * devices, which answer like real ones.
   */
  @Test
  void aRealRobotWithWiringReadsItsCtreHardware() {
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
    sim.build(Constants.Mode.REAL, wired, SensorErrors.NONE);

    sim.run(0.5);

    assertFalse(sim.robot().isNoHardwareAlertActive());
    assertThat(sim.robot().estimate().health().gyroConnected()).as("the Pigeon answers").isTrue();
  }

  private Pose2d[] shown(String path) {
    return (Pose2d[])
        sim.telemetry().getLastValue(path, MockTelemetryBackend.LogStructArrayValue.class).value();
  }

  /** The drive check runs from the Pitwall page, on the robot's own scheduler. */
  @Test
  void theDriveCheckIsAnActionOnThePage() {
    RobotSim.enable(RobotMode.UTILITY, true);
    var started = sim.pitwall().actions().requestStart(RobotContainer.DRIVE_CHECK);
    sim.loops(25);

    assertThat(started.getNow(null)).isEqualTo(new Outcome(Outcome.Result.APPLIED, ""));
    assertThat(sim.pitwall().actions().view())
        .filteredOn(action -> action.name().equals(RobotContainer.DRIVE_CHECK))
        .singleElement()
        .extracting(Actions.State::phase)
        .isEqualTo(Actions.Phase.RUNNING);
    assertThat(sim.robot().drive().getRobotRelativeVelocities().vx)
        .as("driving forward slowly")
        .isBetween(0.4, 0.6);
  }
}
