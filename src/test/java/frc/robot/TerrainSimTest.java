package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.game.Rebuilt;
import frc.robot.lib.field.FieldModel;
import frc.robot.sim.DriveSimulation;
import frc.robot.sim.SensorErrors;
import java.util.List;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * Over REBUILT's bump with no cameras: the terrain tilts the simulated robot, odometry counts the
 * wheels' travel along the ramps instead of throwing it away, and the crest becomes a landmark.
 */
class TerrainSimTest {
  private static final FieldModel FIELD = Rebuilt.field(Robots.SIM.body());

  /** Facing the blue left bump from the alliance zone, on its middle. */
  private static final Pose2d BEFORE_THE_BUMP = new Pose2d(3.0, 5.5, Rotation2d.ZERO);

  private @Nullable RobotContainer container;
  private Scheduler scheduler = Scheduler.createIndependentScheduler();
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
    scheduler = Scheduler.createIndependentScheduler();
    driver = new GamepadSim(0);
    container =
        new RobotContainer(
            Constants.Mode.SIM,
            scheduler,
            RobotConfigBuilder.builder(Robots.SIM).cameras(List.of()).build(),
            Robots.SIM::drivetrain,
            FIELD,
            SensorErrors.NONE);
  }

  @AfterEach
  void tearDown() {
    driver.setLeftY(0);
    driver.notifyNewData();
    robot().close();
    TelemetryRegistry.reset();
    TunableRegistry.reset();
    SimTestSupport.resumeTime();
  }

  private RobotContainer robot() {
    return Objects.requireNonNull(container);
  }

  private DriveSimulation sim() {
    return Objects.requireNonNull(robot().simulation());
  }

  private void tick(int loops) {
    for (int i = 0; i < loops; i++) {
      SimTestSupport.stepLoop();
      robot().periodic();
      scheduler.run();
    }
  }

  /**
   * Sets the robot before the bump, the estimate agreeing, knocks it back as a hit does (felt, but
   * not how far), then drives it over at 1 m/s.
   */
  private void crossTheBump(Transform2d hit) {
    tick(1);
    sim().place(BEFORE_THE_BUMP);
    robot().localization().resetPose(BEFORE_THE_BUMP);
    tick(5);
    if (!hit.equals(Transform2d.ZERO)) {
      sim().hit(hit);
    }
    SimTestSupport.setEnabled(true);
    driver.setLeftY(-0.6);
    driver.notifyNewData();
    tick(SimTestSupport.loops(4.0));
    driver.setLeftY(0);
    driver.notifyNewData();
    tick(SimTestSupport.loops(0.5));
  }

  private double error() {
    return robot()
        .drive()
        .getPose()
        .getTranslation()
        .getDistance(sim().truePose().getTranslation());
  }

  @Test
  void theRampsTiltTheRobotAndOdometryKeepsItsTravel() {
    crossTheBump(Transform2d.ZERO);

    assertThat(sim().truePose().getX()).as("over the bump").isGreaterThan(5.2);
    assertThat(robot().localization().terrainCrossings()).isEqualTo(1);
    assertThat(error()).as("after crossing, no cameras").isLessThan(0.03);
  }

  /** Knocked back 30 cm before the bump: with no cameras, only the crest can say where it is. */
  @Test
  void theCrestPutsAKnockedRobotBackOnTheLine() {
    crossTheBump(new Transform2d(-0.3, 0, Rotation2d.ZERO));

    assertThat(robot().localization().landmarksAccepted()).isEqualTo(1);
    assertThat(error()).as("after the crest, no cameras").isLessThan(0.05);
  }
}
