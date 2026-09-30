package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.lib.field.FieldModel;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.subsystems.drive.DriveSimulation;
import frc.robot.subsystems.drive.SensorErrors;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.GamepadSim;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * The whole robot in simulation, with PhotonVision's simulated cameras rendering the field from
 * where the robot really is: localization end to end, and the pre-event check that finds a camera
 * knocked out of calibration.
 */
class VisionSimTest {
  private static final FieldModel FIELD = Rebuilt.field(Robots.SIM.body());
  private static final double[][] ROUTE = {
    {1.5, -0.5, 0, 0},
    {1.0, 0, 0.7, 0},
    {1.5, 0, 0, 0.6},
    {2.0, 0.4, -0.5, 0.2},
    {1.5, -0.3, -0.3, -0.4},
    {1.0, 0, 0, 0}
  };

  private @Nullable RobotContainer container;
  private GamepadSim driver = new GamepadSim(0);
  private Scheduler scheduler = Scheduler.createIndependentScheduler();

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

  private void build(SensorErrors errors) {
    if (container != null) {
      container.close();
    }
    SimTestSupport.pauseTimeAndDisable();
    DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_2);
    DriverStationSim.notifyNewData();
    scheduler = Scheduler.createIndependentScheduler();
    container =
        new RobotContainer(
            Constants.Mode.SIM,
            scheduler,
            Robots.SIM.body(),
            Robots.SIM::drivetrain,
            Robots.SIM.cameras(),
            FIELD,
            errors);
    driver = new GamepadSim(0);
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

  private double error() {
    DriveSimulation simulation = Objects.requireNonNull(robot().simulation());
    Pose2d truth = simulation.truePose();
    return robot().drive().getPose().getTranslation().getDistance(truth.getTranslation());
  }

  private double drive() {
    tick(SimTestSupport.loops(1.0)); // placed, and the cameras settle
    SimTestSupport.setEnabled(true);
    double worst = 0;
    for (double[] leg : ROUTE) {
      driver.setLeftY(-leg[1]);
      driver.setLeftX(-leg[2]);
      driver.setRightX(-leg[3]);
      driver.notifyNewData();
      for (int i = 0; i < SimTestSupport.loops(leg[0]); i++) {
        tick(1);
        worst = Math.max(worst, error());
      }
    }
    return worst;
  }

  /** How far a camera's tags land from their real places, against the worst of the others. */
  private double standsOut(String camera) {
    double others = 0;
    for (var config : Robots.SIM.cameras()) {
      if (!config.name().equals(camera)) {
        others = Math.max(others, robot().vision().meanTagResidualMeters(config.name()));
      }
    }
    return robot().vision().meanTagResidualMeters(camera) / others;
  }

  /**
   * A season-worn robot driving a minute's worth of moves, four cameras on it: the design's targets
   * are 8 cm while moving and 3 cm standing still.
   */
  @Test
  void theCamerasHoldTheEstimateToTheTruthAsTheRobotDrives() {
    build(SensorErrors.TYPICAL);

    double worst = drive();

    assertThat(worst).as("worst while driving").isLessThan(0.08);
    assertThat(error()).as("stopped").isLessThan(0.03);
    LocalizationEstimate estimate = robot().estimate();
    assertThat(estimate.mode()).isEqualTo(LocalizationEstimate.Mode.TRACKING);
    LocalizationEstimate.Health health = estimate.health();
    assertThat(health.accepted()).isGreaterThan(19 * health.rejected()).isGreaterThan(500);
  }

  /** 2026's calibration drifted all season; a camera 4 in low stands out before a match. */
  @Test
  void aCameraKnockedLowStandsOutInItsTagResiduals() {
    build(
        SensorErrors.TYPICAL.withCameraMountError(
            "front-left", new Transform3d(0, 0, -0.1016, Rotation3d.ZERO)));

    double worst = drive();

    assertThat(standsOut("front-left")).isGreaterThan(2.5);
    assertThat(worst).as("the other cameras and the gate keep the estimate").isLessThan(0.08);
  }

  /** On a ramp, nose up and leaning: the images are solved with the robot tilted as it is. */
  @Test
  void aTiltedRobotStillSeesWhereItIs() {
    build(SensorErrors.TYPICAL);
    tick(SimTestSupport.loops(1.0));
    int before = robot().estimate().health().accepted();

    Objects.requireNonNull(robot().simulation()).tilt(Math.toRadians(-8), Math.toRadians(3));
    tick(SimTestSupport.loops(1.0));

    assertThat(robot().estimate().health().accepted() - before).isGreaterThan(50);
    assertThat(error()).isLessThan(0.03);
  }

  @Test
  void aCameraTurnedTwoDegreesStandsOutToo() {
    build(
        SensorErrors.TYPICAL.withCameraMountError(
            "front-left", new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.toRadians(2)))));

    drive();

    assertThat(standsOut("front-left")).isGreaterThan(1.4);
  }
}
