package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;

import frc.robot.RobotConfigBuilder;
import frc.robot.Robots;
import frc.robot.SimTestSupport;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.config.SwerveConfigBuilder;
import frc.robot.lib.config.SwerveConfigLimitsBuilder;
import frc.robot.lib.swerve.SwerveSim;
import frc.robot.simlab.Maneuvers;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Test plans: realistic, punishing driver maneuvers run through the drive code and physics, with
 * pass/fail limits on the things that lose matches: brownouts, wheel slip, and odometry that no
 * longer knows where the robot is.
 */
class DriveManeuverTest {
  /** Systemcore disables outputs at 6.3 V (configurable); leave margin for an older battery. */
  private static final double MIN_BATTERY_VOLTS = 7.5;

  /** A wheel sliding more than this share of the time wastes acceleration and wears tread. */
  private static final double MAX_SLIP_FRACTION = 0.05;

  /** Odometry error after the whole maneuver, before any vision correction. */
  private static final double MAX_ODOMETRY_ERROR_METERS = 0.10;

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
    SimTestSupport.resumeTime();
  }

  @Test
  void upDownSprintsStayWithinBatteryAndTractionLimits() {
    try (var harness = new DriveSimHarness(Robots.SIM, SwerveSim.Battery.TYPICAL)) {
      var report = harness.run(Maneuvers.UP_DOWN_SPRINTS);

      assertTrue(report.minBatteryVolts() >= MIN_BATTERY_VOLTS, "brownout risk" + report);
      assertTrue(report.worstSlipFraction() <= MAX_SLIP_FRACTION, "wheels slipping" + report);
      assertTrue(report.minNormalForceNewtons() > 0, "a wheel lifted off the carpet" + report);
      assertTrue(
          report.odometryErrorMeters() <= MAX_ODOMETRY_ERROR_METERS, "odometry lost" + report);
    }
  }

  /**
   * Checks the checker: the same sprints with current limits effectively removed must fail. If this
   * test ever passes the bad robot, the test plan above cannot be trusted.
   */
  @Test
  void theTestPlanCatchesARobotWithoutCurrentLimits() {
    SwerveConfig good = Robots.SIM.drivetrain();
    SwerveConfig unlimited =
        SwerveConfigBuilder.builder(good)
            .limits(
                SwerveConfigLimitsBuilder.builder(good.limits())
                    .driveStatorCurrent(Amps.of(1000))
                    .driveSupplyCurrent(Amps.of(1000))
                    .build())
            .build();
    try (var harness =
        new DriveSimHarness(
            RobotConfigBuilder.builder(Robots.SIM).drivetrain(unlimited).build(),
            SwerveSim.Battery.TYPICAL)) {
      var report =
          harness.run(Maneuvers.UP_DOWN_SPRINTS.named("up-down-sprints-no-current-limits"));

      assertTrue(
          report.minBatteryVolts() < MIN_BATTERY_VOLTS
              || report.worstSlipFraction() > MAX_SLIP_FRACTION,
          "expected brownout or slip without current limits" + report);
    }
  }
}
