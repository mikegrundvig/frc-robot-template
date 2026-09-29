package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Robots;
import frc.robot.SimTestSupport;
import frc.robot.lib.swerve.SwerveSim;
import frc.robot.simlab.Maneuver;
import frc.robot.simlab.Maneuvers;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;

/** Does the drivetrain go where the driver points it, and does it know where it went? */
class DriveBasicsTest {
  private DriveSimHarness harness;

  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
  }

  @BeforeEach
  void setUp() {
    SimTestSupport.pauseTimeAndDisable();
    harness = new DriveSimHarness(Robots.SIM, SwerveSim.Battery.TYPICAL);
  }

  @AfterEach
  void tearDown() {
    harness.close();
    SimTestSupport.resumeTime();
  }

  @Test
  void drivesStraightAndOdometryAgreesWithTruth() {
    var report =
        harness.run(new Maneuver("straight-half-stick", 2.0, t -> new double[] {0.5, 0.0, 0.0}));

    Pose2d truth = harness.sim.truePose();
    assertTrue(truth.getX() > 1.5, "moved " + truth.getX() + " m forward");
    assertEquals(0.0, truth.getY(), 0.02, "no sideways drift");
    assertEquals(0.0, truth.getRotation().getDegrees(), 0.5, "no turning");
    assertEquals(0.0, report.odometryErrorMeters(), 0.02, "no slip, so odometry is accurate");
    assertEquals(
        harness.sim.trueFieldVelocities().vx,
        harness.drive.getRobotRelativeVelocities().vx,
        0.02,
        "the wheels measure the robot's speed");
  }

  @Test
  void needsExactlyFourModules() {
    ModuleIO[] three = {new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}};

    assertThrows(
        IllegalArgumentException.class,
        () ->
            new Drive(
                "ThreeModules",
                new GyroIO() {},
                three,
                Robots.SIM.body(),
                Robots.SIM::drivetrain,
                harness.scheduler));
  }

  @Test
  void spinsInPlace() {
    harness.run(new Maneuver("spin-in-place", 2.0, t -> new double[] {0.0, 0.0, 1.0}));

    Pose2d truth = harness.sim.truePose();
    assertEquals(0.0, truth.getTranslation().getNorm(), 0.05, "stays on the spot");
    assertTrue(harness.sim.trueFieldVelocities().omega > 5.0, "turning at full rate");
  }

  @Test
  void drivesStraightDownfieldWhileSpinning() {
    // Field-relative driving: the robot spins, but its path should still run straight along +x.
    harness.run(Maneuvers.TRANSLATE_WHILE_SPINNING);

    Pose2d truth = harness.sim.truePose();
    assertTrue(truth.getX() > 2.0, "made progress downfield: " + truth.getX() + " m");
    assertEquals(0.0, truth.getY(), 0.05, "path stayed straight (skew compensation)");
  }

  @Test
  void keepsDrivingOnThreeModulesWhenOneDisconnects() {
    harness.sim.setModuleConnected(0, false);

    harness.run(new Maneuver("front-left-module-lost", 2.0, t -> new double[] {0.5, 0.0, 0.0}));

    assertTrue(harness.drive.isModuleDisconnectedAlertActive(0), "driver is told");
    assertTrue(harness.sim.truePose().getX() > 1.0, "and can still drive");
  }
}
