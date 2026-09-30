package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Robots;
import frc.robot.SimTestSupport;
import frc.robot.sim.DriveSimHarness;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/** Does the drivetrain go where the driver points it? */
class DriveBasicsTest {
  private DriveSimHarness harness;

  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
  }

  @BeforeEach
  void setUp() {
    SimTestSupport.pauseTimeAndDisable();
    harness = new DriveSimHarness(Robots.SIM);
  }

  @AfterEach
  void tearDown() {
    harness.close();
    SimTestSupport.resumeTime();
  }

  @Test
  void drivesStraightForward() {
    harness.drive(2.0, 1.0, 0.0, 0.0);

    Pose2d pose = harness.drive.getPose();
    assertTrue(pose.getX() > 5.0, "moved " + pose.getX() + " m forward");
    assertEquals(0.0, pose.getY(), 0.02, "no sideways drift");
    assertEquals(0.0, pose.getRotation().getDegrees(), 0.5, "no turning");
    double topSpeed = Robots.SIM.drivetrain().limits().maxLinearSpeed().baseUnitMagnitude();
    assertEquals(topSpeed, harness.drive.getRobotRelativeVelocities().vx, 0.05, "full stick");
  }

  @Test
  void spinsInPlace() {
    harness.drive(1.0, 0.0, 0.0, 1.0);

    assertEquals(0.0, harness.drive.getPose().getTranslation().getNorm(), 0.05, "stays put");
    assertTrue(harness.drive.getRobotRelativeVelocities().omega > 5.0, "turning at full rate");
  }

  @Test
  void drivesStraightDownfieldWhileSpinning() {
    // Field-relative driving: the robot spins, but its path should still run along +x. Broken
    // field-relative driving spirals off by meters. The simulated steering updates once per loop,
    // slower than real motor controllers, so allow some drift.
    harness.drive(3.0, 0.7, 0.0, 0.5);

    Pose2d pose = harness.drive.getPose();
    assertTrue(pose.getX() > 2.0, "made progress downfield: " + pose.getX() + " m");
    assertEquals(0.0, pose.getY(), 0.25, "path stayed straight");
  }

  @Test
  void drivesForwardForAWhileThenStops() {
    harness.run(harness.drive.driveForward(0.5, 1.0), 1.5);

    Pose2d pose = harness.drive.getPose();
    assertEquals(0.5, pose.getX(), 0.05, "half a meter at 0.5 m/s for a second");
    assertEquals(0.0, pose.getY(), 0.02, "straight");
    assertEquals(0.0, harness.drive.getRobotRelativeVelocities().vx, 0.05, "stopped");
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
                harness.scheduler,
                DriveSimHarness.estimatorAt(Pose2d.ZERO)));
  }

  /** An IMU that sends values that aren't numbers counts as lost, and the robot program runs on. */
  @Test
  void anImuThatSendsNaNCountsAsLost() {
    GyroIO sendingNaN =
        new GyroIO() {
          @Override
          public void updateInputs(GyroIOInputs inputs) {
            inputs.connected = true;
            inputs.yaw = new Rotation2d(Double.NaN);
            inputs.pitchRad = Double.NaN;
            inputs.rollRad = Double.NaN;
            inputs.odometryTimestampsNanos = new long[] {1};
            inputs.odometryYawsRad = new double[] {Double.NaN};
          }
        };
    ModuleIO[] modules = {
      new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}
    };
    try (Drive drive =
        new Drive(
            "NaNImu",
            sendingNaN,
            modules,
            Robots.SIM.body(),
            Robots.SIM::drivetrain,
            harness.scheduler,
            DriveSimHarness.estimatorAt(Pose2d.ZERO))) {
      drive.periodic(true);

      assertTrue(drive.isGyroDisconnectedAlertActive());
      assertEquals(0.0, drive.pitchRadians(), "no IMU, no tilt");
      assertTrue(Double.isFinite(drive.getPose().getX()));
    }
  }

  /** A device that missed its configuration runs without current limits: the pit must know. */
  @Test
  void aModuleWhoseDevicesMissedTheirConfigurationRaisesItsAlert() {
    ModuleIO missed =
        new ModuleIO() {
          @Override
          public void updateInputs(ModuleIOInputs inputs) {
            inputs.unconfigured = "drive motor 1";
          }
        };
    ModuleIO[] modules = {missed, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}};
    try (Drive drive =
        new Drive(
            "Unconfigured",
            new GyroIO() {},
            modules,
            Robots.SIM.body(),
            Robots.SIM::drivetrain,
            harness.scheduler,
            DriveSimHarness.estimatorAt(Pose2d.ZERO))) {
      drive.periodic(false);

      assertTrue(drive.isModuleUnconfiguredAlertActive(0));
      assertFalse(drive.isModuleUnconfiguredAlertActive(1));
    }
  }
}
