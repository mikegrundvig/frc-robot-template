package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.sim.DriveSimHarness;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/** Does the drivetrain go where the driver points it? */
@ExtendWith(RobotSim.class)
class DriveBasicsTest {
  private DriveSimHarness harness;

  @BeforeEach
  void setUp() {
    harness = new DriveSimHarness(Robots.SIM);
  }

  @AfterEach
  void tearDown() {
    harness.close();
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
    // field-relative driving spirals off by meters. The wheels get new angles once a loop and take
    // time to reach them, so allow some drift.
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
    assertThrows(
        IllegalArgumentException.class,
        () -> hardware(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}));
  }

  /**
   * An IMU that sends values that aren't numbers is reported as it read, for localization to judge
   * (it counts as lost there), and the drivetrain runs on.
   */
  @Test
  void anImuThatSendsNaNIsReportedAsUnusable() {
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
    try (Drive drive = drive("NaNImu", hardware(sendingNaN, fourModules()))) {
      drive.periodic(true);

      assertTrue(drive.readings().gyro().reporting());
      assertFalse(drive.readings().gyro().usable());
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
    DriveHardware hardware =
        hardware(new GyroIO() {}, missed, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
    try (Drive drive = drive("Unconfigured", hardware)) {
      drive.periodic(false);

      assertTrue(drive.isModuleUnconfiguredAlertActive(0));
      assertFalse(drive.isModuleUnconfiguredAlertActive(1));
    }
  }

  /**
   * Wheels that report connected but send no readings have lost what reads them (on a robot, the
   * odometry thread): the pose would freeze, so the pit must know.
   */
  @Test
  void wheelsThatStopSendingReadingsRaiseAnAlert() {
    ModuleIO[] connectedButSilent = new ModuleIO[4];
    for (int i = 0; i < 4; i++) {
      connectedButSilent[i] =
          new ModuleIO() {
            @Override
            public void updateInputs(ModuleIOInputs inputs) {
              inputs.driveConnected = true;
              inputs.steerConnected = true;
            }
          };
    }
    try (Drive drive = drive("Silent", hardware(new GyroIO() {}, connectedButSilent))) {
      drive.periodic(true);
      boolean afterOneLoop = drive.isOdometryStoppedAlertActive();
      for (int i = 0; i < 10; i++) {
        drive.periodic(true);
      }

      assertFalse(afterOneLoop, "a loop without readings can happen");
      assertTrue(drive.isOdometryStoppedAlertActive());
    }
  }

  private Drive drive(String name, DriveHardware hardware) {
    return new Drive(
        name,
        hardware,
        Robots.SIM.definition(),
        Robots.SIM::drivetrain,
        harness.scheduler,
        () -> Pose2d.ZERO);
  }

  private static ModuleIO[] fourModules() {
    return new ModuleIO[] {
      new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}
    };
  }

  private static DriveHardware hardware(GyroIO gyro, ModuleIO... modules) {
    return DriveHardware.builder()
        .modules(List.of(modules))
        .gyro(gyro)
        .backupImu(new GyroIO() {})
        .odometryLock(new ReentrantLock())
        .build();
  }
}
