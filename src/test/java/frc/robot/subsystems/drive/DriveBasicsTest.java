package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.Robots;
import frc.robot.SimTestSupport;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;

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
}
