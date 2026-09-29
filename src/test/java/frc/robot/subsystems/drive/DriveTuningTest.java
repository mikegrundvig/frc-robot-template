package frc.robot.subsystems.drive;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;

import frc.robot.Robots;
import frc.robot.SimTestSupport;
import frc.robot.lib.swerve.SwerveSim;
import frc.robot.simlab.DriveScenario;
import frc.robot.simlab.Maneuver;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Values tuned while the robot runs reach its motor controllers, the loop they change. */
class DriveTuningTest {
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
  void aTunedCurrentLimitReachesTheMotorControllers() {
    double configured = Robots.SIM.drivetrain().limits().driveStatorCurrent().in(Amps);
    var launch = new Maneuver("full-stick-launch", 1.0, t -> new double[] {1.0, 0.0, 0.0});

    var asCommitted = harness.run(launch);
    harness.drivetrain.set(DriveScenario.withDriveStatorLimit(Robots.SIM.drivetrain(), 15));
    // The drive reads its configuration after the loop's physics step, so a change takes one
    // loop to reach the motor controllers, as on the robot. Measure from the loop after.
    harness.tick();
    var asTuned = harness.run(launch.named("full-stick-launch-15A"));

    assertThat(asCommitted.peakDriveStatorAmps()).isCloseTo(configured, within(0.5));
    assertThat(asTuned.peakDriveStatorAmps()).isLessThanOrEqualTo(15.001);
  }
}
