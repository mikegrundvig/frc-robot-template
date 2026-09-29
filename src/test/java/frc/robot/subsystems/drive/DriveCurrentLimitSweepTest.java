package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;

import frc.robot.Robots;
import frc.robot.simlab.DriveScenario;
import frc.robot.simlab.ManeuverReport;
import frc.robot.simlab.ManeuverRunner;
import frc.robot.simlab.Maneuvers;
import frc.robot.simlab.Parallel;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The configured drive current limit must be robustly tuned: it passes the up-down sprints in every
 * robustness scenario (worn to fresh tread, tired to strong battery, mass +/- 5 kg, a higher center
 * of gravity), and one step higher fails at least one of them, so no acceleration is left unused.
 */
class DriveCurrentLimitSweepTest {
  private static final double STEP_AMPS = 2.5;
  private static final double MAX_SLIP_FRACTION = 0.01;
  private static final double MIN_BATTERY_VOLTS = 7.5;

  private record Job(double amps, DriveScenario scenario) {}

  private static boolean passes(ManeuverReport r) {
    return r.worstSlipFraction() <= MAX_SLIP_FRACTION && r.minBatteryVolts() >= MIN_BATTERY_VOLTS;
  }

  @Test
  void configuredStatorLimitIsRobustlyTuned() {
    double configured = Robots.SIM.drivetrain().limits().driveStatorCurrent().in(Amps);
    List<Job> jobs = new ArrayList<>();
    for (double amps : new double[] {configured, configured + STEP_AMPS}) {
      for (DriveScenario s : DriveScenario.robustnessSpread(Robots.SIM)) {
        jobs.add(new Job(amps, s));
      }
    }

    List<ManeuverReport> reports =
        Parallel.map(
            jobs,
            job ->
                ManeuverRunner.run(
                    job.scenario().withDriveStatorLimit(job.amps()),
                    Maneuvers.UP_DOWN_SPRINTS.named(job.scenario().name()),
                    null));

    List<String> failuresAtConfigured = new ArrayList<>();
    boolean stepUpFailsSomewhere = false;
    for (int i = 0; i < jobs.size(); i++) {
      Job job = jobs.get(i);
      ManeuverReport r = reports.get(i);
      if (job.amps() == configured && !passes(r)) {
        failuresAtConfigured.add(r.toString());
      } else if (job.amps() != configured && !passes(r)) {
        stepUpFailsSomewhere = true;
      }
    }

    assertTrue(
        failuresAtConfigured.isEmpty(),
        configured
            + " A fails in some scenarios; lower it:"
            + String.join("", failuresAtConfigured));
    assertTrue(
        stepUpFailsSomewhere,
        (configured + STEP_AMPS) + " A passes every scenario too; the limit can go higher");
  }
}
