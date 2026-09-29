package frc.robot.simlab;

import frc.robot.lib.swerve.SwerveSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import java.nio.file.Path;
import java.util.concurrent.atomic.AtomicLong;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;

/**
 * Runs one maneuver on one simulated robot, with no global state: no HAL, no shared clock, no
 * shared scheduler. Safe to call from many threads at once.
 */
public final class ManeuverRunner {
  public static final double LOOP_PERIOD_S = 0.02;
  private static final AtomicLong RUN_IDS = new AtomicLong();

  private ManeuverRunner() {}

  /**
   * @param logPath where to write a .wpilog of the run, or null for none
   */
  public static ManeuverReport run(
      DriveScenario scenario, Maneuver maneuver, @Nullable Path logPath) {
    SwerveSim sim =
        new SwerveSim(scenario.body(), scenario.drivetrain(), scenario.battery(), new Pose2d());
    try (Drive drive =
        new Drive(
            "SimLab" + RUN_IDS.incrementAndGet(),
            new GyroIOSim(sim),
            ModuleIOSim.of(sim),
            scenario.body(),
            scenario::drivetrain,
            Scheduler.createIndependentScheduler())) {
      drive.resetPose(sim.truePose());
      try (ManeuverRecorder recorder =
          new ManeuverRecorder(sim, drive, maneuver, LOOP_PERIOD_S, logPath)) {
        int loops = (int) Math.ceil(maneuver.durationSec() / LOOP_PERIOD_S);
        for (int i = 0; i < loops; i++) {
          double time = i * LOOP_PERIOD_S;
          double[] sticks = maneuver.script().sticks(time);
          // One robot loop: physics, sensors, then the driver's command.
          sim.update(LOOP_PERIOD_S, true);
          drive.periodic(true);
          drive.driveWithSticks(sticks[0], sticks[1], sticks[2]);
          recorder.sample(time, LOOP_PERIOD_S, sticks);
        }
        return recorder.report();
      }
    }
  }
}
