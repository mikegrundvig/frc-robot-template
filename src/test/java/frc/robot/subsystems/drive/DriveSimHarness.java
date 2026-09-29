package frc.robot.subsystems.drive;

import frc.robot.RobotConfig;
import frc.robot.SimTestSupport;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.swerve.SwerveSim;
import frc.robot.simlab.Maneuver;
import frc.robot.simlab.ManeuverRecorder;
import frc.robot.simlab.ManeuverReport;
import java.nio.file.Path;
import java.util.concurrent.atomic.AtomicReference;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;

/**
 * Runs scripted maneuvers through the full robot path: simulated Driver Station, the command
 * scheduler, and the joystick drive command, against the physics simulation. Uses the shared HAL,
 * so one harness at a time; for many runs in parallel use {@link frc.robot.simlab.ManeuverRunner}.
 *
 * <p>Every run writes {@code build/maneuvers/<name>.wpilog} for AdvantageScope.
 */
final class DriveSimHarness implements AutoCloseable {
  final SwerveSim sim;
  final Drive drive;
  final Scheduler scheduler = Scheduler.createIndependentScheduler();

  /** The drivetrain configuration the drive reads each loop, as tuning would supply it. */
  final AtomicReference<SwerveConfig> drivetrain;

  DriveSimHarness(RobotConfig robot, SwerveSim.Battery battery) {
    drivetrain = new AtomicReference<>(robot.drivetrain());
    sim = new SwerveSim(robot.body(), robot.drivetrain(), battery, new Pose2d());
    drive =
        new Drive(
            "Drive",
            new GyroIOSim(sim),
            ModuleIOSim.of(sim),
            robot.body(),
            drivetrain::get,
            scheduler);
    drive.resetPose(sim.truePose());
  }

  /** One robot loop, as the robot runs it: physics, the drive reads sensors, commands run. */
  void tick() {
    SimTestSupport.stepLoop();
    boolean enabled = RobotState.isEnabled();
    sim.update(SimTestSupport.LOOP_PERIOD_S, enabled);
    drive.periodic(enabled);
    scheduler.run();
  }

  ManeuverReport run(Maneuver maneuver) {
    SimTestSupport.setEnabled(true);
    double[] sticks = new double[3];
    Command driving = drive.joystickDrive(() -> sticks[0], () -> sticks[1], () -> sticks[2]);
    scheduler.schedule(driving);

    Path logPath = Path.of("build", "maneuvers", maneuver.name() + ".wpilog");
    ManeuverReport report;
    try (ManeuverRecorder recorder =
        new ManeuverRecorder(sim, drive, maneuver, SimTestSupport.LOOP_PERIOD_S, logPath)) {
      int loops = SimTestSupport.loops(maneuver.durationSec());
      for (int i = 0; i < loops; i++) {
        double time = i * SimTestSupport.LOOP_PERIOD_S;
        System.arraycopy(maneuver.script().sticks(time), 0, sticks, 0, 3);
        tick();
        recorder.sample(time, SimTestSupport.LOOP_PERIOD_S, sticks);
      }
      report = recorder.report();
    }
    scheduler.cancel(driving);
    System.out.println(report);
    return report;
  }

  @Override
  public void close() {
    drive.close();
  }
}
