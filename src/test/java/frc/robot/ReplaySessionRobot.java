package frc.robot;

import com.michaelgrundvig.frc.pitwall.experiment.Run;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.Map;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;

/**
 * The template's robot program with a scripted session on top, for {@link ReplayEndToEndTest}. In
 * the simulator it changes things the way people do in the pit: it picks the red alliance's station
 * 1 (which puts the robot on that start), enables utility mode, changes a configuration gain from
 * Pitwall's page (over HTTP), changes a WPILib tunable, runs an action that drives the robot under
 * a trench, and runs the steering's experiment, keeping it going as the page would. It records what
 * those affect, then stops. Replayed, it only records, and the replay must match.
 */
final class ReplaySessionRobot extends Robot {
  /** How many loops the recorded session runs, about five and a half seconds. */
  static final int LOOPS = 280;

  static final String GAIN = "Session/gain";

  private final TunableDouble gain = TunableDouble.create(1.0);
  private final HttpClient http = HttpClient.newHttpClient();
  private int loop;

  ReplaySessionRobot() {
    TunableRegistry.publish(GAIN, gain);
  }

  /** Runs the session robot: {@code java frc.robot.ReplaySessionRobot}. */
  public static void main(String... args) {
    RobotBase.startRobot(ReplaySessionRobot::new);
  }

  @Override
  public void robotPeriodic() {
    // The script runs before the robot's code, the way Driver Station data arrives between loops:
    // AdvantageKit records it at the end of each loop, so a change made after the robot's code had
    // read it would be logged a loop before the robot saw it, and the replay would run a loop
    // early.
    if (Constants.currentMode == Constants.Mode.SIM) {
      script(++loop);
      pitwall().experiments().heard(null); // the page keeping its experiment going
    }
    super.robotPeriodic();
    Logger.recordOutput("Session/Gain", gain.get());
    Logger.recordOutput("Session/SteerKp", container().drivetrainTuning().get().gains().steerKp());
    Logger.recordOutput("Session/Action", pitwall().actions().view().get(0).phase().name());
    Run run = pitwall().experiments().run();
    Logger.recordOutput("Session/Experiment", run == null ? "NONE" : run.state().phase().name());
    Run.@Nullable Scores scores = run == null ? null : run.state().scores();
    Logger.recordOutput(
        "Session/ExperimentRise",
        scores == null
            ? 0.0
            : Objects.requireNonNull(scores.steps().get("Front left"))
                .get(0)
                .orElseThrow()
                .riseSeconds()
                .orElseThrow());
  }

  /** The session, by loop: in a replay none of this runs, and the log supplies it instead. */
  private void script(int loop) {
    switch (loop) {
      case 5 -> {
        DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
        DriverStationSim.notifyNewData();
      }
      case 10 -> {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setRobotMode(RobotMode.UTILITY);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
      }
      case 20 ->
          http.sendAsync(
              HttpRequest.newBuilder(
                      URI.create("http://localhost:" + pitwall().port() + "/api/values"))
                  .header("Content-Type", "application/merge-patch+json")
                  .method(
                      "PATCH",
                      HttpRequest.BodyPublishers.ofString(
                          "{\"/Drive/config/gains/steerKp\": 18.5}"))
                  .build(),
              HttpResponse.BodyHandlers.discarding());
      case 30 -> pitwall().tunables().requestChanges(Map.of(GAIN, 2.5));
      case 40 -> pitwall().actions().requestStart(RobotContainer.DRIVE_CHECK);
      case 150 -> pitwall().experiments().requestStart(RobotContainer.STEER_STEP, null);
      case LOOPS -> {
        Logger.end(); // closes the log
        System.exit(0);
      }
      default -> {
        // nothing this loop
      }
    }
  }
}
