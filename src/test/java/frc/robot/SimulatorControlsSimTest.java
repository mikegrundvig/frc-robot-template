package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.concurrent.CompletableFuture;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.system.RobotController;

/**
 * The simulator's controls on the template's robot, through Pitwall's page the way the Sim view
 * drives them, in WPILib's own simulation: the Driver Station, driving from the page's keyboard or
 * gamepad, the field lock the panel itself is exempt from, and the clock.
 */
@SimulationTest
class SimulatorControlsSimTest {
  private static final String JSON = "application/json";

  @RegisterExtension final RobotSim sim = new RobotSim();

  private final HttpClient client = HttpClient.newHttpClient();

  /** The page enables the robot in teleop: the robot program sees it in the same loop. */
  @Test
  void thePageEnablesTheRobotAndPicksItsStation() throws Exception {
    sim.robot();
    RobotSim.setEnabled(false);
    DriverStationSim.setDsAttached(false);
    DriverStationSim.notifyNewData();

    var answer =
        send(
            "PATCH",
            "/api/simulator",
            "{\"allianceStation\": \"RED_1\", \"mode\": \"UTILITY\", \"enabled\": true}");

    assertThat(answer.statusCode()).isEqualTo(200);
    assertThat(RobotState.isEnabled()).isTrue();
    assertThat(RobotState.getRobotMode()).isEqualTo(RobotMode.UTILITY);
    assertThat(RobotState.isDSAttached()).isTrue();
    assertThat(MatchState.getLocation()).hasValue(1);
    assertThat(sim.robot().allianceStation()).isEqualTo("Red 1");
  }

  /**
   * In the simulator the panel is the field: attaching the field system locks tuning as a real
   * field does, while the panel stays usable to detach it again.
   */
  @Test
  void theFieldLocksTuningButNotThePanel() throws Exception {
    sim.robot();

    assertThat(send("PATCH", "/api/simulator", "{\"fmsAttached\": true}").statusCode())
        .isEqualTo(200);
    var tuned = send("PATCH", "/api/values", "{\"/Drive/config/gains/steerKp\": 18}");
    var detached = send("PATCH", "/api/simulator", "{\"fmsAttached\": false}");

    assertThat(tuned.statusCode()).isEqualTo(409);
    assertThat(detached.statusCode()).isEqualTo(200);
    assertThat(RobotState.isFMSAttached()).isFalse();
  }

  /**
   * The page's input drives the robot, through the same joystick the drive team's controller is;
   * when the page goes quiet, the deadman centers it and the robot stops.
   */
  @Test
  void thePageDrivesTheRobotAndItsDeadmanStopsIt() throws Exception {
    Pose2d start = placedAndEnabled();

    // Left stick pushed forward, as the page's W key or a gamepad sends it: up is negative.
    String forward = "{\"0\": {\"axes\": [0, -0.5, 0, 0, 0, 0]}}";
    for (int i = 0; i < 10; i++) { // 1 s, repeating every 100 ms as the page does
      assertThat(send("PUT", "/api/simulator/joysticks", forward).statusCode()).isEqualTo(204);
      sim.loops(5);
    }
    double driven = travelled(start);
    sim.run(1.5); // quiet: centered after 500 ms, and stopped
    Pose2d stopped = sim.simulation().truePose();
    sim.run(0.5);

    assertThat(driven).as("driven forward").isGreaterThan(0.3);
    assertThat(sim.simulation().truePose().getTranslation().getDistance(stopped.getTranslation()))
        .as("stopped")
        .isLessThan(0.01);
  }

  /** With the clock paused, a step moves it on to the next loop's time, and no further. */
  @Test
  void aStepMovesThePausedClockOnOneLoop() {
    Simulator simulator = sim.pitwall().simulator().orElseThrow();
    long before = RobotController.getTime();

    CompletableFuture<?> stepped = simulator.step(); // tests run with the clock paused
    long after = RobotController.getTime();
    sim.loop();

    assertThat(after - before).isPositive().isLessThanOrEqualTo(20_000_000L);
    assertThat(stepped).isDone();
  }

  /** On its start, enabled in teleop, cameras settled. */
  private Pose2d placedAndEnabled() {
    sim.robot();
    RobotSim.setEnabled(true);
    sim.run(1.0);
    return sim.simulation().truePose();
  }

  private double travelled(Pose2d from) {
    return sim.simulation().truePose().getTranslation().getDistance(from.getTranslation());
  }

  /**
   * Sends the page's request and runs the robot loop until it's answered, as the robot does: the
   * server waits for the loop.
   */
  private HttpResponse<String> send(String method, String path, @Nullable String json)
      throws Exception {
    var request =
        HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + sim.pitwall().port() + path));
    if (json != null) {
      request.header("Content-Type", JSON);
    }
    var answer =
        client.sendAsync(
            request
                .method(
                    method,
                    json == null
                        ? HttpRequest.BodyPublishers.noBody()
                        : HttpRequest.BodyPublishers.ofString(json))
                .build(),
            HttpResponse.BodyHandlers.ofString());
    long end = System.nanoTime() + Duration.ofSeconds(10).toNanos();
    while (!answer.isDone() && System.nanoTime() < end) {
      sim.loop();
    }
    return answer.get();
  }
}
