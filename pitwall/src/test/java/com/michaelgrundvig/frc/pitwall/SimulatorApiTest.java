package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.experiment.ByHand;
import com.michaelgrundvig.frc.pitwall.push.Broadcaster;
import com.michaelgrundvig.frc.pitwall.simulator.FakeHal;
import com.michaelgrundvig.frc.pitwall.simulator.JoystickInput;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.BooleanSupplier;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.tunable.MockTunableBackend;

/**
 * The simulator's controls over real HTTP, on a stand-in for WPILib's simulation, with a thread
 * playing the robot loop: what the page sends, what's refused and why, and what the robot program
 * ends up seeing.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SimulatorApiTest {
  private static final String JSON = "application/json";
  private static final String MERGE_PATCH = "application/merge-patch+json";

  private final HttpClient client = HttpClient.newHttpClient();
  private final FakeHal hal = new FakeHal();
  private final AtomicInteger stops = new AtomicInteger();
  private final Simulator simulator = new Simulator(hal, System::nanoTime, stops::incrementAndGet);
  private final TunableTee tunables =
      new TunableTee(new MockTunableBackend(), () -> null, inputs -> {}, false);
  private final Tuning tuning =
      new Tuning("sim", tunables::configs, tunables::pathOf, true, inputs -> {}, view -> {});
  private final Page page =
      new Page(
          new Status(),
          new Actions(inputs -> {}),
          tuning,
          tunables,
          new ByHand(System::nanoTime).experiments(),
          simulator);
  private final Broadcaster broadcaster =
      new Broadcaster(Broadcaster.MAX_STREAMS, Broadcaster.MAILBOX, simulator::lost);
  private final PitwallServer server = new PitwallServer(page, broadcaster, 0);
  private final ScheduledExecutorService loop = Executors.newSingleThreadScheduledExecutor();

  @BeforeAll
  void start() throws Exception {
    server.start();
    loop.scheduleAtFixedRate(
        () -> {
          simulator.periodic();
          broadcaster.periodic(page::sections, null);
        },
        0,
        5,
        TimeUnit.MILLISECONDS);
  }

  @AfterAll
  void stop() {
    loop.shutdownNow();
    server.close();
  }

  @AfterEach
  void backAsItWas() throws Exception {
    hal.resume();
    send(
        "PATCH",
        "/api/simulator",
        MERGE_PATCH,
        "{\"enabled\": false, \"fmsAttached\": false, \"matchTime\": null}");
  }

  @Test
  void thePageReadsTheSimulatorsState() throws Exception {
    var answer = send("GET", "/api/simulator", null, null);

    assertThat(answer.statusCode()).isEqualTo(200);
    assertThat(answer.body()).contains("\"simulator\":{\"enabled\":false");
  }

  @Test
  void thePageSetsTheDriverStationAndHearsWhatTheRobotSees() throws Exception {
    var answer =
        send(
            "PATCH",
            "/api/simulator",
            MERGE_PATCH,
            "{\"mode\": \"UTILITY\", \"allianceStation\": \"RED_2\", \"fmsAttached\": true,"
                + " \"enabled\": true, \"matchTime\": 135}");

    assertThat(answer.statusCode()).isEqualTo(200);
    assertThat(answer.body())
        .contains("\"enabled\":true")
        .contains("\"mode\":\"UTILITY\"")
        .contains("\"allianceStation\":\"RED_2\"")
        .contains("\"fmsAttached\":true")
        .contains("\"dsAttached\":true");
    assertThat(hal.driverStation().mode()).isEqualTo(RobotMode.UTILITY);
    assertThat(hal.driverStation().allianceStation()).isEqualTo(AllianceStationID.RED_2);
  }

  @Test
  void whatIsntTheSimulatorsToChangeIsRefused() throws Exception {
    var unknown = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"brownout\": true}");
    var mode = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"mode\": \"TEST\"}");
    var notABoolean = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"enabled\": \"yes\"}");
    var matchTime = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"matchTime\": -5}");

    assertThat(unknown.statusCode()).isEqualTo(422);
    assertThat(unknown.body()).contains("brownout isn't the simulator's to change");
    assertThat(mode.statusCode()).isEqualTo(400);
    assertThat(mode.body()).contains("mode is one of AUTONOMOUS, TELEOPERATED, UTILITY, not TEST");
    assertThat(notABoolean.statusCode()).isEqualTo(400);
    assertThat(matchTime.statusCode()).isEqualTo(400);
    assertThat(hal.driverStation().enabled()).isFalse();
  }

  /** The loop pauses the clock; resuming happens at once, on the server's thread. */
  @Test
  void thePagePausesAndResumesTheClock() throws Exception {
    var paused = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"paused\": true}");
    assertThat(paused.statusCode()).isEqualTo(200);
    assertThat(paused.body()).contains("\"paused\":true");
    assertThat(hal.isPaused()).isTrue();

    var resumed = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"paused\": false}");
    assertThat(resumed.statusCode()).isEqualTo(200);
    assertThat(resumed.body()).contains("\"paused\":false");
    assertThat(hal.isPaused()).isFalse();
  }

  @Test
  void aStepIsOneLoopOfAPausedSimulator() throws Exception {
    var running = send("POST", "/api/simulator/step", null, null);
    hal.pause();
    var stepped = send("POST", "/api/simulator/step", null, null);

    assertThat(running.statusCode()).isEqualTo(409);
    assertThat(running.body()).contains("Step runs one loop of a paused simulator");
    assertThat(stepped.statusCode()).isEqualTo(200);
    assertThat(hal.steps()).isEqualTo(1);
  }

  @Test
  void stopAnswersThenTheLoopEndsTheProgram() throws Exception {
    var answer = send("POST", "/api/simulator/stop", null, null);

    assertThat(answer.statusCode()).isEqualTo(202);
    waitFor(() -> stops.get() == 1, "the loop to stop the program");
  }

  /** Input goes to the loop, which hands it to the robot program; the server answers at once. */
  @Test
  void thePageDrivesAJoystick() throws Exception {
    var driven =
        send(
            "PUT",
            "/api/simulator/joysticks",
            JSON,
            "{\"0\": {\"axes\": [0.5, -1], \"buttons\": [false, true], \"povs\": [90]}}");

    assertThat(driven.statusCode()).isEqualTo(204);
    JoystickInput expected =
        new JoystickInput(List.of(0.5, -1.0), List.of(false, true), List.of(90));
    waitFor(() -> expected.equals(hal.joystick(0)), "the input to reach the robot");

    var released = send("PUT", "/api/simulator/joysticks", JSON, "{\"0\": null}");
    assertThat(released.statusCode()).isEqualTo(204);
    waitFor(() -> JoystickInput.CENTERED.equals(hal.joystick(0)), "the joystick to be let go");
  }

  @Test
  void joystickInputThatIsntIsRefused() throws Exception {
    assertThat(put("{\"2\": {\"axes\": [0.1]}}").statusCode()).isEqualTo(422);
    assertThat(put("{\"0\": {\"axes\": [2]}}").body()).contains("An axis is from -1 to 1");
    assertThat(put("{\"0\": {\"povs\": [10]}}").body()).contains("A POV is an angle");
    assertThat(put("{\"0\": {\"povs\": [90.5]}}").statusCode()).isEqualTo(400);
    assertThat(put("{\"0\": {\"buttons\": [1]}}").statusCode()).isEqualTo(400);
    assertThat(put("{\"0\": {\"hats\": []}}").statusCode()).isEqualTo(400);
    assertThat(put("{\"0\": \"left\"}").statusCode()).isEqualTo(400);
    assertThat(put("{\"0\": {\"axes\": 0.5}}").statusCode()).isEqualTo(400);
  }

  private HttpResponse<String> put(String json) throws Exception {
    return send("PUT", "/api/simulator/joysticks", JSON, json);
  }

  private void waitFor(BooleanSupplier done, String what) throws InterruptedException {
    long end = System.nanoTime() + Duration.ofSeconds(5).toNanos();
    while (!done.getAsBoolean() && System.nanoTime() < end) {
      Thread.sleep(5);
    }
    assertThat(done.getAsBoolean()).as(what).isTrue();
  }

  private HttpResponse<String> send(
      String method, String path, @Nullable String type, @Nullable String body) throws Exception {
    var request = HttpRequest.newBuilder(URI.create("http://localhost:" + server.port() + path));
    if (type != null) {
      request.header("Content-Type", type);
    }
    return client.send(
        request
            .method(
                method,
                body == null
                    ? HttpRequest.BodyPublishers.noBody()
                    : HttpRequest.BodyPublishers.ofString(body))
            .build(),
        HttpResponse.BodyHandlers.ofString());
  }
}
