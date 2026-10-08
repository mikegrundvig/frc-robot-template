package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.experiment.ByHand;
import com.michaelgrundvig.frc.pitwall.push.Broadcaster;
import com.michaelgrundvig.frc.pitwall.scene.Scenes;
import com.michaelgrundvig.frc.pitwall.simulator.Component;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.FakeHal;
import com.michaelgrundvig.frc.pitwall.simulator.Fault;
import com.michaelgrundvig.frc.pitwall.simulator.JoystickInput;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
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
import java.util.concurrent.atomic.AtomicBoolean;
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
  private final AtomicBoolean lampOut = new AtomicBoolean();
  private final Components components =
      new Components(said -> {})
          .add(
              Component.of(
                  "Lamp",
                  "light",
                  Fault.lasting("Out")
                      .does("Dark.")
                      .isOn(lampOut::get)
                      .apply(values -> lampOut.set(true))
                      .undo("Lit again.", () -> lampOut.set(false))
                      .build(),
                  Fault.event("Flickers")
                      .does("Flickers this many times.")
                      .parameter("Times", "", Limits.between(1, 10), 3)
                      .apply(values -> {})
                      .undoneBy("It's over.")
                      .build()))
          .scenario("blackout", "The lamp goes out.", List.of(Setting.of("Lamp", "Out")));
  private final Simulator simulator =
      new Simulator(hal, System::nanoTime, components, stops::incrementAndGet, on -> {});
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
          () -> "",
          simulator,
          new Devices(System::nanoTime),
          new Scenes(System::nanoTime, System.out::println));
  private final Broadcaster broadcaster =
      new Broadcaster(Broadcaster.MAX_STREAMS, Broadcaster.MAILBOX, simulator::lost);
  private final PitwallServer server = new PitwallServer(page, broadcaster, 0);
  private final ScheduledExecutorService loop = Executors.newSingleThreadScheduledExecutor();

  @BeforeAll
  void start() throws Exception {
    server.start();
    // The robot loop: none while the clock is paused, until a step lets one through.
    loop.scheduleAtFixedRate(
        () -> {
          if (hal.runsLoop()) {
            simulator.periodic();
            broadcaster.periodic(page::sections, null);
          }
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
    simulator.resume().get(5, TimeUnit.SECONDS);
    send("DELETE", "/api/simulator/faults", null, null);
    send(
        "PATCH",
        "/api/simulator",
        MERGE_PATCH,
        "{\"enabled\": false, \"fmsAttached\": false, \"matchTime\": null}");
  }

  @Test
  void theSimulatorsStateComesWithWhatCanFailAndTheScenarios() throws Exception {
    var answer = send("GET", "/api/simulator", null, null);

    assertThat(answer.statusCode()).isEqualTo(200);
    assertThat(answer.body())
        .contains("\"simulator\":{\"enabled\":false")
        .contains("\"components\":{\"Lamp\":{\"kind\":\"light\"")
        .contains("\"scenarios\":{\"blackout\":{\"description\":\"The lamp goes out.\"");
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

  /**
   * A body of the wrong type or shape is 400; a name that isn't there or a value out of range 422.
   */
  @Test
  void whatIsntTheSimulatorsToChangeIsRefused() throws Exception {
    var unknown = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"brownout\": true}");
    var mode = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"mode\": \"TEST\"}");
    var modeNotText = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"mode\": 3}");
    var notABoolean = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"enabled\": \"yes\"}");
    var matchTime = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"matchTime\": -5}");
    var matchTimeNotANumber =
        send("PATCH", "/api/simulator", MERGE_PATCH, "{\"matchTime\": \"soon\"}");

    assertThat(unknown.statusCode()).isEqualTo(422);
    assertThat(unknown.body()).contains("brownout isn't the simulator's to change");
    assertThat(mode.statusCode()).isEqualTo(422);
    assertThat(mode.body()).contains("mode is one of AUTONOMOUS, TELEOPERATED, UTILITY, not TEST");
    assertThat(modeNotText.statusCode()).isEqualTo(400);
    assertThat(notABoolean.statusCode()).isEqualTo(400);
    assertThat(matchTime.statusCode()).isEqualTo(422);
    assertThat(matchTimeNotANumber.statusCode()).isEqualTo(400);
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

  /** Paused, no loop runs: a change waits, a read answers at once, and a step runs one loop. */
  @Test
  void aStepIsOneLoopOfAPausedSimulator() throws Exception {
    var running = send("POST", "/api/simulator/step", null, null);
    send("PATCH", "/api/simulator", MERGE_PATCH, "{\"paused\": true}");
    var waiting = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"fmsAttached\": true}");
    var read = send("GET", "/api/simulator", null, null);
    var stepped = send("POST", "/api/simulator/step", null, null);

    assertThat(running.statusCode()).isEqualTo(409);
    assertThat(running.body()).contains("Step runs one loop of a paused simulator");
    assertThat(waiting.statusCode()).as("no loop to apply it").isEqualTo(202);
    assertThat(read.statusCode()).isEqualTo(200);
    assertThat(read.body()).contains("\"paused\":true").contains("\"fmsAttached\":false");
    assertThat(stepped.statusCode()).isEqualTo(200);
    assertThat(stepped.body()).contains("\"fmsAttached\":true").contains("\"paused\":true");
    assertThat(hal.steps()).isEqualTo(1);
  }

  /** A pause asked while paused is answered at once, and a resume after it holds. */
  @Test
  void pausingTwiceThenResumingLeavesItRunning() throws Exception {
    send("PATCH", "/api/simulator", MERGE_PATCH, "{\"paused\": true}");
    var again = send("PATCH", "/api/simulator", MERGE_PATCH, "{\"paused\": true}");
    send("PATCH", "/api/simulator", MERGE_PATCH, "{\"paused\": false}");
    var read = send("GET", "/api/simulator", null, null);

    assertThat(again.statusCode()).isEqualTo(200);
    assertThat(hal.isPaused()).isFalse();
    assertThat(read.body()).contains("\"paused\":false");
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
    var driven = put("{\"0\": {\"axes\": [0.5, -1], \"buttons\": [false, true], \"povs\": [90]}}");

    assertThat(driven.statusCode()).isEqualTo(204);
    JoystickInput expected =
        new JoystickInput(List.of(0.5, -1.0), List.of(false, true), List.of(90));
    waitFor(() -> expected.equals(hal.joystick(0)), "the input to reach the robot");

    var released = put("{\"0\": null}");
    assertThat(released.statusCode()).isEqualTo(204);
    waitFor(() -> JoystickInput.CENTERED.equals(hal.joystick(0)), "the joystick to be let go");
  }

  /** A joystick or value that isn't there is 422; input of the wrong type or shape is 400. */
  @Test
  void joystickInputThatIsntIsRefused() throws Exception {
    assertThat(put("{\"2\": {\"axes\": [0.1]}}").statusCode()).isEqualTo(422);
    var axis = put("{\"0\": {\"axes\": [2]}}");
    assertThat(axis.statusCode()).isEqualTo(422);
    assertThat(axis.body()).contains("An axis is from -1 to 1");
    var pov = put("{\"0\": {\"povs\": [10]}}");
    assertThat(pov.statusCode()).isEqualTo(422);
    assertThat(pov.body()).contains("A POV is an angle");
    assertThat(put("{\"0\": {\"povs\": [90.5]}}").statusCode()).isEqualTo(422);
    assertThat(put("{\"0\": {\"axes\": [0,0,0,0,0,0,0,0,0,0,0,0,0]}}").statusCode())
        .as("13 axes")
        .isEqualTo(400);
    assertThat(put("{\"0\": {\"buttons\": [1]}}").statusCode()).isEqualTo(400);
    assertThat(put("{\"0\": {\"hats\": []}}").statusCode()).isEqualTo(400);
    assertThat(put("{\"0\": \"left\"}").statusCode()).isEqualTo(400);
    assertThat(put("{\"0\": {\"axes\": 0.5}}").statusCode()).isEqualTo(400);
  }

  /** Input from no page is refused: the joystick is let go when its page's stream closes. */
  @Test
  void joystickInputNamesItsPagesStream() throws Exception {
    var anonymous = send("PUT", "/api/simulator/joysticks", JSON, "{\"1\": {\"axes\": [0.3]}}");

    assertThat(anonymous.statusCode()).isEqualTo(400);
    assertThat(anonymous.body()).contains("Pitwall-Stream");
    assertThat(hal.joystick(1)).isNull();
  }

  @Test
  void thePageAppliesAndUndoesAFault() throws Exception {
    var applied =
        send(
            "POST", "/api/simulator/faults", JSON, "{\"component\": \"Lamp\", \"fault\": \"Out\"}");
    assertThat(applied.statusCode()).isEqualTo(200);
    assertThat(lampOut).isTrue();
    assertThat(applied.body()).contains("\"Out\":{").contains("\"on\":true");

    var undone =
        send(
            "POST",
            "/api/simulator/faults",
            JSON,
            "{\"component\": \"Lamp\", \"fault\": \"Out\", \"undo\": true}");
    assertThat(undone.statusCode()).isEqualTo(200);
    assertThat(lampOut).isFalse();
    assertThat(undone.body()).contains("\"on\":false");
  }

  @Test
  void aFaultThatCantBeIsRefusedSayingWhy() throws Exception {
    var unknown =
        send("POST", "/api/simulator/faults", JSON, "{\"component\": \"Fan\", \"fault\": \"Out\"}");
    var outOfRange =
        send(
            "POST",
            "/api/simulator/faults",
            JSON,
            "{\"component\": \"Lamp\", \"fault\": \"Flickers\", \"values\": {\"Times\": 99}}");
    var event =
        send(
            "POST",
            "/api/simulator/faults",
            JSON,
            "{\"component\": \"Lamp\", \"fault\": \"Flickers\", \"undo\": true}");
    var malformed = send("POST", "/api/simulator/faults", JSON, "{\"component\": \"Lamp\"}");
    var notANumber =
        send(
            "POST",
            "/api/simulator/faults",
            JSON,
            "{\"component\": \"Lamp\", \"fault\": \"Flickers\", \"values\": {\"Times\": \"3\"}}");

    assertThat(unknown.statusCode()).isEqualTo(422);
    assertThat(unknown.body()).contains("There is no component named Fan; there are Lamp");
    assertThat(outOfRange.statusCode()).isEqualTo(422);
    assertThat(outOfRange.body()).contains("Times: 99 is above its limit of 10");
    assertThat(event.statusCode()).isEqualTo(422);
    assertThat(event.body()).contains("happens once, and nothing undoes it");
    assertThat(malformed.statusCode()).isEqualTo(400);
    assertThat(notANumber.statusCode()).isEqualTo(400);
  }

  @Test
  void undoingEveryFaultPutsTheRobotBackAsBuilt() throws Exception {
    send("POST", "/api/simulator/faults", JSON, "{\"component\": \"Lamp\", \"fault\": \"Out\"}");

    var undone = send("DELETE", "/api/simulator/faults", null, null);

    assertThat(undone.statusCode()).isEqualTo(200);
    assertThat(lampOut).isFalse();
  }

  @Test
  void thePageAppliesAScenario() throws Exception {
    var applied = send("POST", "/api/simulator/scenarios", JSON, "{\"apply\": \"blackout\"}");
    var unknown = send("POST", "/api/simulator/scenarios", JSON, "{\"apply\": \"brownout\"}");
    var malformed = send("POST", "/api/simulator/scenarios", JSON, "{\"start\": \"blackout\"}");

    assertThat(applied.statusCode()).isEqualTo(200);
    assertThat(lampOut).isTrue();
    assertThat(unknown.statusCode()).isEqualTo(422);
    assertThat(malformed.statusCode()).isEqualTo(400);
  }

  /** A joystick's input, as the page sends it, naming its stream. */
  private HttpResponse<String> put(String json) throws Exception {
    return client.send(
        HttpRequest.newBuilder(
                URI.create("http://localhost:" + server.port() + "/api/simulator/joysticks"))
            .header("Content-Type", JSON)
            .header("Pitwall-Stream", "7")
            .PUT(HttpRequest.BodyPublishers.ofString(json))
            .build(),
        HttpResponse.BodyHandlers.ofString());
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
