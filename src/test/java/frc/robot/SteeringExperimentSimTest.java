package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import frc.robot.sim.SensorErrors;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.io.UncheckedIOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.hardware.hal.RobotMode;

/**
 * Tuning the steering by experiment, in the simulator, the way a programmer does it from the
 * Pitwall page: the page follows the robot's event stream, starts the steering's step to 90° and
 * back over HTTP, keeps it going with heartbeats, and reads the scores from the stream. A gain
 * changed from the page changes the next run's answer.
 */
@SimulationTest
class SteeringExperimentSimTest {
  private static final String STEER_KP = "/Drive/config/gains/steerKp";
  private static final String STEER_KD = "/Drive/config/gains/steerKd";

  /** The longest the test waits, in real time, for the network. */
  private static final Duration WAIT = Duration.ofSeconds(10);

  private static final JsonType<Object> JSON = Jsonb.builder().build().type(Object.class);

  @RegisterExtension final RobotSim sim = new RobotSim();

  private final HttpClient client = HttpClient.newHttpClient();
  private final List<AutoCloseable> open = new ArrayList<>();

  @AfterEach
  void close() throws Exception {
    for (AutoCloseable stream : open) {
      stream.close();
    }
  }

  /** A server-sent event: its name and its data, parsed. */
  record Sse(String event, Map<String, Object> data) {}

  /** The page: its event stream, and the requests it sends naming that stream. */
  private final class Page {
    final BlockingQueue<Sse> events = new LinkedBlockingQueue<>();
    final String stream;

    Page() throws Exception {
      Stream<String> lines =
          answered(
                  client.sendAsync(
                      HttpRequest.newBuilder(uri("/api/events")).GET().build(),
                      HttpResponse.BodyHandlers.ofLines()))
              .body();
      open.add(lines::close);
      Thread.ofVirtual().start(() -> read(lines));
      Sse snapshot = nextEvent();
      assertThat(snapshot.event()).isEqualTo("snapshot");
      stream = (String) Objects.requireNonNull(snapshot.data().get("stream"));
    }

    /** The next event, running the robot a loop at a time until it comes. */
    private Sse nextEvent() throws Exception {
      long deadline = System.nanoTime() + WAIT.toNanos();
      while (System.nanoTime() < deadline) {
        sim.loop();
        Sse event = events.poll(50, TimeUnit.MILLISECONDS);
        if (event != null) {
          return event;
        }
      }
      throw new AssertionError("no event within " + WAIT);
    }

    private void read(Stream<String> lines) {
      List<String> event = new ArrayList<>();
      try {
        lines.forEach(
            line -> {
              if (line.isEmpty()) {
                if (event.size() == 2) {
                  events.add(
                      new Sse(
                          event.get(0).substring("event: ".length()),
                          asMap(JSON.fromJson(event.get(1).substring("data: ".length())))));
                }
                event.clear();
              } else {
                event.add(line);
              }
            });
      } catch (UncheckedIOException closed) {
        // The test is over.
      }
    }

    /** Sends a request as the page does, naming its stream, and runs the robot until answered. */
    HttpResponse<String> send(String method, String path, String json) throws Exception {
      CompletableFuture<HttpResponse<String>> answer =
          client.sendAsync(
              HttpRequest.newBuilder(uri(path))
                  .header("Content-Type", "application/merge-patch+json")
                  .header("Pitwall-Stream", stream)
                  .method(method, HttpRequest.BodyPublishers.ofString(json))
                  .build(),
              HttpResponse.BodyHandlers.ofString());
      return answered(answer);
    }

    /** A heartbeat, as the page sends one after 100 ms of quiet: answered at once. */
    private void heartbeat() throws Exception {
      var answer =
          client.send(
              HttpRequest.newBuilder(uri("/api/heartbeat"))
                  .header("Pitwall-Stream", stream)
                  .POST(HttpRequest.BodyPublishers.noBody())
                  .build(),
              HttpResponse.BodyHandlers.discarding());
      assertThat(answer.statusCode()).isEqualTo(204);
    }

    /**
     * Runs the steering's experiment to its end, keeping it going as the page does: a heartbeat
     * every ten loops, 200 ms of the robot's time, on whose clock the deadman runs. Returns the run
     * as the stream described it, with every sample the stream carried.
     */
    Run run() throws Exception {
      var started =
          send("POST", "/api/experiments", "{\"start\": \"" + RobotContainer.STEER_STEP + "\"}");
      assertThat(started.statusCode()).as(started.body()).isEqualTo(200);
      Run run = new Run();
      long deadline = System.nanoTime() + WAIT.toNanos();
      for (int loop = 1; !run.ended(); loop++) {
        assertThat(System.nanoTime()).as("the run ended within " + WAIT).isLessThan(deadline);
        if (loop % 10 == 0) {
          heartbeat();
        }
        sim.loop();
        takeEvents(run);
      }
      // Its last samples follow the patch that says it ended: wait for every one.
      int recorded = recordedAngles().size();
      while (run.samples.getOrDefault("Front left", 0) < recorded) {
        Sse event = events.poll(WAIT.toMillis(), TimeUnit.MILLISECONDS);
        assertThat(event).as("every sample, within " + WAIT).isNotNull();
        run.take(Objects.requireNonNull(event));
      }
      return run;
    }

    /** The front left wheel's angle readings as the robot recorded them. */
    List<?> recordedAngles() throws Exception {
      Map<String, Object> trace = asMap(asMap(JSON.fromJson(get("/api/run").body())).get("trace"));
      return asList(asMap(asMap(trace.get("signals")).get("Front left")).get("t"));
    }

    private void takeEvents(Run run) {
      for (Sse event = events.poll(); event != null; event = events.poll()) {
        run.take(event);
      }
    }

    /** Asks for something, as a page or a tool would. */
    HttpResponse<String> get(String path) throws Exception {
      return client.send(
          HttpRequest.newBuilder(uri(path)).GET().build(), HttpResponse.BodyHandlers.ofString());
    }
  }

  /** A run as the page records it from the stream. */
  static final class Run {
    Map<String, Object> state = Map.of();
    final Map<String, Integer> samples = new HashMap<>();

    void take(Sse event) {
      if (event.event().equals("patch") && event.data().get("run") instanceof Map<?, ?> run) {
        Map<String, Object> merged = new HashMap<>(state);
        run.forEach((key, value) -> merged.put(String.valueOf(key), value));
        state = merged;
      }
      if (event.event().equals("samples")) {
        asMap(event.data().get("signals"))
            .forEach(
                (signal, series) ->
                    samples.merge(signal, asList(asMap(series).get("t")).size(), Integer::sum));
      }
    }

    boolean ended() {
      return "ENDED".equals(state.get("phase")) || "CANCELED".equals(state.get("phase"));
    }

    /** Each jump of each wheel: {"rise", "overshoot", "settling", "error", "samples"}. */
    List<Map<String, Object>> jumps() {
      Map<String, Object> steps = asMap(asMap(state.get("scores")).get("steps"));
      List<Map<String, Object>> jumps = new ArrayList<>();
      steps.values().forEach(wheel -> asList(wheel).forEach(jump -> jumps.add(asMap(jump))));
      return jumps;
    }

    /** The worst of a score over every jump of every wheel. */
    double worst(String score) {
      return jumps().stream().mapToDouble(jump -> number(jump.get(score))).max().orElseThrow();
    }
  }

  /** The simulator's robot, enabled in utility mode, its cameras off to keep the test quick. */
  private Page ready() throws Exception {
    sim.build(RobotSim.WITHOUT_CAMERAS, SensorErrors.NONE);
    RobotSim.enable(RobotMode.UTILITY, true);
    sim.loops(5);
    return new Page();
  }

  @Test
  void aStepOnTheSteeringIsScoredInASaneRange() throws Exception {
    Page page = ready();

    Run run = page.run();

    assertThat(run.state.get("phase"))
        .as(String.valueOf(run.state.get("detail")))
        .isEqualTo("ENDED");
    assertThat(run.jumps()).as("two jumps on each of four wheels").hasSize(8);
    for (Map<String, Object> jump : run.jumps()) {
      assertThat(number(jump.get("rise"))).as("rise, s").isBetween(0.02, 0.3);
      assertThat(number(jump.get("overshoot"))).as("overshoot").isBetween(0.0, 0.3);
      assertThat(number(jump.get("settling"))).as("settling, s").isBetween(0.02, 0.8);
      assertThat(Math.abs(number(jump.get("error")))).as("error, degrees").isLessThan(1.0);
    }
    assertThat(number(asMap(run.state.get("gains")).get(STEER_KP))).isEqualTo(16.0);
    // Every angle reading the robot recorded reached the page, at 250 Hz: about 2.1 s of them.
    List<?> angles = page.recordedAngles();
    assertThat(run.samples.get("Front left")).isEqualTo(angles.size());
    assertThat(angles.size()).isBetween(500, 540);
  }

  /**
   * The simulator repeats itself: the same robot gives the same scores. To a billionth, not
   * exactly: a run starts when its request arrives, so its times on the robot's clock differ, and
   * with them the last bits of the arithmetic.
   */
  @Test
  void theSameRobotGivesTheSameScores() throws Exception {
    Run first = ready().run();
    close();
    open.clear();

    Run second = ready().run();

    List<Map<String, Object>> again = second.jumps();
    assertThat(again).hasSameSizeAs(first.jumps());
    for (int i = 0; i < again.size(); i++) {
      for (String score : List.of("rise", "overshoot", "settling", "error")) {
        assertThat(number(again.get(i).get(score)))
            .as("jump %d's %s", i, score)
            .isCloseTo(number(first.jumps().get(i).get(score)), within(1e-9));
      }
    }
  }

  /**
   * Tuning steerKp from the page: a stiffer gain answers faster and settles sooner, and overshoots
   * more. At the top of its range the module comes in so fast that the motor, held to its current
   * limit, can't stop it in time: it overshoots by about 5%, with the committed steerKd or none.
   */
  @Test
  void raisingSteerKpRisesFasterAndOvershootsMore() throws Exception {
    Page page = ready();
    Run committed = page.run();
    page.send("PATCH", "/api/values", "{\"" + STEER_KP + "\": 100}");
    Run stiff = page.run();
    // Both at once, all or none, as the page sends them.
    var undamped =
        page.send("PATCH", "/api/values", "{\"" + STEER_KP + "\": 60, \"" + STEER_KD + "\": 0}");
    Run softer = page.run();
    page.send("PATCH", "/api/values", "{\"" + STEER_KP + "\": 100}");
    Run harder = page.run();

    assertThat(undamped.statusCode()).as(undamped.body()).isEqualTo(200);
    assertThat(number(asMap(harder.state.get("gains")).get(STEER_KP))).isEqualTo(100.0);
    assertThat(number(asMap(harder.state.get("gains")).get(STEER_KD))).isEqualTo(0.0);
    assertThat(stiff.worst("rise")).as("rise, s").isLessThan(committed.worst("rise") * 0.8);
    assertThat(stiff.worst("settling"))
        .as("settling, s")
        .isLessThan(committed.worst("settling") * 0.6);
    assertThat(committed.worst("overshoot")).isZero();
    assertThat(stiff.worst("overshoot")).isBetween(0.03, 0.07);
    assertThat(harder.worst("overshoot"))
        .as("overshoot at 100, against %s at 60", softer.worst("overshoot"))
        .isGreaterThan(softer.worst("overshoot") * 3)
        .isBetween(0.005, 0.05);
  }

  /**
   * Runs the robot a loop at a time until an answer comes, waiting up to 50 ms of real time for it
   * after each loop: the time it takes is the network's, not the robot's.
   */
  private <T> T answered(CompletableFuture<T> answer) throws Exception {
    long deadline = System.nanoTime() + WAIT.toNanos();
    while (System.nanoTime() < deadline) {
      sim.loop();
      try {
        return answer.get(50, TimeUnit.MILLISECONDS);
      } catch (TimeoutException notYet) {
        // Another loop, then.
      }
    }
    throw new AssertionError("no answer within " + WAIT);
  }

  /** On 127.0.0.1: on Windows, localhost tries IPv6 first and costs about 200 ms each time. */
  private URI uri(String path) {
    return URI.create("http://127.0.0.1:" + sim.pitwall().port() + path);
  }

  private static double number(@Nullable Object json) {
    return ((Number) Objects.requireNonNull(json)).doubleValue();
  }

  private static List<?> asList(@Nullable Object json) {
    return (List<?>) Objects.requireNonNull(json);
  }

  @SuppressWarnings("unchecked") // JSON objects parse as maps from names
  private static Map<String, Object> asMap(@Nullable Object json) {
    return (Map<String, Object>) Objects.requireNonNull(json);
  }
}
