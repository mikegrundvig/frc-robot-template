package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.io.UncheckedIOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.TreeMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;

/**
 * Pitwall's page left open on the template's robot, in the simulator: with nothing happening, the
 * robot sends the page next to nothing. Push rule 1 (an event only when something changed) holds
 * only if the status lines don't change every loop, so this is where a line that churns shows up.
 * (Before the cameras' line stopped counting measurements, it sent 244 patches in these 250 loops.)
 */
@SimulationTest
class PitwallStreamSimTest {
  private static final JsonType<Object> JSON = Jsonb.builder().build().type(Object.class);

  @RegisterExtension final RobotSim sim = new RobotSim();

  private final HttpClient client = HttpClient.newHttpClient();
  private final List<AutoCloseable> open = new ArrayList<>();

  /** Each event's name and data, as the page receives them. */
  private final List<String[]> events = new CopyOnWriteArrayList<>();

  @AfterEach
  void close() throws Exception {
    for (AutoCloseable stream : open) {
      stream.close();
    }
  }

  @Test
  void anIdleRobotSendsAnOpenPageNextToNothing() throws Exception {
    sim.robot();
    sim.loops(250); // settled: placed, the cameras' tags have narrowed the estimate
    openStream();
    waitFor(() -> events.size() >= 1, "the snapshot");

    sim.loops(250); // five seconds, nothing happening
    sim.loops(1);
    settle();

    List<String[]> patches = events.stream().filter(e -> e[0].equals("patch")).toList();
    Map<String, Integer> changed = new TreeMap<>();
    for (String[] patch : patches) {
      @SuppressWarnings("unchecked") // a merge patch is an object of sections
      Map<String, Object> sections = (Map<String, Object>) JSON.fromJson(patch[1]);
      sections.forEach(
          (section, keys) -> {
            if (keys instanceof Map<?, ?> map) {
              map.keySet().forEach(key -> changed.merge(section + "/" + key, 1, Integer::sum));
            } else {
              changed.merge(section, 1, Integer::sum);
            }
          });
    }
    System.out.println("Patches in 250 idle loops: " + patches.size() + ", changing " + changed);
    assertThat(patches.size())
        .as("patches in 250 idle loops, changing %s", changed)
        .isLessThanOrEqualTo(5);
  }

  private void openStream() throws Exception {
    CompletableFuture<HttpResponse<Stream<String>>> connecting =
        client.sendAsync(
            HttpRequest.newBuilder(
                    URI.create("http://127.0.0.1:" + sim.pitwall().port() + "/api/events"))
                .GET()
                .build(),
            HttpResponse.BodyHandlers.ofLines());
    while (!connecting.isDone()) {
      sim.loop();
      Thread.onSpinWait();
    }
    Stream<String> lines = connecting.get().body();
    open.add(lines::close);
    Thread.ofVirtual()
        .start(
            () -> {
              List<String> event = new ArrayList<>();
              try {
                lines.forEach(
                    line -> {
                      if (line.isEmpty()) {
                        if (event.size() == 2) {
                          events.add(
                              new String[] {
                                event.get(0).substring("event: ".length()),
                                event.get(1).substring("data: ".length())
                              });
                        }
                        event.clear();
                      } else {
                        event.add(line);
                      }
                    });
              } catch (UncheckedIOException closed) {
                // The test is over.
              }
            });
  }

  /** Runs loops until something has arrived, for at most ten seconds of real time. */
  private void waitFor(java.util.function.BooleanSupplier arrived, String what) {
    long end = System.nanoTime() + Duration.ofSeconds(10).toNanos();
    while (!arrived.getAsBoolean() && System.nanoTime() < end) {
      sim.loop();
    }
    assertThat(arrived.getAsBoolean()).as(what).isTrue();
  }

  /** Waits, without running the robot, until the events stop arriving. */
  private void settle() throws InterruptedException {
    int seen = -1;
    for (int i = 0; i < 50 && seen != events.size(); i++) {
      seen = events.size();
      Objects.requireNonNull(events);
      Thread.sleep(50);
    }
  }
}
