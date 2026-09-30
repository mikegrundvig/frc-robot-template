package com.michaelgrundvig.frc.pitwall;

import static com.michaelgrundvig.frc.pitwall.Fixtures.machine;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.experiment.ByHand;
import com.michaelgrundvig.frc.pitwall.experiment.Run;
import com.michaelgrundvig.frc.pitwall.push.Broadcaster;
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
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;

/**
 * The event stream over real HTTP, read by Java's own client as a page reads it, with the test
 * playing the robot loop one loop at a time. Pushing only when something changes is measured here:
 * a loop with nothing new sends nothing.
 */
class EventsTest {
  private static final Actions.Gate READY = new Actions.Gate(true, true, false);
  private static final String KP = "/Machine/drive/gains/kP";
  private static final JsonType<Object> JSON = Jsonb.builder().build().type(Object.class);

  /** One server-sent event as it arrived: its name and data. */
  record Sse(String event, String data) {
    Map<String, Object> json() {
      return asMap(JSON.fromJson(data));
    }
  }

  private final Status status = new Status().add("Robot", () -> "test");
  private final Actions actions = new Actions(inputs -> {});
  private final TunableTee tunables =
      new TunableTee(new MockTunableBackend(), this::whyLocked, inputs -> {}, false);
  private final Tuning tuning =
      new Tuning("test", tunables::configs, tunables::pathOf, true, inputs -> {}, view -> {});
  private final AtomicLong wallClock = new AtomicLong();
  private final ByHand byHand = new ByHand(wallClock::get);
  private final Page page = new Page(status, actions, tuning, tunables, byHand.experiments());
  private final List<String> closed = new CopyOnWriteArrayList<>();
  private final Broadcaster broadcaster =
      new Broadcaster(
          Broadcaster.MAX_STREAMS,
          Broadcaster.MAILBOX,
          stream -> {
            closed.add(stream);
            byHand.experiments().lost(stream);
          });
  private final HttpClient client = HttpClient.newHttpClient();
  private final List<AutoCloseable> open = new ArrayList<>();
  private @Nullable PitwallServer server;

  private @Nullable String whyLocked() {
    return tuning.whyLocked();
  }

  EventsTest() {
    tunables.publish("Shooter/kP", TunableDouble.create(0.5));
    TunableRegistry.registerBackend("", tunables);
    TunableRegistry.publish(
        "Machine",
        LiveConfig.of("machine", machine()).number("drive.gains.kP", Limits.between(0, 100)));
    loop();
  }

  @AfterEach
  void stop() throws Exception {
    for (AutoCloseable stream : open) {
      stream.close();
    }
    PitwallServer running = server;
    if (running != null) {
      running.close();
    }
    TunableRegistry.reset();
  }

  /** Starts the page, whose streams say they're still there after {@code idle} of quiet. */
  private PitwallServer start(Duration idle) throws Exception {
    PitwallServer started = new PitwallServer(page, broadcaster, 0, PitwallServer.WAIT, idle);
    started.start(true);
    server = started;
    return started;
  }

  /**
   * One robot loop, as Pitwall runs it, then the scheduler's part and WPILib's tunables; 20 ms pass
   * on the deadman's clock.
   */
  private void loop() {
    step();
    wallClock.addAndGet(20_000_000);
  }

  private void step() {
    tuning.periodic(false);
    actions.periodic(READY);
    byHand.experiments().periodic(READY);
    status.periodic(broadcaster.watched());
    broadcaster.periodic(page::sections, byHand.experiments().run());
    byHand.step();
    tunables.update();
  }

  /**
   * Runs the loop until something is true, a millisecond of real time between loops. The deadman's
   * clock stands still meanwhile: how many loops it takes depends on how busy the computer is.
   */
  private void loopUntil(BooleanSupplier done) throws InterruptedException {
    for (int i = 0; i < 5000 && !done.getAsBoolean(); i++) {
      step();
      Thread.sleep(1);
    }
    assertThat(done.getAsBoolean()).as("it happened within 5000 loops").isTrue();
  }

  /** A page's stream, as its events arrive. */
  private final class Listener {
    final BlockingQueue<Sse> events = new LinkedBlockingQueue<>();
    final HttpResponse<Stream<String>> response;

    Listener(PitwallServer server) throws Exception {
      CompletableFuture<HttpResponse<Stream<String>>> connecting =
          client.sendAsync(
              HttpRequest.newBuilder(uri(server, "/api/events")).GET().build(),
              HttpResponse.BodyHandlers.ofLines());
      loopUntil(connecting::isDone);
      response = connecting.get();
      Stream<String> lines = response.body();
      open.add(lines::close);
      Thread.ofVirtual().start(() -> read(lines));
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
                          event.get(1).substring("data: ".length())));
                }
                event.clear();
              } else {
                event.add(line);
              }
            });
      } catch (UncheckedIOException closed) {
        // The test closed it.
      }
    }

    /** The next event, running the loop until it comes. */
    Sse next() throws InterruptedException {
      loopUntil(() -> !events.isEmpty());
      return Objects.requireNonNull(events.poll());
    }

    /** Waits for the snapshot, and returns it. */
    Sse snapshot() throws InterruptedException {
      Sse snapshot = next();
      assertThat(snapshot.event()).isEqualTo("snapshot");
      return snapshot;
    }

    void close() {
      response.body().close();
    }
  }

  /**
   * The design's acceptance test: with a page open and nothing changing, 500 loops send nothing
   * after the snapshot. Changing one value sends exactly one patch, holding that value and what
   * follows from it (the builder call that commits it).
   */
  @Test
  void aStreamCarriesTheSnapshotThenOnlyWhatChanged() throws Exception {
    Listener page = new Listener(start(PitwallServer.IDLE));

    Map<String, Object> snapshot = page.snapshot().json();

    assertThat(snapshot).containsEntry("stream", "1");
    Map<String, Object> document = asMap(snapshot.get("document"));
    assertThat(document)
        .containsKeys(
            "robot", "status", "units", "actions", "tuning", "tunables", "values", "changes")
        .containsKeys("wpilibTunables", "rigs", "experiments", "run");
    assertThat(asMap(document.get("status"))).containsEntry("Robot", "test");
    assertThat(number(asMap(document.get("values")).get(KP))).isEqualTo(16.0);

    for (int i = 0; i < 500; i++) {
      loop();
    }
    Thread.sleep(200); // anything sent would have arrived

    assertThat(page.events).as("500 loops, nothing changed").isEmpty();

    tunables.requestChanges(Map.of(KP, 18.5));
    Sse patch = page.next();
    for (int i = 0; i < 100; i++) {
      loop();
    }
    Thread.sleep(100);

    assertThat(patch.event()).isEqualTo("patch");
    assertThat(patch.json()).containsOnlyKeys("values", "changes");
    assertThat(asMap(patch.json().get("values"))).containsOnlyKeys(KP);
    assertThat(number(asMap(patch.json().get("values")).get(KP))).isEqualTo(18.5);
    assertThat(asMap(asMap(patch.json().get("changes")).get(KP)))
        .containsEntry("builderCall", ".kP(18.5)");
    assertThat(page.events).as("one change, one patch").isEmpty();
  }

  /** An event, not a comment line: a browser's EventSource never shows the page a comment. */
  @Test
  void aQuietStreamSaysItsStillThereWithAHeartbeat() throws Exception {
    Listener page = new Listener(start(Duration.ofMillis(50)));
    page.snapshot();

    Sse quiet = Objects.requireNonNull(page.events.poll(5, TimeUnit.SECONDS));

    assertThat(quiet).isEqualTo(new Sse("heartbeat", "{}"));
  }

  @Test
  void closingAStreamRemovesIt() throws Exception {
    Listener page = new Listener(start(Duration.ofMillis(20)));
    page.snapshot();
    assertThat(broadcaster.size()).isEqualTo(1);

    page.close();

    loopUntil(() -> broadcaster.size() == 0);
    assertThat(closed).containsExactly("1");
  }

  @Test
  void beyondEightStreamsAPageIsTurnedAway() throws Exception {
    PitwallServer running = start(PitwallServer.IDLE);
    for (int i = 0; i < Broadcaster.MAX_STREAMS; i++) {
      new Listener(running);
    }

    var ninth =
        client.send(
            HttpRequest.newBuilder(uri(running, "/api/events")).GET().build(),
            HttpResponse.BodyHandlers.ofString());

    assertThat(ninth.statusCode()).isEqualTo(503);
    assertThat(ninth.body()).contains("At most 8 pages");
  }

  /**
   * A run started from a page streams its samples every loop, and ends scored. The page keeps it
   * going with heartbeats that name its stream; when they stop, the deadman cancels the next run.
   */
  @Test
  void aRunStreamsItsSamplesWhileThePageKeepsInTouch() throws Exception {
    PitwallServer running = start(PitwallServer.IDLE);
    Listener page = new Listener(running);
    String stream = (String) Objects.requireNonNull(page.snapshot().json().get("stream"));

    CompletableFuture<HttpResponse<String>> started = post(running, "/api/experiments", stream);
    loopUntil(started::isDone);
    assertThat(started.get().statusCode()).isEqualTo(200);
    int samples = 0;
    Run run = Objects.requireNonNull(byHand.experiments().run());
    for (int i = 0; i < 120; i++) {
      if (i % 10 == 0) {
        heartbeat(running, stream); // every 200 ms of the robot's time
      }
      loop();
    }
    Thread.sleep(200);
    for (Sse event = page.events.poll(); event != null; event = page.events.poll()) {
      if (event.event().equals("samples")) {
        samples++;
      }
    }

    assertThat(run.state().phase()).isEqualTo(Run.Phase.ENDED);
    assertThat(samples).as("one event a loop while it ran").isEqualTo(106);

    CompletableFuture<HttpResponse<String>> again = post(running, "/api/experiments", stream);
    loopUntil(again::isDone);
    Run next = Objects.requireNonNull(byHand.experiments().run());
    for (int i = 0; i < 30; i++) {
      loop(); // 600 ms without a word from the page
    }

    assertThat(next.state().detail())
        .isEqualTo("Canceled: the page that started it went quiet for over 500 ms");
  }

  @Test
  void aRunWhosePageClosesItsStreamIsCanceled() throws Exception {
    PitwallServer running = start(Duration.ofMillis(20));
    Listener page = new Listener(running);
    String stream = (String) Objects.requireNonNull(page.snapshot().json().get("stream"));
    CompletableFuture<HttpResponse<String>> started = post(running, "/api/experiments", stream);
    loopUntil(started::isDone);
    Run run = Objects.requireNonNull(byHand.experiments().run());

    page.close();
    loopUntil(() -> run.state().phase() != Run.Phase.RUNNING);

    assertThat(run.state().detail()).isEqualTo("Canceled: the page that started it closed");
  }

  private CompletableFuture<HttpResponse<String>> post(
      PitwallServer running, String path, String stream) {
    return client.sendAsync(
        HttpRequest.newBuilder(uri(running, path))
            .header("Content-Type", "application/json")
            .header("Pitwall-Stream", stream)
            .POST(HttpRequest.BodyPublishers.ofString("{\"start\": \"" + ByHand.STEP + "\"}"))
            .build(),
        HttpResponse.BodyHandlers.ofString());
  }

  private void heartbeat(PitwallServer running, String stream) throws Exception {
    var answer =
        client.send(
            HttpRequest.newBuilder(uri(running, "/api/heartbeat"))
                .header("Pitwall-Stream", stream)
                .POST(HttpRequest.BodyPublishers.noBody())
                .build(),
            HttpResponse.BodyHandlers.ofString());
    assertThat(answer.statusCode()).isEqualTo(204);
  }

  /** On 127.0.0.1: on Windows, localhost tries IPv6 first and costs about 200 ms each time. */
  private static URI uri(PitwallServer server, String path) {
    return URI.create("http://127.0.0.1:" + server.port() + path);
  }

  private static double number(@Nullable Object json) {
    return ((Number) Objects.requireNonNull(json)).doubleValue();
  }

  @SuppressWarnings("unchecked") // JSON objects parse as maps from names
  private static Map<String, Object> asMap(@Nullable Object json) {
    return (Map<String, Object>) Objects.requireNonNull(json);
  }
}
