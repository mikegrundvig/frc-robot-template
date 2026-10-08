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
import java.util.concurrent.TimeoutException;
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

  /** The longest the test waits, in real time, for the network. */
  private static final Duration WAIT = Duration.ofSeconds(10);

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
            byHand.experiments().lost(stream);
            closed.add(stream); // last: a test that sees it closed sees the experiments told
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
   * Runs the loop a step at a time until an answer comes, waiting up to 50 ms of real time for it
   * after each: the time it takes is the network's, not the robot's.
   */
  private <T> T answered(CompletableFuture<T> answer) throws Exception {
    long deadline = System.nanoTime() + WAIT.toNanos();
    while (System.nanoTime() < deadline) {
      step();
      try {
        return answer.get(50, TimeUnit.MILLISECONDS);
      } catch (TimeoutException notYet) {
        // Another loop, then.
      }
    }
    throw new AssertionError("no answer within " + WAIT);
  }

  /**
   * Runs the loop until something the network brings is true, for at most ten seconds of real time.
   * The deadman's clock stands still meanwhile: how many loops it takes depends on how busy the
   * computer is.
   */
  private void loopUntil(BooleanSupplier done) {
    long deadline = System.nanoTime() + WAIT.toNanos();
    while (!done.getAsBoolean() && System.nanoTime() < deadline) {
      step();
      Thread.onSpinWait();
    }
    assertThat(done.getAsBoolean()).as("it happened within " + WAIT).isTrue();
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
      response = answered(connecting);
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

    /** The next event but a heartbeat, running the loop until it comes. */
    Sse next() {
      while (true) {
        loopUntil(() -> !events.isEmpty());
        Sse event = Objects.requireNonNull(events.poll());
        if (!event.event().equals("heartbeat")) {
          return event;
        }
      }
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

    assertThat(snapshot.get("stream")).as("its own id, random").asString().hasSize(36);
    Map<String, Object> document = asMap(snapshot.get("document"));
    assertThat(document)
        .containsKeys(
            "robot", "status", "units", "actions", "tuning", "configs", "tunables", "values")
        .containsKeys("changes")
        .containsKeys("wpilibTunables", "rigs", "experiments", "run");
    assertThat(asMap(document.get("status"))).containsEntry("Robot", "test");
    assertThat(number(asMap(document.get("values")).get(KP))).isEqualTo(16.0);
    assertThat(asMap(asMap(document.get("configs")).get("machine")))
        .as("documentation travels in the snapshot")
        .containsEntry("record", "Fixtures.Machine");
    assertThat(asMap(asMap(document.get("tunables")).get(KP)))
        .containsEntry("record", "Fixtures.Gains")
        .containsEntry("group", "drive.gains")
        .containsEntry("meaning", "volts per meter per second of error");

    for (int i = 0; i < 500; i++) {
      loop();
    }
    // A stream carries its events in order: if the next is this change's patch, nothing came
    // before it.
    tunables.requestChanges(Map.of(KP, 18.5));
    Sse patch = page.next();
    for (int i = 0; i < 100; i++) {
      loop();
    }
    tunables.requestChanges(Map.of(KP, 19.0));
    Sse marker = page.next();

    assertThat(marker.json()).as("one change, one patch").containsOnlyKeys("values", "changes");
    assertThat(number(asMap(marker.json().get("values")).get(KP))).isEqualTo(19.0);
    assertThat(patch.event()).as("500 loops, nothing changed, then the patch").isEqualTo("patch");
    assertThat(patch.json())
        .as("not the documentation, though its section was rebuilt too")
        .containsOnlyKeys("values", "changes");
    assertThat(asMap(patch.json().get("values"))).containsOnlyKeys(KP);
    assertThat(number(asMap(patch.json().get("values")).get(KP))).isEqualTo(18.5);
    assertThat(asMap(asMap(patch.json().get("changes")).get(KP)))
        .containsEntry("builderCall", ".kP(18.5)");
  }

  /**
   * The documentation section is built when the set of configurations changes, never for a value:
   * it was read when they were published.
   */
  @Test
  void documentationIsBuiltOnlyForNewConfigurations() {
    Map<String, Map<String, Object>> before = page.sections();

    tunables.requestChanges(Map.of(KP, 18.5));
    loop();
    Map<String, Map<String, Object>> after = page.sections();

    assertThat(after.get("values")).containsEntry(KP, 18.5);
    assertThat(after.get("tunables"))
        .as("rebuilt for the change")
        .isNotSameAs(before.get("tunables"));
    assertThat(after.get("configs")).isSameAs(before.get("configs"));
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
    String stream = (String) Objects.requireNonNull(page.snapshot().json().get("stream"));
    assertThat(broadcaster.size()).isEqualTo(1);

    page.close();

    loopUntil(() -> broadcaster.size() == 0);
    assertThat(closed).containsExactly(stream);
  }

  /** With eight pages following, a ninth takes the place of the one that wrote least lately. */
  @Test
  void aNinthPageTakesTheStalestPlace() throws Exception {
    PitwallServer running = start(PitwallServer.IDLE);
    List<Object> streams = new ArrayList<>();
    for (int i = 0; i < Broadcaster.MAX_STREAMS; i++) {
      Listener page = new Listener(running);
      streams.add(page.snapshot().json().get("stream"));
    }

    Listener ninth = new Listener(running);

    assertThat(ninth.snapshot().json().get("stream")).isNotIn(streams);
    loopUntil(() -> closed.size() == 1);
    assertThat(streams).contains(closed.getFirst());
    assertThat(broadcaster.size()).isEqualTo(Broadcaster.MAX_STREAMS);
  }

  /** A page that reconnects names the stream it had, which closes at once. */
  @Test
  void aPageThatReconnectsReplacesItsStream() throws Exception {
    PitwallServer running = start(PitwallServer.IDLE);
    String had =
        (String) Objects.requireNonNull(new Listener(running).snapshot().json().get("stream"));

    var again =
        answered(
            client.sendAsync(
                HttpRequest.newBuilder(uri(running, "/api/events?replaces=" + had)).GET().build(),
                HttpResponse.BodyHandlers.ofLines()));
    open.add(again.body()::close);

    loopUntil(() -> closed.contains(had));
    assertThat(broadcaster.size()).isEqualTo(1);
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

    assertThat(answered(post(running, "/api/experiments", stream)).statusCode()).isEqualTo(200);
    Run run = Objects.requireNonNull(byHand.experiments().run());
    for (int i = 0; i < 120; i++) {
      if (i % 10 == 0) {
        heartbeat(running, stream); // every 200 ms of the robot's time
      }
      loop();
    }
    // One samples event a loop while it ran, 106 of them; wait for each, then see none follows.
    int samples = 0;
    while (samples < 106) {
      Sse event = page.events.poll(WAIT.toMillis(), TimeUnit.MILLISECONDS);
      assertThat(event).as("106 samples events, within " + WAIT).isNotNull();
      samples += Objects.requireNonNull(event).event().equals("samples") ? 1 : 0;
    }
    tunables.requestChanges(Map.of(KP, 18.5));
    Sse marker = page.next();

    assertThat(run.state().phase()).isEqualTo(Run.Phase.ENDED);
    assertThat(marker.event())
        .as("nothing after the last samples but the next change")
        .isEqualTo("patch");
    assertThat(asMap(marker.json().get("values"))).containsKey(KP);

    answered(post(running, "/api/experiments", stream));
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
    answered(post(running, "/api/experiments", stream));
    Run run = Objects.requireNonNull(byHand.experiments().run());

    page.close();
    // The server notices at its next write, a heartbeat within 20 ms; the robot waits meanwhile.
    long deadline = System.nanoTime() + WAIT.toNanos();
    while (!closed.contains(stream) && System.nanoTime() < deadline) {
      Thread.onSpinWait();
    }
    loop();

    assertThat(closed).contains(stream);
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
