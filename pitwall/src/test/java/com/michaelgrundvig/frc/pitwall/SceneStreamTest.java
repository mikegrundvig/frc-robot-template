package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.experiment.ByHand;
import com.michaelgrundvig.frc.pitwall.push.Broadcaster;
import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.Scene;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.pitwall.scene.SceneState;
import com.michaelgrundvig.frc.pitwall.scene.Scenes;
import com.michaelgrundvig.frc.pitwall.scene.Shapes;
import com.michaelgrundvig.frc.pitwall.scene.Style;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.io.UncheckedIOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.BooleanSupplier;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;

/**
 * A scene over real HTTP, as a page sees it: a robot driving on a field the size of an FRC one,
 * with its wheels, cameras, sight lines, and uncertainty, and the field's 32 tags. This is where
 * the scene's budget is measured: its description once, and then what a moving robot costs per
 * second, which must stay well under the 250 kbps the design allows (architecture, "Pitwall:
 * contributions and scenes"). A page that doesn't show the scene is sent none of it.
 */
class SceneStreamTest {
  private static final JsonType<Object> JSON = Jsonb.builder().build().type(Object.class);
  private static final long LOOP_NANOS = 20_000_000L;
  private static final Duration WAIT = Duration.ofSeconds(10);
  private static final List<String> CORNERS = List.of("fl", "fr", "bl", "br");

  private final AtomicLong clock = new AtomicLong(1_000_000_000L);
  private final TunableTee tunables =
      new TunableTee(new MockTunableBackend(), () -> null, inputs -> {}, false);
  private final Tuning tuning =
      new Tuning("test", tunables::configs, tunables::pathOf, true, inputs -> {}, view -> {});
  private final Actions actions =
      new Actions(inputs -> {})
          .add(
              "Calibrate the front camera",
              new Actions.Routine() {
                @Override
                public @Nullable String start() {
                  return null;
                }

                @Override
                public boolean isRunning() {
                  return false;
                }

                @Override
                public void cancel() {}
              });
  private final Scenes scenes = new Scenes(clock::get, System.out::println);
  private final Page page =
      new Page(
          new Status().add("Robot", () -> "test"),
          actions,
          tuning,
          tunables,
          new ByHand(clock::get).experiments(),
          () -> "",
          null,
          new Devices(clock::get),
          scenes);
  private final Broadcaster broadcaster =
      new Broadcaster(Broadcaster.MAX_STREAMS, Broadcaster.MAILBOX, id -> {});
  private final HttpClient client = HttpClient.newHttpClient();
  private final List<AutoCloseable> open = new ArrayList<>();
  private final PitwallServer server;

  /** Where the driving robot is, which way its wheels point, and what its cameras see. */
  private double x = 3;

  private double y = 4;
  private double heading;
  private double steer;
  private double speed;
  private int seenTag = 1;
  private boolean accepted = true;

  /** How long the scene's part of the loop took, in nanoseconds, loop by loop. */
  private final List<Long> sceneNanos = new ArrayList<>();

  SceneStreamTest() throws Exception {
    scenes.add(
        Scene.named("world", "World")
            .layer(Layer.of("field", "Field", SceneStreamTest::field))
            .layer(new Robot())
            .layer(new Rays()));
    server = new PitwallServer(page, broadcaster, 0);
    server.start(true);
  }

  @AfterEach
  void stop() throws Exception {
    for (AutoCloseable stream : open) {
      stream.close();
    }
    server.close();
  }

  /** A field of 16.5 by 8 m, its walls, four bumps, and 32 tags with their numbers. */
  private static void field(SceneBuilder scene) {
    scene
        .polyline("walls", "field")
        .point(0, 0, 0)
        .point(16.54, 0, 0)
        .point(16.54, 8.07, 0)
        .point(0, 8.07, 0)
        .closed();
    for (int bump = 0; bump < 4; bump++) {
      double at = 4 + bump * 2.8;
      scene
          .mesh("bump." + bump, "field")
          .vertex(at, 1, 0)
          .vertex(at + 1.1, 1, 0)
          .vertex(at + 0.55, 1, 0.165)
          .vertex(at, 2.8, 0)
          .vertex(at + 1.1, 2.8, 0)
          .vertex(at + 0.55, 2.8, 0.165)
          .triangle(0, 2, 5)
          .triangle(0, 5, 3)
          .triangle(1, 4, 5)
          .triangle(1, 5, 2);
    }
    for (int tag = 1; tag <= 32; tag++) {
      double along = 0.5 + (tag - 1) * 0.49;
      scene.frame("tag." + tag, "field", along, tag % 2 == 0 ? 0.01 : 8.06, 1.12, 0, 0, 1.57);
      scene.box("tag." + tag + ".plate", "tag." + tag).size(0.01, 0.165, 0.165).style(Style.MUTED);
      scene.label("tag." + tag + ".id", "tag." + tag).text(Integer.toString(tag)).at(0, 0, 0.15);
    }
  }

  /** A swerve robot: its frame, bumpers, front, wheels, cameras, ellipse, and the truth. */
  private final class Robot implements Layer {
    @Override
    public String id() {
      return "robot";
    }

    @Override
    public String title() {
      return "Robot";
    }

    @Override
    public void describe(SceneBuilder scene) {
      scene.frame("robot", "field").frame("truth", "field").frame("ellipse", "field");
      scene
          .box("frame", "robot")
          .size(0.71, 0.71, 0.5)
          .at(0, 0, 0.25)
          .outline()
          .style(Style.ACCENT);
      scene.box("bumpers", "robot").size(0.87, 0.87, 0.13).at(0, 0, 0.1).style(Style.ACCENT);
      scene.arrow("front", "robot").from(0, 0, 0.55).to(0.5, 0, 0.55).style(Style.ACCENT);
      for (String corner : CORNERS) {
        double cx = corner.startsWith("f") ? 0.29 : -0.29;
        double cy = corner.endsWith("l") ? 0.29 : -0.29;
        scene.frame("module." + corner, "robot", cx, cy, 0, 0, 0, 0).value("speed." + corner, 0.05);
        scene
            .cylinder("wheel." + corner, "module." + corner)
            .radius(0.05)
            .length(0.04)
            .at(0, 0, 0.05, Math.PI / 2, 0, 0)
            .style(Style.ACCENT);
        scene
            .arrow("drive." + corner, "module." + corner)
            .from(0, 0, 0.12)
            .to(1, 0, 0.12)
            .length("speed." + corner)
            .style(Style.ACCENT);
        String camera = "camera." + corner;
        scene.frame(camera, "robot", cx, cy, 0.53, 0, -0.17, cy > 0 ? 0.4 : -0.4);
        scene.arrow(camera + ".axis", camera).to(0.3, 0, 0).style(Style.ACCENT);
        scene
            .frustum(camera + ".view", camera)
            .fov(1.2, 0.8)
            .range(0, 7)
            .inspect(() -> Inspection.of(camera).action("Calibrate the front camera"));
        scene.label(camera + ".name", camera).text(corner).at(0, 0, 0.08);
      }
      scene.value("ellipse.major", 0.005).value("ellipse.minor", 0.005);
      scene
          .cylinder("ellipse", "ellipse")
          .radii("ellipse.major", "ellipse.minor")
          .outline()
          .style(Style.ACCENT.dashed());
      scene
          .box("ghost", "truth")
          .size(0.87, 0.87, 0.5)
          .at(0, 0, 0.25)
          .outline()
          .style(Style.TRUTH.dashed());
    }

    @Override
    public void update(SceneState state) {
      state.pose("robot", x, y, heading);
      state.pose("truth", x + 0.03, y - 0.02, heading + 0.01);
      state.pose("ellipse", x, y, heading);
      state.value("ellipse.major", 0.05 + speed * 0.01);
      state.value("ellipse.minor", 0.03);
      for (String corner : CORNERS) {
        double cx = corner.startsWith("f") ? 0.29 : -0.29;
        double cy = corner.endsWith("l") ? 0.29 : -0.29;
        state.pose("module." + corner, cx, cy, steer);
        state.value("speed." + corner, speed / 4);
      }
    }
  }

  /** Sight lines from the cameras to the tags they see, rebuilt when what's seen changes. */
  private final class Rays implements Layer {
    private int drawn;
    private boolean drawnAccepted;

    @Override
    public String id() {
      return "rays";
    }

    @Override
    public String title() {
      return "Sight lines";
    }

    @Override
    public void describe(SceneBuilder scene) {}

    @Override
    public void update(SceneState state) {
      if (drawn == seenTag && drawnAccepted == accepted) {
        return;
      }
      drawn = seenTag;
      drawnAccepted = accepted;
      Shapes shapes = state.shapes();
      for (String corner : List.of("fl", "fr")) {
        String id = "ray." + corner + "." + seenTag + (accepted ? ".ok" : ".bad");
        boolean ok = accepted;
        int tag = seenTag;
        shapes
            .line(id, "field")
            .between("camera." + corner, "tag." + seenTag)
            .style(ok ? Style.OK : Style.BAD.dashed())
            .inspect(
                () ->
                    Inspection.of(corner + " → tag " + tag)
                        .detail("Verdict", ok ? "accepted" : "rejected: gate 45.2 > 13.8"));
      }
    }
  }

  /** One robot loop, as Pitwall runs it: the scene's part timed. */
  private void step() {
    long start = System.nanoTime();
    broadcaster.periodic(page::sections, null, scenes.feeds());
    sceneNanos.add(System.nanoTime() - start);
  }

  private void loop() {
    step();
    clock.addAndGet(LOOP_NANOS);
  }

  /** One loop of driving in a circle: steering and speed changing, the tags seen changing. */
  private void drive(int i) {
    speed = 3 + Math.sin(i * 0.05);
    heading += 0.02;
    steer = Math.sin(i * 0.03);
    x += speed * LOOP_NANOS / 1e9 * Math.cos(heading);
    y += speed * LOOP_NANOS / 1e9 * Math.sin(heading);
    if (i % 10 == 0) {
      seenTag = 1 + (i / 10) % 32;
      accepted = (i / 10) % 3 != 0;
    }
    loop();
  }

  @Test
  void aMovingRobotsSceneStaysWithinItsBudgetAndAStillOneSendsNothing() throws Exception {
    Listener world = new Listener("/api/events?views=world");
    Listener tune = new Listener("/api/events");
    loopUntil(() -> world.has("replace") && tune.has("snapshot"));
    for (int i = 0; i < 250; i++) {
      loop();
    }
    String snapshot = world.data("snapshot");
    String replace = world.data("replace");
    int description =
        JSON.toJson(page.scenes().descriptions()).getBytes(StandardCharsets.UTF_8).length;
    int still = world.count("patch");

    world.mark();
    tune.mark();
    int loops = 250; // five seconds of driving
    for (int i = 0; i < loops; i++) {
      drive(i);
    }
    loopUntil(() -> world.arrived() >= 80);
    List<String> patches = world.since("patch");
    long bytes = 0;
    int largest = 0;
    for (String patch : patches) {
      int size = ("event: patch\ndata: " + patch + "\n\n").getBytes(StandardCharsets.UTF_8).length;
      bytes += size;
      largest = Math.max(largest, size);
    }
    double seconds = loops * LOOP_NANOS / 1e9;
    double kbps = bytes * 8 / seconds / 1000;
    List<Long> timed = sceneNanos.subList(sceneNanos.size() - loops, sceneNanos.size());
    double meanMicros = timed.stream().mapToLong(Long::longValue).average().orElse(0) / 1000;
    System.out.printf(
        Locale.ROOT,
        "Scene: description %d B (snapshot %d B, replace %d B); still: %d patches in 250 loops;"
            + " driving: %d patches in %.0f s, %.0f B each on average, %d B at most, %.1f kbps;"
            + " the loop's part %.1f us on average%n",
        description,
        snapshot.length(),
        replace.length(),
        still,
        patches.size(),
        seconds,
        patches.isEmpty() ? 0.0 : (double) bytes / patches.size(),
        largest,
        kbps,
        meanMicros);

    assertThat(description).as("the description, once").isLessThan(60_000);
    assertThat(still).as("patches while the robot stood still").isZero();
    assertThat(patches.size() / seconds).as("patches a second").isLessThanOrEqualTo(20);
    assertThat(kbps).as("kbps while driving").isLessThan(250);
    assertThat(tune.since("patch")).as("a page not showing the scene").isEmpty();
    assertThat(tune.data("snapshot")).contains("\"scenes\":{\"world\":{\"title\":\"World\"");
  }

  @Test
  void aPageThatTurnsToTheSceneIsSentAllOfItThenWhatMoves() throws Exception {
    Listener page = new Listener("/api/events");
    loopUntil(() -> page.has("snapshot"));
    String stream = String.valueOf(asMap(JSON.fromJson(page.data("snapshot"))).get("stream"));

    HttpResponse<String> shown = putViews(stream, "{\"views\": [\"world\"]}");
    assertThat(shown.statusCode()).isEqualTo(204);
    loopUntil(() -> page.has("replace"));
    assertThat(page.data("replace"))
        .startsWith("{\"at\":[\"scenes\",\"world\",\"state\"],\"value\":");
    drive(0);
    drive(1);
    clock.addAndGet(100_000_000L);
    loopUntil(() -> page.has("patch"));
    assertThat(page.data("patch")).contains("\"scenes\":{\"world\":{\"state\":{\"frames\":");

    assertThat(putViews("nobody", "{\"views\": [\"world\"]}").statusCode()).isEqualTo(404);
    assertThat(putViews(stream, "{\"views\": [\"not a name\"]}").statusCode()).isEqualTo(400);
    assertThat(putViews(stream, "{\"views\": \"world\"}").statusCode()).isEqualTo(400);
  }

  @Test
  void aClickIsAnsweredWithItsDetailsAndItsActionsAsTheyStand() throws Exception {
    Listener world = new Listener("/api/events?views=world");
    loopUntil(() -> world.has("replace"));
    accepted = false;
    seenTag = 7;
    loop();

    HttpResponse<String> ray = get("/api/scenes/world/inspect/ray.fl.7.bad");
    assertThat(ray.statusCode()).isEqualTo(200);
    assertThat(ray.body())
        .isEqualTo(
            "{\"scene\":\"world\",\"shape\":\"ray.fl.7.bad\",\"title\":\"fl → tag 7\","
                + "\"details\":[{\"label\":\"Verdict\",\"value\":\"rejected: gate 45.2 > 13.8\"}],"
                + "\"actions\":[]}");
    assertThat(get("/api/scenes/world/inspect/camera.fl.view").body())
        .contains("\"actions\":[{\"name\":\"Calibrate the front camera\"");
    assertThat(get("/api/scenes/world/inspect/walls").statusCode()).isEqualTo(404);
    assertThat(get("/api/scenes/elsewhere/inspect/walls").statusCode()).isEqualTo(404);
    assertThat(get("/api/scenes/world/inspect/..%2Findex.html").statusCode()).isEqualTo(404);
  }

  // ---- The page's side ----

  /** A page's stream, as its events arrive. */
  private final class Listener {
    private final List<String[]> events = new CopyOnWriteArrayList<>();
    private int marked;

    Listener(String path) throws Exception {
      CompletableFuture<HttpResponse<Stream<String>>> connecting =
          client.sendAsync(
              HttpRequest.newBuilder(uri(path)).GET().build(), HttpResponse.BodyHandlers.ofLines());
      loopUntil(connecting::isDone);
      Stream<String> lines = connecting.get().body();
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
        // The test closed it.
      }
    }

    boolean has(String name) {
      return events.stream().anyMatch(event -> event[0].equals(name));
    }

    int count(String name) {
      return (int) events.stream().filter(event -> event[0].equals(name)).count();
    }

    /** The latest event of a name's data. */
    String data(String name) {
      String found = null;
      for (String[] event : events) {
        if (event[0].equals(name)) {
          found = event[1];
        }
      }
      assertThat(found).as(name).isNotNull();
      return String.valueOf(found);
    }

    /** Remembers where the events stand, for {@link #since}. */
    void mark() {
      marked = events.size();
    }

    int arrived() {
      return events.size() - marked;
    }

    List<String> since(String name) {
      List<String> found = new ArrayList<>();
      // A copy first: a view of the list the stream's thread adds to throws as it's read.
      List<String[]> now = List.copyOf(events);
      for (String[] event : now.subList(marked, now.size())) {
        if (event[0].equals(name)) {
          found.add(event[1]);
        }
      }
      return found;
    }
  }

  /** Runs loops (the clock standing still) until the network has brought something. */
  private void loopUntil(BooleanSupplier done) {
    long deadline = System.nanoTime() + WAIT.toNanos();
    while (!done.getAsBoolean() && System.nanoTime() < deadline) {
      step();
      Thread.onSpinWait();
    }
    assertThat(done.getAsBoolean()).as("it happened within " + WAIT).isTrue();
  }

  private HttpResponse<String> putViews(String stream, String body) throws Exception {
    return client.send(
        HttpRequest.newBuilder(uri("/api/views"))
            .header("Content-Type", "application/json")
            .header("Pitwall-Stream", stream)
            .PUT(HttpRequest.BodyPublishers.ofString(body))
            .build(),
        HttpResponse.BodyHandlers.ofString());
  }

  private HttpResponse<String> get(String path) throws Exception {
    CompletableFuture<HttpResponse<String>> asked =
        client.sendAsync(
            HttpRequest.newBuilder(uri(path)).GET().build(), HttpResponse.BodyHandlers.ofString());
    loopUntil(asked::isDone);
    return asked.get();
  }

  private URI uri(String path) {
    return URI.create("http://127.0.0.1:" + server.port() + path);
  }

  @SuppressWarnings("unchecked") // a JSON object
  private static Map<String, Object> asMap(@Nullable Object json) {
    return (Map<String, Object>) Objects.requireNonNull(json);
  }
}
