package com.michaelgrundvig.frc.pitwall;

import static com.michaelgrundvig.frc.pitwall.Fixtures.machine;
import static org.assertj.core.api.Assertions.assertThat;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.InetAddress;
import java.net.Socket;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;

/** The REST API and the page, over real HTTP, with a thread playing the robot loop. */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class PitwallServerTest {
  private static final String MERGE_PATCH = "application/merge-patch+json";

  private final HttpClient client = HttpClient.newHttpClient();
  private final AtomicBoolean fieldConnected = new AtomicBoolean(false);
  private final AtomicBoolean homed = new AtomicBoolean(false);
  private final Status status =
      new Status().add("Robot", () -> "test").add("Arm homed", homed::get);
  private volatile Actions.Gate gate = new Actions.Gate(true, true, false);
  private final AtomicBoolean homing = new AtomicBoolean(false);
  private final Actions actions =
      new Actions(inputs -> {})
          .add(
              "Home arm",
              new Actions.Routine() {
                @Override
                public @Nullable String start() {
                  homing.set(true);
                  return null;
                }

                @Override
                public boolean isRunning() {
                  return homing.get();
                }

                @Override
                public void cancel() {
                  homing.set(false);
                }
              });
  private final TunableDouble shooterKp = TunableDouble.create(0.5);
  private final TunableTee tunables =
      new TunableTee(new MockTunableBackend(), this::whyLocked, inputs -> {}, false);
  private final Tuning tuning =
      new Tuning("test", tunables::configs, tunables::pathOf, true, inputs -> {}, view -> {});
  private final PitwallServer server = new PitwallServer(status, tuning, actions, tunables, 0);

  private @Nullable String whyLocked() {
    return tuning.whyLocked();
  }

  private final ScheduledExecutorService loop = Executors.newSingleThreadScheduledExecutor();

  @BeforeAll
  void start() throws Exception {
    tunables.publish("Shooter/kP", shooterKp);
    TunableRegistry.registerBackend("", tunables);
    TunableRegistry.publish(
        "Machine",
        LiveConfig.of("machine", machine())
            .number("drive.gains.kP", Limits.between(0, 100))
            .number("drive.limits.current", Limits.between(1, 80))
            .number("travel.min", Limits.NONE)
            .number("travel.max", Limits.NONE));
    server.start();
    loop.scheduleAtFixedRate(
        () -> {
          tuning.periodic(fieldConnected.get());
          actions.periodic(gate);
          tunables.update();
          status.periodic();
        },
        0,
        5,
        TimeUnit.MILLISECONDS);
  }

  @AfterAll
  void stop() throws Exception {
    TunableRegistry.reset();
    loop.shutdownNow();
    server.close();
  }

  @AfterEach
  void backToCommitted() throws Exception {
    fieldConnected.set(false);
    gate = new Actions.Gate(true, true, false);
    send("POST", "/api/actions", "application/json", "{\"cancel\": \"Home arm\"}");
    send("PATCH", "/api/tuning", MERGE_PATCH, "{\"enabled\": true}");
    send("DELETE", "/api/changes", null, null);
    send("PATCH", "/api/wpilib-tunables", MERGE_PATCH, "{\"Shooter/kP\": 0.5}");
  }

  @Test
  void servesThePage() throws Exception {
    var page = get("/", "text/html");

    assertThat(page.statusCode()).isEqualTo(200);
    assertThat(page.headers().firstValue("Content-Type")).hasValue("text/html; charset=utf-8");
    assertThat(page.body()).contains("<title>Pitwall</title>");
  }

  @Test
  void showsTheStatusAsOfTheLastLoop() throws Exception {
    homed.set(true);
    Thread.sleep(50); // a few loops

    assertThat(get("/api/status").body())
        .isEqualTo(
            "[{\"name\":\"Robot\",\"value\":\"test\"},"
                + "{\"name\":\"Arm homed\",\"value\":true}]");
  }

  @Test
  void anActionStartsFromThePageAndShowsItsState() throws Exception {
    assertThat(get("/api/actions").body()).contains("{\"name\":\"Home arm\",\"phase\":");

    var started = send("POST", "/api/actions", "application/json", "{\"start\": \"Home arm\"}");

    assertThat(started.statusCode()).isEqualTo(200);
    assertThat(started.body())
        .isEqualTo("[{\"name\":\"Home arm\",\"phase\":\"RUNNING\",\"detail\":\"Running\"}]");
    assertThat(homing.get()).isTrue();

    var canceled = send("POST", "/api/actions", "application/json", "{\"cancel\": \"Home arm\"}");

    assertThat(canceled.statusCode()).isEqualTo(200);
    assertThat(canceled.body()).contains("\"phase\":\"CANCELED\"");
    assertThat(homing.get()).isFalse();
  }

  @Test
  void anActionOutsideUtilityModeIsLockedAndSaysWhy() throws Exception {
    gate = new Actions.Gate(true, false, false);

    var locked = send("POST", "/api/actions", "application/json", "{\"start\": \"Home arm\"}");

    assertThat(locked.statusCode()).isEqualTo(409);
    assertThat(locked.body()).contains("enabled in utility mode");
    assertThat(homing.get()).isFalse();
  }

  @Test
  void onlyKnownActionsAndWellFormedRequestsAreAccepted() throws Exception {
    var unknown = send("POST", "/api/actions", "application/json", "{\"start\": \"Fly\"}");
    var both =
        send(
            "POST",
            "/api/actions",
            "application/json",
            "{\"start\": \"Home arm\", \"cancel\": \"Home arm\"}");
    var notAName = send("POST", "/api/actions", "application/json", "{\"start\": 3}");

    assertThat(unknown.statusCode()).isEqualTo(422);
    assertThat(unknown.body()).contains("There is no action named Fly");
    assertThat(both.statusCode()).isEqualTo(400);
    assertThat(notAName.statusCode()).isEqualTo(400);
  }

  @Test
  void everyOtherWpilibTunableIsListedAndCanBeChanged() throws Exception {
    assertThat(get("/api/wpilib-tunables").body())
        .isEqualTo(
            "[{\"path\":\"/Shooter/kP\",\"type\":\"double\",\"value\":0.5,\"editable\":true,"
                + "\"limits\":\"\"}]");

    var changed = send("PATCH", "/api/wpilib-tunables", MERGE_PATCH, "{\"Shooter/kP\": 0.75}");

    assertThat(changed.statusCode()).isEqualTo(200);
    assertThat(changed.body()).contains("\"value\":0.75");
  }

  @Test
  void onlyTunablesThePageCanChangeTakeOnlyNumbersAndBooleans() throws Exception {
    var unknown = send("PATCH", "/api/wpilib-tunables", MERGE_PATCH, "{\"Nope\": 1}");
    var notAValue =
        send("PATCH", "/api/wpilib-tunables", MERGE_PATCH, "{\"Shooter/kP\": \"fast\"}");
    fieldConnected.set(true);
    var locked = send("PATCH", "/api/wpilib-tunables", MERGE_PATCH, "{\"Shooter/kP\": 0.6}");

    assertThat(unknown.statusCode()).isEqualTo(422);
    assertThat(notAValue.statusCode()).isEqualTo(400);
    assertThat(locked.statusCode()).isEqualTo(409);
  }

  @Test
  void describesTheSession() throws Exception {
    assertThat(get("/api/tuning").body())
        .isEqualTo("{\"name\":\"test\",\"enabled\":true,\"fieldConnected\":false,\"refused\":[]}");
  }

  @Test
  void listsWhatCanBeTunedAndItsValues() throws Exception {
    assertThat(get("/api/tunables").body())
        .contains(
            "\"/Machine/drive/gains/kP\":{\"config\":\"machine\",\"field\":\"drive.gains.kP\",\"unit\":\"\","
                + "\"limits\":\"0 to 100\",\"committed\":16.0}");
    assertThat(get("/api/values").body()).contains("\"/Machine/drive/gains/kP\":16.0");
  }

  @Test
  void aMergePatchChangesSeveralValuesAtOnce() throws Exception {
    var changed =
        send(
            "PATCH",
            "/api/values",
            MERGE_PATCH,
            "{\"/Machine/travel/min\": 12, \"/Machine/travel/max\": 20}");

    assertThat(changed.statusCode()).isEqualTo(200);
    assertThat(changed.body())
        .contains("\"/Machine/travel/min\":12.0", "\"/Machine/travel/max\":20.0");
  }

  @Test
  void aRefusedPatchChangesNothingAndSaysWhy() throws Exception {
    var refused =
        send(
            "PATCH",
            "/api/values",
            MERGE_PATCH,
            "{\"/Machine/drive/gains/kP\": 18.5, \"/Machine/drive/limits/current\": -1}");

    assertThat(refused.statusCode()).isEqualTo(422);
    assertThat(refused.headers().firstValue("Content-Type"))
        .hasValue("application/problem+json; charset=utf-8");
    assertThat(refused.body())
        .isEqualTo(
            "{\"type\":\"about:blank\",\"title\":\"Refused\",\"status\":422,"
                + "\"detail\":\"/Machine/drive/limits/current: -1 is below its limit of 1\"}");
    assertThat(get("/api/values").body()).contains("\"/Machine/drive/gains/kP\":16.0");
  }

  /** Within its limits, but the configuration's own checks refuse it: they say why. */
  @Test
  void aValueTheConfigurationRejectsSaysWhy() throws Exception {
    var refused = send("PATCH", "/api/values", MERGE_PATCH, "{\"/Machine/travel/min\": 12}");

    assertThat(refused.statusCode()).isEqualTo(422);
    assertThat(refused.body()).contains("min must be below max, got 12.0 and 10.0");
  }

  @Test
  void nullReturnsAValueToItsCommittedOne() throws Exception {
    send("PATCH", "/api/values", MERGE_PATCH, "{\"/Machine/drive/gains/kP\": 18.5}");

    var reverted = send("PATCH", "/api/values", MERGE_PATCH, "{\"/Machine/drive/gains/kP\": null}");

    assertThat(reverted.statusCode()).isEqualTo(200);
    assertThat(reverted.body()).contains("\"/Machine/drive/gains/kP\":16.0");
  }

  @Test
  void theChangesComeAsJsonOrAsAReport() throws Exception {
    send("PATCH", "/api/values", "application/json", "{\"/Machine/drive/gains/kP\": 18.5}");

    assertThat(get("/api/changes").body()).contains("\"builderCall\":\".kP(18.5)\"");
    assertThat(get("/api/changes", "text/plain").body())
        .contains("  drive.gains.kP: 16.0 -> 18.5   .kP(18.5)");
  }

  @Test
  void deletingTheChangesRevertsEverything() throws Exception {
    send("PATCH", "/api/values", MERGE_PATCH, "{\"/Machine/drive/gains/kP\": 18.5}");

    var deleted = send("DELETE", "/api/changes", null, null);

    assertThat(deleted.statusCode()).isEqualTo(200);
    assertThat(deleted.body()).isEqualTo("[]");
    assertThat(get("/api/values").body()).contains("\"/Machine/drive/gains/kP\":16.0");
  }

  @Test
  void switchingTuningOffLocksChanges() throws Exception {
    var off = send("PATCH", "/api/tuning", MERGE_PATCH, "{\"enabled\": false}");
    var locked = send("PATCH", "/api/values", MERGE_PATCH, "{\"/Machine/drive/gains/kP\": 18.5}");

    assertThat(off.body()).contains("\"enabled\":false");
    assertThat(locked.statusCode()).isEqualTo(409);
    assertThat(locked.body()).contains("\"detail\":\"Tuning is locked: tuning is off\"");
  }

  @Test
  void theFieldLocksTuning() throws Exception {
    fieldConnected.set(true);

    var change = send("PATCH", "/api/values", MERGE_PATCH, "{\"/Machine/drive/gains/kP\": 18.5}");
    var on = send("PATCH", "/api/tuning", MERGE_PATCH, "{\"enabled\": true}");

    assertThat(change.statusCode()).isEqualTo(409);
    assertThat(on.statusCode()).isEqualTo(409);
    assertThat(on.body()).contains("the field system is connected");
  }

  @Test
  void refusesWhatItCannotUse() throws Exception {
    assertThat(send("PATCH", "/api/values", "text/plain", "{}").statusCode())
        .as("not a merge patch")
        .isEqualTo(415);
    assertThat(send("PATCH", "/api/values", MERGE_PATCH, "not json").statusCode()).isEqualTo(400);
    assertThat(send("PATCH", "/api/values", MERGE_PATCH, "[1, 2]").statusCode()).isEqualTo(400);
    assertThat(send("PATCH", "/api/values", MERGE_PATCH, "{\"/Machine/drive/gains/kP\": \"lots\"}"))
        .extracting(HttpResponse::statusCode)
        .isEqualTo(400);
    var unknown = send("PATCH", "/api/values", MERGE_PATCH, "{\"nothing\": 1}");
    assertThat(unknown.statusCode()).isEqualTo(422);
    assertThat(unknown.body()).contains("There is no tunable value at nothing");
    assertThat(send("PATCH", "/api/tuning", MERGE_PATCH, "{\"name\": \"other\"}").statusCode())
        .as("only enabled can change")
        .isEqualTo(422);
    assertThat(send("PATCH", "/api/tuning", MERGE_PATCH, "{\"enabled\": \"yes\"}").statusCode())
        .isEqualTo(400);
    assertThat(send("PATCH", "/api/tuning", MERGE_PATCH, "{}").statusCode()).isEqualTo(200);
  }

  /** 1e999 parses to infinity: no number that isn't finite gets as far as the robot loop. */
  @Test
  void refusesNumbersThatArentFinite() throws Exception {
    var config = send("PATCH", "/api/values", MERGE_PATCH, "{\"/Machine/drive/gains/kP\": 1e999}");
    var wpilib = send("PATCH", "/api/wpilib-tunables", MERGE_PATCH, "{\"Shooter/kP\": -1e999}");

    assertThat(config.statusCode()).isEqualTo(400);
    assertThat(wpilib.statusCode()).isEqualTo(400);
    assertThat(shooterKp.get()).isEqualTo(0.5);
  }

  /** The parser recurses: a deep enough body would overflow its stack, so it's refused first. */
  @Test
  void refusesDeeplyNestedJson() throws Exception {
    String deep = "{\"a\": " + "[".repeat(20_000) + "]".repeat(20_000) + "}";

    var nested = send("PATCH", "/api/values", MERGE_PATCH, deep);

    assertThat(nested.statusCode()).isEqualTo(400);
    assertThat(nested.body()).contains("Too deeply nested");
    assertThat(send("PATCH", "/api/values", MERGE_PATCH, "{\"note\": \"[[[[[[[[[[\"}"))
        .as("brackets in a string don't count")
        .extracting(HttpResponse::statusCode)
        .isEqualTo(422);
  }

  @Test
  void saysWhichMethodsAResourceAllows() throws Exception {
    var notAllowed = send("DELETE", "/api/values", null, null);

    assertThat(notAllowed.statusCode()).isEqualTo(405);
    assertThat(notAllowed.headers().firstValue("Allow")).hasValue("GET, PATCH");
  }

  @Test
  void anythingElseIsNotFound() throws Exception {
    var missing = get("/admin");

    assertThat(missing.statusCode()).isEqualTo(404);
    assertThat(missing.body()).contains("\"title\":\"Not found\"");
  }

  @Test
  void aChangeTheLoopHasNotReachedIsAccepted() throws Exception {
    // A robot program whose loop never runs, as if paused in a debugger.
    var idle = new Tuning("idle", List::of, (config, field) -> null, true, inputs -> {}, v -> {});
    try (var paused =
        new PitwallServer(
            new Status(),
            idle,
            new Actions(inputs -> {}),
            new TunableTee(new MockTunableBackend(), () -> null, inputs -> {}, false),
            0,
            Duration.ofMillis(50))) {
      paused.start();
      var accepted =
          client.send(
              HttpRequest.newBuilder(
                      URI.create("http://localhost:" + paused.port() + "/api/tuning"))
                  .header("Content-Type", MERGE_PATCH)
                  .method("PATCH", HttpRequest.BodyPublishers.ofString("{\"enabled\": false}"))
                  .build(),
              HttpResponse.BodyHandlers.ofString());

      assertThat(accepted.statusCode()).isEqualTo(202);
      assertThat(idle.view().enabled()).as("the loop never got to it").isTrue();
    }
  }

  @Test
  void startOrExplainStartsThePageAndSaysWhere() throws Exception {
    List<String> said = new ArrayList<>();

    Optional<PitwallServer> started =
        PitwallServer.startOrExplain(status, tuning, actions, tunables, 0, said::add);

    assertThat(started).isPresent();
    try {
      assertThat(said).singleElement().asString().startsWith("Pitwall: http://localhost:");
    } finally {
      started.get().close();
    }
  }

  @Test
  void aPageThatCannotStartOnlySaysWhy() {
    List<String> said = new ArrayList<>();

    // The test's own server already holds this port.
    Optional<PitwallServer> started =
        PitwallServer.startOrExplain(status, tuning, actions, tunables, server.port(), said::add);

    assertThat(started).isEmpty();
    assertThat(said).singleElement().asString().startsWith("Pitwall's page couldn't start");
  }

  /**
   * A web page on a laptop connected to the robot could point a name it controls at the robot's
   * address (DNS rebinding) and use the page as its own: the name it sends gives it away.
   */
  @Test
  void answersOnlyAtTheRobotsOwnAddress() throws Exception {
    assertThat(statusLine("localhost:" + server.port())).endsWith(" 200 OK");
    assertThat(statusLine("robot.local:5820")).endsWith(" 200 OK");
    assertThat(statusLine("10.26.11.2:5820")).endsWith(" 200 OK");
    assertThat(statusLine("rebinding.example.com:5820")).contains(" 403 ");
  }

  @Test
  void changesComeOnlyFromThePagesOwnSite() throws Exception {
    var own =
        client.send(
            HttpRequest.newBuilder(uri("/api/values"))
                .header("Content-Type", MERGE_PATCH)
                .header("Origin", "http://localhost:" + server.port())
                .method(
                    "PATCH",
                    HttpRequest.BodyPublishers.ofString("{\"/Machine/drive/gains/kP\": 18}"))
                .build(),
            HttpResponse.BodyHandlers.ofString());
    var another =
        client.send(
            HttpRequest.newBuilder(uri("/api/values"))
                .header("Content-Type", MERGE_PATCH)
                .header("Origin", "http://another.example.com")
                .method(
                    "PATCH",
                    HttpRequest.BodyPublishers.ofString("{\"/Machine/drive/gains/kP\": 19}"))
                .build(),
            HttpResponse.BodyHandlers.ofString());

    assertThat(own.statusCode()).isEqualTo(200);
    assertThat(another.statusCode()).isEqualTo(403);
    assertThat(get("/api/values").body()).contains("\"/Machine/drive/gains/kP\":18.0");
  }

  @Test
  void thePageCantBeFramedByAnotherSite() throws Exception {
    var page = get("/", "text/html");

    assertThat(page.headers().firstValue("X-Frame-Options")).hasValue("DENY");
    assertThat(page.headers().firstValue("Content-Security-Policy"))
        .hasValueSatisfying(policy -> assertThat(policy).contains("frame-ancestors 'none'"));
    assertThat(page.headers().firstValue("X-Content-Type-Options")).hasValue("nosniff");
  }

  @Test
  void refusesBodiesTooLargeOrOfNoType() throws Exception {
    String large = "{\"note\": \"" + "x".repeat(64 * 1024) + "\"}";

    assertThat(send("PATCH", "/api/values", MERGE_PATCH, large).statusCode()).isEqualTo(413);
    assertThat(send("PATCH", "/api/values", null, "{}").statusCode()).isEqualTo(415);
  }

  /** The status line of a GET naming this host, sent by hand: Java's client won't set Host. */
  private String statusLine(String host) throws Exception {
    try (Socket socket = new Socket(InetAddress.getLoopbackAddress(), server.port());
        var reader =
            new BufferedReader(
                new InputStreamReader(socket.getInputStream(), StandardCharsets.US_ASCII))) {
      socket
          .getOutputStream()
          .write(
              ("GET /api/tuning HTTP/1.1\r\nHost: " + host + "\r\nConnection: close\r\n\r\n")
                  .getBytes(StandardCharsets.US_ASCII));
      return reader.readLine();
    }
  }

  private HttpResponse<String> get(String path) throws Exception {
    return get(path, "application/json");
  }

  private HttpResponse<String> get(String path, String accept) throws Exception {
    return client.send(
        HttpRequest.newBuilder(uri(path)).header("Accept", accept).GET().build(),
        HttpResponse.BodyHandlers.ofString());
  }

  private HttpResponse<String> send(
      String method, String path, @Nullable String type, @Nullable String body) throws Exception {
    var request = HttpRequest.newBuilder(uri(path));
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

  private URI uri(String path) {
    return URI.create("http://localhost:" + server.port() + path);
  }
}
