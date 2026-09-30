package com.michaelgrundvig.frc.pitwall;

import static com.michaelgrundvig.frc.pitwall.Fixtures.machine;
import static org.assertj.core.api.Assertions.assertThat;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
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
  private final Tuning<Fixtures.Machine> tuning =
      new Tuning<>("test", machine(), true, inputs -> {}, view -> {});
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
      new TunableTee(
          new MockTunableBackend(),
          () -> fieldConnected.get() ? "the field system is connected" : null,
          inputs -> {},
          false);
  private final PitwallServer server = new PitwallServer(status, tuning, actions, tunables, 0);
  private final ScheduledExecutorService loop = Executors.newSingleThreadScheduledExecutor();

  @BeforeAll
  void start() throws Exception {
    tunables.publish("Shooter/kP", shooterKp);
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
            "[{\"path\":\"/Shooter/kP\",\"type\":\"double\",\"value\":0.5,\"editable\":true}]");

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
        .isEqualTo("{\"name\":\"test\",\"enabled\":true,\"fieldConnected\":false}");
  }

  @Test
  void listsWhatCanBeTunedAndItsValues() throws Exception {
    assertThat(get("/api/tunables").body())
        .contains("\"drive.gains.kP\":{\"unit\":\"\",\"committed\":16.0}");
    assertThat(get("/api/values").body()).contains("\"drive.gains.kP\":16.0");
  }

  @Test
  void aMergePatchChangesSeveralValuesAtOnce() throws Exception {
    var changed =
        send("PATCH", "/api/values", MERGE_PATCH, "{\"travel.min\": 12, \"travel.max\": 20}");

    assertThat(changed.statusCode()).isEqualTo(200);
    assertThat(changed.body()).contains("\"travel.min\":12.0", "\"travel.max\":20.0");
  }

  @Test
  void aRefusedPatchChangesNothingAndSaysWhy() throws Exception {
    var refused =
        send(
            "PATCH",
            "/api/values",
            MERGE_PATCH,
            "{\"drive.gains.kP\": 18.5, \"drive.limits.current\": -1}");

    assertThat(refused.statusCode()).isEqualTo(422);
    assertThat(refused.headers().firstValue("Content-Type"))
        .hasValue("application/problem+json; charset=utf-8");
    assertThat(refused.body())
        .isEqualTo(
            "{\"type\":\"about:blank\",\"title\":\"Refused\",\"status\":422,"
                + "\"detail\":\"current must be positive\"}");
    assertThat(get("/api/values").body()).contains("\"drive.gains.kP\":16.0");
  }

  @Test
  void nullReturnsAValueToItsCommittedOne() throws Exception {
    send("PATCH", "/api/values", MERGE_PATCH, "{\"drive.gains.kP\": 18.5}");

    var reverted = send("PATCH", "/api/values", MERGE_PATCH, "{\"drive.gains.kP\": null}");

    assertThat(reverted.statusCode()).isEqualTo(200);
    assertThat(reverted.body()).contains("\"drive.gains.kP\":16.0");
  }

  @Test
  void theChangesComeAsJsonOrAsAReport() throws Exception {
    send("PATCH", "/api/values", "application/json", "{\"drive.gains.kP\": 18.5}");

    assertThat(get("/api/changes").body()).contains("\"builderCall\":\".kP(18.5)\"");
    assertThat(get("/api/changes", "text/plain").body())
        .contains("  drive.gains.kP: 16.0 -> 18.5   .kP(18.5)");
  }

  @Test
  void deletingTheChangesRevertsEverything() throws Exception {
    send("PATCH", "/api/values", MERGE_PATCH, "{\"drive.gains.kP\": 18.5}");

    var deleted = send("DELETE", "/api/changes", null, null);

    assertThat(deleted.statusCode()).isEqualTo(200);
    assertThat(deleted.body()).isEqualTo("[]");
    assertThat(get("/api/values").body()).contains("\"drive.gains.kP\":16.0");
  }

  @Test
  void switchingTuningOffLocksChanges() throws Exception {
    var off = send("PATCH", "/api/tuning", MERGE_PATCH, "{\"enabled\": false}");
    var locked = send("PATCH", "/api/values", MERGE_PATCH, "{\"drive.gains.kP\": 18.5}");

    assertThat(off.body()).contains("\"enabled\":false");
    assertThat(locked.statusCode()).isEqualTo(409);
    assertThat(locked.body()).contains("\"detail\":\"Tuning is locked: tuning is off\"");
  }

  @Test
  void theFieldLocksTuning() throws Exception {
    fieldConnected.set(true);

    var change = send("PATCH", "/api/values", MERGE_PATCH, "{\"drive.gains.kP\": 18.5}");
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
    assertThat(send("PATCH", "/api/values", MERGE_PATCH, "{\"drive.gains.kP\": \"lots\"}"))
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
    var config = send("PATCH", "/api/values", MERGE_PATCH, "{\"drive.gains.kP\": 1e999}");
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
    var idle = new Tuning<>("idle", machine(), true, inputs -> {}, view -> {});
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
                      URI.create("http://localhost:" + paused.port() + "/api/values"))
                  .header("Content-Type", MERGE_PATCH)
                  .method(
                      "PATCH", HttpRequest.BodyPublishers.ofString("{\"drive.gains.kP\": 18.5}"))
                  .build(),
              HttpResponse.BodyHandlers.ofString());

      assertThat(accepted.statusCode()).isEqualTo(202);
      assertThat(idle.isTunable("drive.gains.kP")).isTrue();
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
