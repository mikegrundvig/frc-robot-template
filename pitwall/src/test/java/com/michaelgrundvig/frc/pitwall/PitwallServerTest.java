package com.michaelgrundvig.frc.pitwall;

import static com.michaelgrundvig.frc.pitwall.Fixtures.machine;
import static com.michaelgrundvig.frc.pitwall.experiment.ByHand.STEP;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.experiment.ByHand;
import com.michaelgrundvig.frc.pitwall.push.Broadcaster;
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
  private final ByHand byHand = new ByHand(System::nanoTime);
  private final Page page = new Page(status, actions, tuning, tunables, byHand.experiments());
  private final Broadcaster broadcaster =
      new Broadcaster(Broadcaster.MAX_STREAMS, Broadcaster.MAILBOX, stream -> {});
  private final PitwallServer server = new PitwallServer(page, broadcaster, 0);

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
          byHand.experiments().periodic(gate);
          status.periodic(broadcaster.watched());
          broadcaster.periodic(page::sections, byHand.experiments().run());
          byHand.step();
          tunables.update();
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
    send("POST", "/api/experiments", "application/json", "{\"cancel\": \"" + STEP + "\"}");
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
                + "\"limits\":\"0 to 100\",\"committed\":16.0,\"record\":\"Fixtures.Gains\","
                + "\"group\":\"drive.gains\",\"meaning\":\"volts per meter per second of error\"}")
        .as("undocumented")
        .contains("\"record\":\"Fixtures.Range\",\"group\":\"travel\",\"meaning\":\"\"");
    assertThat(get("/api/values").body()).contains("\"/Machine/drive/gains/kP\":16.0");
  }

  /** What the configurations' records say, for the page to show beside their values. */
  @Test
  void documentsEachConfigurationByItsRecords() throws Exception {
    assertThat(get("/api/configs").body())
        .isEqualTo(
            "{\"machine\":{\"record\":\"Fixtures.Machine\",\"doc\":{\"summary\":\"\",\"text\":\"\"},"
                + "\"groups\":["
                + "{\"path\":\"drive\",\"record\":\"Fixtures.Drive\","
                + "\"doc\":{\"summary\":\"A drive.\",\"text\":\"A drive.\"},\"meaning\":\"\"},"
                + "{\"path\":\"drive.gains\",\"record\":\"Fixtures.Gains\","
                + "\"doc\":{\"summary\":\"How the drive is controlled.\",\"text\":\"How the drive"
                + " is controlled. Tuned on a running machine, then committed.\\n\\nStart `kP`"
                + " low.\"},\"meaning\":\"how it's controlled\"},"
                + "{\"path\":\"drive.limits\",\"record\":\"Fixtures.DriveLimits\","
                + "\"doc\":{\"summary\":\"The drive's limits.\",\"text\":\"The drive's limits.\"},"
                + "\"meaning\":\"\"},"
                + "{\"path\":\"travel\",\"record\":\"Fixtures.Range\","
                + "\"doc\":{\"summary\":\"\",\"text\":\"\"},\"meaning\":\"\"}]}}");
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
            new Page(
                new Status(),
                new Actions(inputs -> {}),
                idle,
                new TunableTee(new MockTunableBackend(), () -> null, inputs -> {}, false),
                new ByHand(System::nanoTime).experiments()),
            new Broadcaster(1, 1, stream -> {}),
            0,
            Duration.ofMillis(50),
            PitwallServer.IDLE)) {
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
      var status =
          client.send(
              HttpRequest.newBuilder(
                      URI.create("http://localhost:" + paused.port() + "/api/status"))
                  .GET()
                  .build(),
              HttpResponse.BodyHandlers.ofString());
      assertThat(status.statusCode()).as("no reading passed off as now").isEqualTo(503);
      assertThat(status.body()).contains("didn't read the status within 50 ms");
    }
  }

  @Test
  void startOrExplainStartsThePageAndSaysWhere() throws Exception {
    List<String> said = new ArrayList<>();

    Optional<PitwallServer> started = PitwallServer.startOrExplain(page, broadcaster, 0, said::add);

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
        PitwallServer.startOrExplain(page, broadcaster, server.port(), said::add);

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

  @Test
  void aStatusItemWithAResolutionShowsToItAndItsUnit() throws Exception {
    var battery =
        new Status().add("Battery", () -> 12.345, "V", 0.1).add("Pressure", () -> 99.5, "psi", 1);

    battery.periodic(true);

    assertThat(battery.view())
        .containsExactly(
            new Status.Item("Battery", "12.3 V"), new Status.Item("Pressure", "100 psi"));
  }

  @Test
  void theExperimentsAreListedWithTheirRigs() throws Exception {
    assertThat(get("/api/experiments").body())
        .isEqualTo(
            "{\"rigs\":{\"Arm\":{\"config\":\"arm\",\"gains\":[\"gains.kP\",\"gains.kD\"],"
                + "\"limits\":\"-100 ° to 100 °\",\"unit\":\"°\",\"measurements\":[\"Angle\"],"
                + "\"signals\":[{\"name\":\"Voltage\",\"unit\":\"V\"}]}},"
                + "\"experiments\":{\""
                + STEP
                + "\":{\"rig\":\"Arm\",\"settings\":\"step and back, 0 ° to 90 °, held 0.8 s\"}}}");
  }

  @Test
  void anExperimentStartsFromThePageAndItsRunIsServedWithItsTrace() throws Exception {
    var started = sendAs("9", "{\"start\": \"" + STEP + "\"}");
    Thread.sleep(50); // a few loops

    assertThat(started.statusCode()).isEqualTo(200);
    assertThat(started.body())
        .startsWith("{\"id\":")
        .contains(
            "\"experiment\":\"" + STEP + "\"", "\"phase\":\"RUNNING\"", "\"controller\":\"9\"");
    assertThat(get("/api/run").body()).contains("\"trace\":{\"run\":", "\"Setpoint\":{\"t\":[");
    String id = started.body().substring("{\"id\":".length()).split(",")[0];

    var canceled =
        sendAs(
            "9",
            "{\"cancel\": \""
                + STEP
                + "\", \"run\": "
                + id
                + ", \"reason\": \"this tab was hidden\"}");
    var again = sendAs("9", "{\"cancel\": \"" + STEP + "\", \"run\": " + id + "}");

    assertThat(canceled.body()).contains("\"phase\":\"CANCELED\"", "Canceled: this tab was hidden");
    assertThat(again.statusCode()).as("canceling what isn't running").isEqualTo(409);
    assertThat(again.body()).contains(STEP + " isn't running");
  }

  /** Only a page following the robot can keep a run going, so only a page starts one. */
  @Test
  void aStartMustNameThePagesStream() throws Exception {
    var started =
        send("POST", "/api/experiments", "application/json", "{\"start\": \"" + STEP + "\"}");

    assertThat(started.statusCode()).isEqualTo(400);
    assertThat(started.body()).contains("Start experiments from Pitwall's page");
  }

  @Test
  void onlyKnownExperimentsAndWellFormedRequestsAreAccepted() throws Exception {
    var unknown = sendAs("9", "{\"start\": \"Fly\"}");
    var notAName = sendAs("9", "{\"start\": 3}");
    var neither = sendAs("9", "{\"run\": \"x\"}");
    var both = sendAs("9", "{\"start\": \"" + STEP + "\", \"cancel\": \"" + STEP + "\"}");
    var badRun = sendAs("9", "{\"cancel\": \"" + STEP + "\", \"run\": \"one\"}");
    var badReason = sendAs("9", "{\"cancel\": \"" + STEP + "\", \"reason\": 3}");
    var idle = sendAs("9", "{\"cancel\": \"" + STEP + "\"}");

    assertThat(unknown.statusCode()).isEqualTo(422);
    assertThat(unknown.body()).contains("There is no experiment named Fly");
    assertThat(List.of(notAName, neither, both, badRun, badReason))
        .extracting(HttpResponse::statusCode)
        .containsOnly(400);
    assertThat(idle.statusCode()).isEqualTo(409);
  }

  @Test
  void aHeartbeatIsAcceptedAndSaysNothing() throws Exception {
    var heartbeat = send("POST", "/api/heartbeat", null, null);

    assertThat(heartbeat.statusCode()).isEqualTo(204);
    assertThat(heartbeat.body()).isEmpty();
  }

  /**
   * A page without a simulator, as on a robot: nothing at any of its routes, so nothing can ever
   * write the Driver Station's state there, and no section of it in the event stream. ({@code
   * PitwallRobotTest} checks a robot gets such a page.)
   */
  @Test
  void aRobotHasNoSimulatorControls() throws Exception {
    for (String path :
        List.of(
            "/api/simulator",
            "/api/simulator/step",
            "/api/simulator/stop",
            "/api/simulator/joysticks",
            "/api/simulator/faults",
            "/api/simulator/scenarios")) {
      for (String method : List.of("GET", "PATCH", "POST", "PUT", "DELETE")) {
        var answer = send(method, path, "application/json", method.equals("GET") ? null : "{}");
        assertThat(answer.statusCode()).as(method + " " + path).isEqualTo(404);
        assertThat(answer.body()).contains("a robot has no simulator controls");
      }
    }
    assertThat(page.sections()).doesNotContainKeys("simulator", "components", "scenarios");
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

  /** Posts to the experiments as the page does, naming its stream. */
  private HttpResponse<String> sendAs(String stream, String json) throws Exception {
    return client.send(
        HttpRequest.newBuilder(uri("/api/experiments"))
            .header("Content-Type", "application/json")
            .header("Pitwall-Stream", stream)
            .POST(HttpRequest.BodyPublishers.ofString(json))
            .build(),
        HttpResponse.BodyHandlers.ofString());
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
