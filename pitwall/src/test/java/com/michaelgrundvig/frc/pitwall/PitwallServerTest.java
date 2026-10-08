package com.michaelgrundvig.frc.pitwall;

import static com.michaelgrundvig.frc.pitwall.Fixtures.machine;
import static com.michaelgrundvig.frc.pitwall.experiment.ByHand.STEP;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.experiment.ByHand;
import com.michaelgrundvig.frc.pitwall.experiment.Run;
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
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
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

  /**
   * Waits for the loop as long as {@link Await} would: an answer comes as soon as the loop applies
   * a change, so only a stuck loop is ever waited on that long, and never a busy machine's 202.
   */
  private final PitwallServer server =
      new PitwallServer(page, broadcaster, 0, Await.DEADLINE, PitwallServer.IDLE);

  private @Nullable String whyLocked() {
    return tuning.whyLocked();
  }

  private final ScheduledExecutorService loop = Executors.newSingleThreadScheduledExecutor();

  /** How many times the loop has run. */
  private final AtomicInteger loops = new AtomicInteger();

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
          loops.incrementAndGet();
        },
        0,
        5,
        TimeUnit.MILLISECONDS);
    Await.until("the loop's first run", () -> loops.get() > 0);
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
    // Off the field and free to run, as the loop reports it, before anything is put back: a change
    // the loop applied with the field still connected would be refused.
    Await.until("the field system disconnected", () -> !tuning.isFieldConnected());
    Await.until("actions free to run", () -> actions.whyLocked() == null);
    Await.until("experiments free to run", () -> byHand.experiments().whyLocked() == null);
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

  /** The page is its shell and its modules, each served from the jar as written. */
  @Test
  void servesThePagesModulesAndStyles() throws Exception {
    var main = get("/page/main.js", "*/*");
    var styles = get("/page/pitwall.css", "*/*");

    assertThat(get("/", "text/html").body())
        .contains("src=\"/page/main.js\"", "src=\"/page/theme.js\"")
        .as("the page runs no inline script")
        .doesNotContain("<script>");
    assertThat(get("/page/theme.js", "*/*").statusCode()).isEqualTo(200);
    assertThat(main.statusCode()).isEqualTo(200);
    assertThat(main.headers().firstValue("Content-Type"))
        .hasValue("text/javascript; charset=utf-8");
    assertThat(styles.headers().firstValue("Content-Type")).hasValue("text/css; charset=utf-8");
    assertThat(get("/page/nothing.js").statusCode()).isEqualTo(404);
    assertThat(get("/page/" + "a".repeat(64) + ".js").statusCode())
        .as("a name too long to be one of the page's")
        .isEqualTo(404);
    assertThat(get("/page/%2E%2E/index.html").statusCode())
        .as("only a plain name, never a way out of page/")
        .isEqualTo(404);
    assertThat(send("POST", "/page/main.js", null, null).statusCode()).isEqualTo(405);
  }

  /**
   * A module a view imports but the jar lacks would leave the page blank: each one is here, those
   * loaded only when needed ({@code import("./scene-render.js")}) too, and the third-party modules
   * they import from their fixed routes.
   */
  @Test
  void everyModuleThePageImportsIsServed() throws Exception {
    Pattern imports =
        Pattern.compile("(?:from |import\\()\"(\\./|/three/)([A-Za-z][A-Za-z0-9.-]*\\.js)\"");
    List<String> seen = new ArrayList<>(List.of("main.js"));
    List<String> third = new ArrayList<>();
    for (int i = 0; i < seen.size(); i++) {
      var module = get("/page/" + seen.get(i), "*/*");
      assertThat(module.statusCode()).as(seen.get(i)).isEqualTo(200);
      Matcher found = imports.matcher(module.body());
      while (found.find()) {
        List<String> into = found.group(1).equals("./") ? seen : third;
        if (!into.contains(found.group(2))) {
          into.add(found.group(2));
        }
      }
    }
    for (String module : third) {
      var served = get("/three/" + module, "*/*");
      assertThat(served.statusCode()).as(module).isEqualTo(200);
      assertThat(served.headers().firstValue("Content-Type"))
          .hasValue("text/javascript; charset=utf-8");
    }
    assertThat(third).containsExactlyInAnyOrder("three.module.min.js", "OrbitControls.js");

    assertThat(seen)
        .contains(
            "core.js",
            "pit.js",
            "tune.js",
            "experiment.js",
            "changes.js",
            "sim.js",
            "sim-drive.js",
            "sim-faults.js",
            "sim-outcome.js",
            "world.js",
            "scene-render.js",
            "scene-shapes.js");
  }

  /**
   * three.js comes from its WebJar, its own minified build served whole. Its orbit controls import
   * the module beside them rather than the bare name 'three', which only an inline import map could
   * resolve, and the page's policy runs no inline script.
   */
  @Test
  void threeJsIsServedFromItsWebJarImportingOnlyWhatsBesideIt() throws Exception {
    var module = get("/three/three.module.min.js", "*/*");
    var core = get("/three/three.core.min.js", "*/*");
    var controls = get("/three/OrbitControls.js", "*/*");

    assertThat(module.statusCode()).isEqualTo(200);
    assertThat(module.body()).contains("from\"./three.core.min.js\"").contains("@license");
    assertThat(core.statusCode()).isEqualTo(200);
    assertThat(controls.body())
        .contains("from './three.module.min.js'")
        .doesNotContainPattern("from\\s+['\"]three['\"]");
    assertThat(controls.headers().firstValue("Cache-Control")).hasValue("max-age=3600");
    assertThat(get("/three/three.webgpu.min.js").statusCode())
        .as("only the files the page loads")
        .isEqualTo(404);
  }

  /** A read waits for the next loop, which reads every item then: none is older than the ask. */
  @Test
  void showsTheStatusAsOfTheNextLoop() throws Exception {
    homed.set(true);

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
        .isEqualTo(
            "[{\"name\":\"Home arm\",\"phase\":\"RUNNING\",\"detail\":\"Running\","
                + "\"locked\":\"\",\"when\":\"ENABLED_IN_UTILITY\"}]");
    assertThat(homing.get()).isTrue();

    var canceled = send("POST", "/api/actions", "application/json", "{\"cancel\": \"Home arm\"}");

    assertThat(canceled.statusCode()).isEqualTo(200);
    assertThat(canceled.body()).contains("\"phase\":\"CANCELED\"");
    assertThat(homing.get()).isFalse();
  }

  @Test
  void anActionOutsideUtilityModeIsLockedAndSaysWhy() throws Exception {
    gate = new Actions.Gate(true, false, false);
    Await.until("actions locked", () -> actions.whyLocked() != null);

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
    Await.until("the field system connected", tuning::isFieldConnected);
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
                + "\"limits\":\"0 to 100\",\"min\":0.0,\"max\":100.0,\"committed\":16.0,"
                + "\"record\":\"Fixtures.Gains\",\"group\":\"drive.gains\","
                + "\"meaning\":\"volts per meter per second of error\"}")
        .as("no limits, no ends")
        .contains("\"field\":\"travel.min\",\"unit\":\"\",\"limits\":\"\",\"committed\"")
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
                + "\"detail\":\"/Machine/drive/limits/current: -1 is below its limit of 1\","
                + "\"errors\":[{\"layer\":\"limit\",\"by\":\"\","
                + "\"path\":\"/Machine/drive/limits/current\","
                + "\"paths\":[\"/Machine/drive/limits/current\"],\"config\":\"machine\","
                + "\"group\":\"\",\"text\":\"-1 is below its limit of 1\","
                + "\"line\":\"/Machine/drive/limits/current: -1 is below its limit of 1\"}]}");
    assertThat(get("/api/values").body()).contains("\"/Machine/drive/gains/kP\":16.0");
  }

  /** Within its limits, but the configuration's own checks refuse it: they say why. */
  @Test
  void aValueTheConfigurationRejectsSaysWhy() throws Exception {
    var refused = send("PATCH", "/api/values", MERGE_PATCH, "{\"/Machine/travel/min\": 12}");

    assertThat(refused.statusCode()).isEqualTo(422);
    assertThat(refused.body())
        .contains(
            "\"layer\":\"record\",\"by\":\"Fixtures.Range\",\"path\":\"/Machine/travel/min\"",
            "\"group\":\"travel\",\"text\":\"min must be below max, got 12.0 and 10.0\"");
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
    Await.until("the field system connected", tuning::isFieldConnected);

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
   * address (DNS rebinding) and use the page as its own: the name it sends gives it away. Only the
   * robot's own names pass, never another {@code .local} one, which anyone on the network can
   * announce.
   */
  @Test
  void answersOnlyAtTheRobotsOwnAddress() throws Exception {
    String machine = InetAddress.getLocalHost().getHostName().split("\\.")[0] + ".local";

    assertThat(statusLine("localhost:" + server.port())).endsWith(" 200 OK");
    assertThat(statusLine("robot.local:5820")).endsWith(" 200 OK");
    assertThat(statusLine("10.26.11.2:5820")).endsWith(" 200 OK");
    assertThat(statusLine("[::1]:5820")).endsWith(" 200 OK");
    assertThat(statusLine(machine + ":5820")).as("this computer's own name").endsWith(" 200 OK");
    assertThat(statusLine("evil.local:5820")).contains(" 403 ");
    assertThat(statusLine("localhost.example.com")).contains(" 403 ");
    assertThat(statusLine("rebinding.example.com:5820")).contains(" 403 ");
    assertThat(statusLine("[evil]:5820")).as("not an address").contains(" 403 ");
    assertThat(statusLine(null)).as("no Host").contains(" 403 ");
  }

  /**
   * A browser says which site a request comes from: another site's page can't read the API, or
   * close a page's stream by naming it, even with a request a browser sends without asking.
   */
  @Test
  void theApiAnswersOnlyThePagesOwnSite() throws Exception {
    assertThat(fetchedFrom("same-origin", "/api/tuning").statusCode()).isEqualTo(200);
    assertThat(fetchedFrom("none", "/api/tuning").statusCode()).as("typed in").isEqualTo(200);
    assertThat(fetchedFrom("cross-site", "/api/tuning").statusCode()).isEqualTo(403);
    assertThat(fetchedFrom("same-site", "/api/tuning").statusCode()).isEqualTo(403);
    assertThat(fetchedFrom("cross-site", "/api/events?replaces=x").statusCode()).isEqualTo(403);
    assertThat(fetchedFrom("cross-site", "/").statusCode())
        .as("the page itself, which runs nothing until it's open")
        .isEqualTo(200);
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
        .hasValueSatisfying(
            policy ->
                assertThat(policy)
                    .contains("frame-ancestors 'none'")
                    .as("no inline script or style runs")
                    .doesNotContain("unsafe-inline"));
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
                + "\":{\"rig\":\"Arm\",\"settings\":\"step and back, 0 ° to 90 °, held 0.8 s\","
                + "\"seconds\":2.1}}}");
  }

  @Test
  void anExperimentStartsFromThePageAndItsRunIsServedWithItsTrace() throws Exception {
    var started = sendAs("9", "{\"start\": \"" + STEP + "\"}");
    Run run = Objects.requireNonNull(byHand.experiments().run());
    Await.until("a setpoint traced", () -> run.trace().series("Setpoint").count() > 0);

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

  /**
   * The status line of a GET naming this host, or none, sent by hand: Java's client won't set Host.
   */
  private String statusLine(@Nullable String host) throws Exception {
    try (Socket socket = new Socket(InetAddress.getLoopbackAddress(), server.port());
        var reader =
            new BufferedReader(
                new InputStreamReader(socket.getInputStream(), StandardCharsets.US_ASCII))) {
      socket
          .getOutputStream()
          .write(
              ("GET /api/tuning HTTP/1.1\r\n"
                      + (host == null ? "" : "Host: " + host + "\r\n")
                      + "Connection: close\r\n\r\n")
                  .getBytes(StandardCharsets.US_ASCII));
      return reader.readLine();
    }
  }

  /**
   * A GET as a browser sends it from a page of this site, or another. Never an event stream that
   * opens: it would never end.
   */
  private HttpResponse<String> fetchedFrom(String site, String path) throws Exception {
    return client.send(
        HttpRequest.newBuilder(uri(path)).header("Sec-Fetch-Site", site).GET().build(),
        HttpResponse.BodyHandlers.ofString());
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
