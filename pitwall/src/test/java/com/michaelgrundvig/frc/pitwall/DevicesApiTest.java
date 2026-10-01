package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.pitwall.Devices.Card;
import com.michaelgrundvig.frc.pitwall.Devices.Device;
import com.michaelgrundvig.frc.pitwall.Devices.Level;
import com.michaelgrundvig.frc.pitwall.Devices.Line;
import com.michaelgrundvig.frc.pitwall.Devices.LogEntry;
import com.michaelgrundvig.frc.pitwall.Devices.LogPage;
import com.michaelgrundvig.frc.pitwall.Devices.LogQuery;
import com.michaelgrundvig.frc.pitwall.experiment.ByHand;
import com.michaelgrundvig.frc.pitwall.push.Broadcaster;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.wpilib.tunable.MockTunableBackend;

/**
 * The devices' routes over real HTTP: the cards, a download streamed from the device as the page
 * asks for it, and its log a page at a time or as text; and what keeps them from costing the robot:
 * only the page may ask, two at once, nothing on the field, files and logs bounded. The devices
 * here are stand-ins; the robot program's own ask over the network, on the server's thread, as
 * these do.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DevicesApiTest {
  /** The page names its event stream on every request; any name will do here. */
  private static final String STREAM = "1";

  /** Small limits, so a test can reach them: a 1 KB file, and half a second for a log's text. */
  private static final PitwallServer.DeviceLimits LIMITS =
      new PitwallServer.DeviceLimits(2, 1024, Duration.ofMillis(500));

  private final HttpClient client = HttpClient.newHttpClient();
  private final TunableTee tunables =
      new TunableTee(new MockTunableBackend(), () -> null, inputs -> {}, false);
  private final Tuning tuning =
      new Tuning("test", tunables::configs, tunables::pathOf, true, inputs -> {}, view -> {});
  private final Page page =
      new Page(
          new Status(),
          new Actions(inputs -> {}),
          tuning,
          tunables,
          new ByHand(System::nanoTime).experiments());
  private final Broadcaster broadcaster =
      new Broadcaster(Broadcaster.MAX_STREAMS, Broadcaster.MAILBOX, stream -> {});
  private final PitwallServer server =
      new PitwallServer(page, broadcaster, 0, PitwallServer.WAIT, PitwallServer.IDLE, LIMITS);
  private final ScheduledExecutorService loop = Executors.newSingleThreadScheduledExecutor();

  /** What the stand-in device's log was asked, in order, and how often its files were. */
  private final List<LogQuery> asked = new CopyOnWriteArrayList<>();

  private final AtomicInteger fetched = new AtomicInteger();
  private volatile boolean answering = true;
  private volatile boolean busy;
  private final AtomicBoolean fieldConnected = new AtomicBoolean();

  /** Holds the slow download until a test lets it go. */
  private volatile CountDownLatch slow = new CountDownLatch(0);

  @BeforeAll
  void start() throws Exception {
    page.devices()
        .add(
            Device.named(
                    "vision-front",
                    "Coprocessor",
                    () ->
                        new Card(
                            "Present", Level.OK, List.of(new Line("Version", "matches", Level.OK))))
                .link("Open PhotonVision", "http://10.0.0.11:5800")
                .download("settings", "Download settings backup", this::settings)
                .download("short", "A file that ends early", DevicesApiTest::endsEarly)
                .download("big", "A file over the limit", DevicesApiTest::tooBig)
                .download("unsized", "A file of unknown length", DevicesApiTest::unsized)
                .download("slow", "A file that takes a while", this::slowly)
                .log(this::journal))
        .add(
            Device.named(
                "vision-back", "Coprocessor", () -> new Card("Missing", Level.PROBLEM, List.of())))
        .add(
            Device.named("endless", "Coprocessor", () -> new Card("Present", Level.OK, List.of()))
                .log(DevicesApiTest::endless));
    server.start();
    loop.scheduleAtFixedRate(
        () -> {
          tuning.periodic(fieldConnected.get());
          page.devices().periodic(true);
          broadcaster.periodic(page::sections, null);
        },
        0,
        5,
        TimeUnit.MILLISECONDS);
  }

  @AfterEach
  void offTheField() throws Exception {
    fieldConnected.set(false);
    slow.countDown();
    waitForTheLoop();
  }

  @AfterAll
  void stop() {
    loop.shutdownNow();
    server.close();
  }

  /** Lets the loop run a few times, so what it hands the server is current. */
  private static void waitForTheLoop() throws InterruptedException {
    Thread.sleep(50);
  }

  private Devices.File settings() throws IOException {
    fetched.incrementAndGet();
    if (!answering) {
      throw new IOException("connect timed out");
    }
    return Devices.File.of("vision-front-settings.zip", "application/zip", new byte[] {'P', 'K'});
  }

  /** Says it's 100 bytes long, and ends after 10: as a device's answer cut off would. */
  private static Devices.File endsEarly() {
    return new Devices.File(
        "short.zip", "application/zip", 100, new ByteArrayInputStream(new byte[10]));
  }

  private static Devices.File tooBig() {
    return new Devices.File(
        "big.zip", "application/zip", 2048, new ByteArrayInputStream(new byte[2048]));
  }

  private static Devices.File unsized() {
    return new Devices.File(
        "unsized.zip", "application/zip", -1, new ByteArrayInputStream(new byte[] {1, 2, 3}));
  }

  private Devices.File slowly() throws IOException {
    try {
      slow.await(5, TimeUnit.SECONDS);
    } catch (InterruptedException e) {
      throw new IOException(e);
    }
    return Devices.File.of("slow.zip", "application/zip", new byte[] {1});
  }

  /** A log that never ends, a page every 20 ms. */
  private static LogPage endless(LogQuery query) throws IOException {
    try {
      Thread.sleep(20);
    } catch (InterruptedException e) {
      throw new IOException(e);
    }
    return new LogPage(List.of(new LogEntry("1 s", 6, "kernel", "more")), "x" + query.cursor(), "");
  }

  /**
   * Three entries, one a page: the latest is "c"'s; from the start, "a"'s; after "a", "b"'s; after
   * "b", "c"'s; after "c", nothing yet; before "c", "b"'s.
   */
  private LogPage journal(LogQuery query) throws IOException {
    asked.add(query);
    if (busy) {
      throw new Devices.Busy("another journal request is under way");
    }
    if (!answering) {
      throw new IOException("connect timed out");
    }
    String at =
        switch (query.from()) {
          case LATEST -> "c";
          case START -> "a";
          case AFTER -> query.cursor().equals("a") ? "b" : query.cursor().equals("b") ? "c" : "";
          case BEFORE -> query.cursor().equals("c") ? "b" : query.cursor().equals("b") ? "a" : "";
        };
    return switch (at) {
      case "a" ->
          new LogPage(List.of(new LogEntry("12.0 s", 3, "kernel", "usb 3-1: reset")), "a", "");
      case "b" ->
          new LogPage(
              List.of(new LogEntry("13.5 s", 6, "photonvision.service", "<b>started</b>")),
              "b",
              "b");
      case "c" -> new LogPage(List.of(new LogEntry("14.0 s", 6, "kernel", "later")), "c", "c");
      default -> new LogPage(List.of(), "", "");
    };
  }

  @Test
  void theCardsAreServed() throws Exception {
    // The loop reads the cards once a page watches, or here, every loop.
    HttpResponse<String> cards = get("/api/devices");

    assertThat(cards.statusCode()).isEqualTo(200);
    assertThat(cards.body())
        .contains("\"vision-front\"", "\"Open PhotonVision\"", "\"vision-back\"", "\"Missing\"");
  }

  @Test
  void aDownloadIsStreamedFromTheDeviceAsAnAttachment() throws Exception {
    HttpResponse<byte[]> file =
        client.send(
            request("/api/devices/vision-front/downloads/settings").build(),
            HttpResponse.BodyHandlers.ofByteArray());

    assertThat(file.statusCode()).isEqualTo(200);
    assertThat(file.headers().firstValue("Content-Type")).hasValue("application/zip");
    assertThat(file.headers().firstValue("Content-Length")).hasValue("2");
    assertThat(file.headers().firstValue("Content-Disposition"))
        .hasValue("attachment; filename=\"vision-front-settings.zip\"");
    assertThat(file.headers().firstValue("Content-Security-Policy")).isPresent();
    assertThat(file.body()).containsExactly('P', 'K');
  }

  @Test
  void aFileThatEndsEarlyReachesThePageCutOff() {
    assertThatThrownBy(
            () ->
                client.send(
                    request("/api/devices/vision-front/downloads/short").build(),
                    HttpResponse.BodyHandlers.ofByteArray()))
        .isInstanceOf(IOException.class);
  }

  @Test
  void aFileOverTheLimitIsRefusedNotCut() throws Exception {
    HttpResponse<String> big = get("/api/devices/vision-front/downloads/big");

    assertThat(big.statusCode()).isEqualTo(502);
    assertThat(big.body()).contains("more than Pitwall passes on: 1024 bytes");
  }

  @Test
  void aFileOfUnknownLengthIsSentWhole() throws Exception {
    HttpResponse<byte[]> file =
        client.send(
            request("/api/devices/vision-front/downloads/unsized").build(),
            HttpResponse.BodyHandlers.ofByteArray());

    assertThat(file.statusCode()).isEqualTo(200);
    assertThat(file.body()).containsExactly(1, 2, 3);
  }

  /** A request without the page's stream, as another site's page's would be, asks nothing. */
  @Test
  void onlyThePageAsksADevice() throws Exception {
    fetched.set(0);
    HttpResponse<String> anonymous =
        client.send(
            HttpRequest.newBuilder(uri("/api/devices/vision-front/downloads/settings")).build(),
            HttpResponse.BodyHandlers.ofString());

    assertThat(anonymous.statusCode()).isEqualTo(400);
    assertThat(anonymous.body()).contains("Pitwall-Stream");
    assertThat(fetched).hasValue(0);
  }

  @Test
  void twoRequestsAtOnceAtMostAndTheThirdIsBusy() throws Exception {
    slow = new CountDownLatch(1);
    CompletableFuture<HttpResponse<String>> first =
        client.sendAsync(
            request("/api/devices/vision-front/downloads/slow").build(),
            HttpResponse.BodyHandlers.ofString());
    CompletableFuture<HttpResponse<String>> second =
        client.sendAsync(
            request("/api/devices/vision-front/downloads/slow").build(),
            HttpResponse.BodyHandlers.ofString());
    Thread.sleep(300); // both under way, held

    HttpResponse<String> third = get("/api/devices/vision-front/log");
    slow.countDown();

    assertThat(third.statusCode()).isEqualTo(503);
    assertThat(first.get(5, TimeUnit.SECONDS).statusCode()).isEqualTo(200);
    assertThat(second.get(5, TimeUnit.SECONDS).statusCode()).isEqualTo(200);
    assertThat(get("/api/devices/vision-front/log").statusCode()).as("free again").isEqualTo(200);
  }

  @Test
  void onTheFieldNothingIsDownloaded() throws Exception {
    fieldConnected.set(true);
    waitForTheLoop();
    fetched.set(0);

    HttpResponse<String> file = get("/api/devices/vision-front/downloads/settings");
    HttpResponse<String> text = get("/api/devices/vision-front/log.txt");
    HttpResponse<String> page = get("/api/devices/vision-front/log");

    assertThat(file.statusCode()).isEqualTo(409);
    assertThat(text.statusCode()).isEqualTo(409);
    assertThat(file.body()).contains("field system");
    assertThat(fetched).hasValue(0);
    assertThat(page.statusCode()).as("a page of the log is a glance").isEqualTo(200);
  }

  @Test
  void aDeviceThatDoesntAnswerIsABadGateway() throws Exception {
    answering = false;
    try {
      HttpResponse<String> file = get("/api/devices/vision-front/downloads/settings");
      HttpResponse<String> log = get("/api/devices/vision-front/log");

      assertThat(file.statusCode()).isEqualTo(502);
      assertThat(file.body()).contains("vision-front didn't answer: connect timed out");
      assertThat(log.statusCode()).isEqualTo(502);
    } finally {
      answering = true;
    }
  }

  @Test
  void whatIsntThereIsNotFound() throws Exception {
    assertThat(get("/api/devices/nothing/log").statusCode()).isEqualTo(404);
    assertThat(get("/api/devices/vision-front/downloads/nothing").statusCode()).isEqualTo(404);
    assertThat(get("/api/devices/vision-back/log").statusCode()).isEqualTo(404);
    assertThat(get("/api/devices/vision-back/downloads/settings").statusCode()).isEqualTo(404);
    assertThat(get("/api/devices/Vision-Front/log").statusCode()).isEqualTo(404);
    assertThat(get("/api/devices/vision-front/downloads/../log").statusCode()).isEqualTo(404);
  }

  @Test
  void devicesAreOnlyRead() throws Exception {
    HttpResponse<String> posted =
        client.send(
            request("/api/devices/vision-front/log")
                .POST(HttpRequest.BodyPublishers.noBody())
                .build(),
            HttpResponse.BodyHandlers.ofString());

    assertThat(posted.statusCode()).isEqualTo(405);
    assertThat(posted.headers().firstValue("Allow")).hasValue("GET");
  }

  @Test
  void aLogPageIsAskedWithTheFiltersGiven() throws Exception {
    asked.clear();
    HttpResponse<String> latest = get("/api/devices/vision-front/log?priority=3&unit=kernel");
    HttpResponse<String> newer =
        get("/api/devices/vision-front/log?priority=3&unit=kernel&cursor=a");
    HttpResponse<String> older =
        get("/api/devices/vision-front/log?priority=3&unit=kernel&before=c");
    HttpResponse<String> start = get("/api/devices/vision-front/log?from=start");

    assertThat(latest.statusCode()).isEqualTo(200);
    assertThat(latest.body())
        .isEqualTo(
            "{\"entries\":[{\"time\":\"14.0 s\",\"priority\":6,\"unit\":\"kernel\","
                + "\"message\":\"later\"}],\"newer\":\"c\",\"older\":\"c\"}");
    assertThat(newer.body()).contains("<b>started</b>");
    assertThat(older.body()).contains("<b>started</b>");
    assertThat(start.body()).contains("usb 3-1: reset", "\"older\":\"\"");
    assertThat(asked)
        .containsExactly(
            new LogQuery(Devices.From.LATEST, "", 3, "kernel"),
            new LogQuery(Devices.From.AFTER, "a", 3, "kernel"),
            new LogQuery(Devices.From.BEFORE, "c", 3, "kernel"),
            new LogQuery(Devices.From.START, "", 7, ""));
  }

  @Test
  void aPageIsAtOnePlaceAtMost() throws Exception {
    assertThat(get("/api/devices/vision-front/log?cursor=a&before=c").statusCode()).isEqualTo(400);
    assertThat(get("/api/devices/vision-front/log?from=start&cursor=a").statusCode())
        .isEqualTo(400);
    assertThat(get("/api/devices/vision-front/log?from=yesterday").statusCode()).isEqualTo(400);
  }

  /** A device busy with another such request is passed on as busy, saying when to ask again. */
  @Test
  void aBusyDeviceIsServiceUnavailable() throws Exception {
    busy = true;
    try {
      HttpResponse<String> page = get("/api/devices/vision-front/log");

      assertThat(page.statusCode()).isEqualTo(503);
      assertThat(page.headers().firstValue("Retry-After")).hasValue("1");
      assertThat(page.body()).contains("vision-front is busy");
    } finally {
      busy = false;
    }
  }

  @Test
  void aLogsFiltersAreChecked() throws Exception {
    assertThat(get("/api/devices/vision-front/log?priority=9").statusCode()).isEqualTo(400);
    assertThat(get("/api/devices/vision-front/log?unit=a%20b").statusCode()).isEqualTo(400);
    assertThat(get("/api/devices/vision-front/log?grep=x").statusCode()).isEqualTo(400);
    assertThat(get("/api/devices/vision-front/log?priority=").statusCode()).isEqualTo(200);
  }

  @Test
  void theLogAsTextIsFromTheStart() throws Exception {
    asked.clear();
    HttpResponse<String> text = get("/api/devices/vision-front/log.txt?priority=6");

    assertThat(text.statusCode()).isEqualTo(200);
    assertThat(text.headers().firstValue("Content-Disposition"))
        .hasValue("attachment; filename=\"vision-front-log.txt\"");
    assertThat(text.body())
        .isEqualTo(
            "12.0 s err kernel: usb 3-1: reset\n"
                + "13.5 s info photonvision.service: <b>started</b>\n"
                + "14.0 s info kernel: later\n");
    assertThat(asked)
        .extracting(LogQuery::from, LogQuery::cursor)
        .containsExactly(
            org.assertj.core.groups.Tuple.tuple(Devices.From.START, ""),
            org.assertj.core.groups.Tuple.tuple(Devices.From.AFTER, "a"),
            org.assertj.core.groups.Tuple.tuple(Devices.From.AFTER, "b"),
            org.assertj.core.groups.Tuple.tuple(Devices.From.AFTER, "c"));
  }

  /** A log that never ends is cut off at its deadline, and its last line says so. */
  @Test
  void theLogAsTextIsCutOffAtItsDeadline() throws Exception {
    HttpResponse<String> text = get("/api/devices/endless/log.txt");

    assertThat(text.statusCode()).isEqualTo(200);
    assertThat(text.body())
        .startsWith("1 s info kernel: more\n")
        .endsWith("-- cut off here: gathering it took 0.5 s. Read on in the page's log viewer.\n");
  }

  private HttpRequest.Builder request(String path) {
    return HttpRequest.newBuilder(uri(path)).header("Pitwall-Stream", STREAM);
  }

  private HttpResponse<String> get(String path) throws Exception {
    return client.send(request(path).GET().build(), HttpResponse.BodyHandlers.ofString());
  }

  private URI uri(String path) {
    return URI.create("http://localhost:" + server.port() + path);
  }
}
