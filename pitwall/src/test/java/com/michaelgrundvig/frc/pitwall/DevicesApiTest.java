package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.Devices.Card;
import com.michaelgrundvig.frc.pitwall.Devices.Device;
import com.michaelgrundvig.frc.pitwall.Devices.Level;
import com.michaelgrundvig.frc.pitwall.Devices.Line;
import com.michaelgrundvig.frc.pitwall.Devices.LogEntry;
import com.michaelgrundvig.frc.pitwall.Devices.LogPage;
import com.michaelgrundvig.frc.pitwall.Devices.LogQuery;
import com.michaelgrundvig.frc.pitwall.experiment.ByHand;
import com.michaelgrundvig.frc.pitwall.push.Broadcaster;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.wpilib.tunable.MockTunableBackend;

/**
 * The devices' routes over real HTTP: the cards, a download fetched from the device as the page
 * asks for it, and its log a page at a time or as text. The device here is a stand-in; the robot
 * program's own devices ask over the network, on the server's thread, as these do.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DevicesApiTest {
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
  private final PitwallServer server = new PitwallServer(page, broadcaster, 0);
  private final ScheduledExecutorService loop = Executors.newSingleThreadScheduledExecutor();

  /** What the stand-in device's log was asked, in order. */
  private final List<LogQuery> asked = new CopyOnWriteArrayList<>();

  private volatile boolean answering = true;

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
                .log(this::journal))
        .add(
            Device.named(
                "vision-back", "Coprocessor", () -> new Card("Missing", Level.PROBLEM, List.of())));
    server.start();
    loop.scheduleAtFixedRate(
        () -> {
          page.devices().periodic(true);
          broadcaster.periodic(page::sections, null);
        },
        0,
        5,
        TimeUnit.MILLISECONDS);
  }

  @AfterAll
  void stop() {
    loop.shutdownNow();
    server.close();
  }

  private Devices.File settings() throws IOException {
    if (!answering) {
      throw new IOException("connect timed out");
    }
    return new Devices.File("vision-front-settings.zip", "application/zip", new byte[] {'P', 'K'});
  }

  /** Three pages: the first, then the one after "a", then the last after "b". */
  private LogPage journal(LogQuery query) throws IOException {
    asked.add(query);
    if (!answering) {
      throw new IOException("connect timed out");
    }
    return switch (query.cursor()) {
      case "" -> new LogPage(List.of(new LogEntry("12.0 s", 3, "kernel", "usb 3-1: reset")), "a");
      case "a" ->
          new LogPage(
              List.of(new LogEntry("13.5 s", 6, "photonvision.service", "<b>started</b>")), "b");
      default -> new LogPage(List.of(), "");
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
  void aDownloadIsFetchedFromTheDeviceAsAnAttachment() throws Exception {
    HttpResponse<byte[]> file =
        client.send(
            HttpRequest.newBuilder(uri("/api/devices/vision-front/downloads/settings")).build(),
            HttpResponse.BodyHandlers.ofByteArray());

    assertThat(file.statusCode()).isEqualTo(200);
    assertThat(file.headers().firstValue("Content-Type")).hasValue("application/zip");
    assertThat(file.headers().firstValue("Content-Disposition"))
        .hasValue("attachment; filename=\"vision-front-settings.zip\"");
    assertThat(file.headers().firstValue("Content-Security-Policy")).isPresent();
    assertThat(file.body()).containsExactly('P', 'K');
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
            HttpRequest.newBuilder(uri("/api/devices/vision-front/log"))
                .POST(HttpRequest.BodyPublishers.noBody())
                .build(),
            HttpResponse.BodyHandlers.ofString());

    assertThat(posted.statusCode()).isEqualTo(405);
    assertThat(posted.headers().firstValue("Allow")).hasValue("GET");
  }

  @Test
  void aLogPageIsAskedWithTheFiltersGiven() throws Exception {
    asked.clear();
    HttpResponse<String> first = get("/api/devices/vision-front/log?priority=3&unit=kernel");
    HttpResponse<String> second =
        get("/api/devices/vision-front/log?priority=3&unit=kernel&cursor=a");

    assertThat(first.statusCode()).isEqualTo(200);
    assertThat(first.body())
        .isEqualTo(
            "{\"entries\":[{\"time\":\"12.0 s\",\"priority\":3,\"unit\":\"kernel\","
                + "\"message\":\"usb 3-1: reset\"}],\"next\":\"a\"}");
    assertThat(second.body()).contains("\"next\":\"b\"");
    assertThat(asked)
        .containsExactly(new LogQuery("", 3, "kernel"), new LogQuery("a", 3, "kernel"));
  }

  @Test
  void aLogsFiltersAreChecked() throws Exception {
    assertThat(get("/api/devices/vision-front/log?priority=9").statusCode()).isEqualTo(400);
    assertThat(get("/api/devices/vision-front/log?unit=a%20b").statusCode()).isEqualTo(400);
    assertThat(get("/api/devices/vision-front/log?grep=x").statusCode()).isEqualTo(400);
    assertThat(get("/api/devices/vision-front/log?priority=").statusCode()).isEqualTo(200);
  }

  @Test
  void theLogAsTextFollowsEveryPage() throws Exception {
    asked.clear();
    HttpResponse<String> text = get("/api/devices/vision-front/log.txt?priority=6");

    assertThat(text.statusCode()).isEqualTo(200);
    assertThat(text.headers().firstValue("Content-Disposition"))
        .hasValue("attachment; filename=\"vision-front-log.txt\"");
    assertThat(text.body())
        .isEqualTo(
            "12.0 s err kernel: usb 3-1: reset\n"
                + "13.5 s info photonvision.service: <b>started</b>\n");
    assertThat(asked).extracting(LogQuery::cursor).containsExactly("", "a", "b");
    assertThat(text.body().getBytes(StandardCharsets.UTF_8)).isNotEmpty();
  }

  private HttpResponse<String> get(String path) throws Exception {
    return client.send(
        HttpRequest.newBuilder(uri(path)).GET().build(), HttpResponse.BodyHandlers.ofString());
  }

  private URI uri(String path) {
    return URI.create("http://localhost:" + server.port() + path);
  }
}
