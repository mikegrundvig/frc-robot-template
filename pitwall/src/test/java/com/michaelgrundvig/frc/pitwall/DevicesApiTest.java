package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.pitwall.Devices.Card;
import com.michaelgrundvig.frc.pitwall.Devices.Device;
import com.michaelgrundvig.frc.pitwall.Devices.Level;
import com.michaelgrundvig.frc.pitwall.Devices.Line;
import com.michaelgrundvig.frc.pitwall.experiment.ByHand;
import com.michaelgrundvig.frc.pitwall.push.Broadcaster;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.List;
import java.util.concurrent.CompletableFuture;
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
 * asks for it, a file sent to one and a choice made on its card; and what keeps them from costing
 * the robot: only the page may ask, two downloads at once, nothing on the field, files bounded. The
 * devices here are stand-ins; the robot program's own ask over the network or the disk, on the
 * server's thread, as these do.
 */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class DevicesApiTest {
  /** The page names its event stream on every request; any name will do here. */
  private static final String STREAM = "1";

  /** Small limits, so a test can reach them: a 1 KB file. */
  private static final PitwallServer.DeviceLimits LIMITS = new PitwallServer.DeviceLimits(2, 1024);

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

  /** How often the stand-in device's files were asked for. */
  private final AtomicInteger fetched = new AtomicInteger();

  private volatile boolean answering = true;
  private volatile boolean busy;
  private final AtomicBoolean fieldConnected = new AtomicBoolean();

  /** What the venue stand-in has chosen, by its option's value. */
  private volatile String chosenVenue = "official";

  /** Holds the slow download until a test lets it go; how many are held. */
  private volatile CountDownLatch slow = new CountDownLatch(0);

  private final AtomicInteger held = new AtomicInteger();

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
                .download("settings", "Download settings backup", this::settings)
                .download("short", "A file that ends early", DevicesApiTest::endsEarly)
                .download("big", "A file over the limit", DevicesApiTest::tooBig)
                .download("unsized", "A file of unknown length", DevicesApiTest::unsized)
                .download("slow", "A file that takes a while", this::slowly))
        .add(
            Device.named(
                "vision-back", "Coprocessor", () -> new Card("Missing", Level.PROBLEM, List.of())))
        .add(
            Device.named(
                    "venue",
                    "Field",
                    () ->
                        new Card(
                            "In use",
                            Level.OK,
                            List.of(),
                            List.of(
                                new Devices.Choice(
                                    "venue",
                                    "Venue",
                                    List.of(
                                        new Devices.Option("official", "Official layout"),
                                        new Devices.Option("practice", "Practice field")),
                                    chosenVenue))))
                .upload("survey", "Upload venue", DevicesApiTest::survey)
                .choose("venue", this::chooseVenue));
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
    Await.until("the cards read", () -> page.devices().view().size() == 3);
  }

  @AfterEach
  void offTheField() throws Exception {
    fieldConnected.set(false);
    slow.countDown();
    Await.until("the field system disconnected", () -> !tuning.isFieldConnected());
  }

  @AfterAll
  void stop() {
    loop.shutdownNow();
    server.close();
  }

  private Devices.File settings() throws IOException {
    fetched.incrementAndGet();
    if (busy) {
      throw new Devices.Busy("another backup is under way");
    }
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
    held.incrementAndGet();
    try {
      slow.await(5, TimeUnit.SECONDS);
    } catch (InterruptedException e) {
      throw new IOException(e);
    }
    return Devices.File.of("slow.zip", "application/zip", new byte[] {1});
  }

  /** Takes a survey unless it says "bad", which it refuses saying why. */
  private static String survey(byte[] body) throws IOException {
    String text = new String(body, java.nio.charset.StandardCharsets.UTF_8);
    if (text.equals("bad")) {
      throw new Devices.Refused("tag 26 sits 0.500 m from its official place");
    }
    return "Saved " + text;
  }

  private String chooseVenue(String value) throws IOException {
    if (!value.equals("official") && !value.equals("practice")) {
      throw new Devices.Refused("there is no venue " + value);
    }
    chosenVenue = value;
    return "Chose " + value;
  }

  @Test
  void theCardsAreServed() throws Exception {
    // The loop reads the cards once a page watches, or here, every loop.
    HttpResponse<String> cards = get("/api/devices");

    assertThat(cards.statusCode()).isEqualTo(200);
    assertThat(cards.body())
        .contains(
            "\"vision-front\"", "\"Download settings backup\"", "\"vision-back\"", "\"Missing\"");
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
    held.set(0);
    CompletableFuture<HttpResponse<String>> first =
        client.sendAsync(
            request("/api/devices/vision-front/downloads/slow").build(),
            HttpResponse.BodyHandlers.ofString());
    CompletableFuture<HttpResponse<String>> second =
        client.sendAsync(
            request("/api/devices/vision-front/downloads/slow").build(),
            HttpResponse.BodyHandlers.ofString());
    Await.until("both downloads under way, held", () -> held.get() == 2);

    HttpResponse<String> third = get("/api/devices/vision-front/downloads/settings");
    slow.countDown();

    assertThat(third.statusCode()).isEqualTo(503);
    assertThat(first.get(5, TimeUnit.SECONDS).statusCode()).isEqualTo(200);
    assertThat(second.get(5, TimeUnit.SECONDS).statusCode()).isEqualTo(200);
    assertThat(get("/api/devices/vision-front/downloads/settings").statusCode())
        .as("free again")
        .isEqualTo(200);
  }

  @Test
  void onTheFieldNothingIsDownloaded() throws Exception {
    fieldConnected.set(true);
    Await.until("the field system connected", tuning::isFieldConnected);
    fetched.set(0);

    HttpResponse<String> file = get("/api/devices/vision-front/downloads/settings");

    assertThat(file.statusCode()).isEqualTo(409);
    assertThat(file.body()).contains("field system");
    assertThat(fetched).hasValue(0);
  }

  @Test
  void aDeviceThatDoesntAnswerIsABadGateway() throws Exception {
    answering = false;
    try {
      HttpResponse<String> file = get("/api/devices/vision-front/downloads/settings");

      assertThat(file.statusCode()).isEqualTo(502);
      assertThat(file.body()).contains("vision-front didn't answer: connect timed out");
    } finally {
      answering = true;
    }
  }

  @Test
  void whatIsntThereIsNotFound() throws Exception {
    assertThat(get("/api/devices/nothing/downloads/settings").statusCode()).isEqualTo(404);
    assertThat(get("/api/devices/vision-front/downloads/nothing").statusCode()).isEqualTo(404);
    assertThat(get("/api/devices/vision-back/downloads/settings").statusCode()).isEqualTo(404);
    assertThat(get("/api/devices/Vision-Front/downloads/settings").statusCode()).isEqualTo(404);
    assertThat(get("/api/devices/vision-front/downloads/../settings").statusCode()).isEqualTo(404);
    assertThat(get("/api/devices/vision-front/log").statusCode()).as("no log").isEqualTo(404);
  }

  @Test
  void devicesAreOnlyRead() throws Exception {
    HttpResponse<String> posted =
        client.send(
            request("/api/devices/vision-front/downloads/settings")
                .POST(HttpRequest.BodyPublishers.noBody())
                .build(),
            HttpResponse.BodyHandlers.ofString());

    assertThat(posted.statusCode()).isEqualTo(405);
    assertThat(posted.headers().firstValue("Allow")).hasValue("GET");
  }

  /** A device busy with another such request is passed on as busy, saying when to ask again. */
  @Test
  void aBusyDeviceIsServiceUnavailable() throws Exception {
    busy = true;
    try {
      HttpResponse<String> file = get("/api/devices/vision-front/downloads/settings");

      assertThat(file.statusCode()).isEqualTo(503);
      assertThat(file.headers().firstValue("Retry-After")).hasValue("1");
      assertThat(file.body()).contains("vision-front is busy");
    } finally {
      busy = false;
    }
  }

  private HttpResponse<String> post(String path, HttpRequest.BodyPublisher body, String type)
      throws Exception {
    return client.send(
        request(path).header("Content-Type", type).POST(body).build(),
        HttpResponse.BodyHandlers.ofString());
  }

  @Test
  void aFileSentToADeviceIsTakenOrRefusedSayingWhy() throws Exception {
    HttpResponse<String> taken =
        post(
            "/api/devices/venue/uploads/survey",
            HttpRequest.BodyPublishers.ofString("good"),
            "application/octet-stream");
    assertThat(taken.statusCode()).isEqualTo(200);
    assertThat(taken.body()).contains("Saved good");

    HttpResponse<String> refused =
        post(
            "/api/devices/venue/uploads/survey",
            HttpRequest.BodyPublishers.ofString("bad"),
            "application/octet-stream");
    assertThat(refused.statusCode()).isEqualTo(422);
    assertThat(refused.body()).contains("tag 26 sits 0.500 m");
  }

  @Test
  void aFileOverTheLimitIsRefusedUnread() throws Exception {
    HttpResponse<String> big =
        post(
            "/api/devices/venue/uploads/survey",
            HttpRequest.BodyPublishers.ofByteArray(new byte[64 * 1024 + 1]),
            "application/octet-stream");
    assertThat(big.statusCode()).isEqualTo(413);
  }

  @Test
  void aChoiceShowsOnTheCardAndIsMadeThere() throws Exception {
    assertThat(get("/api/devices").body())
        .contains("\"choices\":[{\"id\":\"venue\",\"label\":\"Venue\"")
        .contains("\"current\":\"official\"")
        .contains("\"uploads\":[{\"id\":\"survey\",\"label\":\"Upload venue\"}]");
    HttpResponse<String> chose =
        post(
            "/api/devices/venue/choices/venue",
            HttpRequest.BodyPublishers.ofString("{\"value\": \"practice\"}"),
            "application/merge-patch+json");
    assertThat(chose.statusCode()).isEqualTo(200);
    assertThat(chose.body()).contains("Chose practice");
    // Cards are read at most once a second (Devices.INTERVAL): the choice shows at the next read.
    Await.until(
        "the card showing the choice",
        () -> answer("/api/devices").contains("\"current\":\"practice\""));

    HttpResponse<String> refused =
        post(
            "/api/devices/venue/choices/venue",
            HttpRequest.BodyPublishers.ofString("{\"value\": \"moon\"}"),
            "application/merge-patch+json");
    assertThat(refused.statusCode()).isEqualTo(422);
    assertThat(refused.body()).contains("there is no venue moon");
    chosenVenue = "official";
  }

  @Test
  void changesToDevicesWaitWhileTheFieldIsConnected() throws Exception {
    fieldConnected.set(true);
    Await.until("the field system connected", tuning::isFieldConnected);
    HttpResponse<String> response =
        post(
            "/api/devices/venue/uploads/survey",
            HttpRequest.BodyPublishers.ofString("good"),
            "application/octet-stream");
    assertThat(response.statusCode()).isEqualTo(409);
  }

  @Test
  void onlyThePageMayChangeADeviceAndOnlyByPost() throws Exception {
    HttpResponse<String> noPage =
        client.send(
            HttpRequest.newBuilder(uri("/api/devices/venue/uploads/survey"))
                .POST(HttpRequest.BodyPublishers.ofString("good"))
                .build(),
            HttpResponse.BodyHandlers.ofString());
    assertThat(noPage.statusCode()).isEqualTo(400);
    HttpResponse<String> got = get("/api/devices/venue/uploads/survey");
    assertThat(got.statusCode()).isEqualTo(405);
    assertThat(got.headers().firstValue("Allow")).contains("POST");
    assertThat(
            post(
                    "/api/devices/venue/uploads/other",
                    HttpRequest.BodyPublishers.ofString("x"),
                    "application/octet-stream")
                .statusCode())
        .isEqualTo(404);
  }

  private HttpRequest.Builder request(String path) {
    return HttpRequest.newBuilder(uri(path)).header("Pitwall-Stream", STREAM);
  }

  private HttpResponse<String> get(String path) throws Exception {
    return client.send(request(path).GET().build(), HttpResponse.BodyHandlers.ofString());
  }

  /** A GET's body, for a condition to wait on. */
  private String answer(String path) {
    try {
      return get(path).body();
    } catch (Exception e) {
      throw new IllegalStateException(e);
    }
  }

  private URI uri(String path) {
    return URI.create("http://localhost:" + server.port() + path);
  }
}
