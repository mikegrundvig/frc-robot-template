package com.michaelgrundvig.frc.coprocessor.agent;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.coprocessor.api.AgentApi;
import com.michaelgrundvig.frc.coprocessor.api.Health;
import com.michaelgrundvig.frc.coprocessor.api.JournalPage;
import com.michaelgrundvig.frc.coprocessor.api.ShutdownAnswer;
import com.michaelgrundvig.frc.coprocessor.api.Stamp;
import com.michaelgrundvig.frc.coprocessor.json.Json;
import com.michaelgrundvig.frc.coprocessor.settings.PhotonVisionDatabase;
import com.michaelgrundvig.frc.coprocessor.settings.Settings;
import com.michaelgrundvig.frc.coprocessor.settings.SettingsDatabase;
import com.michaelgrundvig.frc.coprocessor.settings.SettingsFiles;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.nio.file.Path;
import java.sql.Connection;
import java.sql.SQLException;
import java.time.Duration;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** The API over HTTP, as the robot uses it. */
class AgentServerTest {
  @TempDir Path dir;
  Fixture fixture;
  AgentServer server;
  final HttpClient client = HttpClient.newBuilder().connectTimeout(Duration.ofSeconds(5)).build();

  @BeforeEach
  void anAgent() throws IOException {
    fixture = new Fixture(dir);
    fixture.commands.answer(List.of("journalctl"), Fixture.lines("journal-this-boot.json"));
    fixture.commands.answer(List.of("systemctl", "stop"), List.of());
    fixture.commands.answer(List.of("systemctl", "poweroff"), List.of());
    serve(fixture.agent());
  }

  private void serve(Agent agent) throws IOException {
    if (server != null) {
      server.close();
    }
    server = new AgentServer(agent, new InetSocketAddress(InetAddress.getLoopbackAddress(), 0));
  }

  @AfterEach
  void stop() {
    server.close();
  }

  private HttpResponse<byte[]> get(String path) throws IOException, InterruptedException {
    return send(HttpRequest.newBuilder(uri(path)).GET());
  }

  private HttpResponse<byte[]> post(String path) throws IOException, InterruptedException {
    return send(HttpRequest.newBuilder(uri(path)).POST(HttpRequest.BodyPublishers.noBody()));
  }

  private HttpResponse<byte[]> send(HttpRequest.Builder request)
      throws IOException, InterruptedException {
    return client.send(
        request.timeout(Duration.ofSeconds(10)).build(), HttpResponse.BodyHandlers.ofByteArray());
  }

  private URI uri(String path) {
    return URI.create("http://127.0.0.1:" + server.port() + path);
  }

  private static String text(HttpResponse<byte[]> response) {
    return new String(response.body(), StandardCharsets.UTF_8);
  }

  @Test
  void theStampAndHealthAreJson() throws Exception {
    HttpResponse<byte[]> stamp = get(AgentApi.STAMP);
    assertThat(stamp.statusCode()).isEqualTo(200);
    assertThat(stamp.headers().firstValue("Content-Type"))
        .contains("application/json; charset=utf-8");
    assertThat(stamp.headers().firstValue("Cache-Control")).contains("no-store");
    Stamp read = Stamp.parse(text(stamp));
    assertThat(read.name()).isEqualTo("vision-front");
    assertThat(read.mac()).isEqualTo("c0:74:2b:fe:12:34");

    HttpResponse<byte[]> health = get(AgentApi.HEALTH);
    assertThat(health.statusCode()).isEqualTo(200);
    assertThat(Health.parse(text(health)).thermal()).hasSize(7);
  }

  @Test
  void theJournalIsPagedAndItsQueryChecked() throws Exception {
    HttpResponse<byte[]> page =
        get(AgentApi.JOURNAL + "?priority=err&unit=photonvision.service&limit=2");
    assertThat(page.statusCode()).isEqualTo(200);
    JournalPage read = JournalPage.parse(text(page));
    assertThat(read.entries()).hasSize(2);
    assertThat(read.more()).isTrue();
    assertThat(fixture.commands.ran().get(fixture.commands.ran().size() - 1))
        .contains("--priority=3", "--unit=photonvision.service", "--lines=2");

    assertThat(get(AgentApi.JOURNAL + "?cursor=" + "s%3Da1%3Bi%3D101&priority=4").statusCode())
        .isEqualTo(200);
    assertThat(fixture.commands.ran().get(fixture.commands.ran().size() - 1))
        .contains("--after-cursor=s=a1;i=101", "--priority=4");

    for (String bad :
        List.of(
            "?limit=0",
            "?limit=501",
            "?limit=x",
            "?priority=8",
            "?priority=loud",
            "?unit=a%20b",
            "?cursor=a%20--since",
            "?cursor=%25zz")) {
      HttpResponse<byte[]> refused = get(AgentApi.JOURNAL + bad);
      assertThat(refused.statusCode()).as(bad).isEqualTo(400);
      assertThat(Json.parse(text(refused)).asObject("error").string("error", "")).isNotEmpty();
    }
  }

  @Test
  void theSettingsAreTheDatabasesRowsAndTheirHash() throws Exception {
    Settings settings;
    try (Connection connection = PhotonVisionDatabase.open(fixture.host.path(Fixture.DATABASE))) {
      settings = SettingsDatabase.read(connection);
    } catch (SQLException e) {
      throw new IOException(e);
    }
    HttpResponse<byte[]> json = get(AgentApi.SETTINGS);
    assertThat(json.statusCode()).isEqualTo(200);
    assertThat(text(json)).isEqualTo(Json.compact(settings.toJson()));
    assertThat(Settings.parse(text(json))).isEqualTo(settings);

    HttpResponse<byte[]> zip = get(AgentApi.SETTINGS_ZIP);
    assertThat(zip.statusCode()).isEqualTo(200);
    assertThat(zip.headers().firstValue("Content-Disposition"))
        .contains("attachment; filename=\"vision-front-settings.zip\"");
    Map<String, String> files = new TreeMap<>();
    try (ZipInputStream in = new ZipInputStream(new ByteArrayInputStream(zip.body()))) {
      ZipEntry entry;
      while ((entry = in.getNextEntry()) != null) {
        files.put(entry.getName(), new String(in.readAllBytes(), StandardCharsets.UTF_8));
      }
    }
    Map<String, String> expected = new TreeMap<>();
    SettingsFiles.render(settings)
        .forEach((path, text) -> expected.put("coprocessors/vision-front/settings/" + path, text));
    assertThat(files).isEqualTo(expected);
  }

  @Test
  void withoutADatabaseThereAreNoSettings() throws Exception {
    fixture.delete(Fixture.DATABASE);
    assertThat(get(AgentApi.SETTINGS).statusCode()).isEqualTo(404);
    assertThat(get(AgentApi.SETTINGS_ZIP).statusCode()).isEqualTo(404);
  }

  @Test
  void unknownPathsAndMethodsAreRefused() throws Exception {
    assertThat(get("/v2/health").statusCode()).isEqualTo(404);
    HttpResponse<byte[]> post = post(AgentApi.HEALTH);
    assertThat(post.statusCode()).isEqualTo(405);
    assertThat(post.headers().firstValue("Allow")).contains("GET");
    HttpResponse<byte[]> get = get(AgentApi.SHUTDOWN);
    assertThat(get.statusCode()).isEqualTo(405);
    assertThat(get.headers().firstValue("Allow")).contains("POST");
  }

  @Test
  void onlyTheRobotControllerMayShutTheComputerDown() throws Exception {
    // The stamp's team is 2611: the robot controller is 10.26.11.2, and this test isn't.
    HttpResponse<byte[]> refused = post(AgentApi.SHUTDOWN);
    assertThat(refused.statusCode()).isEqualTo(403);
    assertThat(text(refused)).contains("only the robot controller (10.26.11.2)");
    assertThat(fixture.commands.ran()).noneMatch(command -> command.contains("poweroff"));
  }

  @Test
  void aShutdownStopsPhotonVisionThenPowersOffOnce() throws Exception {
    serve(fixture.agent("127.0.0.1"));
    HttpResponse<byte[]> first = post(AgentApi.SHUTDOWN);
    assertThat(first.statusCode()).isEqualTo(202);
    assertThat(ShutdownAnswer.parse(text(first)).alreadyRequested()).isFalse();
    List<List<String>> ran = fixture.commands.ran();
    assertThat(ran.subList(ran.size() - 2, ran.size()))
        .containsExactly(
            List.of("systemctl", "stop", "photonvision.service"), List.of("systemctl", "poweroff"));
    assertThat(fixture.log.get(0))
        .isEqualTo(
            "Shutdown asked for by 127.0.0.1: stopping photonvision.service, then powering off");

    HttpResponse<byte[]> again = post(AgentApi.SHUTDOWN);
    assertThat(again.statusCode()).isEqualTo(202);
    assertThat(ShutdownAnswer.parse(text(again)).alreadyRequested()).isTrue();
    assertThat(fixture.commands.ran()).hasSize(ran.size());

    fixture.micros.addAndGet(1_000_000);
    assertThat(Health.parse(text(get(AgentApi.HEALTH))).problems())
        .first()
        .isEqualTo("shutting down: asked for by the robot");
  }

  @Test
  void aShutdownGoesOnWhenStoppingPhotonVisionFails() throws Exception {
    fixture.commands.answer(
        List.of("systemctl", "stop"), new Commands.Output(1, List.of(), false, false));
    fixture.commands.answer(
        List.of("systemctl", "poweroff"), new Commands.Output(-1, List.of(), false, true));
    serve(fixture.agent("127.0.0.1"));
    assertThat(post(AgentApi.SHUTDOWN).statusCode()).isEqualTo(202);
    assertThat(fixture.log)
        .contains(
            "Stopping photonvision.service failed (exit 1); powering off anyway",
            "Powering off failed (timed out)");
  }

  @Test
  void aSourceThatThrowsIsAServerError() throws Exception {
    fixture.write("/etc/coprocessor/stamp.json", "[]");
    HttpResponse<byte[]> stamp = get(AgentApi.STAMP);
    assertThat(stamp.statusCode()).isEqualTo(500);
    assertThat(text(stamp)).contains("expected an object");
  }
}
