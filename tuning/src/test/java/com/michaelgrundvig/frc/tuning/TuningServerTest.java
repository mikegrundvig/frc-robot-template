package com.michaelgrundvig.frc.tuning;

import static com.michaelgrundvig.frc.tuning.Fixtures.machine;
import static org.assertj.core.api.Assertions.assertThat;

import java.net.URI;
import java.net.URLEncoder;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

/** The tuning page's web server, over real HTTP on a free port. */
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class TuningServerTest {
  private static final String KP = "drive.gains.kP";

  private final HttpClient client = HttpClient.newHttpClient();
  private final Tuning<Fixtures.Machine> tuning =
      new Tuning<>("test", machine(), true, inputs -> {}, view -> {});
  private final TuningServer server = new TuningServer(tuning, 0);

  @BeforeAll
  void start() throws Exception {
    server.start();
  }

  @AfterAll
  void stop() throws Exception {
    server.close();
  }

  @AfterEach
  void backToCommitted() {
    tuning.requestEnabled(true);
    tuning.requestReset();
    tuning.periodic(false);
  }

  @Test
  void servesThePage() throws Exception {
    HttpResponse<String> page = get("/");

    assertThat(page.statusCode()).isEqualTo(200);
    assertThat(page.headers().firstValue("Content-Type")).hasValue("text/html; charset=utf-8");
    assertThat(page.body()).contains("<title>Robot tuning</title>");
  }

  @Test
  void servesTheState() throws Exception {
    HttpResponse<String> state = get("/api/state");

    assertThat(state.statusCode()).isEqualTo(200);
    assertThat(state.headers().firstValue("Content-Type"))
        .hasValue("application/json; charset=utf-8");
    assertThat(state.body()).contains("\"name\":\"test\"", "\"path\":\"" + KP + "\"");
  }

  @Test
  void changesAreQueuedForTheRobotLoop() throws Exception {
    assertThat(post("/api/set", Map.of("path", KP, "value", "18.5")).statusCode()).isEqualTo(202);
    assertThat(tuning.config().drive().gains().kP()).as("before the loop").isEqualTo(16);

    tuning.periodic(false);

    assertThat(get("/api/state").body()).contains("\"builderCall\":\".kP(18.5)\"");
  }

  @Test
  void refusesUnknownValuesAndNonNumbers() throws Exception {
    HttpResponse<String> unknown = post("/api/set", Map.of("path", "nothing", "value", "1"));
    HttpResponse<String> notANumber = post("/api/set", Map.of("path", KP, "value", "lots"));

    assertThat(unknown.statusCode()).isEqualTo(404);
    assertThat(unknown.body()).isEqualTo("No tunable value at nothing");
    assertThat(notANumber.statusCode()).isEqualTo(400);
    assertThat(notANumber.body()).isEqualTo("Not a number: lots");
  }

  @Test
  void switchesTuningOffAndResets() throws Exception {
    tuning.requestSet(KP, 18.5);
    tuning.periodic(false);

    assertThat(post("/api/reset", Map.of()).statusCode()).isEqualTo(202);
    assertThat(post("/api/enabled", Map.of("on", "false")).statusCode()).isEqualTo(202);
    tuning.periodic(false);

    assertThat(tuning.view().changes()).isEmpty();
    assertThat(tuning.view().enabled()).isFalse();
  }

  @Test
  void startOrExplainStartsThePageAndSaysWhere() throws Exception {
    List<String> said = new ArrayList<>();

    Optional<TuningServer> started = TuningServer.startOrExplain(tuning, 0, said::add);

    assertThat(started).isPresent();
    try {
      assertThat(said).singleElement().asString().startsWith("Tuning page: http://localhost:");
    } finally {
      started.get().close();
    }
  }

  @Test
  void aPageThatCannotStartOnlySaysWhy() {
    List<String> said = new ArrayList<>();

    // The test's own server already holds this port.
    Optional<TuningServer> started = TuningServer.startOrExplain(tuning, server.port(), said::add);

    assertThat(started).isEmpty();
    assertThat(said).singleElement().asString().startsWith("The tuning page couldn't start");
  }

  @Test
  void anythingElseIsNotFound() throws Exception {
    assertThat(get("/admin").statusCode()).isEqualTo(404);
  }

  private HttpResponse<String> get(String path) throws Exception {
    return client.send(
        HttpRequest.newBuilder(uri(path)).GET().build(), HttpResponse.BodyHandlers.ofString());
  }

  private HttpResponse<String> post(String path, Map<String, String> form) throws Exception {
    String body =
        form.entrySet().stream()
            .map(
                field ->
                    URLEncoder.encode(field.getKey(), StandardCharsets.UTF_8)
                        + "="
                        + URLEncoder.encode(field.getValue(), StandardCharsets.UTF_8))
            .collect(Collectors.joining("&"));
    return client.send(
        HttpRequest.newBuilder(uri(path))
            .header("Content-Type", "application/x-www-form-urlencoded")
            .POST(HttpRequest.BodyPublishers.ofString(body))
            .build(),
        HttpResponse.BodyHandlers.ofString());
  }

  private URI uri(String path) {
    return URI.create("http://localhost:" + server.port() + path);
  }
}
