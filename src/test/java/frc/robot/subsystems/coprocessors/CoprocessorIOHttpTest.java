package frc.robot.subsystems.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.coprocessor.api.JournalPage;
import com.michaelgrundvig.frc.coprocessor.json.Json;
import com.michaelgrundvig.frc.coprocessor.table.Board;
import com.michaelgrundvig.frc.coprocessor.table.CompiledTable;
import com.michaelgrundvig.frc.coprocessor.table.Computer;
import com.michaelgrundvig.frc.coprocessor.table.Table;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import frc.robot.subsystems.coprocessors.CoprocessorIO.CoprocessorIOInputs;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * The robot's side of the agent's API over real HTTP: a stand-in agent on this computer serving
 * what the simulated agent would answer, polled on the IO's own thread as on a robot.
 */
class CoprocessorIOHttpTest {
  private static final Computer FRONT =
      new Computer("vision-front", 11, Board.ORANGEPI_5, List.of("front-left"), 5808);
  private static final CompiledTable TABLE =
      new CompiledTable(new Table(2611, 5808, List.of(FRONT)), "v2027.1.0", "recipe-1", Map.of());

  /** Fast polls, for a test: ten a second. */
  private static final Coprocessors.Settings FAST =
      CoprocessorsSettingsBuilder.builder(Coprocessors.Settings.DEFAULTS)
          .pollPeriodSeconds(0.1)
          .connectTimeoutSeconds(0.04)
          .answerTimeoutSeconds(0.05)
          .staleAfterSeconds(1)
          .build();

  private final CoprocessorIOSim agent =
      new CoprocessorIOSim(TABLE, FRONT, Coprocessors.Settings.DEFAULTS);
  private final List<String> asked = new CopyOnWriteArrayList<>();
  private volatile boolean garbled;

  @SuppressWarnings("NullAway.Init")
  private HttpServer server;

  @SuppressWarnings("NullAway.Init")
  private CoprocessorIOHttp io;

  @BeforeEach
  void start() throws IOException {
    agent.step(0);
    server = HttpServer.create(new InetSocketAddress(InetAddress.getLoopbackAddress(), 0), 0);
    server.createContext(
        "/v1/health",
        exchange ->
            send(exchange, garbled ? "{\"stamp\": [" : Json.compact(agent.answer().toJson())));
    server.createContext(
        "/v1/journal",
        exchange -> {
          String query = exchange.getRequestURI().getRawQuery();
          asked.add(query);
          Map<String, String> parameters = parameters(query);
          JournalPage page =
              agent.journal(
                  parameters.getOrDefault("cursor", ""),
                  Integer.parseInt(parameters.getOrDefault("priority", "7")),
                  parameters.getOrDefault("unit", ""));
          send(exchange, garbled ? "not JSON" : Json.compact(page.toJson()));
        });
    server.createContext(
        "/v1/settings.zip",
        exchange -> {
          byte[] zip = agent.settingsBackup();
          exchange.sendResponseHeaders(200, zip.length);
          try (OutputStream out = exchange.getResponseBody()) {
            out.write(zip);
          }
        });
    server.createContext(
        "/v1/shutdown",
        exchange -> {
          asked.add(exchange.getRequestMethod() + " " + exchange.getRequestURI().getPath());
          exchange.sendResponseHeaders(202, -1);
          exchange.close();
        });
    server.start();
    io =
        new CoprocessorIOHttp(
            "vision-front", "127.0.0.1", server.getAddress().getPort(), FAST, System::nanoTime);
  }

  @AfterEach
  void stop() {
    io.close();
    server.stop(0);
  }

  private static void send(HttpExchange exchange, String body) throws IOException {
    byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
    exchange.sendResponseHeaders(200, bytes.length);
    try (OutputStream out = exchange.getResponseBody()) {
      out.write(bytes);
    }
  }

  private static Map<String, String> parameters(String query) {
    Map<String, String> parameters = new LinkedHashMap<>();
    for (String pair : query.split("&")) {
      String[] parts = pair.split("=", 2);
      parameters.put(parts[0], URLDecoder.decode(parts[1], StandardCharsets.UTF_8));
    }
    return parameters;
  }

  /** Reads the inputs as the loop does, until they say so, or five seconds pass. */
  private CoprocessorIOInputs until(BooleanSupplier done, CoprocessorIOInputs inputs)
      throws InterruptedException {
    long deadline = System.nanoTime() + 5_000_000_000L;
    io.updateInputs(inputs);
    while (!done.getAsBoolean() && System.nanoTime() < deadline) {
      Thread.sleep(10);
      io.updateInputs(inputs);
    }
    return inputs;
  }

  @Test
  void theLoopReadsTheLatestAnswer() throws Exception {
    CoprocessorIOInputs inputs = new CoprocessorIOInputs();

    until(() -> inputs.everAnswered, inputs);

    assertThat(inputs.answered).isTrue();
    assertThat(inputs.error).isEmpty();
    assertThat(inputs.name).isEqualTo("vision-front");
    assertThat(inputs.address).isEqualTo("10.26.11.11");
    assertThat(inputs.photonVisionVersion).isEqualTo("v2027.1.0");
    assertThat(inputs.recipeHash).isEqualTo("recipe-1");
    assertThat(inputs.role).containsExactly("front-left");
    assertThat(inputs.camerasPresent).containsExactly("front-left");
    assertThat(inputs.usbCameras).singleElement().asString().contains("Arducam", "5000 Mb/s");
    assertThat(inputs.bigCoreMaxMhz).isEqualTo(2400);
    assertThat(inputs.bigCoreLimitMhz).isEqualTo(2400);
    assertThat(inputs.coreBusyPercent).hasSize(8);
    assertThat(inputs.zones).contains("bigcore0-thermal");
    assertThat(inputs.zoneTripCelsius).containsOnly(85.0);
    assertThat(inputs.photonVisionState).isEqualTo("active/running");
    assertThat(inputs.agentMonotonicNanos).isEqualTo(40_000_000_000L);
    assertThat(inputs.lastShutdownKnown).isTrue();
    assertThat(inputs.driveFound).isTrue();
    assertThat(inputs.ageSeconds).isLessThan(1);
    assertThat(inputs.macsTakingTurns).isEmpty();
  }

  @Test
  void anAnswerThatCantBeReadIsNoAnswer() throws Exception {
    garbled = true;
    CoprocessorIOInputs inputs = new CoprocessorIOInputs();

    until(() -> !inputs.error.isEmpty() && !inputs.error.equals("not asked yet"), inputs);

    assertThat(inputs.everAnswered).isFalse();
    assertThat(inputs.error).startsWith("its answer can't be read");
  }

  @Test
  void aSilentAgentIsSaidWhyAndItsLastAnswerAges() throws Exception {
    CoprocessorIOInputs inputs = new CoprocessorIOInputs();
    until(() -> inputs.everAnswered, inputs);

    server.stop(0);
    until(() -> !inputs.answered, inputs);

    assertThat(inputs.everAnswered).as("its last answer stays").isTrue();
    assertThat(inputs.error).isNotEmpty();
    assertThat(inputs.name).isEqualTo("vision-front");
  }

  @Test
  void itsBackupAndJournalAreFetchedOnRequest() throws Exception {
    agent.restartPhotonVision();

    byte[] zip = io.settingsBackup();
    JournalPage errors = io.journal("", 3, "photonvision.service");
    JournalPage after = io.journal(errors.cursor(), 7, "");

    assertThat(zip).startsWith('P', 'K');
    assertThat(errors.entries())
        .singleElement()
        .satisfies(e -> assertThat(e.priority()).isEqualTo(3));
    assertThat(after.entries()).hasSize(1);
    assertThat(asked)
        .containsExactly(
            "priority=3&limit=500&unit=photonvision.service", "priority=7&limit=500&cursor=sim-3");
  }

  @Test
  void aJournalThatCantBeReadSaysSo() {
    garbled = true;

    assertThatThrownBy(() -> io.journal("", 7, ""))
        .isInstanceOf(IOException.class)
        .hasMessageStartingWith("its journal can't be read");
  }

  /**
   * Asked to power down, it posts the request, then tries both ports until neither answers: here
   * the stand-in agent stops, and nothing listens on PhotonVision's port on this computer.
   */
  @Test
  void poweringDownAsksThenWaitsUntilItsPortsAreGone() throws Exception {
    CoprocessorIOInputs inputs = new CoprocessorIOInputs();
    until(() -> inputs.everAnswered, inputs);

    io.powerDown();
    until(() -> inputs.portsTried, inputs);
    boolean openWhileUp = inputs.agentPortOpen;
    server.stop(0);
    until(() -> !inputs.agentPortOpen, inputs);

    assertThat(inputs.powerDownAsked).isTrue();
    assertThat(inputs.powerDownAnswer).isEqualTo("accepted");
    assertThat(asked).contains("POST /v1/shutdown");
    assertThat(openWhileUp).isTrue();
    assertThat(inputs.agentPortOpen).isFalse();
    assertThat(inputs.photonVisionPortOpen).isFalse();
  }
}
