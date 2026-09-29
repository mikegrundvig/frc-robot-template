package com.michaelgrundvig.frc.pitwall;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.eclipse.jetty.http.HttpHeader;
import org.eclipse.jetty.io.Content;
import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Request;
import org.eclipse.jetty.server.Response;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.ServerConnector;
import org.eclipse.jetty.util.Callback;
import org.eclipse.jetty.util.thread.QueuedThreadPool;
import tools.jackson.core.JacksonException;
import tools.jackson.databind.JsonNode;
import tools.jackson.databind.json.JsonMapper;

/**
 * Pitwall's page and its REST API, served by the robot program itself: http://localhost:5800 in the
 * simulator, http://robot.local:5800 on the robot. Any browser works, a phone's included, and so
 * does anything else that speaks HTTP. So far it serves live tuning ({@link Tuning}):
 *
 * <ul>
 *   <li>{@code GET /api/tuning}: the session, {@code {"name", "enabled", "fieldConnected"}}. {@code
 *       PATCH} it with {@code {"enabled": true}} to switch tuning on or off.
 *   <li>{@code GET /api/tunables}: what can be tuned, {@code {path: {"unit", "committed"}}}.
 *   <li>{@code GET /api/values}: the current values, {@code {path: number}}. {@code PATCH} it with
 *       a JSON merge patch (RFC 7396) to change several at once, all or none; {@code null} returns
 *       a value to its committed one.
 *   <li>{@code GET /api/changes}: what differs from the committed configuration, with the builder
 *       calls that commit it ({@code Accept: text/plain} for the report). {@code DELETE} it to
 *       return every value to its committed one.
 * </ul>
 *
 * <p>A change waits for the robot loop and answers with what became of it: 200 and the new state
 * when applied, 422 when the configuration's checks refuse it, 409 when tuning is off or the field
 * system is connected, and 202 when the loop didn't get to it in time (the next GET shows the
 * result). Errors are problem details (RFC 9457).
 *
 * <p>The server's threads never change the robot: they read {@link Tuning#view()} and queue
 * requests for the robot loop to apply.
 */
public final class PitwallServer implements AutoCloseable {
  /** In the range of ports the field network leaves open for teams. */
  public static final int PORT = 5800;

  /** How long a change waits for the robot loop before answering 202 Accepted. */
  static final Duration WAIT = Duration.ofSeconds(1);

  private final Server server;
  private final ServerConnector connector;

  /**
   * @param port the port to serve on; 0 picks a free one
   */
  public PitwallServer(Tuning<?> tuning, int port) {
    this(tuning, port, WAIT);
  }

  PitwallServer(Tuning<?> tuning, int port, Duration wait) {
    // A few threads are plenty for one page, and daemon threads never keep the program running.
    QueuedThreadPool threads = new QueuedThreadPool(8, 2);
    threads.setName("pitwall");
    threads.setDaemon(true);
    server = new Server(threads);
    connector = new ServerConnector(server, 1, 1);
    connector.setPort(port);
    server.addConnector(connector);
    server.setHandler(new Api(tuning, page(), wait));
  }

  /**
   * Starts the page, or says why it couldn't. The page is a convenience: the robot runs without it.
   *
   * @param port usually {@link #PORT}
   * @param say prints a line where the team sees it
   * @return the running server, or empty if it couldn't start
   */
  public static Optional<PitwallServer> startOrExplain(
      Tuning<?> tuning, int port, Consumer<String> say) {
    PitwallServer server = null;
    try {
      server = new PitwallServer(tuning, port);
      server.start();
      say.accept(
          "Pitwall: http://localhost:"
              + server.port()
              + " in the simulator, http://robot.local:"
              + server.port()
              + " on the robot");
      return Optional.of(server);
    } catch (Exception e) {
      say.accept("Pitwall's page couldn't start, so it is unavailable: " + e);
      if (server != null) {
        stopQuietly(server);
      }
      return Optional.empty();
    }
  }

  private static void stopQuietly(PitwallServer server) {
    try {
      server.close();
    } catch (Exception e) {
      // It never started; there is nothing more to release.
    }
  }

  public void start() throws Exception {
    server.start();
  }

  /** The port it serves on. */
  public int port() {
    return connector.getLocalPort();
  }

  @Override
  public void close() throws Exception {
    server.stop();
  }

  private static String page() {
    try (InputStream in = PitwallServer.class.getResourceAsStream("index.html")) {
      if (in == null) {
        throw new IllegalStateException("Pitwall's page (index.html) is missing");
      }
      return new String(in.readAllBytes(), StandardCharsets.UTF_8);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  /** An answer that says what went wrong, as problem details (RFC 9457). */
  private static final class Problem extends RuntimeException {
    private static final long serialVersionUID = 1;

    private final int status;
    private final String title;

    Problem(int status, String title, String detail) {
      super(detail);
      this.status = status;
      this.title = title;
    }
  }

  /** The page, and the resources of the API. */
  private static final class Api extends Handler.Abstract {
    private static final Map<String, List<String>> METHODS =
        Map.of(
            "/", List.of("GET"),
            "/api/tuning", List.of("GET", "PATCH"),
            "/api/tunables", List.of("GET"),
            "/api/values", List.of("GET", "PATCH"),
            "/api/changes", List.of("GET", "DELETE"));

    private final Tuning<?> tuning;
    private final String page;
    private final Duration wait;
    private final JsonMapper json = JsonMapper.builder().build();

    Api(Tuning<?> tuning, String page, Duration wait) {
      this.tuning = tuning;
      this.page = page;
      this.wait = wait;
    }

    @Override
    public boolean handle(Request request, Response response, Callback callback) {
      String path = Request.getPathInContext(request);
      String method = request.getMethod();
      try {
        List<String> allowed = METHODS.get(path);
        if (allowed == null) {
          throw new Problem(404, "Not found", "There is nothing at " + path);
        }
        if (!allowed.contains(method)) {
          response.getHeaders().put(HttpHeader.ALLOW, String.join(", ", allowed));
          throw new Problem(
              405, "Method not allowed", path + " allows " + String.join(", ", allowed));
        }
        switch (method + " " + path) {
          case "GET /" -> send(response, callback, 200, "text/html", page);
          case "GET /api/tuning" -> sendJson(response, callback, 200, session());
          case "PATCH /api/tuning" -> patchTuning(request, response, callback);
          case "GET /api/tunables" -> sendJson(response, callback, 200, tunables());
          case "GET /api/values" -> sendJson(response, callback, 200, values());
          case "PATCH /api/values" -> patchValues(request, response, callback);
          case "GET /api/changes" -> getChanges(request, response, callback);
          case "DELETE /api/changes" ->
              answer(tuning.requestReset(), response, callback, this::changes);
          default -> throw new IllegalStateException("unrouted: " + method + " " + path);
        }
      } catch (Problem problem) {
        sendProblem(response, callback, problem);
      }
      return true;
    }

    private void patchTuning(Request request, Response response, Callback callback) {
      JsonNode patch = readPatch(request);
      for (String name : patch.propertyNames()) {
        if (!name.equals("enabled")) {
          throw new Problem(422, "Not changeable", "Only enabled can be changed, not " + name);
        }
      }
      JsonNode enabled = patch.get("enabled");
      if (enabled == null) {
        sendJson(response, callback, 200, session()); // an empty patch changes nothing
        return;
      }
      if (!enabled.isBoolean()) {
        throw new Problem(400, "Not true or false", "enabled must be true or false");
      }
      answer(tuning.requestEnabled(enabled.booleanValue()), response, callback, this::session);
    }

    private void patchValues(Request request, Response response, Callback callback) {
      JsonNode patch = readPatch(request);
      Map<String, Tuning.Value> known = new LinkedHashMap<>();
      for (Tuning.Value value : tuning.view().values()) {
        known.put(value.path(), value);
      }
      Map<String, Double> values = new LinkedHashMap<>();
      for (Map.Entry<String, JsonNode> field : patch.properties()) {
        Tuning.Value value = known.get(field.getKey());
        if (value == null) {
          throw new Problem(422, "Not tunable", "There is no tunable value at " + field.getKey());
        }
        JsonNode number = field.getValue();
        if (number.isNull()) {
          values.put(value.path(), value.committed());
        } else if (number.isNumber()) {
          values.put(value.path(), number.doubleValue());
        } else {
          throw new Problem(
              400,
              "Not a number",
              value.path() + " must be a number, or null to return it to its committed value");
        }
      }
      answer(tuning.requestChanges(values), response, callback, this::values);
    }

    private void getChanges(Request request, Response response, Callback callback) {
      String accept = request.getHeaders().get(HttpHeader.ACCEPT);
      if (accept != null && accept.contains("text/plain") && !accept.contains("json")) {
        List<String> report = tuning.report();
        send(response, callback, 200, "text/plain", String.join("\n", report) + "\n");
      } else {
        sendJson(response, callback, 200, changes());
      }
    }

    /** Waits for the robot loop, then answers with what became of the request. */
    private void answer(
        CompletableFuture<Tuning.Outcome> pending,
        Response response,
        Callback callback,
        Supplier<Object> state) {
      Tuning.Outcome outcome;
      try {
        outcome = pending.get(wait.toMillis(), TimeUnit.MILLISECONDS);
      } catch (TimeoutException e) {
        sendJson(
            response,
            callback,
            202,
            Map.of("detail", "Queued for the robot loop; the next GET shows the result"));
        return;
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new Problem(503, "Interrupted", "The server stopped waiting for the robot loop");
      } catch (ExecutionException e) {
        throw new IllegalStateException(e);
      }
      switch (outcome.result()) {
        case APPLIED -> sendJson(response, callback, 200, state.get());
        case REFUSED -> throw new Problem(422, "Refused", outcome.detail());
        case LOCKED -> throw new Problem(409, "Locked", outcome.detail());
      }
    }

    private JsonNode readPatch(Request request) {
      String type = request.getHeaders().get(HttpHeader.CONTENT_TYPE);
      if (type == null
          || !(type.startsWith("application/merge-patch+json")
              || type.startsWith("application/json"))) {
        throw new Problem(
            415,
            "Unsupported media type",
            "Send a JSON merge patch, with Content-Type: application/merge-patch+json");
      }
      try {
        JsonNode patch = json.readTree(Content.Source.asString(request, StandardCharsets.UTF_8));
        if (patch == null || !patch.isObject()) {
          throw new Problem(400, "Not an object", "A merge patch is a JSON object");
        }
        return patch;
      } catch (JacksonException e) {
        throw new Problem(400, "Not JSON", String.valueOf(e.getOriginalMessage()));
      } catch (IOException e) {
        throw new Problem(400, "Unreadable", String.valueOf(e.getMessage()));
      }
    }

    private Map<String, Object> session() {
      Tuning.View view = tuning.view();
      Map<String, Object> session = new LinkedHashMap<>();
      session.put("name", view.name());
      session.put("enabled", view.enabled());
      session.put("fieldConnected", view.fieldConnected());
      return session;
    }

    private Map<String, Object> tunables() {
      Map<String, Object> tunables = new LinkedHashMap<>();
      for (Tuning.Value value : tuning.view().values()) {
        Map<String, Object> description = new LinkedHashMap<>();
        description.put("unit", value.unit());
        description.put("committed", value.committed());
        tunables.put(value.path(), description);
      }
      return tunables;
    }

    private Map<String, Object> values() {
      Map<String, Object> values = new LinkedHashMap<>();
      for (Tuning.Value value : tuning.view().values()) {
        values.put(value.path(), value.current());
      }
      return values;
    }

    private Object changes() {
      return tuning.view().changes();
    }

    private void sendJson(Response response, Callback callback, int status, Object body) {
      send(response, callback, status, "application/json", json.writeValueAsString(body));
    }

    private void sendProblem(Response response, Callback callback, Problem problem) {
      Map<String, Object> body = new LinkedHashMap<>();
      body.put("type", "about:blank");
      body.put("title", problem.title);
      body.put("status", problem.status);
      body.put("detail", problem.getMessage());
      send(
          response,
          callback,
          problem.status,
          "application/problem+json",
          json.writeValueAsString(body));
    }

    private static void send(
        Response response, Callback callback, int status, String type, String body) {
      response.setStatus(status);
      response.getHeaders().put(HttpHeader.CONTENT_TYPE, type + "; charset=utf-8");
      response.getHeaders().put(HttpHeader.CACHE_CONTROL, "no-store");
      Content.Sink.write(response, true, body, callback);
    }
  }
}
