package com.michaelgrundvig.frc.pitwall;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import io.avaje.json.JsonException;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * Pitwall's page and its REST API, served by the robot program itself: http://localhost:5820 in the
 * simulator, http://robot.local:5820 on the robot. Any browser works, a phone's included, and so
 * does anything else that speaks HTTP. It serves the robot's status ({@link Status}), actions
 * ({@link Actions}), live tuning of the configuration ({@link Tuning}), and every WPILib tunable in
 * the program ({@link TunableTee}):
 *
 * <ul>
 *   <li>{@code GET /api/status}: the status items as of the last loop, {@code [{"name", "value"}]}.
 *   <li>{@code GET /api/actions}: every action and where it stands, {@code [{"name", "phase",
 *       "detail"}]}. {@code POST} it {@code {"start": name}} or {@code {"cancel": name}}.
 *   <li>{@code GET /api/tuning}: the session, {@code {"name", "enabled", "fieldConnected"}}. {@code
 *       PATCH} it with {@code {"enabled": true}} to switch tuning on or off.
 *   <li>{@code GET /api/tunables}: what can be tuned, {@code {path: {"unit", "committed"}}}.
 *   <li>{@code GET /api/values}: the current values, {@code {path: number}}. {@code PATCH} it with
 *       a JSON merge patch (RFC 7396) to change several at once, all or none; {@code null} returns
 *       a value to its committed one.
 *   <li>{@code GET /api/changes}: what differs from the committed configuration, with the builder
 *       calls that commit it ({@code Accept: text/plain} for the report). {@code DELETE} it to
 *       return every value to its committed one.
 *   <li>{@code GET /api/wpilib-tunables}: every other WPILib tunable, {@code [{"path", "type",
 *       "value", "editable"}]}. {@code PATCH} it with {@code {path: value}} to change several at
 *       once, all or none.
 * </ul>
 *
 * <p>A change waits for the robot loop and answers with what became of it: 200 and the new state
 * when applied, 422 when the configuration's checks (or the action) refuse it, 409 when it is
 * locked out (tuning off, not enabled in utility mode, or the field system connected), and 202 when
 * the loop didn't get to it in time (the next GET shows the result). Errors are problem details
 * (RFC 9457).
 *
 * <p>The server's threads never change the robot: they read what the robot loop last published
 * ({@link Status#view()}, {@link Actions#view()}, {@link Tuning#view()}) and queue requests for the
 * robot loop to apply.
 *
 * <p>Nothing here is bundled: the web server is the one built into Java ({@code
 * com.sun.net.httpserver}), and JSON is avaje-jsonb, which WPILib gives every robot program.
 */
public final class PitwallServer implements AutoCloseable {
  /**
   * Deliberately outside 5800-5810, the only team ports the field network lets through. Tools that
   * must work on the field crowd into that range: on a Systemcore, Elastic's package serves on
   * 5803, AdvantageScope Lite on 5808, and NetworkTables on 5810, and Elastic's docs have teams
   * serve layouts from the robot program on 5800. Pitwall is locked while the field is connected,
   * so it doesn't need the range. If something on your robot already uses this port, pass another
   * to {@link #startOrExplain}.
   */
  public static final int PORT = 5820;

  /** How long a change waits for the robot loop before answering 202 Accepted. */
  static final Duration WAIT = Duration.ofSeconds(1);

  /** The page's requests are a few hundred bytes; anything much bigger isn't from it. */
  private static final int MAX_BODY_BYTES = 64 * 1024;

  /** How deeply a request's JSON may nest: the page sends flat objects. */
  private static final int MAX_NESTING = 8;

  private final HttpServer server;
  private final ExecutorService threads;
  private final int port;

  /**
   * @param port the port to serve on; 0 picks a free one
   */
  public PitwallServer(
      Status status, Tuning<?> tuning, Actions actions, TunableTee tunables, int port) {
    this(status, tuning, actions, tunables, port, WAIT);
  }

  PitwallServer(
      Status status,
      Tuning<?> tuning,
      Actions actions,
      TunableTee tunables,
      int port,
      Duration wait) {
    this.port = port;
    try {
      server = HttpServer.create();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    // A few threads are plenty for one page, and daemon threads never keep the program running.
    threads =
        Executors.newFixedThreadPool(
            4,
            runnable -> {
              Thread thread = new Thread(runnable, "pitwall");
              thread.setDaemon(true);
              return thread;
            });
    server.setExecutor(threads);
    Api api = new Api(status, tuning, actions, tunables, page(), wait);
    server.createContext("/", api::handle);
  }

  /**
   * Starts the page, or says why it couldn't. The page is a convenience: the robot runs without it.
   *
   * @param port usually {@link #PORT}
   * @param say prints a line where the team sees it
   * @return the running server, or empty if it couldn't start
   */
  public static Optional<PitwallServer> startOrExplain(
      Status status,
      Tuning<?> tuning,
      Actions actions,
      TunableTee tunables,
      int port,
      Consumer<String> say) {
    return startOrExplain(status, tuning, actions, tunables, port, false, say);
  }

  /**
   * Starts the page, or says why it couldn't.
   *
   * @param thisComputerOnly serve only to this computer (the simulator on a laptop, where the
   *     network may be a school's or an event's), not to the network the robot is on
   */
  public static Optional<PitwallServer> startOrExplain(
      Status status,
      Tuning<?> tuning,
      Actions actions,
      TunableTee tunables,
      int port,
      boolean thisComputerOnly,
      Consumer<String> say) {
    PitwallServer server = null;
    try {
      server = new PitwallServer(status, tuning, actions, tunables, port);
      server.start(thisComputerOnly);
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
        server.close();
      }
      return Optional.empty();
    }
  }

  /** Serves the page to the network the robot is on. */
  public void start() throws IOException {
    start(false);
  }

  /**
   * Serves the page.
   *
   * @param thisComputerOnly serve only to this computer, not to the network
   */
  public void start(boolean thisComputerOnly) throws IOException {
    server.bind(
        thisComputerOnly
            ? new InetSocketAddress(InetAddress.getLoopbackAddress(), port)
            : new InetSocketAddress(port),
        0);
    server.start();
  }

  /** The port it serves on, once started; -1 before. */
  public int port() {
    InetSocketAddress address = server.getAddress();
    return address != null ? address.getPort() : -1;
  }

  @Override
  public void close() {
    try {
      server.stop(0);
    } catch (IllegalStateException neverStarted) {
      // There is nothing to stop.
    }
    threads.shutdownNow();
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
  private static final class Api {
    private static final Map<String, List<String>> METHODS =
        Map.of(
            "/", List.of("GET"),
            "/api/status", List.of("GET"),
            "/api/actions", List.of("GET", "POST"),
            "/api/tuning", List.of("GET", "PATCH"),
            "/api/tunables", List.of("GET"),
            "/api/values", List.of("GET", "PATCH"),
            "/api/changes", List.of("GET", "DELETE"),
            "/api/wpilib-tunables", List.of("GET", "PATCH"));

    private final Status status;
    private final Tuning<?> tuning;
    private final Actions actions;
    private final TunableTee tunables;
    private final String page;
    private final Duration wait;
    private final JsonType<Object> json = Jsonb.builder().build().type(Object.class);

    Api(
        Status status,
        Tuning<?> tuning,
        Actions actions,
        TunableTee tunables,
        String page,
        Duration wait) {
      this.status = status;
      this.tuning = tuning;
      this.actions = actions;
      this.tunables = tunables;
      this.page = page;
      this.wait = wait;
    }

    void handle(HttpExchange exchange) throws IOException {
      try (exchange) {
        String path = exchange.getRequestURI().getPath();
        String method = exchange.getRequestMethod();
        try {
          checkAddressed(exchange, method);
          List<String> allowed = METHODS.get(path);
          if (allowed == null) {
            throw new Problem(404, "Not found", "There is nothing at " + path);
          }
          if (!allowed.contains(method)) {
            exchange.getResponseHeaders().set("Allow", String.join(", ", allowed));
            throw new Problem(
                405, "Method not allowed", path + " allows " + String.join(", ", allowed));
          }
          switch (method + " " + path) {
            case "GET /" -> send(exchange, 200, "text/html", page);
            case "GET /api/status" -> sendJson(exchange, 200, statusItems());
            case "GET /api/actions" -> sendJson(exchange, 200, actionStates());
            case "POST /api/actions" -> postAction(exchange);
            case "GET /api/tuning" -> sendJson(exchange, 200, session());
            case "PATCH /api/tuning" -> patchTuning(exchange);
            case "GET /api/tunables" -> sendJson(exchange, 200, tunables());
            case "GET /api/values" -> sendJson(exchange, 200, values());
            case "PATCH /api/values" -> patchValues(exchange);
            case "GET /api/changes" -> getChanges(exchange);
            case "DELETE /api/changes" -> answer(exchange, tuning.requestReset(), this::changes);
            case "GET /api/wpilib-tunables" -> sendJson(exchange, 200, wpilibTunables());
            case "PATCH /api/wpilib-tunables" -> patchWpilibTunables(exchange);
            default -> throw new IllegalStateException("unrouted: " + method + " " + path);
          }
        } catch (Problem problem) {
          sendProblem(exchange, problem);
        } catch (RuntimeException e) {
          sendProblem(exchange, new Problem(500, "Server error", String.valueOf(e)));
        }
      }
    }

    /**
     * Refuses a request addressed to anything but the robot itself, and a change sent from another
     * site's page. A web page open on a laptop connected to the robot could otherwise point a name
     * it controls at the robot's address (DNS rebinding) and use the page as its own; the address
     * the request names gives it away. The robot is reached by address, as localhost, or by its
     * local name (robot.local).
     */
    private static void checkAddressed(HttpExchange exchange, String method) {
      String host = exchange.getRequestHeaders().getFirst("Host");
      if (host == null || !isRobotsOwn(withoutPort(host))) {
        throw new Problem(
            403, "Wrong address", "Pitwall answers only at the robot's own address or name");
      }
      String origin = exchange.getRequestHeaders().getFirst("Origin");
      if (!method.equals("GET") && origin != null && !origin.equals("http://" + host)) {
        throw new Problem(403, "Another site", "Changes come only from Pitwall's own page");
      }
    }

    private static String withoutPort(String host) {
      if (host.startsWith("[")) {
        int end = host.indexOf(']');
        return end < 0 ? host : host.substring(0, end + 1);
      }
      int colon = host.lastIndexOf(':');
      return colon < 0 ? host : host.substring(0, colon);
    }

    private static boolean isRobotsOwn(String name) {
      String host = name.toLowerCase(Locale.ROOT);
      return host.equals("localhost")
          || host.endsWith(".local")
          || host.startsWith("[") // an IPv6 address
          || host.matches("\\d{1,3}(\\.\\d{1,3}){3}"); // an IPv4 address
    }

    private void postAction(HttpExchange exchange) throws IOException {
      Map<String, @Nullable Object> body = readObject(exchange);
      Object start = body.get("start");
      Object cancel = body.get("cancel");
      Object name = start != null ? start : cancel;
      if (body.size() != 1 || name == null) {
        throw new Problem(400, "Start or cancel", "Send {\"start\": name} or {\"cancel\": name}");
      }
      if (!(name instanceof String action)) {
        throw new Problem(400, "Not a name", "An action's name is a string");
      }
      if (!actions.has(action)) {
        throw new Problem(422, "Not an action", "There is no action named " + action);
      }
      answer(
          exchange,
          start != null ? actions.requestStart(action) : actions.requestCancel(action),
          this::actionStates);
    }

    private void patchTuning(HttpExchange exchange) throws IOException {
      Map<String, @Nullable Object> patch = readObject(exchange);
      for (String name : patch.keySet()) {
        if (!name.equals("enabled")) {
          throw new Problem(422, "Not changeable", "Only enabled can be changed, not " + name);
        }
      }
      if (!patch.containsKey("enabled")) {
        sendJson(exchange, 200, session()); // an empty patch changes nothing
        return;
      }
      if (!(patch.get("enabled") instanceof Boolean enabled)) {
        throw new Problem(400, "Not true or false", "enabled must be true or false");
      }
      answer(exchange, tuning.requestEnabled(enabled), this::session);
    }

    private void patchValues(HttpExchange exchange) throws IOException {
      Map<String, @Nullable Object> patch = readObject(exchange);
      Map<String, Tuning.Value> known = new LinkedHashMap<>();
      for (Tuning.Value value : tuning.view().values()) {
        known.put(value.path(), value);
      }
      Map<String, Double> values = new LinkedHashMap<>();
      for (Map.Entry<String, @Nullable Object> field : patch.entrySet()) {
        Tuning.Value value = known.get(field.getKey());
        if (value == null) {
          throw new Problem(422, "Not tunable", "There is no tunable value at " + field.getKey());
        }
        Object number = field.getValue();
        if (number == null) {
          values.put(value.path(), value.committed());
        } else if (number instanceof Number n && Double.isFinite(n.doubleValue())) {
          values.put(value.path(), n.doubleValue());
        } else {
          throw new Problem(
              400,
              "Not a number",
              value.path() + " must be a number, or null to return it to its committed value");
        }
      }
      answer(exchange, tuning.requestChanges(values), this::values);
    }

    private void patchWpilibTunables(HttpExchange exchange) throws IOException {
      Map<String, @Nullable Object> patch = readObject(exchange);
      Map<String, Object> values = new LinkedHashMap<>();
      for (Map.Entry<String, @Nullable Object> field : patch.entrySet()) {
        if (!tunables.isEditable(field.getKey())) {
          throw new Problem(
              422,
              "Not changeable",
              "There is no tunable the page can change at " + field.getKey());
        }
        Object value = field.getValue();
        boolean number = value instanceof Number n && Double.isFinite(n.doubleValue());
        if (!(number || value instanceof Boolean)) {
          throw new Problem(400, "Not a value", field.getKey() + " takes a number or true/false");
        }
        values.put(field.getKey(), value);
      }
      answer(exchange, tunables.requestChanges(values), this::wpilibTunables);
    }

    private void getChanges(HttpExchange exchange) throws IOException {
      String accept = exchange.getRequestHeaders().getFirst("Accept");
      if (accept != null && accept.contains("text/plain") && !accept.contains("json")) {
        List<String> report = tuning.report();
        send(exchange, 200, "text/plain", String.join("\n", report) + "\n");
      } else {
        sendJson(exchange, 200, changes());
      }
    }

    /** Waits for the robot loop, then answers with what became of the request. */
    private void answer(
        HttpExchange exchange, CompletableFuture<Outcome> pending, Supplier<Object> state)
        throws IOException {
      Outcome outcome;
      try {
        outcome = pending.get(wait.toMillis(), TimeUnit.MILLISECONDS);
      } catch (TimeoutException e) {
        sendJson(
            exchange,
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
        case APPLIED -> sendJson(exchange, 200, state.get());
        case REFUSED -> throw new Problem(422, "Refused", outcome.detail());
        case LOCKED -> throw new Problem(409, "Locked", outcome.detail());
      }
    }

    /** The request's body as a JSON object, or the problem with it. */
    private Map<String, @Nullable Object> readObject(HttpExchange exchange) throws IOException {
      String type = exchange.getRequestHeaders().getFirst("Content-Type");
      if (type == null
          || !(type.startsWith("application/merge-patch+json")
              || type.startsWith("application/json"))) {
        throw new Problem(
            415,
            "Unsupported media type",
            "Send a JSON merge patch, with Content-Type: application/merge-patch+json");
      }
      byte[] body;
      try (InputStream in = exchange.getRequestBody()) {
        body = in.readNBytes(MAX_BODY_BYTES + 1);
      }
      if (body.length > MAX_BODY_BYTES) {
        throw new Problem(413, "Too large", "The page's requests are well under 64 KB");
      }
      if (nesting(body) > MAX_NESTING) {
        // Checked before parsing: the parser recurses, and a deep enough body overflows its stack.
        throw new Problem(400, "Too deeply nested", "The page's requests are flat JSON objects");
      }
      Object parsed;
      try {
        parsed = json.fromJson(new String(body, StandardCharsets.UTF_8));
      } catch (JsonException e) {
        throw new Problem(400, "Not JSON", String.valueOf(e.getMessage()));
      }
      if (!(parsed instanceof Map<?, ?> map)) {
        throw new Problem(400, "Not an object", "A merge patch is a JSON object");
      }
      Map<String, @Nullable Object> object = new LinkedHashMap<>();
      map.forEach((key, value) -> object.put(String.valueOf(key), value));
      return object;
    }

    /** How deeply a JSON body's objects and arrays nest, not counting brackets inside strings. */
    private static int nesting(byte[] body) {
      int depth = 0;
      int deepest = 0;
      boolean inString = false;
      boolean escaped = false;
      for (byte b : body) {
        if (inString) {
          if (escaped) {
            escaped = false;
          } else if (b == '\\') {
            escaped = true;
          } else if (b == '"') {
            inString = false;
          }
        } else if (b == '"') {
          inString = true;
        } else if (b == '{' || b == '[') {
          deepest = Math.max(deepest, ++depth);
        } else if (b == '}' || b == ']') {
          depth--;
        }
      }
      return deepest;
    }

    private List<Object> statusItems() {
      List<Object> items = new ArrayList<>();
      for (Status.Item item : status.view()) {
        Map<String, Object> shown = new LinkedHashMap<>();
        shown.put("name", item.name());
        shown.put("value", item.value());
        items.add(shown);
      }
      return items;
    }

    private List<Object> actionStates() {
      List<Object> states = new ArrayList<>();
      for (Actions.State state : actions.view()) {
        Map<String, Object> shown = new LinkedHashMap<>();
        shown.put("name", state.name());
        shown.put("phase", state.phase().name());
        shown.put("detail", state.detail());
        states.add(shown);
      }
      return states;
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

    private List<Object> wpilibTunables() {
      List<Object> listed = new ArrayList<>();
      for (TunableTee.Listed tunable : tunables.view()) {
        Map<String, Object> shown = new LinkedHashMap<>();
        shown.put("path", tunable.path());
        shown.put("type", tunable.type());
        shown.put("value", tunable.value());
        shown.put("editable", tunable.editable());
        listed.add(shown);
      }
      return listed;
    }

    private List<Object> changes() {
      List<Object> changes = new ArrayList<>();
      for (ConfigTuner.Change change : tuning.view().changes()) {
        Map<String, Object> shown = new LinkedHashMap<>();
        shown.put("path", change.path());
        shown.put("committed", change.committed());
        shown.put("tuned", change.tuned());
        shown.put("builderCall", change.builderCall());
        changes.add(shown);
      }
      return changes;
    }

    private void sendJson(HttpExchange exchange, int status, Object body) throws IOException {
      send(exchange, status, "application/json", json.toJson(body));
    }

    private void sendProblem(HttpExchange exchange, Problem problem) throws IOException {
      Map<String, Object> body = new LinkedHashMap<>();
      body.put("type", "about:blank");
      body.put("title", problem.title);
      body.put("status", problem.status);
      body.put("detail", String.valueOf(problem.getMessage()));
      send(exchange, problem.status, "application/problem+json", json.toJson(body));
    }

    private static void send(HttpExchange exchange, int status, String type, String body)
        throws IOException {
      byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
      exchange.getResponseHeaders().set("Content-Type", type + "; charset=utf-8");
      exchange.getResponseHeaders().set("Cache-Control", "no-store");
      // The page may not be framed by another site's page (to trick a click on its buttons), and
      // loads nothing from anywhere else.
      exchange.getResponseHeaders().set("X-Frame-Options", "DENY");
      exchange
          .getResponseHeaders()
          .set(
              "Content-Security-Policy",
              "default-src 'self'; script-src 'self' 'unsafe-inline';"
                  + " style-src 'self' 'unsafe-inline'; frame-ancestors 'none'");
      exchange.getResponseHeaders().set("X-Content-Type-Options", "nosniff");
      exchange.getResponseHeaders().set("Referrer-Policy", "no-referrer");
      exchange.sendResponseHeaders(status, bytes.length > 0 ? bytes.length : -1);
      if (bytes.length > 0) {
        try (OutputStream out = exchange.getResponseBody()) {
          out.write(bytes);
        }
      }
    }
  }
}
