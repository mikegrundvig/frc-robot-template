package com.michaelgrundvig.frc.pitwall;

import com.michaelgrundvig.frc.pitwall.experiment.Experiments;
import com.michaelgrundvig.frc.pitwall.experiment.Run;
import com.michaelgrundvig.frc.pitwall.push.Broadcaster;
import com.michaelgrundvig.frc.pitwall.push.Event;
import com.michaelgrundvig.frc.pitwall.push.Stream;
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
 * ({@link Actions}), live tuning of the mechanisms' configurations ({@link Tuning}, {@link
 * LiveConfig}), every WPILib tunable in the program ({@link TunableTee}), and experiments ({@link
 * com.michaelgrundvig.frc.pitwall.experiment.Experiments}):
 *
 * <ul>
 *   <li>{@code GET /api/events}: the page's state as server-sent events, pushed only when something
 *       changes: a {@code snapshot} first, then {@code patch} events (JSON merge patches) and the
 *       run's {@code samples}; a comment line after 5 s with nothing to say. See {@link
 *       Broadcaster}.
 *   <li>{@code GET /api/status}: the status items, read by the next loop, {@code [{"name",
 *       "value"}]}.
 *   <li>{@code GET /api/actions}: every action and where it stands, {@code [{"name", "phase",
 *       "detail"}]}. {@code POST} it {@code {"start": name}} or {@code {"cancel": name}}.
 *   <li>{@code GET /api/tuning}: the session, {@code {"name", "enabled", "fieldConnected",
 *       "refused"}}, the last being the latest dashboard changes refused and why (a dashboard can't
 *       say). {@code PATCH} it with {@code {"enabled": true}} to switch tuning on or off.
 *   <li>{@code GET /api/tunables}: the configurations' tunable values, by the WPILib tunable each
 *       is published as, {@code {path: {"config", "field", "unit", "limits", "committed"}}}.
 *   <li>{@code GET /api/values}: their current values, {@code {path: number}}. {@code PATCH} it
 *       with a JSON merge patch (RFC 7396) to change several at once, all or none, each checked by
 *       its configuration first; {@code null} returns a value to its committed one.
 *   <li>{@code GET /api/changes}: what differs from the committed configurations, with the builder
 *       calls that commit it, {@code [{"config", "path", "committed", "tuned", "builderCall"}]}
 *       ({@code Accept: text/plain} for the report). {@code DELETE} it to return every value to its
 *       committed one.
 *   <li>{@code GET /api/wpilib-tunables}: every other WPILib tunable, {@code [{"path", "type",
 *       "value", "editable", "limits"}]}, the limits as text ({@code "0 to 30"}, empty for none).
 *       {@code PATCH} it with {@code {path: value}} to change several at once, all or none, each
 *       within its limits.
 *   <li>{@code GET /api/experiments}: the rigs and the experiments that run on them. {@code POST}
 *       it {@code {"start": name}} or {@code {"cancel": name}}.
 *   <li>{@code GET /api/run}: the run in progress, or the latest to end, with its trace.
 *   <li>{@code POST /api/heartbeat}: nothing, but it keeps a run going: while an experiment runs,
 *       the page that started it sends a request at least every 500 ms (the deadman), naming its
 *       event stream in the {@code Pitwall-Stream} header, as it does with every request.
 * </ul>
 *
 * <p>A change waits for the robot loop and answers with what became of it: 200 and the new state
 * when applied, 422 when a limit or the configuration's checks (or the action) refuse it, 409 when
 * it is locked out (tuning off, not enabled in utility mode, or the field system connected), and
 * 202 when the loop didn't get to it in time (the next GET shows the result). Errors are problem
 * details (RFC 9457).
 *
 * <p>The server's threads never change the robot: they read what the robot loop last published
 * ({@link Status#view()}, {@link Actions#view()}, {@link Tuning#view()}) and queue requests for the
 * robot loop to apply. The event stream is written by its own thread from what the loop hands it.
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

  /** How long a stream stays silent before a comment line says it's still there. */
  static final Duration IDLE = Duration.ofSeconds(5);

  /** The header a page names its event stream in, so the deadman knows who's asking. */
  static final String STREAM = "Pitwall-Stream";

  /** The page's requests are a few hundred bytes; anything much bigger isn't from it. */
  private static final int MAX_BODY_BYTES = 64 * 1024;

  /** How deeply a request's JSON may nest: the page sends flat objects. */
  private static final int MAX_NESTING = 8;

  private final HttpServer server;
  private final ExecutorService threads;
  private final int port;

  /**
   * @param page what the page shows and changes
   * @param broadcaster pushes the page's state to each event stream
   * @param port the port to serve on; 0 picks a free one
   */
  public PitwallServer(Page page, Broadcaster broadcaster, int port) {
    this(page, broadcaster, port, WAIT, IDLE);
  }

  PitwallServer(Page page, Broadcaster broadcaster, int port, Duration wait, Duration idle) {
    this.port = port;
    // Small events go out at once, not held back to fill a packet (Nagle's algorithm). The server
    // reads this as it starts, so it's set first; a program that set it already keeps its choice.
    if (System.getProperty("sun.net.httpserver.nodelay") == null) {
      System.setProperty("sun.net.httpserver.nodelay", "true");
    }
    try {
      server = HttpServer.create();
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    // A virtual thread for each request: a stream sleeps on its mailbox between events, so a slow
    // or vanished page stalls only its own thread. They never keep the program running.
    threads = Executors.newVirtualThreadPerTaskExecutor();
    server.setExecutor(threads);
    Api api = new Api(page, broadcaster, page(), wait, idle);
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
      Page page, Broadcaster broadcaster, int port, Consumer<String> say) {
    return startOrExplain(page, broadcaster, port, false, say);
  }

  /**
   * Starts the page, or says why it couldn't.
   *
   * @param thisComputerOnly serve only to this computer (the simulator on a laptop, where the
   *     network may be a school's or an event's), not to the network the robot is on
   */
  public static Optional<PitwallServer> startOrExplain(
      Page page,
      Broadcaster broadcaster,
      int port,
      boolean thisComputerOnly,
      Consumer<String> say) {
    PitwallServer server = null;
    try {
      server = new PitwallServer(page, broadcaster, port);
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
        Map.ofEntries(
            Map.entry("/", List.of("GET")),
            Map.entry("/api/events", List.of("GET")),
            Map.entry("/api/status", List.of("GET")),
            Map.entry("/api/actions", List.of("GET", "POST")),
            Map.entry("/api/tuning", List.of("GET", "PATCH")),
            Map.entry("/api/tunables", List.of("GET")),
            Map.entry("/api/values", List.of("GET", "PATCH")),
            Map.entry("/api/changes", List.of("GET", "DELETE")),
            Map.entry("/api/wpilib-tunables", List.of("GET", "PATCH")),
            Map.entry("/api/experiments", List.of("GET", "POST")),
            Map.entry("/api/run", List.of("GET")),
            Map.entry("/api/heartbeat", List.of("POST")));

    private final Page state;
    private final Status status;
    private final Tuning tuning;
    private final Actions actions;
    private final TunableTee tunables;
    private final Experiments experiments;
    private final Broadcaster broadcaster;
    private final String page;
    private final Duration wait;
    private final Duration idle;
    private final JsonType<Object> json = Jsonb.builder().build().type(Object.class);

    Api(Page state, Broadcaster broadcaster, String page, Duration wait, Duration idle) {
      this.state = state;
      status = state.status();
      tuning = state.tuning();
      actions = state.actions();
      tunables = state.tunables();
      experiments = state.experiments();
      this.broadcaster = broadcaster;
      this.page = page;
      this.wait = wait;
      this.idle = idle;
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
          // Any request from the page that started an experiment keeps it going.
          experiments.heard(exchange.getRequestHeaders().getFirst(STREAM));
          switch (method + " " + path) {
            case "GET /" -> send(exchange, 200, "text/html", page);
            case "GET /api/events" -> stream(exchange);
            case "GET /api/status" -> sendJson(exchange, 200, statusItems(readStatus()));
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
            case "GET /api/experiments" -> sendJson(exchange, 200, state.experimentsView());
            case "POST /api/experiments" -> postExperiment(exchange);
            case "GET /api/run" -> sendJson(exchange, 200, run(true));
            case "POST /api/heartbeat" -> send(exchange, 204, "text/plain", "");
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

    private void postExperiment(HttpExchange exchange) throws IOException {
      Map<String, @Nullable Object> body = readObject(exchange);
      Object start = body.get("start");
      Object cancel = body.get("cancel");
      Object name = start != null ? start : cancel;
      if (body.size() != 1 || name == null) {
        throw new Problem(400, "Start or cancel", "Send {\"start\": name} or {\"cancel\": name}");
      }
      if (!(name instanceof String experiment)) {
        throw new Problem(400, "Not a name", "An experiment's name is a string");
      }
      if (!experiments.has(experiment)) {
        throw new Problem(422, "Not an experiment", "There is no experiment named " + experiment);
      }
      answer(
          exchange,
          start != null
              ? experiments.requestStart(experiment, exchange.getRequestHeaders().getFirst(STREAM))
              : experiments.requestCancel(experiment),
          () -> run(false));
    }

    /**
     * Streams the page's state until the page goes away: events as the robot loop sends them, and a
     * comment line whenever it has been quiet for {@link #IDLE}. A write that fails means the page
     * has gone, and ends the stream.
     */
    private void stream(HttpExchange exchange) throws IOException {
      Stream stream =
          broadcaster
              .open()
              .orElseThrow(
                  () ->
                      new Problem(
                          503,
                          "Too many pages",
                          "At most "
                              + Broadcaster.MAX_STREAMS
                              + " pages can follow the robot at once; close one"));
      String why = "the page went away";
      try {
        headers(exchange, "text/event-stream");
        exchange.sendResponseHeaders(200, 0);
        OutputStream out = exchange.getResponseBody();
        while (true) {
          Event event = stream.next(idle);
          if (!stream.isOpen()) {
            break;
          }
          out.write((event == null ? ":\n" : event.wire()).getBytes(StandardCharsets.UTF_8));
          out.flush();
        }
      } catch (IOException gone) {
        // The page closed, or its connection did: nothing more to send.
      } catch (InterruptedException stopped) {
        Thread.currentThread().interrupt();
        why = "Pitwall stopped";
      } finally {
        stream.close(why);
      }
    }

    /** The status items, read by the next loop; as last read if the loop doesn't get to it. */
    private List<Status.Item> readStatus() {
      try {
        return status.requestRead().get(wait.toMillis(), TimeUnit.MILLISECONDS);
      } catch (TimeoutException | ExecutionException e) {
        return status.view();
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return status.view();
      }
    }

    /** The run in progress, or the latest to end; with its trace so far, if asked. */
    private Map<String, Object> run(boolean withTrace) {
      Run run = experiments.run();
      if (run == null) {
        throw new Problem(404, "No run", "No experiment has run yet");
      }
      Map<String, Object> shown = new LinkedHashMap<>(state.run(run));
      if (withTrace) {
        shown.put("trace", Broadcaster.trace(run));
      }
      return shown;
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
      for (Tuning.Value value : configValues()) {
        known.put(value.path(), value);
      }
      Map<String, Object> values = new LinkedHashMap<>();
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
      // A configuration's values are WPILib tunables: the tee checks them whole, and applies them.
      answer(exchange, tunables.requestChanges(values), this::values);
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

    private static List<Object> statusItems(List<Status.Item> read) {
      List<Object> items = new ArrayList<>();
      for (Status.Item item : read) {
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
      return state.session();
    }

    private Map<String, Object> tunables() {
      return Page.tunables(tuning.view());
    }

    private Map<String, Object> values() {
      return Page.values(tuning.view());
    }

    private List<Tuning.Value> configValues() {
      return tuning.view().configs().stream().flatMap(config -> config.values().stream()).toList();
    }

    private List<Object> wpilibTunables() {
      List<Object> listed = new ArrayList<>();
      for (TunableTee.Listed tunable : tunables.view()) {
        Map<String, Object> shown = new LinkedHashMap<>();
        shown.put("path", tunable.path());
        shown.put("type", tunable.type());
        shown.put("value", tunable.value());
        shown.put("editable", tunable.editable());
        shown.put(
            "limits", tunable.limits().equals(Limits.NONE) ? "" : tunable.limits().toString());
        listed.add(shown);
      }
      return listed;
    }

    private List<Object> changes() {
      List<Object> changes = new ArrayList<>();
      for (Tuning.ConfigView config : tuning.view().configs()) {
        for (LiveConfig.Change change : config.changes()) {
          Map<String, Object> shown = new LinkedHashMap<>();
          shown.put("config", config.name());
          shown.put("path", change.path());
          shown.put("committed", change.committed());
          shown.put("tuned", change.tuned());
          shown.put("builderCall", change.builderCall());
          changes.add(shown);
        }
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
      headers(exchange, type);
      exchange.sendResponseHeaders(status, bytes.length > 0 ? bytes.length : -1);
      if (bytes.length > 0) {
        try (OutputStream out = exchange.getResponseBody()) {
          out.write(bytes);
        }
      }
    }

    private static void headers(HttpExchange exchange, String type) {
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
    }
  }
}
