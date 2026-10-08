package com.michaelgrundvig.frc.pitwall;

import com.michaelgrundvig.frc.pitwall.experiment.Experiments;
import com.michaelgrundvig.frc.pitwall.experiment.Run;
import com.michaelgrundvig.frc.pitwall.push.Broadcaster;
import com.michaelgrundvig.frc.pitwall.push.Event;
import com.michaelgrundvig.frc.pitwall.push.Stream;
import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.DriverStationState;
import com.michaelgrundvig.frc.pitwall.simulator.JoystickInput;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import com.michaelgrundvig.frc.pitwall.simulator.Simulator;
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
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.jspecify.annotations.Nullable;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;

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
 *       run's {@code samples}; a {@code heartbeat} after 5 s with nothing else to say. A page
 *       reconnecting names the stream it had ({@code ?replaces=id}), which closes; with eight open,
 *       the one that wrote least lately makes way. A page names the views it shows ({@code
 *       ?views=world}), and only then is sent their part of the state (a scene's moving parts),
 *       starting with a {@code replace} event holding all of it. See {@link Broadcaster}.
 *   <li>{@code PUT /api/views}: {@code {"views": ["world"]}}, the views the page shows now, naming
 *       its stream; 204.
 *   <li>{@code GET /api/scenes/<id>/inspect/<shape>}: what a click on a scene's shape says, {@code
 *       {"scene", "shape", "title", "details": [{"label", "value"}], "actions"}}, the actions as
 *       {@code /api/actions} gives them; 404 when nothing is known of it now. See {@link
 *       com.michaelgrundvig.frc.pitwall.scene.Scenes}.
 *   <li>{@code GET /api/status}: the status items, read by the next loop, {@code [{"name",
 *       "value"}]}; 503 when the loop doesn't get to it within a second.
 *   <li>{@code GET /api/actions}: every action and where it stands, {@code [{"name", "phase",
 *       "detail", "locked", "when"}]}: why it can't start now (empty while it can), and when it may
 *       run ({@code ENABLED_IN_UTILITY} or {@code DISABLED}). {@code POST} it {@code {"start":
 *       name}} or {@code {"cancel": name}}.
 *   <li>{@code GET /api/tuning}: the session, {@code {"name", "enabled", "fieldConnected",
 *       "refused"}}, the last being the latest dashboard changes refused and why (a dashboard can't
 *       say). {@code PATCH} it with {@code {"enabled": true}} to switch tuning on or off.
 *   <li>{@code GET /api/configs}: each configuration's documentation, from its records' Javadoc
 *       ({@link LiveConfig.Documentation}), {@code {name: {"record", "doc": {"summary", "text"},
 *       "groups": [{"path", "record", "doc", "meaning"}]}}}: its root record, then each record its
 *       values are in, with what the record holding it says it is. A {@code record} is a display
 *       name, such as {@code DrivetrainControl.Gains}.
 *   <li>{@code GET /api/tunables}: the configurations' tunable values, by the WPILib tunable each
 *       is published as, {@code {path: {"config", "field", "unit", "limits", "min", "max",
 *       "committed", "record", "group", "meaning"}}}: its tuning limits as text and their ends as
 *       numbers (an open end has none), and the record that declares the value, its path, and what
 *       its {@code @param} says the value means (empty when there's none).
 *   <li>{@code GET /api/values}: their current values, {@code {path: number}}. {@code PATCH} it
 *       with a JSON merge patch (RFC 7396) to change several at once, all or none, each checked by
 *       its configuration first; {@code null} returns a value to its committed one.
 *   <li>{@code GET /api/changes}: what differs from the committed configurations, with the builder
 *       calls that commit it, {@code [{"config", "field", "committed", "tuned", "builderCall"}]}
 *       ({@code Accept: text/plain} for the report). {@code DELETE} it to return every value to its
 *       committed one.
 *   <li>{@code GET /api/wpilib-tunables}: every other WPILib tunable, {@code [{"path", "type",
 *       "value", "editable", "limits", "min", "max"}]}, the limits as text ({@code "0 to 30"},
 *       empty for none) and their ends as numbers. {@code PATCH} it with {@code {path: value}} to
 *       change several at once, all or none, each within its limits.
 *   <li>{@code GET /api/experiments}: the rigs and the experiments that run on them. {@code POST}
 *       it {@code {"start": name}}, naming the page's stream, or {@code {"cancel": name, "run": id,
 *       "reason": why}}; canceling what isn't running is 409.
 *   <li>{@code GET /api/run}: the run in progress, or the latest to end, with its trace.
 *   <li>{@code POST /api/heartbeat}: nothing, but it keeps a run going: while an experiment runs,
 *       the page that started it sends a request at least every 500 ms (the deadman), naming its
 *       event stream in the {@code Pitwall-Stream} header, as it does with every request.
 *   <li>{@code GET /api/devices}: each device's card as of its last read, as the event stream's
 *       {@code devices} section carries them ({@link Devices}).
 *   <li>{@code GET /api/devices/<name>/downloads/<id>}: a file from a device, fetched now, as an
 *       attachment; 502 when the device doesn't give it, 503 (with {@code Retry-After}) when it's
 *       busy.
 *   <li>{@code POST /api/devices/<name>/uploads/<id>}: a file for a device, the body as it is (64
 *       KB at most), and {@code POST /api/devices/<name>/choices/<id>}: {@code {"value": option}},
 *       a choice made on its card; each {@code {"message"}}, what the robot did with it, or 422
 *       with why it refused.
 * </ul>
 *
 * <p>The device routes ask the device on the server's thread, never the robot loop's; the page
 * never talks to a device itself. Each names the page's event stream ({@code Pitwall-Stream}), as
 * only the page can (400 without), so another site's page can't make the robot ask a device
 * anything; at most two downloads are under way at once (503 beyond); and a download, or a change
 * to a device, waits until the field system disconnects (409). A file is streamed through as it
 * arrives, up to 64 MB.
 *
 * <p>In the simulator only, the simulator's controls ({@link Simulator}); on a robot every one of
 * these is 404, so nothing can write the Driver Station's state there. They are exempt from the
 * field lock: in the simulator, they are the field. See docs/simulator.md for the details.
 *
 * <ul>
 *   <li>{@code GET /api/simulator}: {@code {"simulator", "components", "scenarios"}}, as the event
 *       stream's sections of those names carry them. {@code PATCH} it with any of {@code
 *       "enabled"}, {@code "mode"} ({@code "AUTONOMOUS"}, {@code "TELEOPERATED"}, {@code
 *       "UTILITY"}), {@code "allianceStation"} ({@code "RED_1"} to {@code "BLUE_3"}, or {@code
 *       "UNKNOWN"}), {@code "fmsAttached"}, {@code "dsAttached"}, {@code "matchTime"} (seconds, or
 *       null for none), and {@code "paused"}.
 *   <li>{@code POST /api/simulator/step}: one loop of a paused simulator; 409 while it runs.
 *   <li>{@code POST /api/simulator/stop}: ends the program, as closing the GUI's window did.
 *   <li>{@code PUT /api/simulator/joysticks}: {@code {"0": {"axes": [...], "buttons": [...],
 *       "povs": [...]}, "1": null}}, each joystick's whole input (null lets it go), at most once a
 *       frame and again every 200 ms while anything is pressed, naming the page's event stream; 204
 *       at once.
 *   <li>{@code POST /api/simulator/faults}: {@code {"component", "fault", "values": {name:
 *       number}}} applies a fault, {@code {"component", "fault", "undo": true}} undoes a lasting
 *       one. {@code DELETE} it to undo every lasting fault.
 *   <li>{@code POST /api/simulator/scenarios}: {@code {"apply": name}} applies a scenario's faults.
 * </ul>
 *
 * <p>A change waits for the robot loop and answers with what became of it: 200 and the new state
 * when applied, 422 when a limit or the configuration's checks (or the action) refuse it, 409 when
 * it is locked out (tuning off, not enabled in utility mode, or the field system connected), and
 * 202 when the loop didn't get to it in time (the next GET shows the result). Errors are problem
 * details (RFC 9457). A refused change to tunables also lists each reason in {@code errors}, tied
 * to the tunables it concerns ({@code [{"layer", "by", "path", "paths", "config", "group", "text",
 * "line"}]}), while {@code detail} keeps every reason as a line, as it always has.
 *
 * <p>The server's threads never change the robot: they read what the robot loop last published
 * ({@link Status#view()}, {@link Actions#view()}, {@link Tuning#view()}) and queue requests for the
 * robot loop to apply. The event stream is written by its own thread from what the loop hands it.
 *
 * <p>The page itself is {@code index.html} and the files in {@code page/} beside it ({@code
 * /page/tune.js}, ...): a script module for its core and one for each view, plain JavaScript served
 * as written, with nothing to build.
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
   * to {@link #startOrExplain}, or name it in the {@code pitwall.port} system property, which
   * {@link Pitwall#start(String)} reads.
   */
  public static final int PORT = 5820;

  /** How long a change waits for the robot loop before answering 202 Accepted. */
  static final Duration WAIT = Duration.ofSeconds(1);

  /**
   * How long a stream stays silent before a heartbeat says it's still there. The heartbeat is an
   * event, not a comment line: a browser's EventSource never shows the page a comment, and the page
   * counts 10 s without an event as the link lost.
   */
  static final Duration IDLE = Duration.ofSeconds(5);

  /** What a quiet stream sends: an event the page sees, holding nothing. */
  private static final String HEARTBEAT = "event: heartbeat\ndata: {}\n\n";

  /**
   * Third-party files the page loads from here, as the robot has no internet at an event: uPlot,
   * which draws the experiments' plots, and three.js, which draws the scenes (the World view loads
   * it only when it's first shown). Each is served if it's bundled; three.js is taken out of its
   * WebJar as Pitwall builds (pitwall/build.gradle).
   */
  private static final Map<String, String> BUNDLED =
      Map.of(
          "/uPlot.iife.min.js", "text/javascript",
          "/uPlot.min.css", "text/css",
          "/three/three.module.min.js", "text/javascript",
          "/three/three.core.min.js", "text/javascript",
          "/three/OrbitControls.js", "text/javascript");

  /**
   * The page's own files besides index.html: a script module for its core and one for each view,
   * and its styles, in {@code page/} beside it. A short plain name and nothing else, so no request
   * can reach any other resource.
   */
  private static final Pattern PAGE_FILE = Pattern.compile("/page/[a-z][a-z0-9-]{0,31}\\.(js|css)");

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
    this(page, broadcaster, port, wait, idle, DeviceLimits.DEFAULT);
  }

  PitwallServer(
      Page page,
      Broadcaster broadcaster,
      int port,
      Duration wait,
      Duration idle,
      DeviceLimits deviceLimits) {
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
    Api api = new Api(page, broadcaster, page(), bundled(), wait, idle, deviceLimits);
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

  /** The bundled files that are here, by path. */
  private static Map<String, byte[]> bundled() {
    Map<String, byte[]> found = new LinkedHashMap<>();
    for (String path : BUNDLED.keySet()) {
      try (InputStream in = PitwallServer.class.getResourceAsStream(path.substring(1))) {
        if (in != null) {
          found.put(path, in.readAllBytes());
        }
      } catch (IOException e) {
        throw new UncheckedIOException(e);
      }
    }
    return found;
  }

  /** One of the page's files by name, such as {@code tune.js}; empty if there's none. */
  private static Optional<byte[]> pageFile(String name) {
    try (InputStream in = PitwallServer.class.getResourceAsStream("page/" + name)) {
      return in == null ? Optional.empty() : Optional.of(in.readAllBytes());
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
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

    /** For a refused change to tunables, each reason with the values it concerns. */
    private final transient List<Outcome.Reason> reasons;

    /** For a busy device, when to ask again, in seconds: 0 for nothing to say. */
    private int retryAfterSeconds;

    Problem(int status, String title, String detail) {
      this(status, title, detail, List.of());
    }

    /** This problem, saying when to ask again. */
    Problem retryAfter(int seconds) {
      retryAfterSeconds = seconds;
      return this;
    }

    Problem(int status, String title, String detail, List<Outcome.Reason> reasons) {
      super(detail);
      this.status = status;
      this.title = title;
      this.reasons = reasons;
    }
  }

  /** The simulator's routes, which a robot doesn't have. */
  private static final String SIMULATOR = "/api/simulator";

  /**
   * A device's routes: a download by its id (GET), and a file sent to it or a choice made on its
   * card (POST). Names as {@link Devices#NAME} allows them, so no request can name anything else.
   */
  private static final Pattern DEVICE_ROUTE =
      Pattern.compile(
          "/api/devices/("
              + Devices.NAME.pattern()
              + ")/(downloads/("
              + Devices.NAME.pattern()
              + ")|uploads/("
              + Devices.NAME.pattern()
              + ")|choices/("
              + Devices.NAME.pattern()
              + "))");

  /** A scene's names, as {@link com.michaelgrundvig.frc.pitwall.scene.Scene} allows them. */
  private static final String SCENE_NAMES = "[A-Za-z0-9][A-Za-z0-9._-]{0,63}";

  /** What a view is named: a scene's id. */
  private static final Pattern SCENE_NAME = Pattern.compile(SCENE_NAMES);

  /** A click on a scene's shape: the scene's id, then the shape's. */
  private static final Pattern INSPECT_ROUTE =
      Pattern.compile("/api/scenes/(" + SCENE_NAMES + ")/inspect/(" + SCENE_NAMES + ")");

  /** The most views a page may say it shows at once. */
  private static final int MAX_VIEWS = 8;

  /**
   * What asking devices may cost: how many downloads at once (more are 503, never queued), and the
   * largest file passed on.
   *
   * @param concurrent downloads from devices at once, across every page
   * @param maxFileBytes the largest file passed on from a device
   */
  record DeviceLimits(int concurrent, long maxFileBytes) {
    /** Two at once, 64 MB. */
    static final DeviceLimits DEFAULT = new DeviceLimits(2, 64L * 1024 * 1024);
  }

  /** The page, and the resources of the API. */
  private static final class Api {
    /** The modes the page may choose: a Driver Station's, not "no mode". */
    private static final Set<RobotMode> MODES =
        Set.of(RobotMode.AUTONOMOUS, RobotMode.TELEOPERATED, RobotMode.UTILITY);

    /**
     * The longest match time the page may set: ten minutes, room for any match period (a match is 2
     * min 30 s) and a long practice run.
     */
    private static final double MAX_MATCH_SECONDS = 600;

    private static final Map<String, List<String>> METHODS =
        Map.ofEntries(
            Map.entry("/", List.of("GET")),
            Map.entry("/api/events", List.of("GET")),
            Map.entry("/api/status", List.of("GET")),
            Map.entry("/api/actions", List.of("GET", "POST")),
            Map.entry("/api/tuning", List.of("GET", "PATCH")),
            Map.entry("/api/configs", List.of("GET")),
            Map.entry("/api/tunables", List.of("GET")),
            Map.entry("/api/values", List.of("GET", "PATCH")),
            Map.entry("/api/changes", List.of("GET", "DELETE")),
            Map.entry("/api/wpilib-tunables", List.of("GET", "PATCH")),
            Map.entry("/api/experiments", List.of("GET", "POST")),
            Map.entry("/api/run", List.of("GET")),
            Map.entry("/api/heartbeat", List.of("POST")),
            Map.entry("/api/devices", List.of("GET")),
            Map.entry("/api/views", List.of("PUT")),
            Map.entry("/api/simulator", List.of("GET", "PATCH")),
            Map.entry("/api/simulator/step", List.of("POST")),
            Map.entry("/api/simulator/stop", List.of("POST")),
            Map.entry("/api/simulator/joysticks", List.of("PUT")),
            Map.entry("/api/simulator/faults", List.of("POST", "DELETE")),
            Map.entry("/api/simulator/scenarios", List.of("POST")),
            Map.entry("/uPlot.iife.min.js", List.of("GET")),
            Map.entry("/uPlot.min.css", List.of("GET")),
            Map.entry("/three/three.module.min.js", List.of("GET")),
            Map.entry("/three/three.core.min.js", List.of("GET")),
            Map.entry("/three/OrbitControls.js", List.of("GET")));

    private final Page state;
    private final Status status;
    private final Tuning tuning;
    private final Actions actions;
    private final TunableTee tunables;
    private final Experiments experiments;
    private final Broadcaster broadcaster;
    private final String page;
    private final Map<String, byte[]> bundled;

    /**
     * The page's files, each read once, as first asked for: they never change while it runs. Only
     * files found are kept, so a stranger asking for names that don't exist fills nothing.
     */
    private final Map<String, byte[]> pageFiles = new ConcurrentHashMap<>();

    private final Duration wait;
    private final Duration idle;
    private final DeviceLimits deviceLimits;

    /** Requests to devices under way: a page can't make the robot ask more at once. */
    private final Semaphore deviceRequests;

    /** This computer's name on the local network, once looked up: see {@link #machineName}. */
    private volatile @Nullable String machineName;

    private final JsonType<Object> json = Jsonb.builder().build().type(Object.class);

    Api(
        Page state,
        Broadcaster broadcaster,
        String page,
        Map<String, byte[]> bundled,
        Duration wait,
        Duration idle,
        DeviceLimits deviceLimits) {
      this.state = state;
      this.deviceLimits = deviceLimits;
      deviceRequests = new Semaphore(deviceLimits.concurrent());
      status = state.status();
      tuning = state.tuning();
      actions = state.actions();
      tunables = state.tunables();
      experiments = state.experiments();
      this.broadcaster = broadcaster;
      this.page = page;
      this.bundled = bundled;
      this.wait = wait;
      this.idle = idle;
    }

    void handle(HttpExchange exchange) throws IOException {
      try (exchange) {
        String path = exchange.getRequestURI().getPath();
        String method = exchange.getRequestMethod();
        try {
          checkAddressed(exchange, path, method);
          if (path.startsWith(SIMULATOR) && state.simulator() == null) {
            throw new Problem(
                404, "Not found", "There is no simulator here: a robot has no simulator controls");
          }
          List<String> allowed = METHODS.get(path);
          Matcher deviceRoute = DEVICE_ROUTE.matcher(path);
          if (allowed == null && deviceRoute.matches()) {
            boolean change = deviceRoute.group(4) != null || deviceRoute.group(5) != null;
            allowed = List.of(change ? "POST" : "GET");
          }
          if (allowed == null
              && (PAGE_FILE.matcher(path).matches() || INSPECT_ROUTE.matcher(path).matches())) {
            allowed = List.of("GET");
          }
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
            case "GET /api/configs" -> sendJson(exchange, 200, configs());
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
            case "GET /api/devices" -> sendJson(exchange, 200, state.devices().view());
            case "PUT /api/views" -> putViews(exchange);
            case "GET /api/simulator" -> sendJson(exchange, 200, simulatorView());
            case "PATCH /api/simulator" -> patchSimulator(exchange);
            case "POST /api/simulator/step" ->
                answer(exchange, simulator().step(), this::simulatorState);
            case "POST /api/simulator/stop" -> stopSimulator(exchange);
            case "PUT /api/simulator/joysticks" -> putJoysticks(exchange);
            case "POST /api/simulator/faults" -> postFault(exchange);
            case "DELETE /api/simulator/faults" ->
                answer(exchange, simulator().requestUndoAll(), this::componentsView);
            case "POST /api/simulator/scenarios" -> postScenario(exchange);
            case "GET /uPlot.iife.min.js",
                "GET /uPlot.min.css",
                "GET /three/three.module.min.js",
                "GET /three/three.core.min.js",
                "GET /three/OrbitControls.js" ->
                sendBundled(exchange, path);
            default -> {
              Matcher device = DEVICE_ROUTE.matcher(path);
              Matcher inspect = INSPECT_ROUTE.matcher(path);
              if (device.matches()) {
                device(exchange, device);
              } else if (inspect.matches()) {
                inspect(exchange, inspect.group(1), inspect.group(2));
              } else if (PAGE_FILE.matcher(path).matches()) {
                sendPageFile(exchange, path);
              } else {
                throw new IllegalStateException("unrouted: " + method + " " + path);
              }
            }
          }
        } catch (Problem problem) {
          sendProblem(exchange, problem);
        } catch (RuntimeException e) {
          sendProblem(exchange, new Problem(500, "Server error", String.valueOf(e)));
        }
      }
    }

    /**
     * Refuses a request addressed to anything but the robot itself, a change sent from another
     * site's page, and any use of the API by another site's page. A web page open on a laptop
     * connected to the robot could otherwise point a name it controls at the robot's address (DNS
     * rebinding) and use the page as its own; the address the request names gives it away. The
     * robot is reached by address, as localhost, as robot.local, or by this computer's own name on
     * the local network ({@code fedora.local}): any other name, even another {@code .local} one, is
     * 403. A browser says which site a request comes from ({@code Sec-Fetch-Site}), so another
     * site's page can't even read the API, or close a page's event stream by naming it.
     */
    private void checkAddressed(HttpExchange exchange, String path, String method) {
      String host = exchange.getRequestHeaders().getFirst("Host");
      if (host == null || !isRobotsOwn(withoutPort(host))) {
        throw new Problem(
            403, "Wrong address", "Pitwall answers only at the robot's own address or name");
      }
      String origin = exchange.getRequestHeaders().getFirst("Origin");
      if (!method.equals("GET") && origin != null && !origin.equals("http://" + host)) {
        throw new Problem(403, "Another site", "Changes come only from Pitwall's own page");
      }
      String site = exchange.getRequestHeaders().getFirst("Sec-Fetch-Site");
      if (path.startsWith("/api/")
          && site != null
          && !site.equals("same-origin")
          && !site.equals("none")) {
        throw new Problem(403, "Another site", "Pitwall's API answers only Pitwall's own page");
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

    private boolean isRobotsOwn(String name) {
      String host = name.toLowerCase(Locale.ROOT);
      return host.equals("localhost")
          || host.equals("robot.local")
          || isAddress(host)
          || host.equals(machineName());
    }

    /** Whether a host is an IP address, v4 or v6 in brackets, rather than a name. */
    private static boolean isAddress(String host) {
      try {
        InetAddress.ofLiteral(host); // parses it, and never asks DNS
        return true;
      } catch (IllegalArgumentException name) {
        return false;
      }
    }

    /**
     * This computer's own name on the local network, such as {@code fedora.local}: looked up the
     * first time a request names something else, on a server thread, never the robot loop's. Empty
     * when the computer has no name to look up.
     */
    private String machineName() {
      String name = machineName;
      if (name == null) {
        try {
          String host = InetAddress.getLocalHost().getHostName().toLowerCase(Locale.ROOT);
          int dot = host.indexOf('.');
          name = (dot < 0 ? host : host.substring(0, dot)) + ".local";
        } catch (IOException noName) {
          name = "";
        }
        machineName = name;
      }
      return name;
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

    /**
     * Starts or cancels an experiment. A start names the page's stream, which the deadman watches:
     * only a page following the robot can keep a run going. A cancel names the run, so one meant
     * for a run that has just ended never stops the next, and may say why.
     */
    private void postExperiment(HttpExchange exchange) throws IOException {
      Map<String, @Nullable Object> body = readObject(exchange);
      Object start = body.get("start");
      Object cancel = body.get("cancel");
      Object name = start != null ? start : cancel;
      Set<String> allowed = start != null ? Set.of("start") : Set.of("cancel", "run", "reason");
      if (name == null
          || (start != null && cancel != null)
          || !allowed.containsAll(body.keySet())) {
        throw new Problem(
            400,
            "Start or cancel",
            "Send {\"start\": name}, or {\"cancel\": name, \"run\": id, \"reason\": why}");
      }
      if (!(name instanceof String experiment)) {
        throw new Problem(400, "Not a name", "An experiment's name is a string");
      }
      if (!experiments.has(experiment)) {
        throw new Problem(422, "Not an experiment", "There is no experiment named " + experiment);
      }
      if (start != null) {
        String stream = exchange.getRequestHeaders().getFirst(STREAM);
        if (stream == null || stream.isBlank()) {
          throw new Problem(
              400,
              "No page to keep it going",
              "Start experiments from Pitwall's page: a start names the page's event stream ("
                  + STREAM
                  + "), and the run goes on only while that page keeps in touch");
        }
        answer(exchange, experiments.requestStart(experiment, stream), () -> run(false));
        return;
      }
      long runId = 0;
      if (body.get("run") instanceof Number number) {
        runId = number.longValue();
      } else if (body.containsKey("run")) {
        throw new Problem(400, "Not a run", "A run is named by its number");
      }
      Object reason = body.getOrDefault("reason", "");
      if (!(reason instanceof String why) || why.length() > 200) {
        throw new Problem(400, "Not a reason", "A reason is a sentence of text");
      }
      Run running = experiments.run();
      if (running == null
          || running.state().phase() != Run.Phase.RUNNING
          || !running.experiment().name().equals(experiment)
          || (runId != 0 && running.id() != runId)) {
        throw new Problem(409, "Not running", experiment + " isn't running");
      }
      answer(exchange, experiments.requestCancel(experiment, runId, why), () -> run(false));
    }

    /**
     * Says which views a page shows, for the parts of the state sent only to the pages that show
     * them (a scene's moving parts): {@code {"views": ["world"]}}, naming the page's stream in
     * {@code Pitwall-Stream}. 204 at once; the next loop sends what a newly shown view needs.
     */
    private void putViews(HttpExchange exchange) throws IOException {
      Map<String, @Nullable Object> body = readObject(exchange);
      if (body.size() != 1 || !(body.get("views") instanceof List<?> named)) {
        throw new Problem(400, "Views", "Send {\"views\": [view, ...]}");
      }
      Set<String> views = views(named);
      String stream = exchange.getRequestHeaders().getFirst(STREAM);
      if (stream == null || stream.isBlank()) {
        throw new Problem(
            400,
            "No stream",
            "Name the page's event stream (" + STREAM + ") whose views these are");
      }
      if (!broadcaster.show(stream, views)) {
        throw new Problem(404, "No such stream", "No event stream " + stream + " is open");
      }
      send(exchange, 204, "text/plain", "");
    }

    /** Views a page names, each a scene's name, a few at most. */
    private static Set<String> views(List<?> named) {
      if (named.size() > MAX_VIEWS) {
        throw new Problem(400, "Too many views", "A page shows " + MAX_VIEWS + " views at most");
      }
      Set<String> views = new LinkedHashSet<>();
      for (Object view : named) {
        if (!(view instanceof String name) || !SCENE_NAME.matcher(name).matches()) {
          throw new Problem(400, "Not a view", "A view is named by a scene's id: " + view);
        }
        views.add(name);
      }
      return views;
    }

    /**
     * What a click on a scene's shape says, from what the robot loop last published: {@code
     * {"scene", "shape", "title", "details": [{"label", "value"}], "actions": [action]}}, each
     * action as {@code /api/actions} gives it (one that isn't an action is left out). 404 when
     * there's nothing to say: no such scene, or a shape gone for more than two seconds.
     */
    private void inspect(HttpExchange exchange, String scene, String shape) throws IOException {
      if (!state.scenes().has(scene)) {
        throw new Problem(404, "No such scene", "There is no scene " + scene);
      }
      Optional<Inspection> said;
      try {
        said = state.scenes().inspect(scene, shape);
      } catch (RuntimeException e) {
        throw new Problem(500, "Couldn't inspect", shape + ": " + e);
      }
      if (said.isEmpty()) {
        throw new Problem(
            404,
            "Nothing to say",
            "Nothing is known of " + shape + " now: what moves is kept for two seconds");
      }
      Inspection inspection = said.get();
      List<Object> details = new ArrayList<>();
      for (Inspection.Detail detail : inspection.details()) {
        Map<String, Object> shown = new LinkedHashMap<>();
        shown.put("label", detail.label());
        shown.put("value", detail.value());
        details.add(shown);
      }
      List<Object> offered = new ArrayList<>();
      List<Object> all = actionStates();
      for (String name : inspection.actions()) {
        for (Object action : all) {
          if (action instanceof Map<?, ?> each && name.equals(each.get("name"))) {
            offered.add(action);
          }
        }
      }
      Map<String, Object> shown = new LinkedHashMap<>();
      shown.put("scene", scene);
      shown.put("shape", shape);
      shown.put("title", inspection.title());
      shown.put("details", details);
      shown.put("actions", offered);
      sendJson(exchange, 200, shown);
    }

    /**
     * Streams the page's state until the page goes away: events as the robot loop sends them, and a
     * heartbeat whenever it has been quiet for {@link #IDLE}. A write that fails means the page has
     * gone, and ends the stream.
     */
    private void stream(HttpExchange exchange) throws IOException {
      Map<String, String> asked = query(exchange.getRequestURI().getRawQuery());
      String replaces = asked.get("replaces");
      String shown = asked.getOrDefault("views", "");
      Set<String> views = views(shown.isEmpty() ? List.of() : List.of(shown.split(",")));
      Stream stream =
          broadcaster
              .open(replaces, views)
              .orElseThrow(() -> new Problem(503, "Stopping", "Pitwall is stopping"));
      String why = "the page went away";
      try {
        stream.writtenBy(Thread.currentThread());
        headers(exchange, "text/event-stream");
        exchange.sendResponseHeaders(200, 0);
        OutputStream out = exchange.getResponseBody();
        while (true) {
          Event event = stream.next(idle);
          if (!stream.isOpen()) {
            break;
          }
          out.write((event == null ? HEARTBEAT : event.wire()).getBytes(StandardCharsets.UTF_8));
          out.flush();
          stream.wrote();
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

    /**
     * The status items, read by the next loop. A loop that doesn't get to it in time (stopped, or
     * paused in a debugger) is 503: an old reading passed off as now would mislead.
     */
    private List<Status.Item> readStatus() {
      try {
        return status.requestRead().get(wait.toMillis(), TimeUnit.MILLISECONDS);
      } catch (TimeoutException | ExecutionException e) {
        throw new Problem(
            503,
            "No reading",
            "The robot loop didn't read the status within "
                + wait.toMillis()
                + " ms: it may be stopped or paused");
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new Problem(503, "Interrupted", "The server stopped waiting for the robot loop");
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

    /**
     * A device's download, asked of the device now, on this thread: it may wait on the network,
     * never the robot loop. Only the page asks: a request names its event stream, which another
     * site's page can't send. At most {@link DeviceLimits#concurrent} at once, and none while the
     * field system is connected.
     */
    private void device(HttpExchange exchange, Matcher route) throws IOException {
      String stream = exchange.getRequestHeaders().getFirst(STREAM);
      if (stream == null || stream.isBlank()) {
        throw new Problem(
            400,
            "No page",
            "Devices are asked from Pitwall's page: a request names the page's event stream ("
                + STREAM
                + ")");
      }
      String name = route.group(1);
      Devices.Device device =
          state
              .devices()
              .named(name)
              .orElseThrow(() -> new Problem(404, "No device", "There is no device named " + name));
      String uploadId = route.group(4);
      String choiceId = route.group(5);
      if (uploadId != null || choiceId != null) {
        deviceChange(exchange, name, device, uploadId, choiceId);
        return;
      }
      String id = Objects.requireNonNull(route.group(3));
      Devices.Download download =
          device
              .download(id)
              .orElseThrow(
                  () -> new Problem(404, "No download", name + " has no download named " + id));
      if (tuning.isFieldConnected()) {
        throw new Problem(
            409,
            "On the field",
            "Downloads from devices wait until the field system disconnects: the match has the"
                + " network");
      }
      if (!deviceRequests.tryAcquire()) {
        throw new Problem(
            503,
            "Busy",
            "Pitwall is already asking devices for "
                + deviceLimits.concurrent()
                + " things: try again when one is done");
      }
      try {
        sendDeviceFile(exchange, name, download);
      } finally {
        deviceRequests.release();
      }
    }

    /**
     * A file sent to a device, or a choice made on its card: taken by the robot's own handler, on
     * this thread, never the robot loop. Never while the field system is connected.
     */
    private void deviceChange(
        HttpExchange exchange,
        String name,
        Devices.Device device,
        @Nullable String uploadId,
        @Nullable String choiceId)
        throws IOException {
      if (tuning.isFieldConnected()) {
        throw new Problem(
            409,
            "On the field",
            "Changes to devices wait until the field system disconnects: the match has the robot");
      }
      String message;
      try {
        if (uploadId != null) {
          Devices.Upload upload =
              device
                  .upload(uploadId)
                  .orElseThrow(
                      () ->
                          new Problem(
                              404, "No upload", name + " takes no upload named " + uploadId));
          byte[] body;
          try (InputStream in = exchange.getRequestBody()) {
            body = in.readNBytes(MAX_BODY_BYTES + 1);
          }
          if (body.length > MAX_BODY_BYTES) {
            throw new Problem(413, "Too large", "A file sent to a device is at most 64 KB");
          }
          message = upload.accept().accept(body);
        } else {
          String id = Objects.requireNonNull(choiceId);
          Devices.Choose choose =
              device
                  .chooser(id)
                  .orElseThrow(
                      () -> new Problem(404, "No choice", name + " has no choice named " + id));
          Object value = readObject(exchange).get("value");
          if (!(value instanceof String chosen)) {
            throw new Problem(400, "No value", "A choice is sent as {\"value\": \"...\"}");
          }
          message = choose.choose(chosen);
        }
      } catch (Devices.Refused refused) {
        throw new Problem(422, "Refused", String.valueOf(refused.getMessage()));
      } catch (IOException e) {
        throw new Problem(500, "Failed", name + ": " + e.getMessage());
      }
      sendJson(exchange, 200, Map.of("message", message));
    }

    /**
     * A device's file, streamed through as it arrives when the device says its length: one that
     * ends short reaches the page cut off, as an error. One of unknown length is read whole first,
     * up to the limit, so a file over it is refused rather than cut.
     */
    private void sendDeviceFile(HttpExchange exchange, String name, Devices.Download download)
        throws IOException {
      long most = deviceLimits.maxFileBytes();
      try (Devices.File file = fromDevice(name, download.fetch()::fetch)) {
        long length = file.length();
        if (length > most) {
          throw tooBig(name, file.name(), most);
        }
        if (length < 0) {
          byte[] whole =
              fromDevice(
                  name,
                  () -> file.body().readNBytes((int) Math.min(most + 1, Integer.MAX_VALUE - 8)));
          if (whole.length > most) {
            throw tooBig(name, file.name(), most);
          }
          sendFile(exchange, file.name(), file.contentType(), whole);
          return;
        }
        attachment(exchange, file.name(), file.contentType());
        exchange.sendResponseHeaders(200, length > 0 ? length : -1);
        if (length > 0) {
          // A body that ends early leaves the answer short of its length: closing it then cuts the
          // connection, so the browser reports the download failed.
          try (OutputStream out = exchange.getResponseBody()) {
            InputStream body = file.body();
            byte[] chunk = new byte[64 * 1024];
            long left = length;
            while (left > 0) {
              int read = body.read(chunk, 0, (int) Math.min(chunk.length, left));
              if (read < 0) {
                break;
              }
              out.write(chunk, 0, read);
              left -= read;
            }
          }
        }
      }
    }

    private static Problem tooBig(String name, String file, long most) {
      return new Problem(
          502,
          "Too big",
          name + "'s " + file + " is more than Pitwall passes on: " + most + " bytes");
    }

    /** Something a device gives, or 502 saying why it didn't (503 when it's busy). */
    private static <T> T fromDevice(String name, DeviceCall<T> call) {
      try {
        return call.call();
      } catch (Devices.Busy busy) {
        throw new Problem(503, name + " is busy", name + " is busy: " + busy.getMessage())
            .retryAfter(1);
      } catch (IOException | RuntimeException e) {
        String why = e.getMessage();
        throw new Problem(
            502,
            "No answer from " + name,
            name + " didn't answer: " + (why != null && !why.isBlank() ? why : e.toString()));
      }
    }

    /** A call to a device. */
    @FunctionalInterface
    private interface DeviceCall<T> {
      T call() throws IOException;
    }

    /** A query string's parameters, decoded; the last of a name given twice. */
    private static Map<String, String> query(@Nullable String raw) {
      Map<String, String> parameters = new LinkedHashMap<>();
      if (raw == null || raw.isEmpty()) {
        return parameters;
      }
      if (raw.length() > 2048) {
        throw new Problem(400, "Too long", "A query is at most 2048 characters");
      }
      for (String pair : raw.split("&")) {
        int equals = pair.indexOf('=');
        String key = equals < 0 ? pair : pair.substring(0, equals);
        String value = equals < 0 ? "" : pair.substring(equals + 1);
        try {
          parameters.put(
              URLDecoder.decode(key, StandardCharsets.UTF_8),
              URLDecoder.decode(value, StandardCharsets.UTF_8));
        } catch (IllegalArgumentException e) {
          throw new Problem(400, "Not a query", "The query can't be read: " + e.getMessage());
        }
      }
      return parameters;
    }

    /** A file, to save rather than show: never shown in the page, whatever it holds. */
    private static void sendFile(HttpExchange exchange, String name, String type, byte[] bytes)
        throws IOException {
      attachment(exchange, name, type);
      exchange.sendResponseHeaders(200, bytes.length > 0 ? bytes.length : -1);
      if (bytes.length > 0) {
        try (OutputStream out = exchange.getResponseBody()) {
          out.write(bytes);
        }
      }
    }

    /** The headers of a file to save rather than show. */
    private static void attachment(HttpExchange exchange, String name, String type) {
      headers(exchange, type);
      exchange.getResponseHeaders().set("Content-Type", type);
      exchange
          .getResponseHeaders()
          .set("Content-Disposition", "attachment; filename=\"" + name + "\"");
    }

    /** The simulator's controls: there are none on a robot, and their routes are 404 there. */
    private Simulator simulator() {
      Simulator simulator = state.simulator();
      if (simulator == null) {
        throw new Problem(404, "Not found", "There is no simulator here");
      }
      return simulator;
    }

    /**
     * The simulator as the next loop shows it; while paused, when no loop comes, as the last loop
     * left it.
     */
    private Map<String, Map<String, Object>> simulatorView() {
      Simulator simulator = simulator();
      if (simulator.isPaused()) {
        return simulator.lastView(); // no loop is coming to read it
      }
      try {
        return simulator.requestView().get(wait.toMillis(), TimeUnit.MILLISECONDS);
      } catch (TimeoutException | ExecutionException e) {
        return simulator.lastView();
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        throw new Problem(503, "Interrupted", "The server stopped waiting for the robot loop");
      }
    }

    private Object simulatorState() {
      return simulator().lastView().getOrDefault("simulator", Map.of());
    }

    private Object componentsView() {
      return simulator().lastView().getOrDefault("components", Map.of());
    }

    /**
     * Changes the Driver Station's state, all in one loop, in the order given; resumes the clock at
     * once, or pauses it once the rest is applied.
     */
    private void patchSimulator(HttpExchange exchange) throws IOException {
      Simulator simulator = simulator();
      Map<String, @Nullable Object> patch = readObject(exchange);
      UnaryOperator<DriverStationState> change = null;
      Boolean paused = null;
      for (Map.Entry<String, @Nullable Object> field : patch.entrySet()) {
        String name = field.getKey();
        Object value = field.getValue();
        UnaryOperator<DriverStationState> next =
            switch (name) {
              case "enabled" -> {
                boolean enabled = bool(name, value);
                yield station -> station.withEnabled(enabled);
              }
              case "mode" -> {
                RobotMode mode = named(name, value, RobotMode.class, MODES);
                yield station -> station.withMode(mode);
              }
              case "allianceStation" -> {
                AllianceStationID at =
                    named(name, value, AllianceStationID.class, Set.of(AllianceStationID.values()));
                yield station -> station.withAllianceStation(at);
              }
              case "fmsAttached" -> {
                boolean attached = bool(name, value);
                yield station -> station.withFmsAttached(attached);
              }
              case "dsAttached" -> {
                boolean attached = bool(name, value);
                yield station -> station.withDsAttached(attached);
              }
              case "matchTime" -> {
                double seconds = value == null ? -1 : matchTime(value);
                yield station -> station.withMatchTime(seconds);
              }
              case "paused" -> {
                paused = bool(name, value);
                yield null;
              }
              default ->
                  throw new Problem(
                      422,
                      "Not changeable",
                      name
                          + " isn't the simulator's to change: enabled, mode, allianceStation,"
                          + " fmsAttached, dsAttached, matchTime, or paused");
            };
        if (next != null) {
          UnaryOperator<DriverStationState> before = change;
          change = before == null ? next : station -> next.apply(before.apply(station));
        }
      }
      List<CompletableFuture<Outcome>> pending = new ArrayList<>();
      if (Boolean.FALSE.equals(paused)) {
        pending.add(simulator.resume()); // now: a paused clock runs no loop to do it
      }
      if (change != null) {
        pending.add(simulator.requestDriverStation(change));
      }
      if (Boolean.TRUE.equals(paused)) {
        pending.add(simulator.requestPause()); // once the rest is applied, in the same loop
      }
      if (pending.isEmpty()) {
        sendJson(exchange, 200, simulatorState()); // an empty patch changes nothing
        return;
      }
      answer(exchange, all(pending), this::simulatorState);
    }

    /** Done once every one is, with the first that wasn't applied, or applied. */
    private static CompletableFuture<Outcome> all(List<CompletableFuture<Outcome>> pending) {
      return CompletableFuture.allOf(pending.toArray(CompletableFuture<?>[]::new))
          .thenApply(
              done ->
                  pending.stream()
                      .map(CompletableFuture::join)
                      .filter(outcome -> outcome.result() != Outcome.Result.APPLIED)
                      .findFirst()
                      .orElse(Outcome.applied()));
    }

    /** Says it's stopping, then has the loop stop the program: the answer goes out first. */
    private void stopSimulator(HttpExchange exchange) throws IOException {
      Simulator simulator = simulator();
      sendJson(exchange, 202, Map.of("detail", "Stopping the simulator"));
      simulator.stop();
    }

    /** Hands each joystick's input to the robot loop, which takes the latest. */
    private void putJoysticks(HttpExchange exchange) throws IOException {
      Simulator simulator = simulator();
      Map<String, @Nullable Object> body = readObject(exchange);
      Map<Integer, @Nullable JoystickInput> inputs = new LinkedHashMap<>();
      for (Map.Entry<String, @Nullable Object> joystick : body.entrySet()) {
        int port = port(joystick.getKey());
        Object value = joystick.getValue();
        if (value == null) {
          inputs.put(port, null);
        } else if (value instanceof Map<?, ?> input) {
          inputs.put(port, joystickInput(port, input));
        } else {
          throw new Problem(
              400, "Not a joystick's input", "Send {\"axes\", \"buttons\", \"povs\"}, or null");
        }
      }
      String stream = exchange.getRequestHeaders().getFirst(STREAM);
      if (stream == null || stream.isBlank()) {
        throw new Problem(
            400,
            "No page to drive it",
            "Drive from Pitwall's page: a joystick's input names the page's event stream ("
                + STREAM
                + "), and the joystick is let go when that page goes");
      }
      inputs.forEach((port, input) -> simulator.joystick(port, input, stream));
      send(exchange, 204, "text/plain", "");
    }

    /** Applies a fault, or undoes a lasting one. */
    private void postFault(HttpExchange exchange) throws IOException {
      Simulator simulator = simulator();
      Components components = simulator.components();
      Map<String, @Nullable Object> body = readObject(exchange);
      if (!Set.of("component", "fault", "values", "undo").containsAll(body.keySet())
          || !(body.get("component") instanceof String component)
          || !(body.get("fault") instanceof String fault)) {
        throw new Problem(
            400,
            "A fault",
            "Send {\"component\", \"fault\", \"values\": {name: number}}, or {\"component\","
                + " \"fault\", \"undo\": true}");
      }
      if (Boolean.TRUE.equals(body.get("undo"))) {
        if (body.containsKey("values")) {
          throw new Problem(400, "Undo or apply", "Undoing a fault takes no values");
        }
        String why = components.whyNotUndo(component, fault);
        if (why != null) {
          throw new Problem(422, "Can't be undone", why);
        }
        answer(exchange, simulator.requestUndo(component, fault), this::componentsView);
        return;
      }
      if (body.containsKey("undo") && !Boolean.FALSE.equals(body.get("undo"))) {
        throw new Problem(400, "Not true or false", "undo must be true or false");
      }
      Map<String, Double> values = new LinkedHashMap<>();
      Object given = body.get("values");
      if (given instanceof Map<?, ?> map) {
        for (Map.Entry<?, ?> value : map.entrySet()) {
          if (!(value.getValue() instanceof Number number)) {
            throw new Problem(400, "Not a number", value.getKey() + " must be a number");
          }
          values.put(String.valueOf(value.getKey()), number.doubleValue());
        }
      } else if (given != null) {
        throw new Problem(400, "Not values", "values is an object of numbers, by name");
      }
      Setting setting = Setting.of(component, fault, values);
      String why = components.whyNot(setting);
      if (why != null) {
        throw new Problem(422, "Can't be applied", why);
      }
      answer(exchange, simulator.requestFault(setting), this::componentsView);
    }

    /** Applies a scenario's faults. */
    private void postScenario(HttpExchange exchange) throws IOException {
      Simulator simulator = simulator();
      Map<String, @Nullable Object> body = readObject(exchange);
      if (body.size() != 1 || !(body.get("apply") instanceof String name)) {
        throw new Problem(400, "Apply a scenario", "Send {\"apply\": name}");
      }
      if (!simulator.components().hasScenario(name)) {
        throw new Problem(422, "Not a scenario", "There is no scenario named " + name);
      }
      answer(exchange, simulator.requestScenario(name), this::componentsView);
    }

    private static boolean bool(String name, @Nullable Object value) {
      if (!(value instanceof Boolean bool)) {
        throw new Problem(400, "Not true or false", name + " must be true or false");
      }
      return bool;
    }

    /** One of an enum's names, among those allowed: not text is 400, a name not allowed 422. */
    private static <E extends Enum<E>> E named(
        String name, @Nullable Object value, Class<E> type, Set<E> allowed) {
      if (!(value instanceof String given)) {
        throw new Problem(400, "Not a name", name + " is a name, such as " + type.getSimpleName());
      }
      for (E each : allowed) {
        if (each.name().equals(given)) {
          return each;
        }
      }
      throw new Problem(
          422,
          "Not a " + name,
          name
              + " is one of "
              + String.join(", ", allowed.stream().map(Enum::name).sorted().toList())
              + ", not "
              + value);
    }

    private static double matchTime(Object value) {
      if (!(value instanceof Number number)) {
        throw new Problem(400, "Not a number", "matchTime is a number of seconds, or null");
      }
      if (!(number.doubleValue() >= 0 && number.doubleValue() <= MAX_MATCH_SECONDS)) {
        throw new Problem(
            422,
            "Not a match time",
            "matchTime is seconds from 0 to " + MAX_MATCH_SECONDS + ", or null for none");
      }
      return number.doubleValue();
    }

    private static int port(String name) {
      for (int port = 0; port < Simulator.JOYSTICKS; port++) {
        if (name.equals(Integer.toString(port))) {
          return port;
        }
      }
      throw new Problem(
          422,
          "Not a joystick",
          "The page drives joysticks 0 to " + (Simulator.JOYSTICKS - 1) + ", not " + name);
    }

    private static JoystickInput joystickInput(int port, Map<?, ?> input) {
      if (!Set.of("axes", "buttons", "povs").containsAll(input.keySet())) {
        throw new Problem(
            400, "Not a joystick's input", "A joystick's input is its axes, buttons, and povs");
      }
      List<Double> axes = new ArrayList<>();
      List<Boolean> buttons = new ArrayList<>();
      List<Integer> povs = new ArrayList<>();
      for (Object axis : list(input.get("axes"))) {
        if (!(axis instanceof Number number)) {
          throw new Problem(400, "Not a joystick's input", "Each axis is a number");
        }
        axes.add(number.doubleValue());
      }
      for (Object button : list(input.get("buttons"))) {
        if (!(button instanceof Boolean pressed)) {
          throw new Problem(400, "Not a joystick's input", "Each button is true or false");
        }
        buttons.add(pressed);
      }
      for (Object pov : list(input.get("povs"))) {
        if (!(pov instanceof Number number)) {
          throw new Problem(400, "Not a joystick's input", "Each POV is a number of degrees");
        }
        if (number.doubleValue() != number.intValue()) {
          throw new Problem(
              422, "Not a POV", "Joystick " + port + ": a POV is a whole number of degrees");
        }
        povs.add(number.intValue());
      }
      if (axes.size() > JoystickInput.MAX_AXES
          || buttons.size() > JoystickInput.MAX_BUTTONS
          || povs.size() > JoystickInput.MAX_POVS) {
        throw new Problem(
            400,
            "Not a joystick's input",
            "A joystick has at most "
                + JoystickInput.MAX_AXES
                + " axes, "
                + JoystickInput.MAX_BUTTONS
                + " buttons, and "
                + JoystickInput.MAX_POVS
                + " POVs");
      }
      String why = JoystickInput.whyNot(axes, buttons, povs);
      if (why != null) {
        throw new Problem(422, "Out of range", "Joystick " + port + ": " + why);
      }
      return new JoystickInput(axes, buttons, povs);
    }

    private static List<?> list(@Nullable Object value) {
      if (value == null) {
        return List.of();
      }
      if (!(value instanceof List<?> list)) {
        throw new Problem(400, "Not a joystick's input", "axes, buttons, and povs are lists");
      }
      return list;
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
        case REFUSED -> throw new Problem(422, "Refused", outcome.detail(), outcome.reasons());
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
        shown.put("locked", state.locked());
        shown.put("when", state.when().name());
        states.add(shown);
      }
      return states;
    }

    private Map<String, Object> session() {
      return state.session();
    }

    private Map<String, Object> configs() {
      return Page.configs(tunables.configs());
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
        Page.ends(shown, tunable.limits());
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
          shown.put("field", change.path());
          shown.put("committed", change.committed());
          shown.put("tuned", change.tuned());
          shown.put("builderCall", change.builderCall());
          changes.add(shown);
        }
      }
      return changes;
    }

    private void sendBundled(HttpExchange exchange, String path) throws IOException {
      byte[] file = bundled.get(path);
      if (file == null) {
        throw new Problem(404, "Not bundled", path + " isn't bundled with Pitwall");
      }
      headers(exchange, Objects.requireNonNull(BUNDLED.get(path)));
      // The same for as long as the program runs: the browser may keep it.
      exchange.getResponseHeaders().set("Cache-Control", "max-age=3600");
      exchange.sendResponseHeaders(200, file.length);
      try (OutputStream out = exchange.getResponseBody()) {
        out.write(file);
      }
    }

    /**
     * One of the page's files, from {@code page/} beside index.html. Not kept by the browser, so a
     * page reloaded after a redeploy runs the new program's page.
     */
    private void sendPageFile(HttpExchange exchange, String path) throws IOException {
      String name = path.substring("/page/".length());
      byte[] file = pageFiles.get(name);
      if (file == null) {
        file =
            pageFile(name)
                .orElseThrow(() -> new Problem(404, "Not found", "There is nothing at " + path));
        pageFiles.putIfAbsent(name, file);
      }
      headers(exchange, name.endsWith(".css") ? "text/css" : "text/javascript");
      exchange.sendResponseHeaders(200, file.length);
      try (OutputStream out = exchange.getResponseBody()) {
        out.write(file);
      }
    }

    private void sendJson(HttpExchange exchange, int status, Object body) throws IOException {
      send(exchange, status, "application/json", json.toJson(body));
    }

    private void sendProblem(HttpExchange exchange, Problem problem) throws IOException {
      if (problem.retryAfterSeconds > 0) {
        exchange
            .getResponseHeaders()
            .set("Retry-After", Integer.toString(problem.retryAfterSeconds));
      }
      Map<String, Object> body = new LinkedHashMap<>();
      body.put("type", "about:blank");
      body.put("title", problem.title);
      body.put("status", problem.status);
      body.put("detail", String.valueOf(problem.getMessage()));
      if (!problem.reasons.isEmpty()) {
        // An extension member (RFC 9457 allows them): each reason, tied to its values.
        body.put("errors", problem.reasons.stream().map(Api::error).toList());
      }
      send(exchange, problem.status, "application/problem+json", json.toJson(body));
    }

    /**
     * A reason a change was refused, as the page reads it: which check ({@code layer}: value,
     * limit, record, or tunable), who refused it ({@code by}: a record), the tunables it concerns
     * ({@code paths}, and {@code path} when it's one), its configuration and, for a record's check
     * that names no value, where the record is ({@code group}), why ({@code text}), and its line of
     * {@code detail} ({@code line}).
     */
    private static Map<String, Object> error(Outcome.Reason reason) {
      Map<String, Object> shown = new LinkedHashMap<>();
      shown.put("layer", reason.layer().name().toLowerCase(Locale.ROOT));
      shown.put("by", reason.by());
      if (reason.paths().size() == 1) {
        shown.put("path", reason.paths().getFirst());
      }
      shown.put("paths", reason.paths());
      shown.put("config", reason.config());
      shown.put("group", reason.group());
      shown.put("text", reason.text());
      shown.put("line", reason.line());
      return shown;
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
      // loads nothing from anywhere else. It runs no inline script or style: what a robot's text
      // could slip into the page (it's always set as text) couldn't run.
      exchange.getResponseHeaders().set("X-Frame-Options", "DENY");
      exchange
          .getResponseHeaders()
          .set("Content-Security-Policy", "default-src 'self'; frame-ancestors 'none'");
      exchange.getResponseHeaders().set("X-Content-Type-Options", "nosniff");
      exchange.getResponseHeaders().set("Referrer-Policy", "no-referrer");
    }
  }
}
