package com.michaelgrundvig.frc.coprocessor.agent;

import com.michaelgrundvig.frc.coprocessor.api.AgentApi;
import com.michaelgrundvig.frc.coprocessor.api.ShutdownAnswer;
import com.michaelgrundvig.frc.coprocessor.json.Json;
import com.michaelgrundvig.frc.coprocessor.json.JsonValue;
import com.michaelgrundvig.frc.coprocessor.settings.SettingsFiles;
import com.michaelgrundvig.frc.coprocessor.settings.SettingsRow;
import com.michaelgrundvig.frc.coprocessor.settings.SettingsText;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.net.InetSocketAddress;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;
import org.jspecify.annotations.Nullable;

/**
 * The agent's HTTP API, version 1, on the JDK's own server with two threads: JSON answers, each
 * bounded; nothing a request says is run. The one action, {@code POST /v1/shutdown}, is the robot
 * controller's alone. The only class that touches the web server.
 */
final class AgentServer implements AutoCloseable {
  /** The most any JSON answer may be, but the settings: health is about 2 KB. */
  static final int MAX_ANSWER = 4 * 1024 * 1024;

  /** The most a settings answer may be, in characters of the settings' text. */
  static final long MAX_SETTINGS = 48L * 1024 * 1024;

  private static final Set<String> PATHS =
      Set.of(
          AgentApi.STAMP,
          AgentApi.HEALTH,
          AgentApi.JOURNAL,
          AgentApi.SETTINGS,
          AgentApi.SETTINGS_ZIP,
          AgentApi.SHUTDOWN);

  private static final Map<String, Integer> PRIORITIES =
      Map.of(
          "emerg", 0, "alert", 1, "crit", 2, "err", 3, "warning", 4, "notice", 5, "info", 6,
          "debug", 7);

  private final Agent agent;
  private final HttpServer server;
  private final ExecutorService threads;

  /** A request that can't be answered as asked, with its status. */
  private static final class Refused extends Exception {
    private static final long serialVersionUID = 1L;
    final int status;
    final String reason;

    Refused(int status, String reason) {
      super(reason);
      this.status = status;
      this.reason = reason;
    }
  }

  AgentServer(Agent agent, InetSocketAddress address) throws IOException {
    this.agent = agent;
    this.threads =
        Executors.newFixedThreadPool(
            2,
            task -> {
              Thread thread = new Thread(task, "coprocessor-agent-http");
              thread.setDaemon(true);
              return thread;
            });
    this.server = HttpServer.create(address, 16);
    server.setExecutor(threads);
    server.createContext("/", this::handle);
    server.start();
  }

  /** The port it's serving on. */
  int port() {
    return server.getAddress().getPort();
  }

  @Override
  public void close() {
    server.stop(0);
    threads.shutdownNow();
  }

  private void handle(HttpExchange exchange) throws IOException {
    try (exchange) {
      String path = exchange.getRequestURI().getRawPath();
      String method = exchange.getRequestMethod();
      exchange.getResponseHeaders().set("Cache-Control", "no-store");
      try {
        if (!PATHS.contains(path)) {
          throw new Refused(404, "no such resource: " + path);
        }
        boolean shutdown = path.equals(AgentApi.SHUTDOWN);
        if (shutdown ? !method.equals("POST") : !method.equals("GET")) {
          exchange.getResponseHeaders().set("Allow", shutdown ? "POST" : "GET");
          throw new Refused(405, path + " takes " + (shutdown ? "POST" : "GET") + " only");
        }
        switch (path) {
          case AgentApi.STAMP -> json(exchange, 200, agent.stamp().toJson());
          case AgentApi.HEALTH -> json(exchange, 200, agent.health().toJson());
          case AgentApi.JOURNAL -> journal(exchange);
          case AgentApi.SETTINGS -> settings(exchange);
          case AgentApi.SETTINGS_ZIP -> settingsZip(exchange);
          default -> shutdown(exchange);
        }
      } catch (Refused e) {
        json(exchange, e.status, error(e.reason));
      } catch (IOException | RuntimeException e) {
        json(exchange, 500, error(e.getClass().getSimpleName() + ": " + e.getMessage()));
      }
    } catch (IOException e) {
      // The client went away mid-answer: nothing to do.
    }
  }

  private void journal(HttpExchange exchange) throws IOException, Refused {
    Map<String, String> query = query(exchange.getRequestURI().getRawQuery());
    String cursor = query.getOrDefault("cursor", "");
    if (!cursor.isEmpty() && !JournalSource.CURSOR.matcher(cursor).matches()) {
      throw new Refused(400, "cursor isn't a journal cursor");
    }
    String unit = query.getOrDefault("unit", "");
    if (!unit.isEmpty() && !JournalSource.UNIT.matcher(unit).matches()) {
      throw new Refused(400, "unit isn't a systemd unit's name");
    }
    int priority = -1;
    String level = query.getOrDefault("priority", "").toLowerCase(Locale.ROOT);
    if (!level.isEmpty()) {
      Integer named = PRIORITIES.get(level);
      priority = named != null ? named : number(level, 0, 7, "priority");
    }
    int limit = AgentApi.DEFAULT_JOURNAL_PAGE;
    if (query.containsKey("limit")) {
      limit = number(query.getOrDefault("limit", ""), 1, AgentApi.MAX_JOURNAL_PAGE, "limit");
    }
    json(exchange, 200, agent.journal(cursor, priority, unit, limit).toJson());
  }

  /** The settings as JSON, written a row at a time: the same as {@code Settings.toJson()}. */
  private void settings(HttpExchange exchange) throws IOException, Refused {
    SettingsSource.Snapshot snapshot = snapshot();
    exchange.getResponseHeaders().set("Content-Type", "application/json; charset=utf-8");
    exchange.sendResponseHeaders(200, 0);
    try (Writer out = new OutputStreamWriter(exchange.getResponseBody(), StandardCharsets.UTF_8)) {
      out.write("{\"hash\":");
      out.write(Json.compact(JsonValue.of(snapshot.hash())));
      out.write(",\"userVersion\":" + snapshot.text().userVersion() + ",\"rows\":[");
      boolean first = true;
      for (SettingsText.Row text : snapshot.text().rows()) {
        SettingsRow row = text.parse();
        out.write(first ? "" : ",");
        first = false;
        out.write(
            Json.compact(
                JsonValue.Obj.builder()
                    .put("table", row.table())
                    .put("key", row.key())
                    .put("columns", row.columns())
                    .build()));
      }
      out.write("]}");
    }
  }

  /**
   * The settings as a zip laid out like the repository: {@code coprocessors/<computer>/settings/}
   * and its files, so unzipping it at the repository's root puts them in place.
   */
  private void settingsZip(HttpExchange exchange) throws IOException, Refused {
    SettingsSource.Snapshot snapshot = snapshot();
    String name = agent.stampFile().stamp().name();
    String computer = name.isEmpty() ? "coprocessor" : name;
    String folder = SettingsFiles.folder(computer) + "/";
    List<String> paths =
        snapshot.text().rows().stream()
            .map(row -> SettingsFiles.path(row.table(), row.key()))
            .toList();
    SettingsFiles.checkCase(paths);
    exchange.getResponseHeaders().set("Content-Type", "application/zip");
    exchange
        .getResponseHeaders()
        .set("Content-Disposition", "attachment; filename=\"" + computer + "-settings.zip\"");
    exchange.sendResponseHeaders(200, 0);
    try (ZipOutputStream zip = new ZipOutputStream(exchange.getResponseBody())) {
      entry(
          zip,
          folder + SettingsFiles.DATABASE_FILE,
          SettingsFiles.database(snapshot.text().userVersion()));
      int i = 0;
      for (SettingsText.Row text : snapshot.text().rows()) {
        entry(zip, folder + paths.get(i++), SettingsFiles.text(text.parse()));
      }
    }
  }

  private static void entry(ZipOutputStream zip, String path, String text) throws IOException {
    zip.putNextEntry(new ZipEntry(path));
    zip.write(text.getBytes(StandardCharsets.UTF_8));
    zip.closeEntry();
  }

  private SettingsSource.Snapshot snapshot() throws IOException, Refused {
    Optional<SettingsSource.Snapshot> snapshot = agent.settings();
    if (snapshot.isEmpty()) {
      throw new Refused(404, "PhotonVision has no settings database yet");
    }
    if (snapshot.get().text().length() > MAX_SETTINGS) {
      throw new Refused(500, "the settings are larger than the agent sends");
    }
    return snapshot.get();
  }

  private void shutdown(HttpExchange exchange) throws IOException, Refused {
    String from = exchange.getRemoteAddress().getAddress().getHostAddress();
    String controller = agent.controller();
    if (!from.equals(controller)) {
      throw new Refused(
          403, "only the robot controller (" + controller + ") may shut this computer down");
    }
    boolean first = agent.shutdown(from);
    json(exchange, 202, new ShutdownAnswer(!first).toJson());
  }

  private static JsonValue.Obj error(String message) {
    return JsonValue.Obj.builder().put("error", message).build();
  }

  private static void json(HttpExchange exchange, int status, JsonValue body) throws IOException {
    byte[] bytes = Json.compact(body).getBytes(StandardCharsets.UTF_8);
    int sent = status;
    if (bytes.length > MAX_ANSWER) {
      sent = 500;
      bytes =
          Json.compact(error("the answer is larger than the agent sends"))
              .getBytes(StandardCharsets.UTF_8);
    }
    exchange.getResponseHeaders().set("Content-Type", "application/json; charset=utf-8");
    exchange.sendResponseHeaders(sent, bytes.length);
    try (OutputStream out = exchange.getResponseBody()) {
      out.write(bytes);
    }
  }

  private static Map<String, String> query(@Nullable String raw) throws Refused {
    Map<String, String> query = new HashMap<>();
    if (raw == null || raw.isEmpty()) {
      return query;
    }
    for (String pair : raw.split("&")) {
      int equals = pair.indexOf('=');
      String key = equals < 0 ? pair : pair.substring(0, equals);
      String value = equals < 0 ? "" : pair.substring(equals + 1);
      try {
        query.put(
            URLDecoder.decode(key, StandardCharsets.UTF_8),
            URLDecoder.decode(value, StandardCharsets.UTF_8));
      } catch (IllegalArgumentException e) {
        throw new Refused(400, "the query isn't URL-encoded: " + e.getMessage());
      }
    }
    return query;
  }

  private static int number(String text, int min, int max, String what) throws Refused {
    try {
      int value = Integer.parseInt(text);
      if (value >= min && value <= max) {
        return value;
      }
    } catch (NumberFormatException e) {
      // refused below
    }
    throw new Refused(400, what + " must be a number from " + min + " to " + max);
  }
}
