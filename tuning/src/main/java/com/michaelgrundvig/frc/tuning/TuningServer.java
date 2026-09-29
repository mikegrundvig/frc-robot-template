package com.michaelgrundvig.frc.tuning;

import java.io.IOException;
import java.io.InputStream;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.Optional;
import java.util.function.Consumer;
import org.eclipse.jetty.http.HttpHeader;
import org.eclipse.jetty.io.Content;
import org.eclipse.jetty.server.Handler;
import org.eclipse.jetty.server.Request;
import org.eclipse.jetty.server.Response;
import org.eclipse.jetty.server.Server;
import org.eclipse.jetty.server.ServerConnector;
import org.eclipse.jetty.util.Callback;
import org.eclipse.jetty.util.Fields;
import org.eclipse.jetty.util.thread.QueuedThreadPool;

/**
 * The tuning page, served by the robot program itself: http://localhost:5800 in the simulator,
 * http://robot.local:5800 on the robot. Any browser works, a phone's included.
 *
 * <p>The server's threads never change the robot: they read {@link Tuning#view()} and queue
 * requests for the robot loop to apply.
 */
public final class TuningServer implements AutoCloseable {
  /** In the range of ports the field network leaves open for teams. */
  public static final int PORT = 5800;

  private final Server server;
  private final ServerConnector connector;

  /**
   * @param port the port to serve on; 0 picks a free one
   */
  public TuningServer(Tuning<?> tuning, int port) {
    // A few threads are plenty for one page, and daemon threads never keep the program running.
    QueuedThreadPool threads = new QueuedThreadPool(8, 2);
    threads.setName("tuning-page");
    threads.setDaemon(true);
    server = new Server(threads);
    connector = new ServerConnector(server, 1, 1);
    connector.setPort(port);
    server.addConnector(connector);
    server.setHandler(new Routes(tuning, page()));
  }

  /**
   * Starts the page, or says why it couldn't. The page is a convenience: the robot runs without it.
   *
   * @param port usually {@link #PORT}
   * @param say prints a line where the team sees it
   * @return the running server, or empty if it couldn't start
   */
  public static Optional<TuningServer> startOrExplain(
      Tuning<?> tuning, int port, Consumer<String> say) {
    TuningServer server = null;
    try {
      server = new TuningServer(tuning, port);
      server.start();
      say.accept(
          "Tuning page: http://localhost:"
              + server.port()
              + " in the simulator, http://robot.local:"
              + server.port()
              + " on the robot");
      return Optional.of(server);
    } catch (Exception e) {
      say.accept("The tuning page couldn't start, so it is unavailable: " + e);
      if (server != null) {
        stopQuietly(server);
      }
      return Optional.empty();
    }
  }

  private static void stopQuietly(TuningServer server) {
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
    try (InputStream in = TuningServer.class.getResourceAsStream("index.html")) {
      if (in == null) {
        throw new IllegalStateException("the tuning page (index.html) is missing");
      }
      return new String(in.readAllBytes(), StandardCharsets.UTF_8);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }

  /** The page, the state it polls, and the requests it sends. */
  private static final class Routes extends Handler.Abstract {
    private final Tuning<?> tuning;
    private final String page;

    Routes(Tuning<?> tuning, String page) {
      this.tuning = tuning;
      this.page = page;
    }

    @Override
    public boolean handle(Request request, Response response, Callback callback) throws Exception {
      switch (request.getMethod() + " " + Request.getPathInContext(request)) {
        case "GET /" -> send(response, callback, 200, "text/html", page);
        case "GET /api/state" ->
            send(response, callback, 200, "application/json", Json.view(tuning.view()));
        case "POST /api/set" -> set(request, response, callback);
        case "POST /api/reset" -> {
          tuning.requestReset();
          send(response, callback, 202, "text/plain", "");
        }
        case "POST /api/enabled" -> {
          Fields fields = Request.getParameters(request);
          tuning.requestEnabled("true".equals(fields.getValue("on")));
          send(response, callback, 202, "text/plain", "");
        }
        default -> send(response, callback, 404, "text/plain", "Not found");
      }
      return true;
    }

    private void set(Request request, Response response, Callback callback) throws Exception {
      Fields fields = Request.getParameters(request);
      String path = String.valueOf(fields.getValue("path"));
      String value = String.valueOf(fields.getValue("value"));
      if (!tuning.isTunable(path)) {
        send(response, callback, 404, "text/plain", "No tunable value at " + path);
        return;
      }
      double number;
      try {
        number = Double.parseDouble(value);
      } catch (NumberFormatException e) {
        send(response, callback, 400, "text/plain", "Not a number: " + value);
        return;
      }
      tuning.requestSet(path, number);
      send(response, callback, 202, "text/plain", "");
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
