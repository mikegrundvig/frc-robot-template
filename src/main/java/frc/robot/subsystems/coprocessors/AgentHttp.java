package frc.robot.subsystems.coprocessors;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;
import java.net.URI;
import java.net.URISyntaxException;

/**
 * GETs from a coprocessor's health agent, within a connect timeout and a timeout on the whole
 * answer, and reads at most so many bytes: the agent bounds its answers, and this never trusts it
 * to. Plain {@link HttpURLConnection}, which every Java runtime has, the robot's included.
 */
final class AgentHttp {
  /** How much to read at a time. */
  private static final int CHUNK = 8192;

  private final String host;
  private final int port;
  private final int connectTimeoutMillis;
  private final int answerTimeoutMillis;
  private final int maxBytes;

  /**
   * @param host the coprocessor's address, such as {@code 10.0.0.11}
   * @param port the agent's port
   * @param connectTimeoutSeconds how long to wait for a connection
   * @param answerTimeoutSeconds how long the whole answer may take, once connected
   * @param maxBytes the most an answer may hold: a longer one is refused
   */
  AgentHttp(
      String host,
      int port,
      double connectTimeoutSeconds,
      double answerTimeoutSeconds,
      int maxBytes) {
    this.host = host;
    this.port = port;
    connectTimeoutMillis = millis(connectTimeoutSeconds);
    answerTimeoutMillis = millis(answerTimeoutSeconds);
    this.maxBytes = maxBytes;
  }

  /** The same agent, with other timeouts and another size limit: for a download, say. */
  AgentHttp with(double connectTimeoutSeconds, double answerTimeoutSeconds, int maxBytes) {
    return new AgentHttp(host, port, connectTimeoutSeconds, answerTimeoutSeconds, maxBytes);
  }

  /** The agent's address and port, as a URL's authority: {@code 10.0.0.11:5808}. */
  String authority() {
    return host + ":" + port;
  }

  /**
   * An answer's body.
   *
   * @param pathAndQuery such as {@code /v1/health} or {@code /v1/journal?priority=3}
   * @throws IOException saying why there's none: no connection or answer in time, an answer other
   *     than 200, or one too long
   */
  byte[] get(String pathAndQuery) throws IOException {
    HttpURLConnection connection = open(pathAndQuery);
    long deadline = System.nanoTime() + answerTimeoutMillis * 1_000_000L;
    try {
      int status = connection.getResponseCode();
      if (status != HttpURLConnection.HTTP_OK) {
        throw new IOException("answered " + status + " to " + pathAndQuery);
      }
      try (InputStream in = connection.getInputStream()) {
        return readBounded(in, deadline);
      }
    } finally {
      connection.disconnect();
    }
  }

  /**
   * POSTs to the agent, with no body, and says how it answered.
   *
   * @return the answer's status, such as 202
   * @throws IOException saying why there's no answer: no connection, or none in time
   */
  int post(String path) throws IOException {
    HttpURLConnection connection = open(path);
    try {
      connection.setRequestMethod("POST");
      connection.setDoOutput(true);
      connection.setFixedLengthStreamingMode(0);
      connection.getOutputStream().close();
      return connection.getResponseCode();
    } finally {
      connection.disconnect();
    }
  }

  /**
   * Whether anything accepts a connection on a port of the agent's computer, within the connect
   * timeout: a computer that's gone refuses, or doesn't answer at all.
   */
  boolean accepts(int port) {
    try (Socket socket = new Socket()) {
      socket.connect(new InetSocketAddress(host, port), connectTimeoutMillis);
      return true;
    } catch (IOException e) {
      return false;
    }
  }

  /** The agent's port. */
  int port() {
    return port;
  }

  private HttpURLConnection open(String pathAndQuery) throws IOException {
    if (!pathAndQuery.startsWith("/")) {
      throw new IllegalArgumentException("a path on the agent starts with /: " + pathAndQuery);
    }
    HttpURLConnection connection;
    try {
      connection =
          (HttpURLConnection)
              new URI("http://" + authority() + pathAndQuery).toURL().openConnection();
    } catch (URISyntaxException e) {
      throw new IllegalArgumentException("not a path on the agent: " + pathAndQuery, e);
    }
    connection.setConnectTimeout(connectTimeoutMillis);
    connection.setReadTimeout(answerTimeoutMillis);
    connection.setUseCaches(false);
    connection.setInstanceFollowRedirects(false);
    connection.setRequestProperty("Accept", "*/*");
    return connection;
  }

  private byte[] readBounded(InputStream in, long deadline) throws IOException {
    ByteArrayOutputStream body = new ByteArrayOutputStream();
    byte[] chunk = new byte[CHUNK];
    while (true) {
      int read = in.read(chunk);
      if (read < 0) {
        return body.toByteArray();
      }
      if (body.size() + read > maxBytes) {
        throw new IOException("answered more than " + maxBytes + " bytes");
      }
      body.write(chunk, 0, read);
      if (System.nanoTime() > deadline) {
        throw new SocketTimeoutException("no whole answer within " + answerTimeoutMillis + " ms");
      }
    }
  }

  private static int millis(double seconds) {
    if (!(seconds > 0) || seconds > 3600) {
      throw new IllegalArgumentException("a timeout, in seconds: " + seconds);
    }
    return (int) Math.max(1, Math.round(seconds * 1000));
  }
}
