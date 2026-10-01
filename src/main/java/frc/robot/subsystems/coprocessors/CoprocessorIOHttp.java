package frc.robot.subsystems.coprocessors;

import com.michaelgrundvig.frc.coprocessor.api.AgentApi;
import com.michaelgrundvig.frc.coprocessor.api.Health;
import com.michaelgrundvig.frc.coprocessor.api.JournalPage;
import com.michaelgrundvig.frc.coprocessor.json.JsonException;
import java.io.IOException;
import java.net.URLEncoder;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.function.LongSupplier;
import org.jspecify.annotations.Nullable;

/**
 * A coprocessor's health agent, over the robot's network: asked for its health ({@code /v1/health},
 * which carries its stamp) once a period on a thread of this IO's own, within a connect timeout and
 * a timeout on the whole answer. The robot loop takes only the latest answer. Downloads (its
 * settings backup, its journal) are asked on the caller's thread, with longer timeouts.
 */
public final class CoprocessorIOHttp implements CoprocessorIO {
  /** The most a health answer may hold: the agent's are about 2 KB. */
  static final int MAX_HEALTH_BYTES = 64 * 1024;

  /** The most a download may hold: a settings backup, calibrations included, or a journal page. */
  static final int MAX_DOWNLOAD_BYTES = 64 * 1024 * 1024;

  private record Answer(Health health, List<String> macsTakingTurns) {}

  /**
   * Where powering it down stands.
   *
   * @param asked whether it was asked
   * @param answer how its agent answered: "accepted", or why not; empty until it did
   * @param tried whether its ports have been tried since
   * @param agentOpen whether its agent's port still accepts a connection
   * @param photonVisionOpen whether PhotonVision's does
   */
  private record PowerDown(
      boolean asked, String answer, boolean tried, boolean agentOpen, boolean photonVisionOpen) {}

  /** Where its agent takes a request to power down. */
  static final String SHUTDOWN = "/v1/shutdown";

  private final AgentHttp agent;
  private final AgentHttp downloads;
  private final LongSupplier robotClock;
  private final CoprocessorPoller<Answer> poller;

  /** Seen only on the poller's thread. */
  private final MacWatch macs;

  /** The answer the inputs hold, seen only on the loop's thread. */
  private @Nullable Answer filled;

  private final String name;
  private final long periodMillis;
  private volatile PowerDown powerDown = new PowerDown(false, "", false, false, false);
  private @Nullable Thread poweringDown;

  /**
   * Starts polling the agent, on a thread of its own.
   *
   * @param name the coprocessor's name, for its thread
   * @param address its address, such as {@code 10.0.0.11}
   * @param port its agent's port
   * @param settings how often to ask, and how long to wait
   * @param robotClock the robot's time, in nanoseconds
   */
  public CoprocessorIOHttp(
      String name,
      String address,
      int port,
      Coprocessors.Settings settings,
      LongSupplier robotClock) {
    agent =
        new AgentHttp(
            address,
            port,
            settings.connectTimeoutSeconds(),
            settings.answerTimeoutSeconds(),
            MAX_HEALTH_BYTES);
    downloads =
        agent.with(
            settings.connectTimeoutSeconds(),
            settings.downloadTimeoutSeconds(),
            MAX_DOWNLOAD_BYTES);
    this.robotClock = robotClock;
    this.name = name;
    periodMillis = Math.round(settings.pollPeriodSeconds() * 1000);
    macs = new MacWatch(settings.turnsWindowSeconds());
    poller = new CoprocessorPoller<>(name, this::ask, settings.pollPeriodSeconds(), robotClock);
    poller.start();
  }

  /** One poll, on the poller's thread. */
  private Answer ask() throws IOException {
    Health health = health(agent.get(AgentApi.HEALTH));
    macs.heard(System.nanoTime(), health.stamp().mac());
    return new Answer(health, macs.takingTurns());
  }

  /** An answer's health, or why it can't be read. */
  static Health health(byte[] body) throws IOException {
    try {
      return Health.parse(new String(body, StandardCharsets.UTF_8));
    } catch (JsonException | IllegalArgumentException e) {
      throw new IOException("its answer can't be read: " + e.getMessage(), e);
    }
  }

  @Override
  public void updateInputs(CoprocessorIOInputs inputs) {
    CoprocessorPoller.Snapshot<Answer> latest = poller.latest();
    inputs.answered = latest.answered();
    inputs.error = latest.error();
    Answer answer = latest.answer();
    inputs.everAnswered = answer != null;
    inputs.ageSeconds =
        answer == null ? 0 : Math.max(0, (robotClock.getAsLong() - latest.answeredNanos()) / 1e9);
    if (answer != null && answer != filled) {
      // Only a new answer is flattened again: the same one stays as it was.
      inputs.answeredNanos = latest.answeredNanos();
      Answers.fill(inputs, answer.health());
      inputs.macsTakingTurns = answer.macsTakingTurns().toArray(String[]::new);
      filled = answer;
    }
    PowerDown down = powerDown;
    inputs.powerDownAsked = down.asked();
    inputs.powerDownAnswer = down.answer();
    inputs.portsTried = down.tried();
    inputs.agentPortOpen = down.agentOpen();
    inputs.photonVisionPortOpen = down.photonVisionOpen();
  }

  @Override
  public void powerDown() {
    Thread was = poweringDown;
    if (was != null) {
      was.interrupt();
    }
    powerDown = new PowerDown(true, "", false, true, true);
    poweringDown =
        Thread.ofVirtual().name("Coprocessor " + name + " powering down").start(this::powerDownNow);
  }

  /**
   * Asks the agent to power down, then tries both ports once a period until neither accepts a
   * connection, on the powering-down thread.
   */
  private void powerDownNow() {
    String answer;
    try {
      int status = agent.post(SHUTDOWN);
      answer = status == 202 ? "accepted" : "answered " + status;
    } catch (IOException e) {
      answer = CoprocessorPoller.why(e);
    }
    while (!Thread.currentThread().isInterrupted()) {
      boolean agentOpen = agent.accepts(agent.port());
      boolean photonVisionOpen = agent.accepts(Coprocessors.PHOTONVISION_PORT);
      powerDown = new PowerDown(true, answer, true, agentOpen, photonVisionOpen);
      if (!agentOpen && !photonVisionOpen) {
        return;
      }
      try {
        Thread.sleep(periodMillis);
      } catch (InterruptedException e) {
        return;
      }
    }
  }

  @Override
  public byte[] settingsBackup() throws IOException {
    return downloads.get(AgentApi.SETTINGS_ZIP);
  }

  @Override
  public JournalPage journal(String cursor, int priority, String unit) throws IOException {
    StringBuilder query =
        new StringBuilder(AgentApi.JOURNAL)
            .append("?priority=")
            .append(priority)
            .append("&limit=")
            .append(AgentApi.MAX_JOURNAL_PAGE);
    if (!cursor.isEmpty()) {
      query.append("&cursor=").append(URLEncoder.encode(cursor, StandardCharsets.UTF_8));
    }
    if (!unit.isEmpty()) {
      query.append("&unit=").append(URLEncoder.encode(unit, StandardCharsets.UTF_8));
    }
    byte[] body = downloads.get(query.toString());
    try {
      return JournalPage.parse(new String(body, StandardCharsets.UTF_8));
    } catch (JsonException | IllegalArgumentException e) {
      throw new IOException("its journal can't be read: " + e.getMessage(), e);
    }
  }

  @Override
  public void close() {
    poller.close();
    Thread down = poweringDown;
    if (down != null) {
      down.interrupt();
    }
  }
}
