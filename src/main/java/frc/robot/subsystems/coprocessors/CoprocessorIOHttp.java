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
  /** The most a health answer may hold: the agent's are 3 to 4 KB. */
  static final int MAX_HEALTH_BYTES = 64 * 1024;

  /** The most a settings backup may hold, calibrations included: it's streamed, never held. */
  static final int MAX_BACKUP_BYTES = 64 * 1024 * 1024;

  /** The most a page of the journal may hold: twice what the agent sends at most. */
  static final int MAX_JOURNAL_BYTES = 8 * 1024 * 1024;

  private record Answer(Health health, List<String> macsTakingTurns) {}

  private final AgentHttp agent;
  private final AgentHttp backups;
  private final AgentHttp journals;
  private final LongSupplier robotClock;
  private final CoprocessorPoller<Answer> poller;

  /** Seen only on the poller's thread. */
  private final MacWatch macs;

  /** The answer the inputs hold, seen only on the loop's thread. */
  private @Nullable Answer filled;

  /** Asks it to power down, and watches it go, on a thread of its own. */
  private final PowerDowner powerDowner;

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
    this(name, address, port, Coprocessors.PHOTONVISION_PORT, settings, robotClock);
  }

  /**
   * @param photonVisionPort where PhotonVision's page answers: 5800, but for a test's stand-in
   */
  CoprocessorIOHttp(
      String name,
      String address,
      int port,
      int photonVisionPort,
      Coprocessors.Settings settings,
      LongSupplier robotClock) {
    agent =
        new AgentHttp(
            address,
            port,
            settings.connectTimeoutSeconds(),
            settings.answerTimeoutSeconds(),
            MAX_HEALTH_BYTES);
    backups =
        agent.with(
            settings.connectTimeoutSeconds(), settings.downloadTimeoutSeconds(), MAX_BACKUP_BYTES);
    journals =
        agent.with(
            settings.connectTimeoutSeconds(), settings.downloadTimeoutSeconds(), MAX_JOURNAL_BYTES);
    this.robotClock = robotClock;
    macs = new MacWatch(settings.turnsWindowSeconds());
    powerDowner =
        new PowerDowner(
            name,
            new PowerDowner.Target() {
              @Override
              public int ask() throws IOException {
                return agent.post(AgentApi.SHUTDOWN);
              }

              @Override
              public boolean agentAnswers() {
                return agent.accepts(port);
              }

              @Override
              public boolean photonVisionAnswers() {
                return agent.accepts(photonVisionPort);
              }
            },
            Math.round(settings.pollPeriodSeconds() * 1000),
            Math.round(settings.powerDownTimeoutSeconds() * 1000));
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
    PowerDowner.State down = powerDowner.state();
    inputs.powerDownAsked = down.asked();
    inputs.powerDownAccepted = down.accepted();
    inputs.powerDownAnswer = down.answer();
    inputs.portsTried = down.tried();
    inputs.agentPortOpen = down.agentOpen();
    inputs.photonVisionPortOpen = down.photonVisionOpen();
  }

  @Override
  public void powerDown() {
    powerDowner.ask();
  }

  @Override
  public void stopPowerDown() {
    powerDowner.stop();
  }

  @Override
  public Backup settingsBackup() throws IOException {
    AgentHttp.Streamed answer = backups.stream(AgentApi.SETTINGS_ZIP);
    return new Backup(answer.length(), answer.body());
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
    byte[] body = journals.get(query.toString());
    try {
      return JournalPage.parse(new String(body, StandardCharsets.UTF_8));
    } catch (JsonException | IllegalArgumentException e) {
      throw new IOException("its journal can't be read: " + e.getMessage(), e);
    }
  }

  @Override
  public void close() {
    poller.close();
    powerDowner.close();
  }
}
