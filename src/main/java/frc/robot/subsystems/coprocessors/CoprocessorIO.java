package frc.robot.subsystems.coprocessors;

import com.michaelgrundvig.frc.coprocessor.api.JournalPage;
import java.io.IOException;
import java.io.InputStream;
import org.littletonrobotics.junction.AutoLog;

/**
 * One coprocessor's door to its health agent: over the network on the robot ({@link
 * CoprocessorIOHttp}), a simulated agent in the simulator ({@link CoprocessorIOSim}), or nothing in
 * a replay, where the inputs come from the log. The agent is asked on a thread of the IO's own,
 * once a period; the robot loop reads only the latest answer, and how old it is.
 *
 * <p>A file or a page of the journal is fetched on the asker's thread (Pitwall's server, for a
 * download), never the robot loop's.
 */
public interface CoprocessorIO extends AutoCloseable {
  /**
   * What the robot loop knows of one coprocessor this loop: the latest poll, and the latest
   * answer's stamp and health ({@code /v1/health}), with when it was given on the robot's clock.
   * What an answer holds stays as it was until the next answer; before the first, it's empty.
   */
  @AutoLog
  class CoprocessorIOInputs {
    // The poll.

    /** Whether the latest poll was answered. */
    public boolean answered = false;

    /** Why the latest poll wasn't answered (no connection, no answer in time), or empty. */
    public String error = "";

    /** Whether any poll has been answered since the robot program started. */
    public boolean everAnswered = false;

    /** How old the latest answer is, on the robot's clock, in seconds; 0 before the first. */
    public double ageSeconds = 0;

    /**
     * When the latest answer was given, on the robot's clock, in nanoseconds: halfway between
     * asking and hearing. With {@link #agentMonotonicNanos}, it places the coprocessor's journal on
     * the match's timeline.
     */
    public long answeredNanos = 0;

    // The stamp: which image it runs, and as which computer.

    public String name = "";
    public int team = 0;

    /** Its address as stamped, such as {@code 10.0.0.11}. */
    public String address = "";

    public String board = "";

    /** The cameras its image was stamped to run: its role. */
    public String[] role = new String[] {};

    public String release = "";
    public String recipeHash = "";
    public String photonVisionVersion = "";

    /** The hash of the settings its image was stamped with; empty when none were. */
    public String stampedSettingsHash = "";

    /** The hash of PhotonVision's settings now, as the agent reads them. */
    public String settingsHash = "";

    /** The MAC of the board that answered. */
    public String mac = "";

    /**
     * The MACs that took turns answering at its address lately: two boards at one address. Empty
     * while one board answers, or after one replaced another.
     */
    public String[] macsTakingTurns = new String[] {};

    // This boot, and the agent's clock.

    public String bootId = "";

    /** The agent's monotonic clock as it answered, in nanoseconds since its boot. */
    public long agentMonotonicNanos = 0;

    public double uptimeSeconds = 0;

    /** Whether the boot before this one shut down cleanly, and whether that's known. */
    public boolean lastShutdownKnown = false;

    public boolean lastShutdownClean = true;

    /** What it booted from, such as {@code /dev/nvme0n1p2}, and whether it's read-only. */
    public String rootDevice = "";

    public boolean rootReadOnly = false;

    // Load and heat.

    /** Each core's busy time, in percent, by core number. */
    public double[] coreBusyPercent = new double[] {};

    /** The busiest big core's busy time, in percent: PhotonVision runs on the big cores. */
    public double bigCoreBusyPercent = 0;

    /**
     * The big cores' clock now, the most they're allowed now, and the most they can run, in MHz.
     * Allowed less than they can, they're throttled (by heat, or power).
     */
    public double bigCoreMhz = 0;

    public double bigCoreLimitMhz = 0;
    public double bigCoreMaxMhz = 0;

    /**
     * Each thermal zone: its name, its temperature, and its first passive trip point (where the
     * kernel slows the cores), in °C; NaN for a zone with none.
     */
    public String[] zones = new String[] {};

    public double[] zoneCelsius = new double[] {};
    public double[] zoneTripCelsius = new double[] {};

    // PhotonVision, and the cameras.

    /** PhotonVision's service as systemd says, such as {@code active/running}. */
    public String photonVisionState = "";

    /** How often systemd has restarted PhotonVision this boot. */
    public int photonVisionRestarts = 0;

    /** The cameras in its role that are plugged in where PhotonVision's settings expect them. */
    public String[] camerasPresent = new String[] {};

    /** Every USB camera plugged in: its port, product, and speed. */
    public String[] usbCameras = new String[] {};

    // The journal, the drive, and what the agent couldn't read.

    /** Errors in the journal this boot: USB, UVC, the filesystem, memory, heat, and the rest. */
    public int journalErrors = 0;

    /**
     * Those errors by what they're about, each kind with its count: {@code usb}, {@code uvc},
     * {@code filesystem}, {@code oom}, {@code thermal}, and {@code error} for the rest.
     */
    public String[] journalCategories = new String[] {};

    public long[] journalCounts = new long[] {};

    /** The latest few, oldest first. */
    public String[] latestErrors = new String[] {};

    /** Whether it has an NVMe drive the agent reads, and that drive's state. */
    public boolean driveFound = false;

    public double driveCelsius = 0;

    /** How much of its rated endurance the drive has used, in percent. */
    public int drivePercentUsed = 0;

    /** Power cuts the drive has counted, over its life. */
    public long unsafeShutdowns = 0;

    /** What the agent couldn't read, each saying what and why. */
    public String[] problems = new String[] {};

    // Powering down, once the robot asked it to.

    /** Whether the robot asked it to power down. */
    public boolean powerDownAsked = false;

    /**
     * Whether its agent took the request (202, a request already under way included): only then can
     * it count as gone.
     */
    public boolean powerDownAccepted = false;

    /**
     * How its agent answered: "accepted", "refused: answered 404", or why there was no answer;
     * empty until it answered.
     */
    public String powerDownAnswer = "";

    /**
     * Whether its ports have been tried since it was asked, and whether its agent's port and
     * PhotonVision's (5800) still accept a connection: when neither does, it's gone.
     */
    public boolean portsTried = false;

    public boolean agentPortOpen = false;
    public boolean photonVisionPortOpen = false;
  }

  /** Reads the latest answer, and how old it is: never waits. */
  default void updateInputs(CoprocessorIOInputs inputs) {}

  /**
   * Asks it to power down ({@code POST /v1/shutdown}), again each poll period until its agent takes
   * the request (or refuses, or the power-down timeout passes), then tries its ports until it's
   * gone, on a thread of the IO's own: never waits. Asked again, it starts again.
   */
  default void powerDown() {}

  /** Stops asking it to power down, and watching its ports: where it stood stays in the inputs. */
  default void stopPowerDown() {}

  /**
   * A settings backup as it arrives from the agent, to pass on without holding it whole: close it
   * once read. Asked once more, a second on, when the agent is busy with another such request.
   *
   * @param length its length, or -1 when the agent didn't give it
   * @param body what it holds
   */
  record Backup(long length, InputStream body) {}

  /**
   * Its settings backup ({@code /v1/settings.zip}): a zip laid out like the repository's folder for
   * the computer. Fetched now, on the caller's thread.
   *
   * @throws IOException saying why there's none
   */
  default Backup settingsBackup() throws IOException {
    throw new IOException("there's no coprocessor to ask here");
  }

  /** Where in its journal a page is. */
  enum JournalPlace {
    /** The latest entries. */
    LATEST,
    /** The entries just after a cursor. */
    AFTER,
    /** The entries just before a cursor. */
    BEFORE,
    /** The first entries of its current boot. */
    BOOT
  }

  /**
   * A page of its journal ({@code /v1/journal}), at most 500 entries, oldest first. Fetched now, on
   * the caller's thread; asked once more, a second on, when the agent is busy with another such
   * request.
   *
   * @param place where the page is
   * @param cursor the cursor it's after or before; empty for the latest or the boot's start
   * @param priority that priority and worse: 0 (emergency) to 7 (debug)
   * @param unit one of the units the agent serves ({@code AgentApi.JOURNAL_UNITS}); empty for all
   * @throws IOException saying why there's none: {@link AgentHttp.Answered} with 503 when the agent
   *     is still busy
   */
  default JournalPage journal(JournalPlace place, String cursor, int priority, String unit)
      throws IOException {
    throw new IOException("there's no coprocessor to ask here");
  }

  @Override
  default void close() {}
}
