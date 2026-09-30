package frc.robot.lib.localization;

import frc.robot.lib.config.ConfigRecord;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.util.MathUtil;

/**
 * Watches the IMU for the robot crossing raised terrain (a bump, a ramp, a platform's edge): up,
 * over the crest, down, and level again. Autos sequence on it (1678 drove over REBUILT's bumps
 * until the pitch turned downhill past 10°, then until it was flat within 5°, then waited for the
 * cameras to re-localize); the log shows each crossing.
 *
 * <p>It reads the slope along the direction the robot moves, so a bump taken at an angle, sideways,
 * or backwards reads the same as one taken head on. Backing down the way it came is no crossing.
 *
 * <p>It also marks the crest: halfway between the last steep reading going up and the first going
 * down, the moment the robot's center was over the top of a symmetric bump, driven at a steady
 * speed. A crest is a landmark.
 */
public final class TerrainCrossing {
  /** Where the robot is on the terrain. */
  public enum Phase {
    LEVEL,
    CLIMBING,
    DESCENDING
  }

  private final double steepRadians;
  private final double levelRadians;
  private final long levelNanos;
  private final long topNanos;
  private Phase phase = Phase.LEVEL;
  private boolean climbed;
  private long levelSince = Long.MIN_VALUE;
  private int crossings;
  private long lastSteepClimb = Long.MIN_VALUE;
  private double climbDirection = Double.NaN;
  private @Nullable Crest crest;

  /**
   * The top of a crossing.
   *
   * @param timestampNanos when the robot's center was over the crest
   * @param directionRadians which way it was going, in the field frame; NaN if unknown
   */
  public record Crest(long timestampNanos, double directionRadians) {}

  /**
   * What counts as climbing, level, and one feature.
   *
   * @param steepDegrees a slope at least this steep, along the way the robot moves, is climbing or
   *     descending (1678 used 10°)
   * @param levelDegrees a tilt within this is level (1678 used 5°)
   * @param levelSeconds how long the robot must stay level to count as off the terrain
   * @param topSeconds how long the robot may take between going up and coming down the far side of
   *     one feature (a flat top): longer is two features, or a stop on top
   */
  @ConfigRecord
  public record Settings(
      double steepDegrees, double levelDegrees, double levelSeconds, double topSeconds) {
    /** 1678's thresholds, and a second and a half to cross a flat top. */
    public static final Settings DEFAULTS =
        builder().steepDegrees(10).levelDegrees(5).levelSeconds(0.1).topSeconds(1.5).build();

    public Settings {
      if (!(levelDegrees > 0)
          || !(steepDegrees > levelDegrees)
          || !(steepDegrees < 90)
          || !(levelSeconds >= 0)
          || !(topSeconds > 0)
          || Double.isInfinite(levelSeconds + topSeconds)) {
        throw new IllegalArgumentException(
            "steep must be steeper than level, both positive, and the times positive");
      }
    }

    /** Starts building the settings, one named value at a time. */
    public static TerrainCrossingSettingsBuilder.SteepDegreesStage builder() {
      return TerrainCrossingSettingsBuilder.builder();
    }
  }

  public TerrainCrossing(Settings settings) {
    steepRadians = Math.toRadians(settings.steepDegrees());
    levelRadians = Math.toRadians(settings.levelDegrees());
    levelNanos = Timestamps.fromSeconds(settings.levelSeconds());
    topNanos = Timestamps.fromSeconds(settings.topSeconds());
  }

  /**
   * The slope under the robot along the way it moves, from the IMU's pitch and roll (WPILib's
   * convention: positive pitch is nose down, positive roll lifts the left side).
   *
   * @param directionRadians the way the robot moves, in its own frame: 0 forward, π/2 left
   * @return positive uphill
   */
  public static double climb(double pitchRadians, double rollRadians, double directionRadians) {
    // The height the robot's own direction of travel gains per unit along it.
    double rise =
        -Math.cos(directionRadians) * Math.sin(pitchRadians)
            + Math.sin(directionRadians) * Math.cos(pitchRadians) * Math.sin(rollRadians);
    return Math.asin(Math.clamp(rise, -1.0, 1.0));
  }

  /**
   * A reading, once a loop.
   *
   * @param climbRadians the slope along the way the robot moves ({@link #climb}); zero while it
   *     stands still, when there's no way it moves
   * @param tiltRadians how far the robot tilts from level, whichever way
   * @param directionRadians which way the robot moves, in the field frame; NaN while it stands
   *     still
   * @return true on the reading a crossing completes: up, over, down, and level again
   */
  public boolean update(
      long timestampNanos, double climbRadians, double tiltRadians, double directionRadians) {
    crest = null;
    boolean level = tiltRadians <= levelRadians;
    if (!level) {
      levelSince = Long.MIN_VALUE;
    } else if (levelSince == Long.MIN_VALUE) {
      levelSince = timestampNanos;
    }
    boolean settled = level && timestampNanos - levelSince >= levelNanos;
    if (climbRadians >= steepRadians) {
      lastSteepClimb = timestampNanos;
      climbDirection = directionRadians;
    }
    switch (phase) {
      case LEVEL -> {
        if (climbRadians >= steepRadians) {
          phase = Phase.CLIMBING;
          climbed = true;
        } else if (climbRadians <= -steepRadians) {
          // Down the far side of a flat top just climbed, or off a platform: not a crossing.
          phase = Phase.DESCENDING;
          climbed = crestPassed(timestampNanos, directionRadians);
        }
      }
      case CLIMBING -> {
        if (climbRadians <= -steepRadians) {
          phase = Phase.DESCENDING;
          climbed = crestPassed(timestampNanos, directionRadians);
        } else if (settled) {
          phase = Phase.LEVEL; // backed off, or up onto a flat top
          climbed = false;
        }
      }
      case DESCENDING -> {
        if (climbRadians >= steepRadians) {
          phase = Phase.CLIMBING; // another rise before reaching level
          climbed = true;
        } else if (settled) {
          phase = Phase.LEVEL;
          if (climbed) {
            climbed = false;
            crossings++;
            return true;
          }
        }
      }
    }
    return false;
  }

  /**
   * On the first steep reading down after a climb the same way, the robot has passed the crest:
   * halfway between that climb's last steep reading and this one. Down the way it came up is no
   * crest.
   */
  private boolean crestPassed(long timestampNanos, double directionRadians) {
    if (lastSteepClimb == Long.MIN_VALUE || timestampNanos - lastSteepClimb > topNanos) {
      return false;
    }
    boolean sameWay =
        Double.isNaN(directionRadians)
            || Double.isNaN(climbDirection)
            || Math.abs(MathUtil.angleModulus(directionRadians - climbDirection)) < Math.PI / 2;
    if (!sameWay) {
      return false;
    }
    crest = new Crest(lastSteepClimb + (timestampNanos - lastSteepClimb) / 2, climbDirection);
    lastSteepClimb = Long.MIN_VALUE;
    return true;
  }

  public Phase phase() {
    return phase;
  }

  /** The crest this reading found, if it found one. */
  public Optional<Crest> crest() {
    return Optional.ofNullable(crest);
  }

  /** How many crossings have completed. */
  public int crossings() {
    return crossings;
  }
}
