package frc.robot.lib.localization;

/**
 * Watches the IMU for the robot crossing raised terrain (a bump, a ramp, a platform's edge): up,
 * over the crest, down, and level again. Autos sequence on it (1678 drove over REBUILT's bumps
 * until the pitch turned downhill past 10°, then until it was flat within 5°, then waited for the
 * cameras to re-localize); the log shows each crossing.
 *
 * <p>It reads the slope along the direction the robot moves, so a bump taken at an angle, sideways,
 * or backwards reads the same as one taken head on.
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
  private Phase phase = Phase.LEVEL;
  private boolean climbed;
  private long levelSince = Long.MIN_VALUE;
  private int crossings;

  /**
   * @param steepRadians a slope at least this steep, along the way the robot moves, is climbing or
   *     descending (1678 used 10°)
   * @param levelRadians a tilt within this is level (1678 used 5°)
   * @param levelSeconds how long the robot must stay level to count as off the terrain
   */
  public TerrainCrossing(double steepRadians, double levelRadians, double levelSeconds) {
    if (!(levelRadians > 0) || !(steepRadians > levelRadians) || !(levelSeconds >= 0)) {
      throw new IllegalArgumentException(
          "steep must be steeper than level, and both positive; the level time zero or more");
    }
    this.steepRadians = steepRadians;
    this.levelRadians = levelRadians;
    levelNanos = Timestamps.fromSeconds(levelSeconds);
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
   * @return true on the reading a crossing completes: up, over, down, and level again
   */
  public boolean update(long timestampNanos, double climbRadians, double tiltRadians) {
    boolean level = tiltRadians <= levelRadians;
    if (!level) {
      levelSince = Long.MIN_VALUE;
    } else if (levelSince == Long.MIN_VALUE) {
      levelSince = timestampNanos;
    }
    boolean settled = level && timestampNanos - levelSince >= levelNanos;
    switch (phase) {
      case LEVEL -> {
        if (climbRadians >= steepRadians) {
          phase = Phase.CLIMBING;
          climbed = true;
        } else if (climbRadians <= -steepRadians) {
          phase = Phase.DESCENDING; // down off a platform: not a crossing on its own
          climbed = false;
        }
      }
      case CLIMBING -> {
        if (climbRadians <= -steepRadians) {
          phase = Phase.DESCENDING;
        } else if (settled) {
          phase = Phase.LEVEL; // backed off, or up onto a platform
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

  public Phase phase() {
    return phase;
  }

  /** How many crossings have completed. */
  public int crossings() {
    return crossings;
  }
}
