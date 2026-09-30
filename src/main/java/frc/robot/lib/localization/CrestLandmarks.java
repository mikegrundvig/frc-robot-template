package frc.robot.lib.localization;

import frc.robot.lib.field.Bump;
import frc.robot.lib.field.Terrain;
import java.util.Optional;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.MathUtil;

/**
 * Turns a crest the robot passed into a landmark: the bump it was on, from where the estimate had
 * the robot then, and the line that bump's crest puts the robot's center on.
 */
public final class CrestLandmarks {
  /** A crest counts only if the robot crossed it within this of straight across. */
  private static final double MAX_ANGLE_FROM_ACROSS = Math.toRadians(60);

  private final Terrain terrain;
  private final double stdDevMeters;
  private final double reachMeters;

  /**
   * @param terrain the field's bumps
   * @param stdDevMeters how exactly a crest places the robot's center across it: the crest's
   *     timing, and how far the bump and the robot are from symmetric
   * @param reachMeters how far off its footprint the estimate may put the robot and still name the
   *     bump; farther, it's a crest the terrain doesn't know
   */
  public CrestLandmarks(Terrain terrain, double stdDevMeters, double reachMeters) {
    if (!(stdDevMeters > 0) || !(reachMeters >= 0)) {
      throw new IllegalArgumentException("a positive spread, and a reach of zero or more");
    }
    this.terrain = terrain;
    this.stdDevMeters = stdDevMeters;
    this.reachMeters = reachMeters;
  }

  /**
   * The landmark a crest gives, if a bump's crest is within reach and the robot crossed it rather
   * than driving along it.
   *
   * @param crest when the robot's center was over a crest, and which way it went
   * @param estimated where the estimate had the robot then
   */
  public Optional<LandmarkEvent> landmark(TerrainCrossing.Crest crest, Translation2d estimated) {
    Optional<Bump> under = terrain.under(estimated, reachMeters);
    if (under.isEmpty()) {
      return Optional.empty();
    }
    Bump bump = under.get();
    double way = crest.directionRadians();
    if (!Double.isNaN(way)
        && Math.abs(Math.cos(MathUtil.angleModulus(way - bump.across().getRadians())))
            < Math.cos(MAX_ANGLE_FROM_ACROSS)) {
      return Optional.empty();
    }
    return Optional.of(
        new LandmarkEvent(
            crest.timestampNanos(), bump.name(), bump.crestStart(), bump.across(), stdDevMeters));
  }
}
