package frc.robot.lib.field;

import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Translation2d;

/**
 * The field's raised features, on both alliances' halves; flat everywhere else. The simulator tilts
 * the robot by it, odometry tells a tilt the ground explains from one it doesn't, and a crossing of
 * a bump's crest becomes a landmark.
 */
public final class Terrain {
  /** A field with nothing to drive over. */
  public static final Terrain FLAT = new Terrain(List.of());

  private final List<Bump> bumps;

  public Terrain(List<Bump> bumps) {
    this.bumps = List.copyOf(bumps);
  }

  public List<Bump> bumps() {
    return bumps;
  }

  /** How high the ground is at a point: the highest bump there, or the floor. */
  public double heightAt(Translation2d point) {
    double height = 0;
    for (Bump bump : bumps) {
      height = Math.max(height, bump.heightAt(point));
    }
    return height;
  }

  /**
   * The bump whose footprint is at or near a point: the one to explain a tilt there.
   *
   * @param marginMeters how far outside a footprint still counts, for a point that may be off
   */
  public Optional<Bump> under(Translation2d point, double marginMeters) {
    Bump nearest = null;
    for (Bump bump : bumps) {
      if (bump.covers(point, marginMeters)
          && (nearest == null
              || Math.abs(bump.acrossOffset(point)) < Math.abs(nearest.acrossOffset(point)))) {
        nearest = bump;
      }
    }
    return Optional.ofNullable(nearest);
  }
}
