package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Translation2d;

/**
 * The field's raised ground, on both alliances' halves: its {@link Surface}s, and the floor
 * everywhere else. The simulator rides the robot over it, odometry tells a tilt the ground explains
 * from one it doesn't, and a crossing of a crest becomes a landmark.
 */
public final class Terrain {
  /** A field with nothing to drive over. */
  public static final Terrain FLAT = new Terrain(List.of());

  private final List<Surface> surfaces;

  public Terrain(List<? extends Surface> surfaces) {
    this.surfaces = List.copyOf(surfaces);
  }

  public List<Surface> surfaces() {
    return surfaces;
  }

  /** Every surface's crossings: the lines that place a robot crossing them. */
  public List<Crossing> crossings() {
    List<Crossing> crossings = new ArrayList<>();
    for (Surface surface : surfaces) {
      crossings.addAll(surface.crossings());
    }
    return List.copyOf(crossings);
  }

  /** How high the ground is at a point: the highest surface there, or the floor. */
  public double heightAt(Translation2d point) {
    double height = 0;
    for (Surface surface : surfaces) {
      height = Math.max(height, surface.heightAt(point));
    }
    return height;
  }

  /** Which way the ground faces at a point: the highest surface's way there, or straight up. */
  public Vector3 normalAt(Translation2d point) {
    double height = 0;
    Vector3 normal = Vector3.UP;
    for (Surface surface : surfaces) {
      double here = surface.heightAt(point);
      if (here > height) {
        height = here;
        normal = surface.normalAt(point);
      }
    }
    return normal;
  }

  /**
   * The surface whose footprint is at or nearest a point: the one to explain a tilt there.
   *
   * @param marginMeters how far outside a footprint still counts, for a point that may be off
   */
  public Optional<Surface> under(Translation2d point, double marginMeters) {
    Surface nearest = null;
    double nearestDistance = Double.POSITIVE_INFINITY;
    for (Surface surface : surfaces) {
      if (surface.covers(point, marginMeters)) {
        double distance = surface.footprint().distance(point);
        if (distance < nearestDistance) {
          nearest = surface;
          nearestDistance = distance;
        }
      }
    }
    return Optional.ofNullable(nearest);
  }
}
