package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Translation2d;

/**
 * Raised ground of any shape, given as triangles: the escape hatch for what {@link Bump}, {@link
 * Ramp} and {@link Platform} can't say, such as a shape from the field's CAD. Its heights are found
 * by casting a line straight down onto the triangles: the highest one over a point is the ground
 * there. Its footprint is the outline of its triangles seen from above, made convex; its crossings
 * are given, since triangles don't say where a robot crossing them can tell.
 *
 * <p>The triangles over each part of the field are looked up in a grid made once, so a CAD mesh of
 * thousands of triangles costs a few of them a lookup.
 */
public final class MeshSurface implements Surface {
  /** About how many triangles stand over each square of the lookup grid. */
  private static final int TRIANGLES_PER_CELL = 4;

  private final String name;
  private final Material material;
  private final TriangleMesh mesh;
  private final List<Crossing> crossings;
  private final Polygon2d footprint;

  // The lookup grid over the footprint's bounds: each cell lists the triangles over it, walls left
  // out, since a line straight down only grazes a wall.
  private final double minX;
  private final double minY;
  private final double cellMeters;
  private final int columns;
  private final int rows;
  private final int[][] cells;

  /**
   * A surface covered in the field's carpet.
   *
   * @param name for the log
   * @param mesh its triangles, in the field frame, counterclockwise seen from outside
   * @param crossings the lines on it that place a robot crossing them, if any
   * @throws IllegalArgumentException if it has nothing to stand on (only walls), or a crossing runs
   *     off it
   */
  public MeshSurface(String name, TriangleMesh mesh, List<Crossing> crossings) {
    this(name, mesh, crossings, Material.CARPET);
  }

  /**
   * @param name for the log
   * @param mesh its triangles, in the field frame, counterclockwise seen from outside
   * @param crossings the lines on it that place a robot crossing them, if any
   * @param material what it's covered in
   * @throws IllegalArgumentException if it has nothing to stand on (only walls), or a crossing runs
   *     off it
   */
  public MeshSurface(String name, TriangleMesh mesh, List<Crossing> crossings, Material material) {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a surface needs a name");
    }
    this.name = name;
    this.material = material;
    this.mesh = mesh;
    this.crossings = List.copyOf(crossings);
    List<Integer> ground = new ArrayList<>();
    List<Translation2d> seen = new ArrayList<>();
    for (int i = 0; i < mesh.triangles().size(); i++) {
      TriangleMesh.Triangle triangle = mesh.triangles().get(i);
      if (!mesh.vertical(triangle)) {
        ground.add(i);
        for (Vector3 corner : mesh.corners(triangle)) {
          seen.add(new Translation2d(corner.x(), corner.y()));
        }
      }
    }
    if (ground.isEmpty()) {
      throw new IllegalArgumentException(name + ": it has nothing to stand on, only walls");
    }
    footprint = Polygon2d.hull(seen);
    for (Crossing crossing : this.crossings) {
      if (!footprint.contains(crossing.start()) || !footprint.contains(crossing.end())) {
        throw new IllegalArgumentException(name + ": " + crossing.name() + " runs off it");
      }
    }
    double maxX = Double.NEGATIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    double lowX = Double.POSITIVE_INFINITY;
    double lowY = Double.POSITIVE_INFINITY;
    for (Translation2d corner : footprint.corners()) {
      lowX = Math.min(lowX, corner.getX());
      lowY = Math.min(lowY, corner.getY());
      maxX = Math.max(maxX, corner.getX());
      maxY = Math.max(maxY, corner.getY());
    }
    minX = lowX;
    minY = lowY;
    int wanted = Math.max(1, ground.size() / TRIANGLES_PER_CELL);
    cellMeters = Math.sqrt((maxX - minX) * (maxY - minY) / wanted);
    columns = Math.max(1, (int) Math.ceil((maxX - minX) / cellMeters));
    rows = Math.max(1, (int) Math.ceil((maxY - minY) / cellMeters));
    List<List<Integer>> grid = new ArrayList<>();
    for (int i = 0; i < columns * rows; i++) {
      grid.add(new ArrayList<>());
    }
    for (int index : ground) {
      double fromX = Double.POSITIVE_INFINITY;
      double fromY = Double.POSITIVE_INFINITY;
      double toX = Double.NEGATIVE_INFINITY;
      double toY = Double.NEGATIVE_INFINITY;
      for (Vector3 corner : mesh.corners(mesh.triangles().get(index))) {
        fromX = Math.min(fromX, corner.x());
        fromY = Math.min(fromY, corner.y());
        toX = Math.max(toX, corner.x());
        toY = Math.max(toY, corner.y());
      }
      for (int column = column(fromX); column <= column(toX); column++) {
        for (int row = row(fromY); row <= row(toY); row++) {
          grid.get(column * rows + row).add(index);
        }
      }
    }
    cells = new int[grid.size()][];
    for (int i = 0; i < grid.size(); i++) {
      cells[i] = grid.get(i).stream().mapToInt(Integer::intValue).toArray();
    }
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public Material material() {
    return material;
  }

  /** Its triangles' outline seen from above, made convex. */
  @Override
  public Polygon2d footprint() {
    return footprint;
  }

  /** The highest triangle over the point, or the floor where there's none. */
  @Override
  public double heightAt(Translation2d point) {
    return castDown(point).map(TriangleMesh.Hit::height).orElse(0.0);
  }

  /** The highest triangle's, turned to face up; straight up where there's none. */
  @Override
  public Vector3 normalAt(Translation2d point) {
    return castDown(point).map(TriangleMesh.Hit::normal).orElse(Vector3.UP);
  }

  @Override
  public TriangleMesh mesh() {
    return mesh;
  }

  @Override
  public List<Crossing> crossings() {
    return crossings;
  }

  /** Its steepest triangle's slope, walls aside. */
  @Override
  public double steepestSlopeRadians() {
    double steepest = 0;
    for (TriangleMesh.Triangle triangle : mesh.triangles()) {
      if (!mesh.vertical(triangle)) {
        Vector3 normal = mesh.normal(triangle);
        steepest =
            Math.max(
                steepest, Math.atan2(Math.hypot(normal.x(), normal.y()), Math.abs(normal.z())));
      }
    }
    return steepest;
  }

  @Override
  public String toString() {
    return "MeshSurface[" + name + ", " + mesh.triangles().size() + " triangles]";
  }

  /**
   * What a line straight down meets: the highest of the triangles over the point's cell. A point
   * off the grid looks in the nearest cell, whose triangles it misses.
   */
  private Optional<TriangleMesh.Hit> castDown(Translation2d point) {
    TriangleMesh.Triangle best = null;
    double height = Double.NEGATIVE_INFINITY;
    for (int index : cells[column(point.getX()) * rows + row(point.getY())]) {
      TriangleMesh.Triangle triangle = mesh.triangles().get(index);
      OptionalDouble over = mesh.heightOver(triangle, point);
      if (over.isPresent() && over.getAsDouble() > height) {
        height = over.getAsDouble();
        best = triangle;
      }
    }
    return best == null ? Optional.empty() : Optional.of(mesh.hit(best, height));
  }

  private int column(double x) {
    return Math.clamp((long) Math.floor((x - minX) / cellMeters), 0, columns - 1);
  }

  private int row(double y) {
    return Math.clamp((long) Math.floor((y - minY) / cellMeters), 0, rows - 1);
  }
}
