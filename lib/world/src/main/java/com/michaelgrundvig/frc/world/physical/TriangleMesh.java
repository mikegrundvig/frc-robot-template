package com.michaelgrundvig.frc.world.physical;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.UnaryOperator;
import org.wpilib.math.geometry.Translation2d;

/**
 * Triangles in 3D sharing corners by index: a shape exactly, as a physics engine, a 3D view, or a
 * glTF file takes it. Each triangle's corners run counterclockwise seen from outside, so its
 * normal, by the right-hand rule, points out of the shape: up, for ground.
 *
 * @param corners every corner, in meters, in the mesh's frame
 * @param triangles each by its three corners' indices
 */
public record TriangleMesh(List<Vector3> corners, List<Triangle> triangles) {
  /** No triangles at all. */
  public static final TriangleMesh EMPTY = new TriangleMesh(List.of(), List.of());

  /**
   * A triangle, by its corners' indices in the mesh, counterclockwise seen from outside.
   *
   * @param a its first corner
   * @param b the next, counterclockwise
   * @param c the last
   */
  public record Triangle(int a, int b, int c) {}

  /**
   * Where a line straight down meets the mesh.
   *
   * @param height how high, in meters
   * @param normal the face it meets there, a unit vector turned to face up
   */
  public record Hit(double height, Vector3 normal) {}

  public TriangleMesh {
    corners = List.copyOf(corners);
    triangles = List.copyOf(triangles);
    for (int i = 0; i < corners.size(); i++) {
      Vector3 corner = corners.get(i);
      if (!Double.isFinite(corner.x() + corner.y() + corner.z())) {
        throw new IllegalArgumentException("corner " + i + " isn't a point: " + corner);
      }
    }
    for (int i = 0; i < triangles.size(); i++) {
      Triangle triangle = triangles.get(i);
      for (int corner : new int[] {triangle.a(), triangle.b(), triangle.c()}) {
        if (corner < 0 || corner >= corners.size()) {
          throw new IllegalArgumentException(
              "triangle " + i + " names corner " + corner + " of " + corners.size());
        }
      }
      Vector3 a = corners.get(triangle.a());
      if (!(corners.get(triangle.b()).minus(a).cross(corners.get(triangle.c()).minus(a)).length()
          > 0)) {
        throw new IllegalArgumentException("triangle " + i + " has no area");
      }
    }
  }

  /** A mesh built a corner and a triangle at a time. */
  public static Builder builder() {
    return new Builder();
  }

  /** A triangle's three corners, in its order. */
  public List<Vector3> corners(Triangle triangle) {
    return List.of(corners.get(triangle.a()), corners.get(triangle.b()), corners.get(triangle.c()));
  }

  /** Which way a triangle faces, a unit vector out of the shape. */
  public Vector3 normal(Triangle triangle) {
    Vector3 a = corners.get(triangle.a());
    return corners.get(triangle.b()).minus(a).cross(corners.get(triangle.c()).minus(a)).unit();
  }

  /**
   * Whether a triangle stands on edge, so that seen from above it's a line: a wall, which a line
   * straight down only grazes.
   */
  public boolean vertical(Triangle triangle) {
    Vector3 a = corners.get(triangle.a());
    Vector3 b = corners.get(triangle.b());
    Vector3 c = corners.get(triangle.c());
    double ux = b.x() - a.x();
    double uy = b.y() - a.y();
    double vx = c.x() - a.x();
    double vy = c.y() - a.y();
    return Math.abs(ux * vy - uy * vx) <= 1e-12 * (ux * ux + uy * uy + vx * vx + vy * vy);
  }

  /**
   * How high a line straight up and down through a point meets a triangle; empty if it passes
   * beside the triangle, or the triangle stands on edge (a wall), which it only grazes. A point on
   * the triangle's edge meets it.
   */
  public OptionalDouble heightOver(Triangle triangle, Translation2d point) {
    if (vertical(triangle)) {
      return OptionalDouble.empty();
    }
    Vector3 a = corners.get(triangle.a());
    Vector3 b = corners.get(triangle.b());
    Vector3 c = corners.get(triangle.c());
    double ux = b.x() - a.x();
    double uy = b.y() - a.y();
    double vx = c.x() - a.x();
    double vy = c.y() - a.y();
    double area = ux * vy - uy * vx; // twice the area seen from above, signed
    // The point as a + s (b - a) + t (c - a), seen from above.
    double px = point.getX() - a.x();
    double py = point.getY() - a.y();
    double s = (px * vy - py * vx) / area;
    double t = (ux * py - uy * px) / area;
    double edge = 1e-9;
    if (s < -edge || t < -edge || s + t > 1 + edge) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(a.z() + s * (b.z() - a.z()) + t * (c.z() - a.z()));
  }

  /**
   * What a line cast straight down through a point meets first: the highest triangle over the
   * point, passing walls by. Empty where it meets none.
   */
  public Optional<Hit> castDown(Translation2d point) {
    Triangle best = null;
    double height = Double.NEGATIVE_INFINITY;
    for (Triangle triangle : triangles) {
      OptionalDouble over = heightOver(triangle, point);
      if (over.isPresent() && over.getAsDouble() > height) {
        height = over.getAsDouble();
        best = triangle;
      }
    }
    return best == null ? Optional.empty() : Optional.of(hit(best, height));
  }

  /** A hit on a triangle at a height, its normal turned to face up. */
  public Hit hit(Triangle triangle, double height) {
    Vector3 normal = normal(triangle);
    return new Hit(height, normal.z() < 0 ? normal.times(-1) : normal);
  }

  /** Both meshes as one. */
  public TriangleMesh plus(TriangleMesh other) {
    List<Vector3> joined = new ArrayList<>(corners);
    joined.addAll(other.corners);
    List<Triangle> all = new ArrayList<>(triangles);
    int offset = corners.size();
    for (Triangle triangle : other.triangles) {
      all.add(new Triangle(triangle.a() + offset, triangle.b() + offset, triangle.c() + offset));
    }
    return new TriangleMesh(joined, all);
  }

  /**
   * The mesh with each corner moved, facing out still.
   *
   * @param move where each corner goes
   * @param mirrors whether the move is a mirror image, which turns counterclockwise into clockwise:
   *     each triangle's corners are then listed the other way round
   */
  public TriangleMesh map(UnaryOperator<Vector3> move, boolean mirrors) {
    List<Vector3> moved = corners.stream().map(move).toList();
    List<Triangle> turned =
        mirrors
            ? triangles.stream()
                .map(triangle -> new Triangle(triangle.a(), triangle.c(), triangle.b()))
                .toList()
            : triangles;
    return new TriangleMesh(moved, turned);
  }

  /** Builds a mesh a corner and a triangle at a time. */
  public static final class Builder {
    private final List<Vector3> corners = new ArrayList<>();
    private final List<Triangle> triangles = new ArrayList<>();

    private Builder() {}

    /** A corner, by its index: the first is 0. */
    public int corner(double x, double y, double z) {
      return corner(new Vector3(x, y, z));
    }

    /** A corner, by its index: the first is 0. */
    public int corner(Vector3 at) {
      corners.add(at);
      return corners.size() - 1;
    }

    /** A triangle between three corners, counterclockwise seen from outside. */
    public Builder triangle(int a, int b, int c) {
      triangles.add(new Triangle(a, b, c));
      return this;
    }

    /** A flat face of four corners, counterclockwise seen from outside, as two triangles. */
    public Builder quad(int a, int b, int c, int d) {
      return triangle(a, b, c).triangle(a, c, d);
    }

    public TriangleMesh build() {
      return new TriangleMesh(corners, triangles);
    }
  }
}
