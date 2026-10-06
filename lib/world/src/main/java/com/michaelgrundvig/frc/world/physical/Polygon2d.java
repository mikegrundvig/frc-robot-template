package com.michaelgrundvig.frc.world.physical;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.shape.Rectangle2d;

/**
 * A polygon, in meters: where something stands on the floor, seen from above, or a shape's outline
 * in its own x-y plane. Its corners run counterclockwise seen from above (from +z), so its inside
 * is on the left of each edge, and its edges never cross. Edge {@code i} runs from corner {@code i}
 * to the next, and the last edge back to the first corner.
 *
 * @param corners in order, counterclockwise seen from above
 */
public record Polygon2d(List<Translation2d> corners) {
  /** How close to an edge a point counts as on it: a nanometer, as WPILib compares points. */
  private static final double ON_EDGE_METERS = 1e-9;

  public Polygon2d {
    corners = List.copyOf(corners);
    if (corners.size() < 3) {
      throw new IllegalArgumentException(
          "a polygon needs three corners or more, not " + corners.size());
    }
    for (int i = 0; i < corners.size(); i++) {
      Translation2d corner = corners.get(i);
      if (!Double.isFinite(corner.getX()) || !Double.isFinite(corner.getY())) {
        throw new IllegalArgumentException("corner " + i + " isn't a point: " + corner);
      }
    }
    for (int i = 0; i < corners.size(); i++) {
      if (!(corners.get(i).getDistance(corners.get((i + 1) % corners.size())) > 0)) {
        throw new IllegalArgumentException(
            "corners " + i + " and " + ((i + 1) % corners.size()) + " are the same point");
      }
    }
    double area = signedArea(corners);
    if (area < 0) {
      throw new IllegalArgumentException(
          "list a polygon's corners counterclockwise, seen from above");
    }
    if (!(area > 0)) {
      throw new IllegalArgumentException("a polygon needs some area: its corners are in a line");
    }
    String crossing = crossingEdges(corners);
    if (crossing != null) {
      throw new IllegalArgumentException(crossing);
    }
  }

  /**
   * A rectangle centered on a pose, its sides along and across the pose's heading.
   *
   * @param xLength along the heading
   * @param yWidth across it
   */
  public static Polygon2d rectangle(Pose2d center, double xLength, double yWidth) {
    if (!(xLength > 0) || !(yWidth > 0) || Double.isInfinite(xLength + yWidth)) {
      throw new IllegalArgumentException(
          "a rectangle's sides must be positive: " + xLength + " by " + yWidth);
    }
    List<Translation2d> corners = new ArrayList<>();
    double[][] signs = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    for (double[] sign : signs) {
      corners.add(
          center
              .getTranslation()
              .plus(
                  new Translation2d(sign[0] * xLength / 2, sign[1] * yWidth / 2)
                      .rotateBy(center.getRotation())));
    }
    return new Polygon2d(corners);
  }

  /**
   * The rectangle between two opposite corners, its sides along the axes: an area of the field
   * between two lines across it and two along it.
   */
  public static Polygon2d rectangle(Translation2d corner, Translation2d opposite) {
    double x0 = Math.min(corner.getX(), opposite.getX());
    double x1 = Math.max(corner.getX(), opposite.getX());
    double y0 = Math.min(corner.getY(), opposite.getY());
    double y1 = Math.max(corner.getY(), opposite.getY());
    if (!(x1 > x0) || !(y1 > y0)) {
      throw new IllegalArgumentException(
          "a rectangle's sides must be positive: from " + corner + " to " + opposite);
    }
    return new Polygon2d(
        List.of(
            new Translation2d(x0, y0),
            new Translation2d(x1, y0),
            new Translation2d(x1, y1),
            new Translation2d(x0, y1)));
  }

  /** WPILib's rectangle, as a polygon. */
  public static Polygon2d of(Rectangle2d rectangle) {
    return rectangle(rectangle.getCenter(), rectangle.getXWidth(), rectangle.getYWidth());
  }

  /**
   * The smallest convex polygon around some points: what a shape covers, seen from above. Corners
   * in a line along its edge are left out.
   *
   * @throws IllegalArgumentException if the points are all on one line, which encloses nothing
   */
  public static Polygon2d hull(Collection<Translation2d> points) {
    List<Translation2d> sorted = new ArrayList<>(points);
    sorted.sort(
        Comparator.comparingDouble(Translation2d::getX).thenComparingDouble(Translation2d::getY));
    // Andrew's monotone chain: the lower hull left to right, then the upper right to left.
    List<Translation2d> hull = new ArrayList<>();
    for (int pass = 0; pass < 2; pass++) {
      int start = hull.size();
      for (int k = 0; k < sorted.size(); k++) {
        Translation2d point = sorted.get(pass == 0 ? k : sorted.size() - 1 - k);
        while (hull.size() >= start + 2
            && hull.get(hull.size() - 1)
                    .minus(hull.get(hull.size() - 2))
                    .cross(point.minus(hull.get(hull.size() - 1)))
                <= 0) {
          hull.remove(hull.size() - 1);
        }
        hull.add(point);
      }
      hull.remove(hull.size() - 1); // the next pass starts where this one ended
    }
    if (hull.size() < 3) {
      throw new IllegalArgumentException("points all on one line enclose nothing");
    }
    return new Polygon2d(hull);
  }

  /** A corner by its index, counting around: the corner after the last is the first. */
  public Translation2d corner(int index) {
    return corners.get(Math.floorMod(index, corners.size()));
  }

  /** Square meters. */
  public double area() {
    return signedArea(corners);
  }

  /** Its center of area. */
  public Translation2d centroid() {
    double x = 0;
    double y = 0;
    for (int i = 0; i < corners.size(); i++) {
      Translation2d a = corners.get(i);
      Translation2d b = corner(i + 1);
      double cross = a.cross(b);
      x += (a.getX() + b.getX()) * cross;
      y += (a.getY() + b.getY()) * cross;
    }
    double area = area();
    return new Translation2d(x / (6 * area), y / (6 * area));
  }

  /** Whether every corner turns left, or goes straight on: no corner points in. */
  public boolean convex() {
    for (int i = 0; i < corners.size(); i++) {
      if (corner(i + 1).minus(corner(i)).cross(corner(i + 2).minus(corner(i + 1))) < 0) {
        return false;
      }
    }
    return true;
  }

  /** Whether a point is inside the polygon or on its edge. */
  public boolean contains(Translation2d point) {
    boolean inside = false;
    for (int i = 0; i < corners.size(); i++) {
      Translation2d a = corners.get(i);
      Translation2d b = corner(i + 1);
      if (fromSegment(point, a, b) <= ON_EDGE_METERS) {
        return true;
      }
      // Count the edges a ray from the point toward +x crosses: odd is inside.
      if ((a.getY() > point.getY()) != (b.getY() > point.getY())) {
        double x =
            a.getX() + (b.getX() - a.getX()) * (point.getY() - a.getY()) / (b.getY() - a.getY());
        if (point.getX() < x) {
          inside = !inside;
        }
      }
    }
    return inside;
  }

  /**
   * Whether it and another polygon share a point: one inside the other, or their edges crossing or
   * touching.
   */
  public boolean touches(Polygon2d other) {
    for (Translation2d corner : corners) {
      if (other.contains(corner)) {
        return true;
      }
    }
    for (Translation2d corner : other.corners) {
      if (contains(corner)) {
        return true;
      }
    }
    for (int i = 0; i < corners.size(); i++) {
      for (int j = 0; j < other.corners.size(); j++) {
        if (intersect(corners.get(i), corner(i + 1), other.corners.get(j), other.corner(j + 1))) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * Whether the straight line from one point to another touches the polygon: an end inside it or on
   * its edge, or the line crossing or touching an edge.
   */
  public boolean touches(Translation2d from, Translation2d to) {
    if (contains(from) || contains(to)) {
      return true;
    }
    for (int i = 0; i < corners.size(); i++) {
      if (intersect(from, to, corners.get(i), corner(i + 1))) {
        return true;
      }
    }
    return false;
  }

  /** How far a point is from the polygon, in meters: zero inside it or on its edge. */
  public double distance(Translation2d point) {
    if (contains(point)) {
      return 0;
    }
    double nearest = Double.POSITIVE_INFINITY;
    for (int i = 0; i < corners.size(); i++) {
      nearest = Math.min(nearest, fromSegment(point, corners.get(i), corner(i + 1)));
    }
    return nearest;
  }

  /**
   * How far a point inside the polygon goes, heading one way, before it reaches the edge: zero for
   * a point outside it, or on its edge heading out.
   */
  public double reach(Translation2d from, Rotation2d way) {
    if (!contains(from)) {
      return 0;
    }
    Translation2d heading = new Translation2d(way.getCos(), way.getSin());
    double nearest = Double.POSITIVE_INFINITY;
    for (int i = 0; i < corners.size(); i++) {
      Translation2d a = corners.get(i);
      Translation2d edge = corner(i + 1).minus(a);
      double turn = heading.cross(edge);
      if (turn == 0) {
        continue; // heading along the edge, which it never reaches
      }
      // from + t heading = a + s edge, for t ahead and s along the edge.
      Translation2d toEdge = a.minus(from);
      double t = toEdge.cross(edge) / turn;
      double s = toEdge.cross(heading) / turn;
      if (t > ON_EDGE_METERS && s >= -ON_EDGE_METERS && s <= 1 + ON_EDGE_METERS) {
        nearest = Math.min(nearest, t);
      }
    }
    return Double.isInfinite(nearest) ? 0 : nearest;
  }

  /**
   * Its area cut into triangles of its corners, each by its corners' indices and counterclockwise
   * seen from above. A convex polygon is cut into a fan from its first corner.
   */
  public List<TriangleMesh.Triangle> triangles() {
    // Ear clipping: cut off a corner that turns left and holds no other corner, until three remain.
    List<Integer> left = new ArrayList<>();
    for (int i = 0; i < corners.size(); i++) {
      left.add(i);
    }
    List<TriangleMesh.Triangle> triangles = new ArrayList<>();
    int at = 1;
    int stalled = 0;
    while (left.size() > 3) {
      int n = left.size();
      at = Math.floorMod(at, n);
      int previous = left.get(Math.floorMod(at - 1, n));
      int corner = left.get(at);
      int next = left.get((at + 1) % n);
      Translation2d in = corners.get(corner).minus(corners.get(previous));
      Translation2d out = corners.get(next).minus(corners.get(corner));
      double turn = in.cross(out);
      if (Math.abs(turn) <= 1e-12 * in.getNorm() * out.getNorm()) {
        left.remove(at); // straight on: nothing to cut off
        stalled = 0;
      } else if (turn > 0 && holdsNone(previous, corner, next, left)) {
        triangles.add(new TriangleMesh.Triangle(previous, corner, next));
        left.remove(at);
        stalled = 0;
      } else {
        at++;
        if (++stalled > n) {
          throw new IllegalStateException("couldn't cut " + this + " into triangles");
        }
      }
    }
    Translation2d a = corners.get(left.get(0));
    if (corners.get(left.get(1)).minus(a).cross(corners.get(left.get(2)).minus(a)) > 0) {
      triangles.add(new TriangleMesh.Triangle(left.get(0), left.get(1), left.get(2)));
    }
    return List.copyOf(triangles);
  }

  /** Whether the triangle of three corners holds none of the others still left, edges included. */
  private boolean holdsNone(int a, int b, int c, List<Integer> left) {
    for (int other : left) {
      if (other != a
          && other != b
          && other != c
          && inTriangle(corners.get(other), corners.get(a), corners.get(b), corners.get(c))) {
        return false;
      }
    }
    return true;
  }

  private static boolean inTriangle(
      Translation2d point, Translation2d a, Translation2d b, Translation2d c) {
    return b.minus(a).cross(point.minus(a)) >= 0
        && c.minus(b).cross(point.minus(b)) >= 0
        && a.minus(c).cross(point.minus(c)) >= 0;
  }

  /** How far a point is from the segment between two others. */
  private static double fromSegment(Translation2d point, Translation2d a, Translation2d b) {
    Translation2d edge = b.minus(a);
    double along = point.minus(a).dot(edge) / edge.getSquaredNorm();
    return point.getDistance(a.plus(edge.times(Math.max(0, Math.min(1, along)))));
  }

  /** Twice the shoelace sum, halved: positive when the corners run counterclockwise. */
  private static double signedArea(List<Translation2d> corners) {
    double sum = 0;
    for (int i = 0; i < corners.size(); i++) {
      sum += corners.get(i).cross(corners.get((i + 1) % corners.size()));
    }
    return sum / 2;
  }

  /** What's wrong with the edges, if any cross or double back; null when none do. */
  private static @Nullable String crossingEdges(List<Translation2d> corners) {
    int n = corners.size();
    for (int i = 0; i < n; i++) {
      Translation2d a = corners.get(i);
      Translation2d b = corners.get((i + 1) % n);
      Translation2d c = corners.get((i + 2) % n);
      if (b.minus(a).cross(c.minus(b)) == 0 && b.minus(a).dot(c.minus(b)) < 0) {
        return "the polygon doubles back at corner " + ((i + 1) % n);
      }
      for (int j = i + 2; j < n; j++) {
        if (i == 0 && j == n - 1) {
          continue; // the last edge meets the first at corner 0
        }
        if (intersect(a, b, corners.get(j), corners.get((j + 1) % n))) {
          return "the polygon's edges " + i + " and " + j + " cross";
        }
      }
    }
    return null;
  }

  /** Whether two segments meet, touching included. */
  private static boolean intersect(
      Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
    double abc = b.minus(a).cross(c.minus(a));
    double abd = b.minus(a).cross(d.minus(a));
    double cda = d.minus(c).cross(a.minus(c));
    double cdb = d.minus(c).cross(b.minus(c));
    if (((abc > 0 && abd < 0) || (abc < 0 && abd > 0))
        && ((cda > 0 && cdb < 0) || (cda < 0 && cdb > 0))) {
      return true;
    }
    return (abc == 0 && onSegment(c, a, b))
        || (abd == 0 && onSegment(d, a, b))
        || (cda == 0 && onSegment(a, c, d))
        || (cdb == 0 && onSegment(b, c, d));
  }

  /** Whether a point known to be on a segment's line is between its ends. */
  private static boolean onSegment(Translation2d point, Translation2d a, Translation2d b) {
    return point.getX() >= Math.min(a.getX(), b.getX())
        && point.getX() <= Math.max(a.getX(), b.getX())
        && point.getY() >= Math.min(a.getY(), b.getY())
        && point.getY() <= Math.max(a.getY(), b.getY());
  }
}
