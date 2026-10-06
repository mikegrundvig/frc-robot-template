package com.michaelgrundvig.frc.world.physical;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.shape.Rectangle2d;

class Polygon2dTest {
  /** An L, 3 m by 2 m, its notch cut from the top right: concave at corner 3. */
  private static final Polygon2d L = polygon(0, 0, 3, 0, 3, 1, 1, 1, 1, 2, 0, 2);

  private static final Polygon2d SQUARE = polygon(0, 0, 2, 0, 2, 2, 0, 2);

  @Test
  void itKnowsItsAreaCenterAndShape() {
    assertThat(SQUARE.area()).isEqualTo(4);
    assertThat(SQUARE.centroid()).isEqualTo(new Translation2d(1, 1));
    assertThat(SQUARE.convex()).isTrue();
    assertThat(L.area()).isCloseTo(4, within(1e-12));
    assertThat(L.convex()).isFalse();
    // Two rectangles: 3 x 1 centered at (1.5, 0.5), and 1 x 1 at (0.5, 1.5).
    assertThat(L.centroid().getX()).isCloseTo((3 * 1.5 + 0.5) / 4, within(1e-12));
    assertThat(L.centroid().getY()).isCloseTo((3 * 0.5 + 1.5) / 4, within(1e-12));
    assertThat(L.corner(6)).as("counting around").isEqualTo(L.corner(0));
    assertThat(L.corner(-1)).isEqualTo(new Translation2d(0, 2));
  }

  @Test
  void aPointIsInsideOnTheEdgeOrSomeDistanceOff() {
    assertThat(L.contains(new Translation2d(0.5, 1.5))).isTrue();
    assertThat(L.contains(new Translation2d(2, 0.5))).isTrue();
    assertThat(L.contains(new Translation2d(2, 1.5))).as("in the notch").isFalse();
    assertThat(L.contains(new Translation2d(3, 0.5))).as("on an edge").isTrue();
    assertThat(L.contains(new Translation2d(1, 1))).as("at the inside corner").isTrue();
    assertThat(L.distance(new Translation2d(0.5, 0.5))).isZero();
    assertThat(L.distance(new Translation2d(2, 1.5)))
        .as("from the notch")
        .isCloseTo(0.5, within(1e-12));
    assertThat(L.distance(new Translation2d(4, 3))).isCloseTo(Math.hypot(1, 2), within(1e-12));
  }

  @Test
  void aWayFromInsideReachesTheEdge() {
    Translation2d middle = new Translation2d(1, 1);
    assertThat(SQUARE.reach(middle, Rotation2d.ZERO)).isCloseTo(1, within(1e-12));
    assertThat(SQUARE.reach(middle, Rotation2d.fromDegrees(45)))
        .isCloseTo(Math.sqrt(2), within(1e-12));
    assertThat(L.reach(new Translation2d(0.5, 0.5), Rotation2d.ZERO)).isCloseTo(2.5, within(1e-12));
    assertThat(L.reach(new Translation2d(2, 0.5), Rotation2d.CCW_PI_2))
        .as("up into the notch, which is outside")
        .isCloseTo(0.5, within(1e-12));
    assertThat(SQUARE.reach(new Translation2d(3, 1), Rotation2d.PI)).as("from outside").isZero();
    assertThat(SQUARE.reach(new Translation2d(2, 1), Rotation2d.ZERO))
        .as("on the edge, heading out")
        .isZero();
  }

  @Test
  void itCutsIntoTrianglesThatCoverItExactly() {
    for (Polygon2d polygon : List.of(SQUARE, L, star())) {
      List<TriangleMesh.Triangle> triangles = polygon.triangles();
      assertThat(triangles).hasSize(polygon.corners().size() - 2);
      double area = 0;
      for (TriangleMesh.Triangle triangle : triangles) {
        Translation2d a = polygon.corners().get(triangle.a());
        double twice =
            polygon
                .corners()
                .get(triangle.b())
                .minus(a)
                .cross(polygon.corners().get(triangle.c()).minus(a));
        assertThat(twice).as("counterclockwise").isPositive();
        area += twice / 2;
      }
      assertThat(area).isCloseTo(polygon.area(), within(1e-9));
    }
    assertThat(SQUARE.triangles())
        .as("a convex polygon is a fan from its first corner")
        .containsExactly(new TriangleMesh.Triangle(0, 1, 2), new TriangleMesh.Triangle(0, 2, 3));
  }

  @Test
  void cornersInAStraightLineAddNoTriangle() {
    Polygon2d withAStraightCorner = polygon(0, 0, 1, 0, 2, 0, 2, 1, 0, 1);

    assertThat(withAStraightCorner.convex()).isTrue();
    double area = 0;
    for (TriangleMesh.Triangle triangle : withAStraightCorner.triangles()) {
      Translation2d a = withAStraightCorner.corners().get(triangle.a());
      area +=
          withAStraightCorner
                  .corners()
                  .get(triangle.b())
                  .minus(a)
                  .cross(withAStraightCorner.corners().get(triangle.c()).minus(a))
              / 2;
    }
    assertThat(area).isCloseTo(2, within(1e-12));
  }

  @Test
  void aRectangleTurnsWithItsHeading() {
    Polygon2d turned = Polygon2d.rectangle(new Pose2d(5, 5, Rotation2d.CCW_PI_2), 4, 2);

    assertThat(turned.area()).isCloseTo(8, within(1e-12));
    assertThat(turned.contains(new Translation2d(5, 6.9))).as("its length now along y").isTrue();
    assertThat(turned.contains(new Translation2d(5.9, 5))).isTrue();
    assertThat(turned.contains(new Translation2d(6.1, 5))).isFalse();
    Polygon2d wpilibs =
        Polygon2d.of(new Rectangle2d(new Translation2d(1, 1), new Translation2d(3, 2)));
    assertThat(wpilibs.area()).isCloseTo(2, within(1e-12));
    assertThat(wpilibs.centroid().getX()).isCloseTo(2, within(1e-12));
  }

  @Test
  void theHullIsTheConvexOutlineOfPoints() {
    List<Translation2d> points = new ArrayList<>(L.corners());
    points.add(new Translation2d(1, 0.5)); // inside
    points.add(new Translation2d(1.5, 0)); // on an edge

    Polygon2d hull = Polygon2d.hull(points);

    assertThat(hull.convex()).isTrue();
    assertThat(hull.area()).isCloseTo(5, within(1e-12));
    assertThat(hull.corners()).hasSize(5);
    assertThatThrownBy(
            () ->
                Polygon2d.hull(
                    List.of(
                        new Translation2d(0, 0), new Translation2d(1, 1), new Translation2d(2, 2))))
        .hasMessageContaining("one line");
  }

  @Test
  void twoPolygonsTouchWhereTheyShareAPoint() {
    assertThat(SQUARE.touches(polygon(1, 1, 3, 1, 3, 3, 1, 3))).as("overlapping").isTrue();
    assertThat(SQUARE.touches(polygon(0.5, 0.5, 1, 0.5, 1, 1))).as("one inside").isTrue();
    assertThat(polygon(0.5, 0.5, 1, 0.5, 1, 1).touches(SQUARE)).as("the other way").isTrue();
    assertThat(SQUARE.touches(polygon(2, 2, 3, 2, 3, 3))).as("corner to corner").isTrue();
    assertThat(SQUARE.touches(polygon(2, 0.5, 3, 0.5, 3, 1))).as("along an edge").isTrue();
    // A bar across the square, its corners all outside it, and the square's all outside the bar.
    assertThat(SQUARE.touches(polygon(-1, 0.5, 3, 0.5, 3, 1.5, -1, 1.5))).as("across").isTrue();
    assertThat(SQUARE.touches(polygon(2.1, 0, 3, 0, 3, 1))).as("apart").isFalse();
    assertThat(L.touches(polygon(1.5, 1.5, 2.5, 1.5, 2.5, 2.5, 1.5, 2.5)))
        .as("in the L's notch")
        .isFalse();
  }

  @Test
  void aRectangleBetweenTwoCornersRunsAlongTheAxes() {
    Polygon2d rectangle = Polygon2d.rectangle(new Translation2d(3, 1), new Translation2d(1, 2));

    assertThat(rectangle.corners())
        .containsExactly(
            new Translation2d(1, 1),
            new Translation2d(3, 1),
            new Translation2d(3, 2),
            new Translation2d(1, 2));
    assertThatThrownBy(() -> Polygon2d.rectangle(new Translation2d(1, 1), new Translation2d(1, 2)))
        .hasMessageContaining("positive");
  }

  @Test
  void aLineTouchesAPolygonWhereItCrossesOrReachesIt() {
    assertThat(SQUARE.touches(new Translation2d(-1, 1), new Translation2d(3, 1)))
        .as("across")
        .isTrue();
    assertThat(SQUARE.touches(new Translation2d(1, 1), new Translation2d(1, 1)))
        .as("a point in it")
        .isTrue();
    assertThat(SQUARE.touches(new Translation2d(-1, 1), new Translation2d(0, 1)))
        .as("to its edge")
        .isTrue();
    assertThat(SQUARE.touches(new Translation2d(-1, 1), new Translation2d(-0.1, 1)))
        .as("short")
        .isFalse();
    assertThat(L.touches(new Translation2d(1.5, 1.5), new Translation2d(2.5, 1.5)))
        .as("within the L's notch")
        .isFalse();
    assertThat(L.touches(new Translation2d(2, 1.5), new Translation2d(2, 0.5)))
        .as("into the L")
        .isTrue();
  }

  @Test
  void aPolygonMustBeOne() {
    assertThatThrownBy(() -> polygon(0, 0, 1, 0)).hasMessageContaining("three corners");
    assertThatThrownBy(() -> polygon(0, 0, 0, 1, 1, 0)).hasMessageContaining("counterclockwise");
    assertThatThrownBy(() -> polygon(0, 0, 1, 0, 2, 0)).hasMessageContaining("some area");
    assertThatThrownBy(() -> polygon(0, 0, 1, 0, 1, 0, 0, 1)).hasMessageContaining("same point");
    assertThatThrownBy(() -> polygon(0, 0, 2, 0, 0, 1, 2, 1, 1, 3))
        .as("a bow tie with a roof")
        .hasMessageContaining("cross");
    assertThatThrownBy(() -> polygon(0, 0, 2, 0, 1, 0, 1, 1)).hasMessageContaining("doubles back");
    assertThatThrownBy(() -> polygon(0, 0, Double.NaN, 0, 1, 1))
        .hasMessageContaining("isn't a point");
    assertThatThrownBy(() -> Polygon2d.rectangle(Pose2d.ZERO, 0, 1))
        .hasMessageContaining("positive");
  }

  /** A five-pointed star: concave at every other corner. */
  private static Polygon2d star() {
    List<Translation2d> corners = new ArrayList<>();
    for (int i = 0; i < 10; i++) {
      double radius = i % 2 == 0 ? 2 : 0.8;
      double angle = Math.PI / 2 + i * Math.PI / 5;
      corners.add(new Translation2d(radius * Math.cos(angle), radius * Math.sin(angle)));
    }
    return new Polygon2d(corners);
  }

  static Polygon2d polygon(double... xy) {
    List<Translation2d> corners = new ArrayList<>();
    for (int i = 0; i < xy.length; i += 2) {
      corners.add(new Translation2d(xy[i], xy[i + 1]));
    }
    return new Polygon2d(corners);
  }
}
