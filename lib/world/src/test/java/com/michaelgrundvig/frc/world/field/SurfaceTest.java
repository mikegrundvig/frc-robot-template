package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Every kind of surface, set at odd angles, and each one's images under both symmetries: its two
 * descriptions agree. Its triangles, met by a line cast straight down, give its heights within a
 * millimeter; its normals are its heights' slopes; its steepest slope is the steepest anywhere on
 * it; and its crossings are where it says.
 */
class SurfaceTest {
  private static final double LENGTH = 16;
  private static final double WIDTH = 8;

  /** How closely the triangles must give the heights: a millimeter, as simulation.md asks. */
  private static final double MESH_METERS = 0.001;

  /** The step for the slopes of the heights, by finite differences. */
  private static final double STEP = 1e-4;

  static Stream<Arguments> surfaces() {
    List<Surface> written =
        List.of(
            new Bump("Bump", new Translation2d(4, 2), new Translation2d(5, 4.5), 0.1654, 1.128),
            new Ramp("Ramp", new Translation2d(2, 6), new Translation2d(3.2, 6.7), 1.5, 0.3),
            platform(),
            new Platform(
                "Box",
                Polygon2d.rectangle(new Pose2d(6, 6, Rotation2d.fromDegrees(10)), 1, 0.6),
                0.4,
                List.of(
                    Platform.Side.EDGE,
                    Platform.Side.EDGE,
                    Platform.Side.EDGE,
                    Platform.Side.EDGE)),
            hill(),
            tent());
    List<Arguments> all = new ArrayList<>();
    for (Surface surface : written) {
      all.add(Arguments.of(surface.name(), surface));
      for (Symmetry symmetry : Symmetry.values()) {
        all.add(
            Arguments.of(surface.name() + ", " + symmetry, symmetry.apply(surface, LENGTH, WIDTH)));
      }
    }
    return all.stream();
  }

  /** A pentagon at an angle, ramps down three of its sides and edges on two, 25 cm up. */
  static Platform platform() {
    List<Translation2d> corners = new ArrayList<>();
    for (int i = 0; i < 5; i++) {
      double angle = Math.toRadians(17 + 72 * i);
      corners.add(new Translation2d(9 + 0.8 * Math.cos(angle), 3 + 0.8 * Math.sin(angle)));
    }
    return new Platform(
        "Platform",
        new Polygon2d(corners),
        0.25,
        List.of(
            Platform.Side.ramp(0.6),
            Platform.Side.EDGE,
            Platform.Side.ramp(0.4),
            Platform.Side.EDGE,
            Platform.Side.ramp(0.5)));
  }

  /** A smooth hill from CAD, as a grid of triangles: 30 cm at its top, falling away all round. */
  static MeshSurface hill() {
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    int n = 12;
    int[][] at = new int[n + 1][n + 1];
    for (int i = 0; i <= n; i++) {
      for (int j = 0; j <= n; j++) {
        double x = -1.5 + 3.0 * i / n;
        double y = -1.5 + 3.0 * j / n;
        at[i][j] = mesh.corner(12 + x, 6 + y, 0.3 * Math.exp(-(x * x + y * y)));
      }
    }
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        mesh.quad(at[i][j], at[i + 1][j], at[i + 1][j + 1], at[i][j + 1]);
      }
    }
    return new MeshSurface("Hill", mesh.build(), List.of());
  }

  /** A ridge from CAD, closed at its ends and underneath: a crest along y at x = 14.5. */
  static MeshSurface tent() {
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    int frontLeft = mesh.corner(14, 1, 0);
    int frontRight = mesh.corner(15, 1, 0);
    int frontTop = mesh.corner(14.5, 1, 0.2);
    int backLeft = mesh.corner(14, 2.5, 0);
    int backRight = mesh.corner(15, 2.5, 0);
    int backTop = mesh.corner(14.5, 2.5, 0.2);
    TriangleMesh tent =
        mesh.quad(frontLeft, frontTop, backTop, backLeft)
            .quad(frontTop, frontRight, backRight, backTop)
            .triangle(frontLeft, frontRight, frontTop)
            .triangle(backLeft, backTop, backRight)
            .quad(frontLeft, backLeft, backRight, frontRight)
            .build();
    return new MeshSurface(
        "Tent",
        tent,
        List.of(
            new Crossing(
                "Tent crest",
                new Translation2d(14.5, 1),
                new Translation2d(14.5, 2.5),
                Crossing.Kind.CREST)));
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("surfaces")
  void itsTrianglesGiveItsHeightsWithinAMillimeter(String name, Surface surface) {
    for (Translation2d point : around(surface)) {
      double meshed = surface.mesh().castDown(point).map(TriangleMesh.Hit::height).orElse(0.0);
      assertThat(meshed)
          .as("%s at %s", name, point)
          .isCloseTo(surface.heightAt(point), within(MESH_METERS));
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("surfaces")
  void itsNormalIsItsHeightsSlope(String name, Surface surface) {
    int compared = 0;
    int raised = 0;
    for (Translation2d point : around(surface)) {
      Vector3 normal = surface.normalAt(point);
      assertThat(normal.length()).as("a unit vector").isCloseTo(1, within(1e-12));
      assertThat(normal.z()).as("facing up").isPositive();
      if (surface.heightAt(point) > 0) {
        raised++;
      }
      Vector3 slopes = slopes(surface, point);
      if (slopes == null) {
        continue; // on a fold, where the slope changes
      }
      compared++;
      Vector3 expected = new Vector3(-slopes.x(), -slopes.y(), 1).unit();
      assertThat(normal.x()).as("%s at %s", name, point).isCloseTo(expected.x(), within(1e-6));
      assertThat(normal.y()).as("%s at %s", name, point).isCloseTo(expected.y(), within(1e-6));
      assertThat(normal.z()).as("%s at %s", name, point).isCloseTo(expected.z(), within(1e-6));
      if (surface.heightAt(point) > 0) {
        Vector3 meshed = surface.mesh().castDown(point).orElseThrow().normal();
        assertThat(meshed.minus(normal).length())
            .as("%s's triangle at %s", name, point)
            .isLessThan(1e-6);
      }
    }
    assertThat(compared).as("points away from any fold").isGreaterThan(around(surface).size() / 2);
    assertThat(raised).as("points on it").isGreaterThan(20);
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("surfaces")
  void itsSteepestSlopeIsTheSteepestAnywhereOnIt(String name, Surface surface) {
    double steepest = 0;
    for (Translation2d point : around(surface)) {
      Vector3 normal = surface.normalAt(point);
      steepest = Math.max(steepest, Math.acos(Math.min(1, normal.z())));
    }
    assertThat(steepest).isLessThanOrEqualTo(surface.steepestSlopeRadians() + 1e-9);
    assertThat(steepest).isCloseTo(surface.steepestSlopeRadians(), within(Math.toRadians(0.5)));
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("surfaces")
  void itsCrossingsAreWhereItSays(String name, Surface surface) {
    for (Crossing crossing : surface.crossings()) {
      Translation2d middle = crossing.middle();
      Translation2d across = new Translation2d(0.05, crossing.across());
      double here = surface.heightAt(middle);
      assertThat(here).as("%s is on raised ground", crossing.name()).isPositive();
      assertThat(surface.covers(middle, 0)).isTrue();
      switch (crossing.kind()) {
        case CREST -> {
          assertThat(surface.heightAt(middle.plus(across))).isLessThan(here);
          assertThat(surface.heightAt(middle.minus(across))).isLessThan(here);
        }
        case TOP ->
            assertThat(surface.heightAt(middle.minus(across)))
                .as("%s: the way across is the way up", crossing.name())
                .isLessThan(here);
      }
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("surfaces")
  void itsFootprintHoldsAllOfIt(String name, Surface surface) {
    for (Translation2d point : around(surface)) {
      if (surface.heightAt(point) > 0) {
        assertThat(surface.footprint().contains(point)).as("%s at %s", name, point).isTrue();
        assertThat(surface.covers(point, 0)).isTrue();
      }
    }
    for (var corner : surface.mesh().corners()) {
      assertThat(surface.footprint().distance(new Translation2d(corner.x(), corner.y())))
          .isLessThan(1e-9);
    }
  }

  /**
   * Its triangles face out: its ground up (a mesh's underside aside), and each wall away from its
   * footprint.
   */
  @ParameterizedTest(name = "{0}")
  @MethodSource("surfaces")
  void itsTrianglesFaceOut(String name, Surface surface) {
    TriangleMesh mesh = surface.mesh();
    for (TriangleMesh.Triangle triangle : mesh.triangles()) {
      Vector3 normal = mesh.normal(triangle);
      Vector3 middle =
          mesh.corners(triangle).stream().reduce(Vector3.ZERO, Vector3::plus).times(1.0 / 3);
      Translation2d onFloor = new Translation2d(middle.x(), middle.y());
      if (mesh.vertical(triangle)) {
        Translation2d out = new Translation2d(normal.x(), normal.y()).times(0.01);
        assertThat(surface.footprint().contains(onFloor.plus(out)))
            .as("%s: a wall at %s faces out", name, onFloor)
            .isFalse();
        assertThat(surface.footprint().contains(onFloor.minus(out))).isTrue();
      } else if (!(surface instanceof MeshSurface) || middle.z() > 0) {
        assertThat(normal.z()).as("%s: ground at %s faces up", name, middle).isPositive();
      }
    }
  }

  /** Each surface's image has the same heights at the images of the same points. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("surfaces")
  void itsImagesHaveItsShape(String name, Surface surface) {
    for (Symmetry symmetry : Symmetry.values()) {
      Surface image = symmetry.apply(surface, LENGTH, WIDTH);
      Surface back = symmetry.apply(image, LENGTH, WIDTH);
      assertThat(image.name()).isEqualTo(surface.name());
      assertThat(image.footprint().area()).isCloseTo(surface.footprint().area(), within(1e-9));
      assertThat(image.steepestSlopeRadians())
          .isCloseTo(surface.steepestSlopeRadians(), within(1e-9));
      assertThat(image.crossings()).hasSameSizeAs(surface.crossings());
      for (Translation2d point : around(surface)) {
        Translation2d there = symmetry.apply(point, LENGTH, WIDTH);
        assertThat(image.heightAt(there)).isCloseTo(surface.heightAt(point), within(1e-9));
        assertThat(back.heightAt(point)).isCloseTo(surface.heightAt(point), within(1e-9));
      }
    }
  }

  /**
   * The slopes of the heights at a point, along x and y, by finite differences; null where the
   * slope changes within a step, on a fold.
   */
  private static @Nullable Vector3 slopes(Surface surface, Translation2d point) {
    double here = surface.heightAt(point);
    double east = surface.heightAt(point.plus(new Translation2d(STEP, 0))) - here;
    double west = here - surface.heightAt(point.minus(new Translation2d(STEP, 0)));
    double north = surface.heightAt(point.plus(new Translation2d(0, STEP))) - here;
    double south = here - surface.heightAt(point.minus(new Translation2d(0, STEP)));
    if (Math.abs(east - west) > 1e-11 || Math.abs(north - south) > 1e-11) {
      return null;
    }
    return new Vector3((east + west) / (2 * STEP), (north + south) / (2 * STEP), 0);
  }

  /** A grid of points over the surface and a little around it, off the grid of its corners. */
  private static List<Translation2d> around(Surface surface) {
    double minX = Double.POSITIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY;
    double maxX = Double.NEGATIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    for (Translation2d corner : surface.footprint().corners()) {
      minX = Math.min(minX, corner.getX());
      minY = Math.min(minY, corner.getY());
      maxX = Math.max(maxX, corner.getX());
      maxY = Math.max(maxY, corner.getY());
    }
    List<Translation2d> points = new ArrayList<>();
    int n = 41;
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        // An irrational nudge keeps the points off the corners' own grid.
        double x = minX - 0.2 + (maxX - minX + 0.4) * (i + 0.5 * Math.sqrt(2) - 0.5) / (n - 1);
        double y = minY - 0.2 + (maxY - minY + 0.4) * (j + 0.5 * Math.sqrt(3) - 0.5) / (n - 1);
        points.add(new Translation2d(x, y));
      }
    }
    return points;
  }
}
