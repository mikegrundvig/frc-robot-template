package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import java.util.List;
import java.util.Random;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

class MeshSurfaceTest {
  /**
   * Its grid of triangles finds the same ground as testing every triangle, on a mesh of thousands
   * of them, ragged and overlapping: a CAD export's.
   */
  @Test
  void itsLookupFindsWhatEveryTriangleWould() {
    Random random = new Random(2611);
    TriangleMesh.Builder builder = TriangleMesh.builder();
    for (int i = 0; i < 2000; i++) {
      double x = 3 + 4 * random.nextDouble();
      double y = 2 + 3 * random.nextDouble();
      int a = builder.corner(x, y, 0.5 * random.nextDouble());
      int b = builder.corner(x + 0.3 * random.nextDouble() + 0.01, y, 0.5 * random.nextDouble());
      int c = builder.corner(x, y + 0.3 * random.nextDouble() + 0.01, 0.5 * random.nextDouble());
      builder.triangle(a, b, c);
    }
    TriangleMesh mesh = builder.build();
    MeshSurface surface = new MeshSurface("Rubble", mesh, List.of());

    for (int i = 0; i < 2000; i++) {
      Translation2d point =
          new Translation2d(2.5 + 5 * random.nextDouble(), 1.5 + 4 * random.nextDouble());
      assertThat(surface.heightAt(point))
          .as("at %s", point)
          .isEqualTo(mesh.castDown(point).map(TriangleMesh.Hit::height).orElse(0.0));
    }
    assertThat(surface.toString()).contains("Rubble", "2000 triangles");
  }

  @Test
  void itsCrossingsAreGivenAndMustBeOnIt() {
    TriangleMesh slab = SurfaceTest.tent().mesh();
    Crossing off =
        new Crossing(
            "Off", new Translation2d(14.5, 1), new Translation2d(14.5, 4), Crossing.Kind.CREST);

    assertThatThrownBy(() -> new MeshSurface("Tent", slab, List.of(off)))
        .hasMessageContaining("Off runs off it");
    assertThat(new MeshSurface("Tent", slab, List.of()).crossings()).isEmpty();
    assertThat(SurfaceTest.tent().heightAt(new Translation2d(14.5, 1.5)))
        .isCloseTo(0.2, within(1e-12));
  }

  @Test
  void aMeshSurfaceNeedsGroundToStandOn() {
    TriangleMesh.Builder wall = TriangleMesh.builder();
    int a = wall.corner(0, 0, 0);
    int b = wall.corner(1, 0, 0);
    int c = wall.corner(1, 0, 1);

    assertThatThrownBy(() -> new MeshSurface("Wall", wall.triangle(a, b, c).build(), List.of()))
        .hasMessageContaining("nothing to stand on");
    assertThatThrownBy(() -> new MeshSurface(" ", SurfaceTest.tent().mesh(), List.of()))
        .hasMessageContaining("name");
  }
}
