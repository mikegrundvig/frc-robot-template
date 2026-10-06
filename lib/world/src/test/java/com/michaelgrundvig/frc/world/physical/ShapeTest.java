package com.michaelgrundvig.frc.world.physical;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/** Shapes: where each kind is from its pose, and how far a point is from it. */
class ShapeTest {
  private static final Material ALUMINUM = Material.builder().name("aluminum").build();

  /** A 2 x 1 x 0.5 m box centered at (1, 1, 1), turned a quarter turn about z. */
  private static final Shape.Box BOX =
      Shape.Box.builder()
          .pose(new Pose3d(1, 1, 1, new Rotation3d(0, 0, Math.PI / 2)))
          .size(new Translation3d(2, 1, 0.5))
          .material(ALUMINUM)
          .build();

  /** The points that stand in for a shape where only its extent matters: what reaches farthest. */
  @Test
  void aShapesPointsBoundIt() {
    List<Translation3d> corners = BOX.points();

    assertThat(corners).hasSize(8);
    assertThat(corners.stream().mapToDouble(Translation3d::getX).max().orElseThrow())
        .isCloseTo(1.5, within(1e-12));
    assertThat(corners.stream().mapToDouble(Translation3d::getY).max().orElseThrow())
        .isCloseTo(2, within(1e-12));
  }

  @Test
  void aBoxIsCenteredOnItsPoseAlongItsAxes() {
    // Turned a quarter turn, its 2 m length runs along y.
    assertThat(BOX.contains(new Translation3d(1, 1.9, 1))).isTrue();
    assertThat(BOX.contains(new Translation3d(1.9, 1, 1))).isFalse();
    assertThat(BOX.distance(new Translation3d(1, 1, 2))).isCloseTo(0.75, within(1e-12));
    assertThat(BOX.distance(new Translation3d(1.5 + 0.3, 2 + 0.4, 1)))
        .isCloseTo(0.5, within(1e-12));
    assertThat(BOX.center()).isEqualTo(new Translation3d(1, 1, 1));
    assertThat(BOX.material()).isEqualTo(ALUMINUM);
  }

  @Test
  void aShapeMustBeOne() {
    assertThatThrownBy(
            () ->
                Shape.Box.builder()
                    .pose(Pose3d.ZERO)
                    .size(new Translation3d(1, 0, 1))
                    .material(ALUMINUM)
                    .build())
        .hasMessageContaining("Box.size")
        .hasMessageContaining("positive along each axis");
    assertThatThrownBy(
            () ->
                Shape.Box.builder()
                    .pose(new Pose3d(Double.NaN, 0, 0, Rotation3d.ZERO))
                    .size(new Translation3d(1, 1, 1))
                    .material(ALUMINUM)
                    .build())
        .hasMessageContaining("Box.pose");
    assertThatThrownBy(() -> Material.builder().name(" ").build())
        .hasMessageContaining("Material.name");
    assertThat(Material.CARPET.name()).isEqualTo("carpet");
  }
}
