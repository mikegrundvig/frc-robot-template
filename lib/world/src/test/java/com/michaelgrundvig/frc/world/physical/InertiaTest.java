package com.michaelgrundvig.frc.world.physical;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Pounds;

import com.michaelgrundvig.frc.world.config.ConfigException;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/** Mass properties: how much a part weighs, where, and how hard it is to turn. */
class InertiaTest {
  @Test
  void aBoxsInertiaIsItsMassSpreadEvenly() {
    // A 60 kg robot, 0.8 m by 0.7 m by 0.5 m tall.
    Inertia box = Inertia.box(Kilograms.of(60), new Translation3d(0.8, 0.7, 0.5));

    assertThat(box.izz()).isCloseTo(60 * (0.64 + 0.49) / 12, within(1e-12));
    assertThat(box.ixx()).isCloseTo(60 * (0.49 + 0.25) / 12, within(1e-12));
    assertThat(box.iyy()).isCloseTo(60 * (0.64 + 0.25) / 12, within(1e-12));
    assertThat(box.ixy()).isZero();
    assertThat(box.ixz()).isZero();
    assertThat(box.iyz()).isZero();
  }

  @Test
  void aTurnedPartHasProductsOfInertia() {
    // A 1 kg bar, 1 m by 0.2 m by 0.2 m, turned 45° about z: its tensor's own entries.
    double across = (0.04 + 0.04) / 12;
    double along = (1 + 0.04) / 12;
    Inertia turned =
        Inertia.builder()
            .ixx((across + along) / 2)
            .iyy((across + along) / 2)
            .izz(along)
            .ixy((across - along) / 2)
            .ixz(0)
            .iyz(0)
            .build();

    assertThat(turned.ixy()).isNegative();
    assertThatThrownBy(
            () ->
                Inertia.builder()
                    .ixx((across + along) / 2)
                    .iyy((across + along) / 2)
                    .izz(along)
                    .ixy(0.1)
                    .ixz(0)
                    .iyz(0)
                    .build())
        .isInstanceOf(ConfigException.class)
        .hasMessageContaining("Inertia.ixy")
        .hasMessageContaining("no body turns that way");
  }

  @Test
  void anInertiaMustBeABodys() {
    assertThatThrownBy(() -> Inertia.builder().ixx(-1).iyy(1).izz(1).ixy(0).ixz(0).iyz(0).build())
        .hasMessageContaining("Inertia.ixx: -1 must be zero or more");
    assertThatThrownBy(() -> Inertia.builder().ixx(3).iyy(1).izz(1).ixy(0).ixz(0).iyz(0).build())
        .hasMessageContaining("each must be at most the sum of the other two");
    assertThatThrownBy(
            () -> Inertia.builder().ixx(1).iyy(1).izz(1).ixy(Double.NaN).ixz(0).iyz(0).build())
        .hasMessageContaining("Inertia.ixy");
  }

  @Test
  void massPropertiesSayWhereTheWeightIs() {
    MassProperties robot =
        MassProperties.builder()
            .mass(Pounds.of(135))
            .cg(new Translation3d(0.02, -0.01, 0.2))
            .inertia(Inertia.box(Pounds.of(135), new Translation3d(0.8, 0.7, 0.5)))
            .build();

    assertThat(robot.mass().in(Kilograms)).isCloseTo(61.235, within(1e-3));
    assertThat(robot.cg().getZ()).isEqualTo(0.2);
    assertThatThrownBy(
            () ->
                MassProperties.builder()
                    .mass(Kilograms.of(-1))
                    .cg(new Translation3d(0, 0, Double.NaN))
                    .inertia(robot.inertia())
                    .build())
        .hasMessageContaining("MassProperties.mass")
        .hasMessageContaining("must be zero or more");
    assertThatThrownBy(
            () ->
                MassProperties.builder()
                    .mass(Kilograms.of(1))
                    .cg(new Translation3d(0, 0, Double.NaN))
                    .inertia(robot.inertia())
                    .build())
        .hasMessageContaining("MassProperties.cg");
  }

  /**
   * What nobody has given is zero, not a guess: no mass, at the frame's origin, turning freely. It
   * moves, turns, adds and comes away as nothing; and a point mass, with no inertia of its own, is
   * a part too. Only a whole body must be solid.
   */
  @Test
  void noneKnownIsZeroAndAddsNothing() {
    MassProperties none = MassProperties.NONE;
    MassProperties seen =
        none.transformedBy(
            new Transform3d(new Translation3d(1, 2, 3), new Rotation3d(0.3, -0.2, 1.1)));
    MassProperties point =
        MassProperties.builder()
            .mass(Kilograms.of(2))
            .cg(new Translation3d(0.5, 0, 0))
            .inertia(Inertia.ZERO)
            .build();

    assertThat(none.isMassless()).isTrue();
    assertThat(none.inertia().isZero()).isTrue();
    assertThat(seen.isMassless()).isTrue();
    assertThat(seen.inertia().isZero()).isTrue();
    assertThat(MassProperties.sum(List.of(none, none))).isEqualTo(MassProperties.NONE);
    assertThat(MassProperties.sum(List.of(none, cube(0.5))).cg().getX()).isEqualTo(0.5);
    assertThat(cube(0.5).minus(none)).isEqualTo(cube(0.5));
    assertThat(point.inertia().rotated(new Rotation3d(0, 0, 1)).isZero()).isTrue();
    assertThat(Inertia.ZERO.whyNotSolid()).contains("turns too easily");
    assertThat(Inertia.box(Kilograms.of(1), new Translation3d(1, 1, 1)).whyNotSolid()).isNull();
    assertThat(Inertia.builder().ixx(0).iyy(1).izz(1).ixy(0).ixz(0).iyz(0).build().whyNotSolid())
        .as("a thin rod along x: a part, but no body")
        .isNotNull();
  }

  /** A bar along x, turned a quarter turn about z, lies along y: x and y trade places. */
  @Test
  void aTurnedInertiaIsTheSameBodyTurned() {
    Inertia bar = Inertia.box(Kilograms.of(1), new Translation3d(1, 0.2, 0.2));
    Inertia turned = bar.rotated(new Rotation3d(0, 0, Math.PI / 2));

    assertThat(turned.ixx()).isCloseTo(bar.iyy(), within(1e-12));
    assertThat(turned.iyy()).isCloseTo(bar.ixx(), within(1e-12));
    assertThat(turned.izz()).isCloseTo(bar.izz(), within(1e-12));
    assertThat(turned.ixy()).isCloseTo(0, within(1e-12));
    assertThat(bar.about(new Vector3(0, 0, 3))).as("an axis of any length").isEqualTo(bar.izz());
    assertThat(bar.about(new Vector3(1, 1, 0)))
        .isCloseTo((bar.ixx() + bar.iyy()) / 2, within(1e-12));
    // Turned 45 degrees, it's as hard to turn about x as the unturned bar about that diagonal.
    Inertia diagonal = bar.rotated(new Rotation3d(0, 0, Math.PI / 4));
    assertThat(diagonal.ixx()).isCloseTo(bar.about(new Vector3(1, -1, 0)), within(1e-12));
    assertThat(diagonal.ixy()).as("the tensor's own entry").isNegative();
  }

  /** Two cubes side by side are a box twice as long; take one away and the other is left. */
  @Test
  void partsAddAndComeApart() {
    MassProperties left = cube(-0.5);
    MassProperties right = cube(0.5);
    MassProperties both = MassProperties.sum(List.of(left, right));
    Inertia box = Inertia.box(Kilograms.of(2), new Translation3d(2, 1, 1));

    assertThat(both.mass().in(Kilograms)).isEqualTo(2);
    assertThat(both.cg().getNorm()).isZero();
    assertThat(both.inertia().ixx()).isCloseTo(box.ixx(), within(1e-12));
    assertThat(both.inertia().iyy()).isCloseTo(box.iyy(), within(1e-12));
    assertThat(both.inertia().izz()).isCloseTo(box.izz(), within(1e-12));
    MassProperties less = both.minus(right);
    assertThat(less.cg().getX()).isCloseTo(-0.5, within(1e-12));
    assertThat(less.inertia().izz()).isCloseTo(left.inertia().izz(), within(1e-12));
    assertThatThrownBy(() -> left.minus(both))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageStartingWith("nothing is left: 2 Kg taken from 1 Kg");
    assertThatThrownBy(() -> MassProperties.sum(List.of()))
        .isInstanceOf(IllegalArgumentException.class);
  }

  /** A part given in its joint's frame, seen from the robot's: moved, and turned with it. */
  @Test
  void aPartSeenFromAnotherFrame() {
    MassProperties bar =
        MassProperties.builder()
            .mass(Kilograms.of(1))
            .cg(new Translation3d(0.5, 0, 0))
            .inertia(Inertia.box(Kilograms.of(1), new Translation3d(1, 0.2, 0.2)))
            .build();
    MassProperties seen =
        bar.transformedBy(
            new Transform3d(new Translation3d(0, 0, 1), new Rotation3d(0, 0, Math.PI / 2)));

    assertThat(seen.cg().getX()).isCloseTo(0, within(1e-12));
    assertThat(seen.cg().getY()).isCloseTo(0.5, within(1e-12));
    assertThat(seen.cg().getZ()).isEqualTo(1);
    assertThat(seen.inertia().iyy()).isCloseTo(bar.inertia().ixx(), within(1e-12));
  }

  private static MassProperties cube(double x) {
    return MassProperties.builder()
        .mass(Kilograms.of(1))
        .cg(new Translation3d(x, 0, 0))
        .inertia(Inertia.box(Kilograms.of(1), new Translation3d(1, 1, 1)))
        .build();
  }
}
