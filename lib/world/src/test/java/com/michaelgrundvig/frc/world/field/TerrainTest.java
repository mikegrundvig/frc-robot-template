package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

/** Bumps: their shape, where they are, and both halves' from one written for blue. */
class TerrainTest {
  /** A bump like REBUILT's: its crest along y at x = 4, 6.5 in tall, 44.4 in deep. */
  private static final Bump BUMP =
      new Bump("Bump", new Translation2d(4, 2), new Translation2d(4, 4), 0.1654, 1.128);

  @Test
  void aBumpIsARidgeDownToTheFloorOnBothSides() {
    assertThat(BUMP.heightAt(new Translation2d(4, 3))).isCloseTo(0.1654, within(1e-12));
    assertThat(BUMP.heightAt(new Translation2d(4 - 0.282, 3))).isCloseTo(0.0827, within(1e-12));
    assertThat(BUMP.heightAt(new Translation2d(4 + 0.282, 3))).isCloseTo(0.0827, within(1e-12));
    assertThat(BUMP.heightAt(new Translation2d(4.6, 3))).isZero();
    assertThat(BUMP.heightAt(new Translation2d(4, 4.1))).as("past the crest's end").isZero();
    assertThat(Math.toDegrees(BUMP.slopeRadians())).isCloseTo(16.3, within(0.1));
  }

  @Test
  void aBumpSaysWhereAPointIsAcrossItsCrestAndAlong() {
    Translation2d point = new Translation2d(3.7, 2.5);

    // Along the crest is +y, so across it is -x.
    assertThat(BUMP.along().getDegrees()).isCloseTo(90, within(1e-9));
    assertThat(BUMP.across().getDegrees()).isCloseTo(180, within(1e-9));
    assertThat(BUMP.acrossOffset(point)).isCloseTo(0.3, within(1e-12));
    assertThat(BUMP.alongOffset(point)).isCloseTo(0.5, within(1e-12));
    assertThat(BUMP.length()).isEqualTo(2);
    assertThat(BUMP.spans(new Translation2d(4, 1.9), 0.2)).isTrue();
    assertThat(BUMP.covers(new Translation2d(3.3, 3), 0.1)).isFalse();
    assertThat(BUMP.covers(new Translation2d(3.3, 3), 0.2)).isTrue();
  }

  @Test
  void theTerrainIsTheHighestBumpAndFindsTheOneUnderAPoint() {
    Bump other = new Bump("Other", new Translation2d(8, 2), new Translation2d(8, 4), 0.3, 1.0);
    Terrain terrain = new Terrain(List.of(BUMP, other));

    assertThat(terrain.heightAt(new Translation2d(8, 3))).isCloseTo(0.3, within(1e-12));
    assertThat(terrain.heightAt(new Translation2d(6, 3))).isZero();
    assertThat(terrain.under(new Translation2d(4.2, 3), 0)).contains(BUMP);
    assertThat(terrain.under(new Translation2d(6, 3), 0.5)).isEmpty();
    assertThat(Terrain.FLAT.heightAt(new Translation2d(4, 3))).isZero();
  }

  @Test
  void aBumpNeedsARealShape() {
    Translation2d here = new Translation2d(1, 1);
    assertThatThrownBy(() -> new Bump(" ", here, new Translation2d(1, 2), 0.1, 1))
        .hasMessageContaining("name");
    assertThatThrownBy(() -> new Bump("B", here, here, 0.1, 1))
        .hasMessageContaining("two different points");
    assertThatThrownBy(() -> new Bump("B", here, new Translation2d(1, 2), 0, 1))
        .hasMessageContaining("positive");
  }

  @Test
  void theFieldPutsEachBlueBumpsPartnerOnRedsHalf() {
    FieldModel field =
        new FieldModel(
            TestFields.layout(), Symmetry.ROTATED, List.of(), List.of(), List.of(), List.of(BUMP));

    List<Bump> bumps = field.terrain().bumps();

    assertThat(bumps).extracting(Bump::name).containsExactly("Bump (blue)", "Bump (red)");
    Bump red = bumps.get(1);
    assertThat(red.crestStart().getX()).isCloseTo(12, within(1e-9));
    assertThat(red.crestStart().getY()).isCloseTo(6, within(1e-9));
    assertThat(red.heightAt(new Translation2d(12, 5))).isCloseTo(0.1654, within(1e-12));
    assertThat(red.heightAt(new Translation2d(4, 5))).isZero();
  }

  @Test
  void aBumpOffTheFieldIsCaughtWhenTheFieldIsBuilt() {
    Bump off = new Bump("Off", new Translation2d(-1, 2), new Translation2d(-1, 4), 0.1, 1);

    assertThatThrownBy(
            () ->
                new FieldModel(
                    TestFields.layout(),
                    Symmetry.ROTATED,
                    List.of(),
                    List.of(),
                    List.of(),
                    List.of(off)))
        .hasMessageContaining("off the field");
  }
}
