package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

/** A ramp on its own: one slope up from its foot to its top edge, then straight down. */
class RampTest {
  /** Climbing toward +x from x = 2 to x = 3.2, 1.5 m wide, to 30 cm. */
  private static final Ramp RAMP =
      new Ramp("Ramp", new Translation2d(2, 4), new Translation2d(3.2, 4), 1.5, 0.3);

  @Test
  void itRisesEvenlyFromItsFootToItsTop() {
    assertThat(RAMP.runMeters()).isCloseTo(1.2, within(1e-12));
    assertThat(RAMP.up().getDegrees()).isCloseTo(0, within(1e-12));
    assertThat(RAMP.heightAt(new Translation2d(2, 4))).isZero();
    assertThat(RAMP.heightAt(new Translation2d(2.6, 4.7))).isCloseTo(0.15, within(1e-12));
    assertThat(RAMP.heightAt(new Translation2d(3.2, 3.3))).isCloseTo(0.3, within(1e-12));
    assertThat(RAMP.heightAt(new Translation2d(3.21, 4))).as("past its top").isZero();
    assertThat(RAMP.heightAt(new Translation2d(2.6, 4.8))).as("beside it").isZero();
    assertThat(Math.toDegrees(RAMP.slopeRadians())).isCloseTo(14.04, within(0.01));
    assertThat(RAMP.steepestSlopeRadians()).isEqualTo(RAMP.slopeRadians());
    assertThat(RAMP.normalAt(new Translation2d(2.6, 4)).x()).as("leaning back down").isNegative();
    assertThat(RAMP.normalAt(new Translation2d(4, 4)).z()).isEqualTo(1);
  }

  @Test
  void itsTopIsACrossingWhoseWayAcrossIsUp() {
    Crossing top = RAMP.crossings().get(0);

    assertThat(RAMP.crossings()).hasSize(1);
    assertThat(top.name()).isEqualTo("Ramp");
    assertThat(top.kind()).isEqualTo(Crossing.Kind.TOP);
    assertThat(top.length()).isCloseTo(1.5, within(1e-12));
    assertThat(top.across().getDegrees()).isCloseTo(0, within(1e-9));
    assertThat(top.middle()).isEqualTo(new Translation2d(3.2, 4));
    assertThat(RAMP.footprint().area()).isCloseTo(1.8, within(1e-12));
  }

  @Test
  void aRampNeedsARealShape() {
    Translation2d here = new Translation2d(1, 1);
    Translation2d there = new Translation2d(2, 1);
    assertThatThrownBy(() -> new Ramp(" ", here, there, 1, 1)).hasMessageContaining("name");
    assertThatThrownBy(() -> new Ramp("R", here, here, 1, 1))
        .hasMessageContaining("two different points");
    assertThatThrownBy(() -> new Ramp("R", here, there, 0, 1)).hasMessageContaining("positive");
    assertThatThrownBy(() -> new Ramp("R", here, there, 1, Double.POSITIVE_INFINITY))
        .hasMessageContaining("positive");
  }
}
