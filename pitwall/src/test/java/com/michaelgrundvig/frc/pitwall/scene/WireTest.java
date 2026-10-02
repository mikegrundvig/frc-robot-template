package com.michaelgrundvig.frc.pitwall.scene;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.util.List;
import org.junit.jupiter.api.Test;

/** A scene's numbers as they go to the page: to the millimeter and the tenth of a degree. */
class WireTest {
  @Test
  void lengthsGoToTheMillimeterAndWholeNumbersWithoutADecimal() {
    assertThat(Wire.meters(1.23449)).isEqualTo(1.234);
    assertThat(Wire.meters(1.2345001)).isEqualTo(1.235);
    assertThat(Wire.meters(2.0)).isEqualTo(2L);
    assertThat(Wire.meters(-0.0004)).as("no negative zero").isEqualTo(0L);
    assertThat(Wire.meters(-3.5)).isEqualTo(-3.5);
  }

  @Test
  void anglesGoInDegreesToTheTenthBetweenMinusAndPlusHalfATurn() {
    assertThat(Wire.degrees(Math.toRadians(45.04))).isEqualTo(45L);
    assertThat(Wire.degrees(Math.toRadians(45.06))).isEqualTo(45.1);
    assertThat(Wire.degrees(Math.toRadians(270))).isEqualTo(-90L);
    assertThat(Wire.degrees(Math.toRadians(-180))).isEqualTo(180L);
    assertThat(Wire.degrees(Math.toRadians(540))).isEqualTo(180L);
  }

  @Test
  void aValueGoesToItsOwnResolutionWithItsDigitsOnly() {
    assertThat(Wire.toResolution(0.3000004, 0.1)).isEqualTo(0.3);
    assertThat(Wire.toResolution(0.26, 0.25)).isEqualTo(0.25);
    assertThat(Wire.toResolution(0.38, 0.25)).isEqualTo(0.5);
    assertThat(Wire.toResolution(1234.0, 10)).isEqualTo(1230L);
    assertThat(Wire.toResolution(1.234567, 0.001)).isEqualTo(1.235);
  }

  @Test
  void aLevelPoseGoesAsThreeNumbersAndAnyOtherAsSix() {
    assertThat(Wire.pose(1, 2, 0.0004, 0, 0, Math.PI / 2)).isEqualTo(List.of(1L, 2L, 90L));
    assertThat(Wire.pose(1, 2, 0.5, 0, Math.toRadians(10), 0))
        .isEqualTo(List.of(1L, 2L, 0.5, 0L, 10L, 0L));
  }

  @Test
  void anglesApartAreMeasuredTheShortWayRound() {
    assertThat(Wire.angleBetween(Math.toRadians(179), Math.toRadians(-179)))
        .isCloseTo(Math.toRadians(2), org.assertj.core.data.Offset.offset(1e-12));
  }

  @Test
  void namesAreShortAndSafeInAnAddress() {
    assertThat(Wire.name("A frame", "camera.front-left_2")).isEqualTo("camera.front-left_2");
    assertThatThrownBy(() -> Wire.name("A frame", "front left"))
        .hasMessageContaining("A frame \"front left\" isn't a name");
    assertThatThrownBy(() -> Wire.name("A frame", "-x"))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> Wire.name("A frame", "x".repeat(65)))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> Wire.meters(Double.NaN))
        .hasMessageContaining("a scene's numbers are finite");
  }
}
