package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A platform like 2023's charge station: a 2 m square top 25 cm up, from (4, 2) to (6, 4), ramps
 * 0.5 m long down its front and back (toward -x and +x), and edges on its sides.
 */
class PlatformTest {
  static final Platform STATION =
      new Platform(
          "Station",
          new Polygon2d(
              List.of(
                  new Translation2d(4, 2),
                  new Translation2d(6, 2),
                  new Translation2d(6, 4),
                  new Translation2d(4, 4))),
          0.25,
          List.of(
              Platform.Side.EDGE,
              Platform.Side.ramp(0.5),
              Platform.Side.EDGE,
              Platform.Side.ramp(0.5)));

  @Test
  void itsTopIsFlatAndItsRampsRunDownFromItsSides() {
    assertThat(STATION.heightAt(new Translation2d(5, 3))).isEqualTo(0.25);
    assertThat(STATION.heightAt(new Translation2d(6, 3))).as("the top's edge").isEqualTo(0.25);
    assertThat(STATION.heightAt(new Translation2d(6.25, 3))).isCloseTo(0.125, within(1e-12));
    assertThat(STATION.heightAt(new Translation2d(3.9, 3))).isCloseTo(0.2, within(1e-12));
    assertThat(STATION.heightAt(new Translation2d(6.6, 3))).as("past the ramp's foot").isZero();
    assertThat(STATION.heightAt(new Translation2d(5, 4.01))).as("off an edge").isZero();
    assertThat(STATION.heightAt(new Translation2d(6.2, 4.2))).as("between at a corner").isZero();
    assertThat(STATION.normalAt(new Translation2d(6.25, 3)).x()).as("leaning out").isPositive();
    assertThat(STATION.normalAt(new Translation2d(3.75, 3)).x()).isNegative();
    assertThat(STATION.normalAt(new Translation2d(5, 3)).z()).isEqualTo(1);
    assertThat(Math.toDegrees(STATION.steepestSlopeRadians()))
        .isCloseTo(Math.toDegrees(Math.atan2(0.25, 0.5)), within(1e-9));
  }

  @Test
  void itsFootprintLeavesTheFloorBetweenRampsOut() {
    // The top, 4 square meters, and two ramps of 1.
    assertThat(STATION.footprint().area()).isCloseTo(6, within(1e-12));
    assertThat(STATION.footprint().contains(new Translation2d(6.2, 3))).isTrue();
    assertThat(STATION.footprint().contains(new Translation2d(6.2, 4.2))).isFalse();
    assertThat(STATION.covers(new Translation2d(6.2, 4.2), 0.25)).isTrue();
  }

  @Test
  void eachRampsTopIsACrossingNamedForItsSide() {
    List<Crossing> crossings = STATION.crossings();

    assertThat(crossings)
        .extracting(Crossing::name)
        .containsExactly("Station, side 2", "Station, side 4");
    assertThat(crossings).allMatch(crossing -> crossing.kind() == Crossing.Kind.TOP);
    assertThat(crossings.get(0).across().getDegrees())
        .as("up the +x ramp is -x")
        .isCloseTo(180, within(1e-9));
    assertThat(crossings.get(1).across().getDegrees()).isCloseTo(0, within(1e-9));
  }

  @Test
  void aPlatformWithoutRampsHasOnlyItsTop() {
    Platform box =
        new Platform(
            "Box",
            Polygon2d.rectangle(new Pose2d(3, 3, Rotation2d.ZERO), 1, 1),
            0.1,
            List.of(
                Platform.Side.EDGE, Platform.Side.EDGE, Platform.Side.EDGE, Platform.Side.EDGE));

    assertThat(box.crossings()).isEmpty();
    assertThat(box.steepestSlopeRadians()).isZero();
    assertThat(box.footprint()).isEqualTo(box.top());
    assertThat(box.mesh().triangles()).as("its top, and two for each wall").hasSize(2 + 4 * 2);
  }

  @Test
  void aPlatformMustBeOne() {
    Polygon2d square = STATION.top();
    List<Platform.Side> four = STATION.sides();
    assertThatThrownBy(() -> new Platform(" ", square, 1, four)).hasMessageContaining("name");
    assertThatThrownBy(() -> new Platform("P", square, 0, four)).hasMessageContaining("positive");
    assertThatThrownBy(() -> new Platform("P", square, 1, four.subList(0, 3)))
        .hasMessageContaining("4 edges, not 3");
    Polygon2d notch =
        new Polygon2d(
            List.of(
                new Translation2d(0, 0),
                new Translation2d(2, 0),
                new Translation2d(1, 0.5),
                new Translation2d(2, 1),
                new Translation2d(0, 1)));
    assertThatThrownBy(
            () ->
                new Platform(
                    "P",
                    notch,
                    1,
                    List.of(
                        Platform.Side.EDGE,
                        Platform.Side.EDGE,
                        Platform.Side.EDGE,
                        Platform.Side.EDGE,
                        Platform.Side.EDGE)))
        .hasMessageContaining("convex");
    Polygon2d straight =
        new Polygon2d(
            List.of(
                new Translation2d(0, 0),
                new Translation2d(1, 0),
                new Translation2d(2, 0),
                new Translation2d(2, 1),
                new Translation2d(0, 1)));
    assertThatThrownBy(
            () ->
                new Platform(
                    "P",
                    straight,
                    1,
                    List.of(
                        Platform.Side.ramp(1),
                        Platform.Side.ramp(1),
                        Platform.Side.EDGE,
                        Platform.Side.EDGE,
                        Platform.Side.EDGE)))
        .hasMessageContaining("corner 1 goes straight on between two ramps");
    assertThatThrownBy(() -> Platform.Side.ramp(0)).hasMessageContaining("some distance");
    assertThatThrownBy(() -> new Platform.Side(-1)).hasMessageContaining("some distance");
  }
}
