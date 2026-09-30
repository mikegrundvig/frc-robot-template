package frc.robot.lib.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import frc.robot.lib.field.Bump;
import frc.robot.lib.field.Terrain;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

/** A crest the robot passed, matched to the bump it was on. */
class CrestLandmarksTest {
  private static final Bump BUMP =
      new Bump(
          "Left bump (blue)",
          new Translation2d(4.6, 4.6),
          new Translation2d(4.6, 6.5),
          0.165,
          1.13);
  private static final CrestLandmarks LANDMARKS =
      new CrestLandmarks(new Terrain(List.of(BUMP)), 0.05, 1.0);

  @Test
  void aCrestNearABumpPutsTheRobotOnItsCrestLine() {
    TerrainCrossing.Crest crest = new TerrainCrossing.Crest(2_000_000_000L, 0.1); // heading +x

    LandmarkEvent landmark =
        LANDMARKS.landmark(crest, new Translation2d(4.3, 5.5)).orElseThrow(); // 30 cm short

    assertThat(landmark.name()).isEqualTo("Left bump (blue)");
    assertThat(landmark.timestampNanos()).isEqualTo(2_000_000_000L);
    assertThat(landmark.stdDevMeters()).isEqualTo(0.05);
    assertThat(Math.abs(landmark.offset(new Translation2d(4.3, 5.5)))).isCloseTo(0.3, within(1e-9));
    assertThat(landmark.offset(new Translation2d(4.6, 1.0)))
        .as("anywhere along it")
        .isCloseTo(0, within(1e-12));
  }

  @Test
  void aCrestFarFromAnyBumpIsNoLandmark() {
    TerrainCrossing.Crest crest = new TerrainCrossing.Crest(0, 0);

    assertThat(LANDMARKS.landmark(crest, new Translation2d(2.0, 5.5))).isEmpty();
  }

  @Test
  void drivingAlongACrestIsNoLandmark() {
    TerrainCrossing.Crest alongIt = new TerrainCrossing.Crest(0, Math.PI / 2);
    TerrainCrossing.Crest unknownWay = new TerrainCrossing.Crest(0, Double.NaN);

    assertThat(LANDMARKS.landmark(alongIt, new Translation2d(4.6, 5.5))).isEmpty();
    assertThat(LANDMARKS.landmark(unknownWay, new Translation2d(4.6, 5.5))).isPresent();
  }

  @Test
  void landmarksAreCheckedWhenMade() {
    assertThatThrownBy(() -> new CrestLandmarks(Terrain.FLAT, 0, 1))
        .isInstanceOf(IllegalArgumentException.class);
    Translation2d here = new Translation2d(1, 1);
    assertThatThrownBy(() -> new LandmarkEvent(-1, "x", here, BUMP.across(), 0.05))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> new LandmarkEvent(0, " ", here, BUMP.across(), 0.05))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> new LandmarkEvent(0, "x", here, BUMP.across(), 0))
        .isInstanceOf(IllegalArgumentException.class);
  }
}
