package frc.robot.studies;

import static frc.robot.studies.StudyFixtures.WALL;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.geometry.Translation2d;

/** Weights: regions written for blue, counted on both halves. */
class WeightsTest {
  @TempDir Path folder;

  private Path file(String... lines) throws IOException {
    return Files.write(folder.resolve("weights.csv"), List.of(lines));
  }

  @Test
  void aRegionWeighsItsImageOnRedsHalfToo() throws IOException {
    Weights weights =
        Weights.read(
            file(
                "# name, x min, y min, x max, y max, weight",
                "",
                "Scoring, 2, 3, 4, 5, 3",
                "Blocked, 3, 3, 4, 4, 0"));

    assertThat(weights.regions()).hasSize(2);
    assertThat(weights.at(WALL, new Translation2d(2.5, 4.5))).isEqualTo(3.0);
    // The field is turned half a turn for red: (2.5, 4.5) on blue's half is (13.5, 3.5) on red's.
    assertThat(weights.at(WALL, new Translation2d(13.5, 3.5))).isEqualTo(3.0);
    assertThat(weights.at(WALL, new Translation2d(3.5, 3.5))).as("both: 3 x 0").isZero();
    assertThat(weights.at(WALL, new Translation2d(8, 4))).as("in none").isEqualTo(1.0);
  }

  @Test
  void aBadLineIsNamed() throws IOException {
    assertThatThrownBy(() -> Weights.read(file("Scoring, 2, 3, 4, 5")))
        .hasMessageContaining("weights.csv line 1")
        .hasMessageContaining("6 values");
    assertThatThrownBy(() -> Weights.read(file("# a comment", "Scoring, 2, 3, four, 5, 1")))
        .hasMessageContaining("line 2");
    assertThatThrownBy(() -> Weights.read(file("Backwards, 4, 3, 2, 5, -1")))
        .hasMessageContaining("Region[Backwards].weight: -1 must be zero or more")
        .hasMessageNotContaining("xMax");
    assertThatThrownBy(() -> Weights.read(file("Backwards, 4, 3, 2, 5, 1")))
        .hasMessageContaining("Region[Backwards].xMax: 2 must be more than xMin");
  }
}
