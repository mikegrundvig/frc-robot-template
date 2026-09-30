package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.fields.Fields;

/** The study end to end, on a coarse grid: its options, and the report it writes. */
class CameraStudyTest {
  @TempDir Path folder;

  private final ByteArrayOutputStream printed = new ByteArrayOutputStream();

  private void run(String... args) throws IOException {
    CameraStudy.run(args, new PrintStream(printed, true, StandardCharsets.UTF_8));
  }

  private String printed() {
    return printed.toString(StandardCharsets.UTF_8);
  }

  @Test
  void aCoarseRunRanksEveryLayoutAndWritesTheReport() throws IOException {
    Path weights =
        Files.writeString(folder.resolve("weights.csv"), "Hub, 4.03, 3.43, 5.22, 4.64, 0\n");

    run(
        "--grid-meters=3",
        "--headings=2",
        "--frames=1",
        "--threads=2",
        "--weights=" + weights,
        "--out=" + folder.resolve("report"));

    List<String> candidates = Files.readAllLines(folder.resolve("report/candidates.csv"));
    assertThat(candidates).hasSize(1 + 157);
    assertThat(candidates.get(0)).startsWith("rank,layout,cameras,localized");
    assertThat(candidates).anyMatch(line -> line.contains("\"The robot's own: sim\""));
    String summary = Files.readString(folder.resolve("report/summary.md"));
    assertThat(summary)
        .contains("on 2026 FRC Rebuilt Welded (WPILib's layout): 32 tags, rotated")
        .contains("Weights from `" + weights + "`: Hub 0.0")
        .contains("## The best at each camera count")
        .contains("## The robot's own layout")
        .contains("### The robot's own: sim");
    // 6 x 3 positions, at 2 headings, one image each: for the robot's own layout and the best.
    assertThat(Files.readAllLines(folder.resolve("report/poses.csv"))).hasSize(1 + 2 * 36);
    assertThat(printed()).contains("100%").contains("Best: ").contains("summary.md");
  }

  @Test
  void aLayoutFileTakesThePlaceOfWpilibsField() throws IOException {
    Path layout = folder.resolve("field.json");
    Files.writeString(layout, Fields.FRC_2025_REEFSCAPE_WELDED.loadField().getJson());

    run(
        "--layout=" + layout,
        "--grid-meters=6",
        "--headings=1",
        "--frames=1",
        "--out=" + folder.resolve("report"));

    assertThat(Files.readString(folder.resolve("report/summary.md")))
        .contains("(from `" + layout + "`)")
        .contains("Every spot counts the same");
  }

  @Test
  void helpShowsTheOptions() throws IOException {
    run("--help");

    assertThat(printed()).contains("--grid-meters=M").contains("docs/camera-placement.md");
  }

  @Test
  void aWrongOptionSaysWhich() {
    assertThatThrownBy(() -> run("--grid=0.5")).hasMessageContaining("not an option: --grid=0.5");
    assertThatThrownBy(() -> run("grid-meters=0.5")).hasMessageContaining("not an option");
    assertThatThrownBy(() -> run("--grid-meters=half"))
        .hasMessageContaining("--grid-meters must be a number: half");
    assertThatThrownBy(() -> run("--headings=2.5"))
        .hasMessageContaining("--headings must be a whole number");
    assertThatThrownBy(() -> run("--frames=0"))
        .hasMessageContaining("frames: 0 must be at least 1");
    assertThatThrownBy(() -> run("--robot=comp"))
        .hasMessageContaining("no robot named comp")
        .hasMessageContaining("sim");
    assertThatThrownBy(() -> run("--field=FRC_2027")).hasMessageContaining("no field named");
  }
}
