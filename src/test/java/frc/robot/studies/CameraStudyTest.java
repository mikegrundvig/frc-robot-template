package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import frc.robot.Robots;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
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
        Files.writeString(folder.resolve("weights.csv"), "Field element, 7.5, 3.5, 9.0, 4.5, 0\n");

    run(
        // 2026's, pinned, so the test stands when WPILib's default becomes the next season's.
        "--field=FRC_2026_REBUILT_WELDED",
        "--grid-meters=3",
        "--headings=2",
        "--frames=1",
        "--threads=2",
        "--weights=" + weights,
        "--out=" + folder.resolve("report"));

    List<String> candidates = Files.readAllLines(folder.resolve("report/candidates.csv"));
    assertThat(candidates).hasSize(1 + Candidates.of(Robots.SIM).size());
    assertThat(candidates.get(0)).startsWith("rank,layout,cameras,position_p95_m");
    assertThat(candidates).anyMatch(line -> line.contains("\"The robot's own: sim\""));
    String summary = Files.readString(folder.resolve("report/summary.md"));
    assertThat(summary)
        .contains("on 2026 FRC Rebuilt Welded (WPILib's layout): 32 tags, rotated")
        .contains("Weights from `" + weights + "`: Field element 0.0")
        .contains("the family around its frame, of up to 4 cameras")
        .contains("No other robots in the way.")
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

  /** CAD's places: every layout of up to two cameras at them, and the robot's own. */
  @Test
  void aRunOnCadsPlaces() throws IOException {
    Path mounts =
        Files.write(
            folder.resolve("mounts.csv"),
            List.of(
                "name, forward (in), left (in), up (in), yaws (deg), tilts (deg)",
                "front, 12, 0, 20, 0 20, 10",
                "back, -12, 0, 20, 180, 10 20",
                "left, 0, 12, 20, 90, 10"));

    run(
        "--field=FRC_2026_REBUILT_WELDED",
        "--mounts=" + mounts,
        "--cameras=2",
        "--other-robots=3-5",
        "--grid-meters=6",
        "--headings=2",
        "--frames=1",
        "--out=" + folder.resolve("report"));

    // Ways of facing: 2, 2, 1. One camera: 5; two: 2 x 2 + 2 x 1 + 2 x 1 = 8; the robot's own.
    assertThat(Files.readAllLines(folder.resolve("report/candidates.csv"))).hasSize(1 + 14);
    assertThat(Files.readString(folder.resolve("report/summary.md")))
        .contains("all of up to 2 cameras at the places in `" + mounts + "`")
        .contains("Other robots in the cameras' way: 3 to 5 around each pose, each a solid box")
        .contains("0.76 m tall")
        .contains("2 cameras: front 0°, up 10°; back 180°, up 20°");
  }

  /** Robustness: the nominal scoring, then each trial's, and the layouts ranked by the worst. */
  @Test
  void aRobustnessRunRanksByTheWorstTrial() throws IOException {
    run(
        "--field=FRC_2026_REBUILT_WELDED",
        "--trials=2",
        "--cameras=2",
        "--grid-meters=6",
        "--headings=2",
        "--frames=1",
        "--out=" + folder.resolve("report"));

    List<String> candidates = Files.readAllLines(folder.resolve("report/candidates.csv"));
    assertThat(candidates.get(0))
        .startsWith("rank,layout,cameras,worst_p95_m,median_p95_m,worst_within_5cm")
        .contains("trial_1_p95_m,trial_2_p95_m,nominal_rank,nominal_position_p95_m");
    assertThat(Files.readString(folder.resolve("report/summary.md")))
        .contains("*Ranked by* the worst of 2 trials' 95% error")
        .contains("- Trial 1: corners off ")
        .contains("rank correlation is ")
        .contains("## The most robust at each camera count")
        .contains("## The robot's own layout, across the trials")
        .contains("## In the nominal conditions");
    assertThat(printed()).contains("Trial 2 of 2: ").contains("Most robust: ");

    // Each row's worst is the worst of its trials; down the ranking, the worst never improves (to
    // the millimeter, the ranking's own resolution: a tie there goes to fewer cameras).
    List<String> header = columns(candidates.get(0));
    int worst = header.indexOf("worst_p95_m");
    int first = header.indexOf("trial_1_p95_m");
    long previous = Long.MIN_VALUE;
    for (String line : candidates.subList(1, candidates.size())) {
      List<String> row = columns(line);
      double most = Math.max(meters(row.get(first)), meters(row.get(first + 1)));
      assertThat(meters(row.get(worst))).as(line).isEqualTo(most);
      long millimeters = Math.round(meters(row.get(worst)) * 1000);
      assertThat(millimeters).as(line).isGreaterThanOrEqualTo(previous);
      previous = millimeters;
    }
  }

  /** A CSV line's values, a quoted one's commas kept. */
  private static List<String> columns(String line) {
    List<String> columns = new ArrayList<>();
    StringBuilder column = new StringBuilder();
    boolean quoted = false;
    for (char c : line.toCharArray()) {
      if (c == '"') {
        quoted = !quoted;
      } else if (c == ',' && !quoted) {
        columns.add(column.toString());
        column.setLength(0);
      } else {
        column.append(c);
      }
    }
    columns.add(column.toString());
    return columns;
  }

  /** A distance as the CSV writes it: empty when there's none, which is past any distance. */
  private static double meters(String value) {
    return value.isEmpty() ? Double.POSITIVE_INFINITY : Double.parseDouble(value);
  }

  @Test
  void theOptionsSetTheErrorsAndTheTrials() {
    CameraStudyConfig set =
        CameraStudy.config(
            CameraStudy.options(
                new String[] {
                  "--corner-noise-pixels=0.4",
                  "--corner-spread-pixels=0.1",
                  "--sight-meters=5",
                  "--gyro-error-degrees=0.5",
                  "--mount-error-mm=3",
                  "--mount-error-degrees=0.5",
                  "--other-robots=3-5",
                  "--other-robots-height-meters=1.2",
                  "--robust"
                }));

    assertThat(set.cornerNoiseMeanPixels()).isEqualTo(0.4);
    assertThat(set.cornerNoiseStdDevPixels()).isEqualTo(0.1);
    assertThat(set.sightMeters()).isEqualTo(5);
    assertThat(set.gyroErrorDegrees()).isEqualTo(0.5);
    assertThat(set.mountErrorMillimeters()).isEqualTo(3);
    assertThat(set.mountErrorDegrees()).isEqualTo(0.5);
    assertThat(set.otherRobotsLeast()).isEqualTo(3);
    assertThat(set.otherRobotsMost()).isEqualTo(5);
    assertThat(set.otherRobotsHeightMeters()).isEqualTo(1.2);
    assertThat(set.trials()).isEqualTo(CameraStudyConfig.ROBUST_TRIALS);
    CameraStudyConfig four =
        CameraStudy.config(CameraStudy.options(new String[] {"--other-robots=4"}));
    assertThat(four.otherRobotsLeast()).isEqualTo(4);
    assertThat(four.otherRobotsMost()).isEqualTo(4);
    assertThat(CameraStudy.config(CameraStudy.options(new String[] {"--trials=3"})).trials())
        .isEqualTo(3);
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
    assertThatThrownBy(() -> run("--other-robots=3-5-7"))
        .hasMessageContaining("--other-robots must be a whole number, or two with a dash");
    assertThatThrownBy(() -> run("--other-robots=6"))
        .hasMessageContaining("6 must be from 0 to 5: an FRC field has five other robots");
    assertThatThrownBy(() -> run("--other-robots=5-3"))
        .hasMessageContaining("otherRobotsMost: 3 must be at least otherRobotsLeast (5)");
    assertThatThrownBy(() -> run("--headings=99999999999"))
        .as("too big to count")
        .isInstanceOf(CameraStudy.OptionException.class);
  }

  /** A file that isn't what it's for says so, without the usage: the options were right. */
  @Test
  void aBadFileIsntAnOptionError() throws IOException {
    Path weights = Files.writeString(folder.resolve("weights.csv"), "Field element, 7.5\n");

    assertThatThrownBy(() -> run("--weights=" + weights))
        .isNotInstanceOf(CameraStudy.OptionException.class)
        .hasMessageContaining("weights.csv line 1");
  }
}
