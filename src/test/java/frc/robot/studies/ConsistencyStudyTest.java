package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import frc.robot.SimulationTest;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.SensorErrors;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** The consistency study end to end, one short run: its options, and the report it writes. */
@SimulationTest
class ConsistencyStudyTest {
  @TempDir Path folder;

  private final ByteArrayOutputStream printed = new ByteArrayOutputStream();

  private void run(String... args) throws IOException {
    ConsistencyStudy.run(args, new PrintStream(printed, true, StandardCharsets.UTF_8));
  }

  private String printed() {
    return printed.toString(StandardCharsets.UTF_8);
  }

  @Test
  void aRunWritesTheReport() throws IOException {
    run(
        "--runs=turning",
        "--seeds=1",
        "--seed=3",
        "--mount-error-mm=2",
        "--mount-error-degrees=0.1",
        "--tag-error-mm=3",
        "--noise-scale=0.5",
        "--out=" + folder);

    String summary = Files.readString(folder.resolve("summary.md"));
    assertThat(summary)
        .contains("1 seed of each run, seeds 3 to 3")
        .contains(
            "up to 2 mm along and 0.1° about each axis, and each tag off the layout by up to 3")
        .contains("The vision noise model scaled by 0.5.")
        .contains("| turning | ")
        .contains("- **turning:** 5 s standing, two turns in place");
    List<String> loops = Files.readAllLines(folder.resolve("loops.csv"));
    assertThat(loops.get(0))
        .isEqualTo("run,loop,seconds,position_nees,heading_nees,error_m,claimed_m");
    assertThat(loops).hasSize(1 + RobotLoop.loops(20));
    assertThat(printed()).contains("turning ").contains("summary.md");
  }

  @Test
  void itSaysHowToUseItAndRefusesWhatItDoesntKnow() throws IOException {
    run("--help");
    assertThat(printed()).startsWith("Usage: ./gradlew consistencyStudy");

    assertThatThrownBy(() -> run("--grid-meters=1")).hasMessageContaining("not an option");
    assertThatThrownBy(() -> run("--runs=dancing")).hasMessageContaining("The runs: driving");
    assertThatThrownBy(() -> run("--seeds=many")).hasMessageContaining("must be a number");
    assertThatThrownBy(() -> run("--seeds=0")).hasMessageContaining("1 or more");
    assertThatThrownBy(() -> run("--mount-error-mm=-1")).hasMessageContaining("zero or more");
    assertThatThrownBy(() -> run("--noise-scale=0")).hasMessageContaining("more than 0");
  }

  @Test
  void eachSeedDrawsEachCamerasAndEachTagsOwnError() {
    ConsistencyRuns.Placement placement = ConsistencyRuns.Placement.CALIBRATED;

    assertThat(placement.mount(1, 0)).isEqualTo(placement.mount(1, 0));
    assertThat(placement.mount(1, 0)).isNotEqualTo(placement.mount(1, 1));
    assertThat(placement.mount(1, 0)).isNotEqualTo(placement.mount(2, 0));
    assertThat(Math.abs(placement.mount(1, 0).getX())).isLessThanOrEqualTo(0.005);
    assertThat(Math.abs(placement.mount(1, 0).getRotation().getZ()))
        .isLessThanOrEqualTo(Math.toRadians(0.25));
    assertThat(placement.tag(1, 26)).isEqualTo(placement.tag(1, 26));
    assertThat(placement.tag(1, 26)).isNotEqualTo(placement.tag(1, 25));
    assertThat(Math.abs(placement.tag(1, 26).getY())).isLessThanOrEqualTo(0.0127);
    assertThat(ConsistencyRuns.Placement.EXACT.mount(1, 0).getTranslation().getNorm()).isZero();
    assertThat(ConsistencyRuns.Placement.EXACT.tag(1, 26).getNorm()).isZero();
  }

  @Test
  void theRobotsSensorsErrAsTheRunAndTheSeedSay() {
    SensorErrors errors =
        ConsistencyRuns.errors(
            ConsistencyRuns.named("camera-low"), 7, ConsistencyRuns.Placement.CALIBRATED);

    assertThat(errors.seed()).isEqualTo(7);
    assertThat(errors.tagErrors()).hasSize(32);
    assertThat(errors.cameraMountError("front-left").getZ()).isCloseTo(-0.1016, within(1e-9));
    assertThat(errors.cameraMountError("back-right"))
        .isEqualTo(ConsistencyRuns.Placement.CALIBRATED.mount(7, 3));
  }
}
