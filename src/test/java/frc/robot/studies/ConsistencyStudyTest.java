package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import frc.robot.SimulationTest;
import frc.robot.lib.timing.RobotLoop;
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
        "--out=" + folder);

    String summary = Files.readString(folder.resolve("summary.md"));
    assertThat(summary)
        .contains("1 seed of each run, seeds 3 to 3")
        .contains("up to 2 mm along and 0.1° about each axis")
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
  }

  @Test
  void eachSeedDrawsEachCamerasOwnMountError() {
    ConsistencyRuns.MountErrors mounts = ConsistencyRuns.MountErrors.CALIBRATED;

    assertThat(mounts.draw(1, 0)).isEqualTo(mounts.draw(1, 0));
    assertThat(mounts.draw(1, 0)).isNotEqualTo(mounts.draw(1, 1));
    assertThat(mounts.draw(1, 0)).isNotEqualTo(mounts.draw(2, 0));
    assertThat(Math.abs(mounts.draw(1, 0).getX())).isLessThanOrEqualTo(0.005);
    assertThat(Math.abs(mounts.draw(1, 0).getRotation().getZ()))
        .isLessThanOrEqualTo(Math.toRadians(0.25));
    assertThat(ConsistencyRuns.MountErrors.NONE.draw(1, 0).getTranslation().getNorm()).isZero();
  }
}
