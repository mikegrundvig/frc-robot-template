package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import frc.robot.Robots;
import frc.robot.SimulationTest;
import frc.robot.lib.localization.EstimatorConfig;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.Scenarios;
import frc.robot.sim.SensorErrors;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
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
        "--gyro-scale-error=0.001",
        "--backup-scale-error=0.002",
        "--noise-scale=0.7",
        "--out=" + folder);

    String summary = Files.readString(folder.resolve("summary.md"));
    assertThat(summary)
        .contains("1 seed of each run, seeds 3 to 3")
        .contains(
            "each camera off its mount by 2 mm along and 0.1° about each axis, each tag off the"
                + " layout by 3 mm along each, the gyro's scale off by 0.1% and the backup IMU's by"
                + " 0.2%")
        .contains("The vision noise model scaled by 0.7.")
        .doesNotContain("matched")
        .contains("| Outside the ellipse")
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
    ConsistencyRuns.Spread spread = ConsistencyRuns.Setup.MATCHED.spread();

    assertThat(spread.mount(1, 0)).isEqualTo(spread.mount(1, 0));
    assertThat(spread.mount(1, 0)).isNotEqualTo(spread.mount(1, 1));
    assertThat(spread.mount(1, 0)).isNotEqualTo(spread.mount(2, 0));
    assertThat(spread.tag(1, 26)).isEqualTo(spread.tag(1, 26));
    assertThat(spread.tag(1, 26)).isNotEqualTo(spread.tag(1, 25));
    assertThat(ConsistencyRuns.Spread.EXACT.mount(1, 0).getTranslation().getNorm()).isZero();
    assertThat(ConsistencyRuns.Spread.EXACT.tag(1, 26).getNorm()).isZero();
    assertThat(ConsistencyRuns.Spread.EXACT.gyroScale(1)).isEqualTo(1);
  }

  /**
   * No seed shares a draw with another: seed 2's first camera isn't seed 1's second, as it was when
   * a seed and a camera's number were added before hashing; nor is a camera's draw a tag's.
   */
  @Test
  void seedsNeverShareADraw() {
    ConsistencyRuns.Spread spread = ConsistencyRuns.Setup.MATCHED.spread();

    assertThat(spread.mount(2, 0)).isNotEqualTo(spread.mount(1, 1));
    assertThat(spread.tag(2, 1)).isNotEqualTo(spread.tag(1, 2));
    assertThat(spread.gyroScale(2)).isNotEqualTo(spread.gyroScale(1));
    assertThat(spread.gyroScale(1)).isNotEqualTo(spread.backupScale(1));
    Set<Double> yaws = new HashSet<>();
    for (long seed = 1; seed <= 12; seed++) {
      for (int camera = 0; camera < 4; camera++) {
        yaws.add(spread.mount(seed, camera).getRotation().getZ());
      }
    }
    assertThat(yaws).as("48 cameras, 48 draws").hasSize(48);
  }

  /** The matched spread draws at the standard deviations the estimator's configuration claims. */
  @Test
  void theMatchedSpreadDrawsWhatTheConfigurationClaims() {
    EstimatorConfig config = Robots.SIM.estimator();
    ConsistencyRuns.Spread spread = ConsistencyRuns.Setup.MATCHED.spread();
    double yaws = 0;
    double tags = 0;
    double scales = 0;
    int draws = 4000;
    for (int seed = 1; seed <= draws; seed++) {
      yaws += Math.pow(spread.mount(seed, 0).getRotation().getZ(), 2);
      tags += Math.pow(spread.tag(seed, 26).getX(), 2);
      scales += Math.pow(spread.gyroScale(seed) - 1, 2);
    }

    double turn = Math.toRadians(config.cameraTurnStdDevDegrees());
    assertThat(Math.sqrt(yaws / draws)).isCloseTo(turn, within(0.05 * turn));
    double shift = config.cameraShiftStdDevMeters();
    assertThat(Math.sqrt(tags / draws)).isCloseTo(shift, within(0.05 * shift));
    double scale = config.headingStdDevDegreesPerTurn() / 360;
    assertThat(Math.sqrt(scales / draws)).isCloseTo(scale, within(0.05 * scale));
  }

  @Test
  void eachSeedBuildsTheRobotWithItsOwnMountsTagsAndImus() {
    ConsistencyRuns.Spread spread = ConsistencyRuns.Setup.MATCHED.spread();
    SensorErrors errors = ConsistencyRuns.errors(7, spread);

    assertThat(errors.seed()).isEqualTo(7);
    assertThat(errors.tagErrors()).hasSize(32).containsEntry(26, spread.tag(7, 26));
    assertThat(errors.cameraMountError("back-right")).isEqualTo(spread.mount(7, 3));
    assertThat(errors.gyro().scale()).isEqualTo(spread.gyroScale(7));
    assertThat(errors.backupGyro().scale()).isEqualTo(spread.backupScale(7));
  }

  /** A scenario's runs: its sensor faults through the simulator's controls, its events too. */
  @Test
  void theScenarioRunsCarryTheirScenarios() {
    assertThat(ConsistencyRuns.named("camera-low").scenario())
        .isEqualTo(Scenarios.named("camera-low"));
    assertThat(ConsistencyRuns.named("collision").scenario())
        .isEqualTo(Scenarios.named("collision"));
    assertThat(ConsistencyRuns.named("driving").scenario()).isNull();
  }
}
