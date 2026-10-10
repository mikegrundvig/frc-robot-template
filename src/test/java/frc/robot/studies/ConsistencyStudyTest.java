package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.runtime.sim.Scenarios;
import com.michaelgrundvig.frc.simulation.SensorErrors;
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
        .contains("## The mount check")
        .contains("## The gates")
        .contains("| turning | doubted 0/1")
        .contains("| turning | ")
        .contains("- **turning:** 5 s standing, two turns in place");
    List<String> loops = Files.readAllLines(folder.resolve("loops.csv"));
    assertThat(loops.get(0))
        .isEqualTo("run,loop,seconds,position_nees,heading_nees,error_m,claimed_m");
    assertThat(loops).hasSize(1 + RobotLoop.loops(20));
    assertThat(printed())
        .contains("turning ")
        .contains("summary.md")
        .containsPattern("Every loop's error and claim, to the bit: [0-9a-f]{16}");
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
    assertThatThrownBy(() -> run("--heading=sometimes")).hasMessageContaining("--heading must be");
  }

  @Test
  void theHeadingOptionNamesEachWay() {
    assertThat(ConsistencyStudy.heading("measurements"))
        .isEqualTo(EstimatorConfig.HeadingCorrection.HEADING_MEASUREMENTS);
    assertThat(ConsistencyStudy.heading("doubt"))
        .isEqualTo(EstimatorConfig.HeadingCorrection.HEADING_MEASUREMENTS_AND_MOUNT_DOUBT);
    assertThat(ConsistencyStudy.heading("every"))
        .isEqualTo(EstimatorConfig.HeadingCorrection.EVERY_FIX);
    assertThat(ConsistencyStudy.heading(null))
        .isEqualTo(ExampleRobot.SIM.estimator().headingCorrection());
  }

  /** Each gate on the claimed uncertainty, at its edges. */
  @Test
  void theGatesPassWhatTheRobotsCodeWould() {
    PoseCovariance sure = PoseCovariance.fromStdDevs(0.02, 0.02, Math.toRadians(0.5));
    PoseCovariance loose = PoseCovariance.fromStdDevs(0.2, 0.2, Math.toRadians(6));
    Consistency.Sample tracking = new Consistency.Sample(0, 0, 0, sure, true);
    Consistency.Sample lost = new Consistency.Sample(0, 0, 0, sure, false);
    Consistency.Sample unsure = new Consistency.Sample(0, 0, 0, loose, true);

    assertThat(Gates.calibrating(tracking)).isTrue();
    assertThat(Gates.calibrating(lost)).isFalse();
    assertThat(Gates.judgingMounts(tracking)).isTrue();
    assertThat(Gates.judgingMounts(unsure)).isFalse();
    assertThat(Gates.scoring(tracking)).isTrue();
    assertThat(Gates.scoring(unsure)).isFalse();
    assertThat(Gates.constraining(tracking)).isTrue();
    assertThat(Gates.constraining(unsure)).isFalse();
    assertThat(Gates.asPlaced(tracking)).isTrue();
    assertThat(Gates.asPlaced(unsure)).isFalse();
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
    EstimatorConfig config = ExampleRobot.SIM.estimator();
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
    double placement = config.tagPlacementStdDevMeters();
    assertThat(Math.sqrt(tags / draws)).isCloseTo(placement, within(0.05 * placement));
    assertThat(spread.mountMillimeters() / 1000)
        .as("a mount's shift, within what the estimator considers of a camera's")
        .isLessThanOrEqualTo(config.cameraShiftStdDevMeters());
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
