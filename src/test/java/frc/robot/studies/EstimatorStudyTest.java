package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.runtime.SimulationTest;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * The estimator study runs, and it feeds WPILib's estimator rightly: with wheels and IMUs exactly
 * as configured and nothing going wrong, every estimator agrees with the truth, loosely. A wrong
 * unit, clock, or reset would put WPILib's far off here. Which estimator is better is the study's
 * to report (docs/estimator-comparison.md), never a test's to demand.
 */
@SimulationTest
class EstimatorStudyTest {
  @TempDir Path folder;

  private final ByteArrayOutputStream printed = new ByteArrayOutputStream();

  private void run(String... args) throws IOException {
    EstimatorStudy.run(args, new PrintStream(printed, true, StandardCharsets.UTF_8));
  }

  private String printed() {
    return printed.toString(StandardCharsets.UTF_8);
  }

  @Test
  void withIdealSensorsEveryEstimatorAgreesWithTheTruth() {
    EstimatorRuns.Played played = EstimatorRuns.play(EstimatorScenarios.named("ideal"), 1);

    Map<String, EstimatorScores.Measures> measures = played.measures();
    assertThat(measures.keySet()).contains("A", "B", "C", "D", "O");
    for (Map.Entry<String, EstimatorScores.Measures> each : measures.entrySet()) {
      // Unfiltered, WPILib's takes every ambiguous tag: looser.
      double meters = each.getKey().equals("D") ? 0.5 : 0.05;
      assertThat(each.getValue().rmsMeters()).as(each.getKey() + ", position").isLessThan(meters);
      assertThat(each.getValue().rmsDegrees()).as(each.getKey() + ", heading").isLessThan(5.0);
    }
    assertThat(played.samplesPerLoop()).as("wheel readings a loop").isGreaterThanOrEqualTo(1);
    assertThat(played.added().get("B")).as("measurements WPILib took").isGreaterThan(100);
  }

  /** Every scenario but the match-long ones plays through, its faults applied and undone. */
  @Test
  void everyShortScenarioPlays() {
    for (EstimatorRuns.Scenario scenario : EstimatorScenarios.ALL) {
      if (scenario.route() == EstimatorScenarios.MATCH) {
        continue;
      }
      EstimatorRuns.Played played = EstimatorRuns.play(scenario, 2);

      assertThat(played.measures())
          .as(scenario.name())
          .hasEntrySatisfying("A", ours -> assertThat(ours.rmsMeters()).isFinite());
      if (scenario.name().startsWith("misread") || scenario.name().startsWith("flipped")) {
        assertThat(played.misreads()).as(scenario.name()).isPositive();
      }
    }
  }

  @Test
  void aRunWritesItsResults() throws IOException {
    run("--scenarios=ideal,shove", "--seeds=2", "--seed=3", "--trace", "--cost", "--out=" + folder);

    assertThat(Files.readAllLines(folder.resolve("runs.csv")))
        .hasSize(1 + 2 * 2 * (1 + EstimatorRuns.contenders().size()))
        .first()
        .asString()
        .startsWith("scenario,seed,contender,rms_m,p95_m,worst_m,rms_deg");
    assertThat(Files.readAllLines(folder.resolve("summary.csv")).get(0))
        .isEqualTo("scenario,contender,measure,median,q1,q3");
    assertThat(Files.readAllLines(folder.resolve("fed.csv"))).hasSizeGreaterThan(4);
    assertThat(Files.readAllLines(folder.resolve("cost.csv")))
        .hasSize(5)
        .first()
        .isEqualTo("contender,median_us,p99_us,worst_us,bytes_per_loop");
    assertThat(Files.readAllLines(folder.resolve("trace-shove.csv")).get(0))
        .startsWith("seconds,true_x,true_y,true_degrees,A_m,A_deg");
    assertThat(Files.readString(folder.resolve("summary.md")))
        .contains("2 seeds of each scenario, seeds 3 to 4")
        .contains("## Verdicts: A against C")
        .contains("| shove |")
        .contains("## ideal");
    assertThat(printed()).contains("ideal ").contains("cost per loop").contains("summary.md");

    // Two studies' runs put together read as one.
    Path merged = folder.resolve("merged");
    run("--merge=" + folder + "," + folder, "--out=" + merged);
    assertThat(Files.readAllLines(merged.resolve("runs.csv")))
        .hasSize(1 + 2 * 2 * 2 * (1 + EstimatorRuns.contenders().size()));
    assertThat(Files.readString(merged.resolve("summary.md"))).contains("| shove |");
  }

  @Test
  void itSaysHowToUseItAndRefusesWhatItDoesntKnow() throws IOException {
    run("--help");
    assertThat(printed()).startsWith("Usage: ./gradlew estimatorStudy");
    run("--list");
    assertThat(printed()).contains("clean-match").contains("tag 26 at");

    assertThatThrownBy(() -> run("--grid=1")).hasMessageContaining("not an option");
    assertThatThrownBy(() -> run("--scenarios=dancing")).hasMessageContaining("The scenarios:");
    assertThatThrownBy(() -> run("--seeds=many")).hasMessageContaining("must be a number");
    assertThatThrownBy(() -> run("--seeds=0")).hasMessageContaining("1 or more");
  }

  /** A verdict is "same" unless the medians differ by more than the seeds do. */
  @Test
  void aVerdictNeedsADifferenceBeyondTheSeedsSpread() {
    EstimatorReport.Spread tight = new EstimatorReport.Spread(0.010, 0.009, 0.011);
    EstimatorReport.Spread wide = new EstimatorReport.Spread(0.012, 0.008, 0.016);
    EstimatorReport.Spread far = new EstimatorReport.Spread(0.050, 0.049, 0.051);
    EstimatorReport.Spread never =
        new EstimatorReport.Spread(Double.POSITIVE_INFINITY, 1, Double.POSITIVE_INFINITY);

    assertThat(EstimatorReport.verdict(tight, wide)).isEqualTo(EstimatorReport.Verdict.SAME);
    assertThat(EstimatorReport.verdict(tight, far)).isEqualTo(EstimatorReport.Verdict.FIRST_BETTER);
    assertThat(EstimatorReport.verdict(far, tight))
        .isEqualTo(EstimatorReport.Verdict.SECOND_BETTER);
    assertThat(EstimatorReport.verdict(tight, never))
        .isEqualTo(EstimatorReport.Verdict.FIRST_BETTER);
    assertThat(EstimatorReport.verdict(never, never)).isEqualTo(EstimatorReport.Verdict.SAME);
    assertThat(EstimatorReport.words(EstimatorReport.Verdict.SECOND_BETTER, far, tight, 100, "cm"))
        .isEqualTo("WPILib better by 4.0 cm");
  }

  /** Recovered is back within 5 cm and 1° and staying there; never, if it never is. */
  @Test
  void recoveryIsTheTimeUntilTheEstimateStaysBack() {
    double[] meters = new double[100];
    double[] degrees = new double[100];
    for (int i = 10; i < 40; i++) {
      meters[i] = 0.5;
    }
    meters[45] = 0.2; // back for less than 0.2 s, then off again

    assertThat(EstimatorScores.recoverySeconds(meters, degrees, 10)).isEqualTo(36 * 0.02);
    assertThat(EstimatorScores.recoverySeconds(meters, degrees, 50)).isZero();
    java.util.Arrays.fill(meters, 60, 100, 1.0);
    assertThat(EstimatorScores.recoverySeconds(meters, degrees, 55)).isInfinite();
    assertThat(EstimatorScores.percentile(new double[] {1, 2, 3, 4, 5}, 0.5)).isEqualTo(3);
    assertThat(EstimatorScores.percentile(new double[0], 0.5)).isNaN();
    assertThat(List.of(EstimatorScores.names().get(0), EstimatorScores.names().get(12)))
        .containsExactly("rms_m", "recovery_s");
  }
}
