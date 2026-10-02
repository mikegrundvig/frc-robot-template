package frc.robot.studies;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.simulation.Seeds;
import frc.robot.Robots;
import frc.robot.lib.timing.RobotLoop;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import org.jspecify.annotations.Nullable;

/**
 * The estimator consistency study: does the pose estimator's uncertainty tell the truth? The whole
 * simulated robot, cameras and all, goes through each run ({@link ConsistencyRuns}) once per seed,
 * and every loop's estimate is compared with the truth the simulator knows ({@link Consistency}).
 * See docs/testing.md.
 *
 * <pre>
 *   ./gradlew consistencyStudy
 *   ./gradlew consistencyStudy --args="--runs=standing,gyro-scale --seeds=50"
 * </pre>
 */
public final class ConsistencyStudy {
  static final String USAGE =
      """
      Usage: ./gradlew consistencyStudy --args="[options]"   (see docs/testing.md)
        --runs=A,B,...            which runs (default: all of them)
        --seeds=N                 runs of each, each with its own noise and mounts (default 12)
        --seed=N                  the first seed (default 1)
      Each seed draws its errors from Gaussians with these standard deviations; the defaults are
      what the estimator's configuration claims, so an honest estimator reads a NEES near 2.
        --mount-error-mm=M        each camera off its mount along each axis (default 3)
        --mount-error-degrees=D   and turned about each axis (default: cameraTurnStdDevDegrees)
        --tag-error-mm=M          each tag off the field's layout along each axis (default: cameraShiftStdDevMeters)
        --gyro-scale-error=F      the gyro's scale off by this share (default: headingStdDevDegreesPerTurn / 360)
        --backup-scale-error=F    the backup IMU's (default: backupHeadingStdDevDegreesPerTurn / 360)
        --noise-scale=F           the vision noise model scaled by this (default 0.5, a little wider than the simulated cameras)
        --heading=W               what corrects the heading beside the gyro: measurements (the default:
                                  measurements with a heading of their own), doubt (those, and each
                                  camera's mount doubt in what it repeats), or every (every fix: a full update)
        --out=DIR                 where the report goes (default build/studies/consistency)
      """;

  /** Runs of each, by default: the study takes about seven minutes. */
  static final int SEEDS = 12;

  private static final Set<String> OPTIONS =
      Set.of(
          "runs",
          "seeds",
          "seed",
          "mount-error-mm",
          "mount-error-degrees",
          "tag-error-mm",
          "gyro-scale-error",
          "backup-scale-error",
          "noise-scale",
          "heading",
          "out");

  private ConsistencyStudy() {}

  public static void main(String[] args) throws IOException {
    try {
      run(args, System.out);
    } catch (IllegalArgumentException e) {
      System.err.println(e.getMessage());
      System.err.println();
      System.err.print(USAGE);
      System.exit(2);
    }
  }

  /**
   * What one run showed, over every seed.
   *
   * @param doubted by camera, how many seeds' mount checks doubted it at some loop
   * @param mostDoubtMeters by camera, the most it was doubted, averaged over the seeds
   * @param outOfCalibration by camera, how many seeds ended with its tags landing past the
   *     tolerance
   */
  record Result(
      ConsistencyRuns.Run run,
      Consistency consistency,
      Map<String, Integer> doubted,
      Map<String, Double> mostDoubtMeters,
      Map<String, Integer> outOfCalibration) {}

  /**
   * Runs the study as the options say, and writes its report.
   *
   * @param out where progress and the headline go
   * @throws IllegalArgumentException for an option that isn't one, or a value it can't take
   */
  static void run(String[] args, PrintStream out) throws IOException {
    Map<String, String> options = options(args);
    if (options.containsKey("help")) {
      out.print(USAGE);
      return;
    }
    List<ConsistencyRuns.Run> runs = new ArrayList<>();
    String named = options.get("runs");
    if (named == null) {
      runs.addAll(ConsistencyRuns.ALL);
    } else {
      for (String name : named.split(",")) {
        runs.add(ConsistencyRuns.named(name.trim()));
      }
    }
    int seeds = (int) number(options, "seeds", SEEDS);
    long first = (long) number(options, "seed", 1);
    if (seeds < 1) {
      throw new IllegalArgumentException("--seeds must be 1 or more: " + seeds);
    }
    ConsistencyRuns.Spread matched = ConsistencyRuns.Setup.MATCHED.spread();
    ConsistencyRuns.Spread spread =
        new ConsistencyRuns.Spread(
            number(options, "mount-error-mm", matched.mountMillimeters()),
            number(options, "mount-error-degrees", matched.mountDegrees()),
            number(options, "tag-error-mm", matched.tagMillimeters()),
            number(options, "gyro-scale-error", matched.gyroScale()),
            number(options, "backup-scale-error", matched.backupScale()));
    double noiseScale = number(options, "noise-scale", ConsistencyRuns.Setup.MATCHED.noiseScale());
    ConsistencyRuns.Setup setup =
        new ConsistencyRuns.Setup(spread, noiseScale, heading(options.get("heading")));

    long start = System.nanoTime();
    List<Result> results = study(runs, seeds, first, setup, out);
    double seconds = (System.nanoTime() - start) / 1e9;

    Path directory = Path.of(options.getOrDefault("out", "build/studies/consistency"));
    Files.createDirectories(directory);
    List<String> described =
        List.of(
            format(
                "%d %s of each run, seeds %d to %d, on the simulator's robot with a season's wear"
                    + " (SensorErrors.TYPICAL).",
                seeds, seeds == 1 ? "seed" : "seeds", first, first + seeds - 1),
            format(
                "Each seed draws, from Gaussians: each camera off its mount by %s mm along and %s°"
                    + " about each axis, each tag off the layout by %s mm along each, the gyro's"
                    + " scale off by %s%% and the backup IMU's by %s%% (standard deviations). The"
                    + " vision noise model scaled by %s.%s",
                shown(spread.mountMillimeters()),
                shown(spread.mountDegrees()),
                shown(spread.tagMillimeters()),
                shown(100 * spread.gyroScale()),
                shown(100 * spread.backupScale()),
                shown(noiseScale),
                setup.spread().equals(ConsistencyRuns.Setup.MATCHED.spread())
                        && noiseScale == ConsistencyRuns.Setup.MATCHED.noiseScale()
                    ? " As the estimator's configuration claims: matched."
                    : ""),
            format("The heading corrected by: %s.", setup.heading()),
            format("%.0f s.", seconds));
    Files.writeString(directory.resolve("summary.md"), summary(described, results));
    Files.write(directory.resolve("loops.csv"), loops(results), StandardCharsets.UTF_8);
    out.printf(Locale.ROOT, "Done in %.0f s.%n", seconds);
    out.println("Report: " + directory.toAbsolutePath().resolve("summary.md"));
  }

  /** A fingerprint carried on through every bit of these loops' errors and claims. */
  static long fingerprint(long fingerprint, List<Consistency.Sample> samples) {
    long mixed = fingerprint;
    for (Consistency.Sample sample : samples) {
      mixed =
          Seeds.mix(
              mixed,
              sample.errorX(),
              sample.errorY(),
              sample.errorHeading(),
              sample.covariance().xx(),
              sample.covariance().xy(),
              sample.covariance().yy(),
              sample.covariance().heading(),
              sample.tracking() ? 1 : 0);
    }
    return mixed;
  }

  /** Every run, once per seed: one robot at a time, since WPILib's simulation is one a program. */
  static List<Result> study(
      List<ConsistencyRuns.Run> runs,
      int seeds,
      long first,
      ConsistencyRuns.Setup setup,
      PrintStream out) {
    List<Result> results = new ArrayList<>();
    double tolerance = Robots.SIM.vision().mountToleranceMeters();
    long fingerprint = 0;
    for (ConsistencyRuns.Run run : runs) {
      Consistency consistency = new Consistency();
      Map<String, Integer> doubted = new LinkedHashMap<>();
      Map<String, Double> mostDoubt = new LinkedHashMap<>();
      Map<String, Integer> outOfCalibration = new LinkedHashMap<>();
      for (long seed = first; seed < first + seeds; seed++) {
        ConsistencyRuns.Played played = ConsistencyRuns.play(run, seed, setup);
        List<Consistency.Sample> samples = played.samples();
        consistency.add(samples);
        fingerprint = fingerprint(fingerprint, samples);
        for (Map.Entry<String, Double> camera : played.mostDoubtMeters().entrySet()) {
          doubted.merge(camera.getKey(), camera.getValue() > 0 ? 1 : 0, Integer::sum);
          mostDoubt.merge(camera.getKey(), camera.getValue() / seeds, Double::sum);
        }
        for (Map.Entry<String, Double> camera : played.offsetMeters().entrySet()) {
          outOfCalibration.merge(
              camera.getKey(), camera.getValue() > tolerance ? 1 : 0, Integer::sum);
        }
        Consistency alone = new Consistency();
        alone.add(samples);
        Consistency.Summary one = alone.summary();
        out.printf(
            Locale.ROOT,
            "  seed %d (gyro %+.2f%%, cameras turned %s): NEES %.2f, heading %.2f, outside the"
                + " ellipse %s, error %.1f cm, %.2f°%n",
            seed,
            100 * (setup.spread().gyroScale(seed) - 1),
            turns(setup.spread(), seed),
            one.positionNees(),
            one.headingNees(),
            percent(one.ellipseExceedance()),
            100 * one.meanErrorMeters(),
            one.meanHeadingErrorDegrees());
      }
      Consistency.Summary summary = consistency.summary();
      out.printf(
          Locale.ROOT,
          "%-14s NEES %.2f (2), heading %.2f (1), outside the ellipse %s, past %s σ %s, worst %.1f"
              + " σ, error %.1f cm%n",
          run.name(),
          summary.positionNees(),
          summary.headingNees(),
          percent(summary.ellipseExceedance()),
          Consistency.BOUND_TEXT,
          percent(summary.exceedance()),
          summary.worstRatio(),
          100 * summary.meanErrorMeters());
      results.add(new Result(run, consistency, doubted, mostDoubt, outOfCalibration));
    }
    // Two studies with the same options print the same: the simulation repeats to the bit.
    out.printf(Locale.ROOT, "Every loop's error and claim, to the bit: %016x%n", fingerprint);
    return results;
  }

  /** How far each of the robot's cameras is turned on its mount for a seed, in degrees. */
  private static String turns(ConsistencyRuns.Spread spread, long seed) {
    List<String> turns = new ArrayList<>();
    for (int camera = 0; camera < Robots.SIM.definition().cameras().size(); camera++) {
      turns.add(format("%+.2f°", Math.toDegrees(spread.mount(seed, camera).getRotation().getZ())));
    }
    return String.join(" ", turns);
  }

  /** The report: what ran, then a row for each run, then what each run was. */
  static String summary(List<String> setup, List<Result> results) {
    StringBuilder md = new StringBuilder("# Estimator consistency\n\n");
    for (String line : setup) {
      md.append(line).append('\n');
    }
    int runs = results.isEmpty() ? 0 : results.get(0).consistency().runs();
    double[] position = Consistency.band(2, runs);
    double[] heading = Consistency.band(1, runs);
    md.append(
        format(
            """

            Honest is NEES near 2 for position and 1 for heading, about 2.5%% of loops above and
            below the band (%.2f to %.2f for position, %.2f to %.2f for heading, averaged over %d
            runs), and at most 0.1%% of loops outside the claimed covariance's 99.9%% ellipse
            (position NEES past 13.8) or past %s σ along its long axis, give or take what that many
            loops allow (docs/testing.md).

            | Run | NEES (2) | Above, below | Heading NEES (1) | Above, below | Outside the ellipse \
            | Past %s σ | Allowed | Worst | Mean error | Worst error | Heading error | Claimed σ |
            |---|---|---|---|---|---|---|---|---|---|---|---|---|
            """,
            position[0],
            position[1],
            heading[0],
            heading[1],
            runs,
            Consistency.BOUND_TEXT,
            Consistency.BOUND_TEXT));
    for (Result result : results) {
      Consistency.Summary s = result.consistency().summary();
      md.append(
          format(
              "| %s | %.2f | %s, %s | %.2f | %s, %s | %s | %s | %s | %.1f σ | %.1f cm | %.1f cm |"
                  + " %.2f° | %.1f cm |%n",
              result.run().name(),
              s.positionNees(),
              percent(s.positionAbove()),
              percent(s.positionBelow()),
              s.headingNees(),
              percent(s.headingAbove()),
              percent(s.headingBelow()),
              percent(s.ellipseExceedance()),
              percent(s.exceedance()),
              percent(s.allowedShare()),
              s.worstRatio(),
              100 * s.meanErrorMeters(),
              100 * s.worstErrorMeters(),
              s.meanHeadingErrorDegrees(),
              100 * s.meanSpreadMeters()));
    }
    md.append(
        """

        ## The mount check

        For each camera, how many seeds' mount checks doubted it at some loop (so its measurements
        counted for less), the most it was doubted on average, and how many seeds ended with its
        tags landing past the tolerance (the alert to recalibrate it). The first camera is the one
        the camera runs knock.

        """);
    md.append("| Run | ");
    List<String> cameras =
        Robots.SIM.definition().cameras().stream()
            .map(com.michaelgrundvig.frc.world.robot.CameraConfig::name)
            .toList();
    md.append(String.join(" | ", cameras)).append(" |\n|---|");
    md.append("---|".repeat(cameras.size())).append('\n');
    for (Result result : results) {
      md.append("| ").append(result.run().name()).append(" |");
      int seeds = result.consistency().runs();
      for (String camera : cameras) {
        md.append(
            format(
                " doubted %d/%d, %.1f cm; out %d/%d |",
                result.doubted().getOrDefault(camera, 0),
                seeds,
                100 * result.mostDoubtMeters().getOrDefault(camera, 0.0),
                result.outOfCalibration().getOrDefault(camera, 0),
                seeds));
      }
      md.append('\n');
    }
    md.append(
        """

        ## The gates

        How often the robot's code would act on the estimate's claimed uncertainty: the share of
        loops each gate on it passes. Calibrating a camera needs the pose tracking and sure within
        5 cm; judging a camera's mount needs it tracking, and its position and heading together
        sure within the mount tolerance at the tag's distance (here 3 m); scoring needs it sure
        within 15 cm (`LocalizationGate`, beside its other tests); the vision pipeline solves with
        the gyro's heading while the heading is sure within 5°; and a robot placed on its start
        claims its heading to 2°.

        """);
    md.append("| Run | Calibrating (σ ≤ 5 cm) | Judging a mount (3 m) | Scoring (σ ≤ 15 cm)");
    md.append(" | Constraining (σθ ≤ 5°) | As placed (σθ ≤ 2°) |\n");
    md.append("|---|---|---|---|---|---|\n");
    for (Result result : results) {
      Consistency consistency = result.consistency();
      md.append(
          format(
              "| %s | %s | %s | %s | %s | %s |%n",
              result.run().name(),
              percent(consistency.share(Gates::calibrating)),
              percent(consistency.share(Gates::judgingMounts)),
              percent(consistency.share(Gates::scoring)),
              percent(consistency.share(Gates::constraining)),
              percent(consistency.share(Gates::asPlaced))));
    }
    md.append("\n## The runs\n\n");
    for (Result result : results) {
      md.append(format("- **%s:** %s%n", result.run().name(), result.run().description()));
    }
    return md.toString();
  }

  /** Each run's means across its seeds, loop by loop, for plotting. */
  static List<String> loops(List<Result> results) {
    List<String> lines = new ArrayList<>();
    lines.add("run,loop,seconds,position_nees,heading_nees,error_m,claimed_m");
    for (Result result : results) {
      double[] position = result.consistency().positionNeesByLoop();
      double[] heading = result.consistency().headingNeesByLoop();
      double[] error = result.consistency().errorByLoop();
      double[] spread = result.consistency().spreadByLoop();
      for (int i = 0; i < position.length; i++) {
        lines.add(
            format(
                "%s,%d,%.2f,%.4f,%.4f,%.4f,%.4f",
                result.run().name(),
                i,
                (i + 1) * RobotLoop.PERIOD_SECONDS,
                position[i],
                heading[i],
                error[i],
                spread[i]));
      }
    }
    return lines;
  }

  /** The way to correct the heading an option names; the configuration's without one. */
  static EstimatorConfig.HeadingCorrection heading(@Nullable String name) {
    if (name == null) {
      return ConsistencyRuns.Setup.MATCHED.heading();
    }
    return switch (name) {
      case "measurements" -> EstimatorConfig.HeadingCorrection.HEADING_MEASUREMENTS;
      case "doubt" -> EstimatorConfig.HeadingCorrection.HEADING_MEASUREMENTS_AND_MOUNT_DOUBT;
      case "every" -> EstimatorConfig.HeadingCorrection.EVERY_FIX;
      default ->
          throw new IllegalArgumentException(
              "--heading must be measurements, doubt, or every: " + name);
    };
  }

  /** The options, by name: {@code --seeds=50} is {@code seeds} = {@code 50}. */
  static Map<String, String> options(String[] args) {
    Map<String, String> options = new LinkedHashMap<>();
    for (String arg : args) {
      if (arg.equals("--help")) {
        options.put("help", "");
        continue;
      }
      int equals = arg.indexOf('=');
      String name = arg.startsWith("--") && equals > 2 ? arg.substring(2, equals) : "";
      if (!OPTIONS.contains(name)) {
        throw new IllegalArgumentException("not an option: " + arg);
      }
      options.put(name, arg.substring(equals + 1));
    }
    return options;
  }

  private static double number(Map<String, String> options, String name, double otherwise) {
    String text = options.get(name);
    try {
      return text == null ? otherwise : Double.parseDouble(text);
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException("--" + name + " must be a number: " + text, e);
    }
  }

  private static String percent(double share) {
    return share == 0 ? "0" : format("%.2f%%", 100 * share);
  }

  private static String shown(double value) {
    return format("%.2f", value).replaceAll("\\.?0+$", "");
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }
}
