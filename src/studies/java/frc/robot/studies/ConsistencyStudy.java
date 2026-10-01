package frc.robot.studies;

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
        --mount-error-mm=M        each camera off its mount by up to this along each axis (default 5)
        --mount-error-degrees=D   and turned by up to this about each axis (default 0.25)
        --noise-scale=F           the vision noise model scaled by this: fit tighter, or looser (default 1)
        --out=DIR                 where the report goes (default build/studies/consistency)
      """;

  /** Runs of each, by default: the study takes about four minutes. */
  static final int SEEDS = 12;

  private static final Set<String> OPTIONS =
      Set.of(
          "runs", "seeds", "seed", "mount-error-mm", "mount-error-degrees", "noise-scale", "out");

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

  /** What one run showed, over every seed. */
  record Result(ConsistencyRuns.Run run, Consistency consistency) {}

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
    ConsistencyRuns.MountErrors mounts =
        new ConsistencyRuns.MountErrors(
            number(options, "mount-error-mm", ConsistencyRuns.MountErrors.CALIBRATED.millimeters()),
            number(
                options, "mount-error-degrees", ConsistencyRuns.MountErrors.CALIBRATED.degrees()));

    double noiseScale = number(options, "noise-scale", 1);
    if (!(noiseScale > 0)) {
      throw new IllegalArgumentException("--noise-scale must be more than 0: " + noiseScale);
    }

    long start = System.nanoTime();
    List<Result> results = study(runs, seeds, first, mounts, noiseScale, out);
    double seconds = (System.nanoTime() - start) / 1e9;

    Path directory = Path.of(options.getOrDefault("out", "build/studies/consistency"));
    Files.createDirectories(directory);
    List<String> setup =
        List.of(
            format(
                "%d %s of each run, seeds %d to %d, on the simulator's robot with a season's wear"
                    + " (SensorErrors.TYPICAL).",
                seeds, seeds == 1 ? "seed" : "seeds", first, first + seeds - 1),
            format(
                "Each camera off its mount by up to %s mm along and %s° about each axis, drawn"
                    + " for each seed.%s",
                shown(mounts.millimeters()),
                shown(mounts.degrees()),
                noiseScale == 1
                    ? ""
                    : " The vision noise model scaled by " + shown(noiseScale) + "."),
            format("%.0f s.", seconds));
    Files.writeString(directory.resolve("summary.md"), summary(setup, results));
    Files.write(directory.resolve("loops.csv"), loops(results), StandardCharsets.UTF_8);
    out.printf(Locale.ROOT, "Done in %.0f s.%n", seconds);
    out.println("Report: " + directory.toAbsolutePath().resolve("summary.md"));
  }

  /** Every run, once per seed: one robot at a time, since WPILib's simulation is one a program. */
  static List<Result> study(
      List<ConsistencyRuns.Run> runs,
      int seeds,
      long first,
      ConsistencyRuns.MountErrors mounts,
      double noiseScale,
      PrintStream out) {
    List<Result> results = new ArrayList<>();
    for (ConsistencyRuns.Run run : runs) {
      Consistency consistency = new Consistency();
      for (long seed = first; seed < first + seeds; seed++) {
        consistency.add(ConsistencyRuns.play(run, seed, mounts, noiseScale));
      }
      Consistency.Summary summary = consistency.summary();
      out.printf(
          Locale.ROOT,
          "%-14s NEES %.1f (2), heading %.1f (1), past %s σ %s, worst %.1f σ, error %.1f cm%n",
          run.name(),
          summary.positionNees(),
          summary.headingNees(),
          Consistency.BOUND_TEXT,
          percent(summary.exceedance()),
          summary.worstRatio(),
          100 * summary.meanErrorMeters());
      results.add(new Result(run, consistency));
    }
    return results;
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
            runs), and the error past %s σ in at most 0.1%% of loops (docs/testing.md).

            | Run | NEES (2) | Above, below | Heading NEES (1) | Above, below | Past %s σ \
            | Worst | Mean error | Worst error | Heading error | Claimed σ |
            |---|---|---|---|---|---|---|---|---|---|---|
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
              "| %s | %.2f | %s, %s | %.2f | %s, %s | %s | %.1f σ | %.1f cm | %.1f cm | %.2f° |"
                  + " %.1f cm |%n",
              result.run().name(),
              s.positionNees(),
              percent(s.positionAbove()),
              percent(s.positionBelow()),
              s.headingNees(),
              percent(s.headingAbove()),
              percent(s.headingBelow()),
              percent(s.exceedance()),
              s.worstRatio(),
              100 * s.meanErrorMeters(),
              100 * s.worstErrorMeters(),
              s.meanHeadingErrorDegrees(),
              100 * s.meanSpreadMeters()));
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
