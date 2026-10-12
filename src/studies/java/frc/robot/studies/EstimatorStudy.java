package frc.robot.studies;

import com.michaelgrundvig.frc.runtime.lib.timing.RobotLoop;
import com.michaelgrundvig.frc.world.field.Tag;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;

/**
 * The estimator study: the robot's own pose estimator against WPILib's {@code
 * SwerveDrivePoseEstimator}, on the same simulated robot in the same loop, scored against the
 * simulator's true pose. WPILib's runs three ways: given what the robot's vision pipeline hands its
 * own estimator (B), behind the filtering strong teams' public code puts in front of it (C), and
 * given every image unfiltered (D). See docs/estimator-comparison.md.
 *
 * <pre>
 *   ./gradlew estimatorStudy
 *   ./gradlew estimatorStudy --args="--scenarios=clean,shove --seeds=20"
 * </pre>
 */
public final class EstimatorStudy {
  static final String USAGE =
      """
      Usage: ./gradlew estimatorStudy --args="[options]"   (see docs/estimator-comparison.md)
        --scenarios=A,B,...   which scenarios (default: all of them); --list names them
        --seeds=N             runs of each, each with its own noise (default 20)
        --seed=N              the first seed (default 1)
        --out=DIR             where the results go (default build/studies/estimators)
        --merge=DIR,DIR,...   run nothing: put earlier results' runs.csv files together into one
                              report in --out (a long study is run a few scenarios at a time)
        --cost                also time each estimator and count its allocations, on the first
                              scenario's first seed (cost.csv)
        --trace               also write the first seed of each scenario loop by loop
        --list                the scenarios, and the field's tags
      """;

  /** Runs of each scenario, by default. */
  static final int SEEDS = 20;

  private static final Set<String> OPTIONS = Set.of("scenarios", "seeds", "seed", "out", "merge");

  private EstimatorStudy() {}

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

  /** One run's row: a scenario, a seed, an estimator, and its measures. */
  record Row(String scenario, long seed, String contender, EstimatorScores.Measures measures) {}

  /**
   * Runs the study as the options say, and writes its results.
   *
   * @param out where progress goes
   * @throws IllegalArgumentException for an option that isn't one, or a value it can't take
   */
  static void run(String[] args, PrintStream out) throws IOException {
    Map<String, String> options = options(args);
    if (options.containsKey("help")) {
      out.print(USAGE);
      return;
    }
    if (options.containsKey("list")) {
      out.print(listing());
      return;
    }
    List<EstimatorRuns.Scenario> scenarios = new ArrayList<>();
    String named = options.get("scenarios");
    if (named == null) {
      scenarios.addAll(EstimatorScenarios.ALL);
    } else {
      for (String name : named.split(",")) {
        scenarios.add(EstimatorScenarios.named(name.trim()));
      }
    }
    int seeds = (int) number(options, "seeds", SEEDS);
    long first = (long) number(options, "seed", 1);
    if (seeds < 1) {
      throw new IllegalArgumentException("--seeds must be 1 or more: " + seeds);
    }
    Path directory = Path.of(options.getOrDefault("out", "build/studies/estimators"));
    Files.createDirectories(directory);
    String merged = options.get("merge");
    if (merged != null) {
      List<Row> rows = new ArrayList<>();
      for (String each : merged.split(",")) {
        rows.addAll(read(Path.of(each.trim()).resolve("runs.csv")));
      }
      long firstSeed = rows.stream().mapToLong(Row::seed).min().orElse(first);
      long lastSeed = rows.stream().mapToLong(Row::seed).max().orElse(first);
      write(directory, rows, (int) (lastSeed - firstSeed + 1), firstSeed);
      out.println("Results: " + directory.toAbsolutePath().resolve("summary.md"));
      return;
    }

    long start = System.nanoTime();
    List<Row> rows = new ArrayList<>();
    List<String> fed = new ArrayList<>();
    fed.add("scenario,seed,contender,offered,added,later_discarded,misreads,samples_per_loop");
    for (EstimatorRuns.Scenario scenario : scenarios) {
      for (long seed = first; seed < first + seeds; seed++) {
        EstimatorRuns.Played played = EstimatorRuns.play(scenario, seed);
        for (Map.Entry<String, EstimatorScores.Measures> each : played.measures().entrySet()) {
          rows.add(new Row(scenario.name(), seed, each.getKey(), each.getValue()));
        }
        if (seed == first && options.containsKey("trace")) {
          Files.write(
              directory.resolve("trace-" + scenario.name() + ".csv"),
              trace(played),
              StandardCharsets.UTF_8);
        }
        fed.add(
            format(
                "%s,%d,%s,%d,%d,0,%d,%.2f",
                scenario.name(),
                seed,
                EstimatorRuns.OURS,
                played.oursOffered(),
                played.oursAccepted(),
                played.misreads(),
                played.samplesPerLoop()));
        for (String contender : played.added().keySet()) {
          fed.add(
              format(
                  "%s,%d,%s,%d,%d,%d,%d,%.2f",
                  scenario.name(),
                  seed,
                  contender,
                  played.offered().getOrDefault(contender, 0),
                  played.added().getOrDefault(contender, 0),
                  played.laterDiscarded().getOrDefault(contender, 0),
                  played.misreads(),
                  played.samplesPerLoop()));
        }
      }
      out.print(EstimatorReport.headline(scenario.name(), rows));
    }
    if (options.containsKey("cost")) {
      EstimatorRuns.Played played = EstimatorRuns.play(scenarios.get(0), first, true);
      List<String> lines = new ArrayList<>();
      lines.add("contender,median_us,p99_us,worst_us,bytes_per_loop");
      for (EstimatorCost.Cost cost : EstimatorCost.measure(played.loops(), played.start())) {
        lines.add(cost.csv());
        out.println("cost per loop: " + cost);
      }
      Files.write(directory.resolve("cost.csv"), lines, StandardCharsets.UTF_8);
    }
    double seconds = (System.nanoTime() - start) / 1e9;
    write(directory, rows, seeds, first);
    Files.write(directory.resolve("fed.csv"), fed, StandardCharsets.UTF_8);
    out.printf(Locale.ROOT, "Done in %.0f s.%n", seconds);
    out.println("Results: " + directory.toAbsolutePath().resolve("summary.md"));
  }

  /** Writes every run, the medians, and the page. */
  private static void write(Path directory, List<Row> rows, int seeds, long first)
      throws IOException {
    Files.write(directory.resolve("runs.csv"), EstimatorReport.runs(rows), StandardCharsets.UTF_8);
    Files.write(
        directory.resolve("summary.csv"), EstimatorReport.summary(rows), StandardCharsets.UTF_8);
    Files.writeString(
        directory.resolve("summary.md"),
        EstimatorReport.markdown(EstimatorScenarios.ALL, rows, seeds, first));
  }

  /** The runs an earlier study wrote. */
  static List<Row> read(Path runs) throws IOException {
    List<Row> rows = new ArrayList<>();
    List<String> lines = Files.readAllLines(runs);
    for (String line : lines.subList(1, lines.size())) {
      String[] values = line.split(",");
      double[] measures = new double[values.length - 3];
      for (int i = 0; i < measures.length; i++) {
        measures[i] = Double.parseDouble(values[i + 3]);
      }
      rows.add(
          new Row(
              values[0],
              Long.parseLong(values[1]),
              values[2],
              EstimatorScores.Measures.of(measures)));
    }
    return rows;
  }

  /** One run loop by loop: where the robot really was, and how far off each estimator was. */
  static List<String> trace(EstimatorRuns.Played played) {
    List<String> lines = new ArrayList<>();
    StringBuilder header = new StringBuilder("seconds,true_x,true_y,true_degrees");
    for (String contender : played.tracks().keySet()) {
      header.append(',').append(contender).append("_m,").append(contender).append("_deg");
    }
    lines.add(header.toString());
    EstimatorScores.Track ours =
        Objects.requireNonNull(played.tracks().get(EstimatorRuns.OURS), "the robot's own track");
    for (int loop = 0; loop < ours.loops(); loop++) {
      Pose2d truth = ours.truth(loop);
      StringBuilder line =
          new StringBuilder(
              format(
                  "%.2f,%.3f,%.3f,%.2f",
                  loop * RobotLoop.PERIOD_SECONDS - EstimatorRuns.DISABLED_SECONDS,
                  truth.getX(),
                  truth.getY(),
                  truth.getRotation().getDegrees()));
      for (EstimatorScores.Track track : played.tracks().values()) {
        line.append(format(",%.4f,%.3f", track.meters(loop), track.degrees(loop)));
      }
      lines.add(line.toString());
    }
    return lines;
  }

  /** The scenarios, then the field's tags: what a route is written against. */
  static String listing() {
    StringBuilder text = new StringBuilder();
    for (EstimatorRuns.Scenario scenario : EstimatorScenarios.ALL) {
      text.append(
          format(
              "%-22s %3.0f s  %s%n",
              scenario.name(), scenario.route().seconds(), scenario.description()));
    }
    text.append(
        format(
            "%nThe field: %.2f m by %.2f m.%n",
            EstimatorRuns.FIELD.length(), EstimatorRuns.FIELD.width()));
    EstimatorRuns.FIELD.tags().values().stream()
        .sorted(Comparator.comparingInt(Tag::id))
        .forEach(
            tag -> {
              Pose3d pose = tag.pose();
              text.append(
                  format(
                      "tag %2d at %5.2f, %5.2f, %4.2f facing %4.0f°%n",
                      tag.id(),
                      pose.getX(),
                      pose.getY(),
                      pose.getZ(),
                      Math.toDegrees(pose.getRotation().getZ())));
            });
    return text.toString();
  }

  /** The options, by name: {@code --seeds=50} is {@code seeds} = {@code 50}. */
  static Map<String, String> options(String[] args) {
    Map<String, String> options = new LinkedHashMap<>();
    for (String arg : args) {
      if (arg.equals("--help")
          || arg.equals("--list")
          || arg.equals("--trace")
          || arg.equals("--cost")) {
        options.put(arg.substring(2), "");
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

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }
}
