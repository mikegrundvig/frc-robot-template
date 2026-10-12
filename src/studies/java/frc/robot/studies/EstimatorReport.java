package frc.robot.studies;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.function.Function;

/**
 * The estimator study's results as files: every run's measures, their medians across the seeds with
 * the interquartile range, and a page that sets the robot's own estimator beside WPILib's for each
 * scenario with a plain verdict.
 *
 * <p>A verdict compares two medians. A difference smaller than the wider of the two interquartile
 * ranges is "same": the seeds differ from each other by more than the estimators do.
 */
final class EstimatorReport {
  private EstimatorReport() {}

  /**
   * A measure's spread across the seeds.
   *
   * @param median the middle seed's
   * @param q1 a quarter of the seeds are at or below this
   * @param q3 and three quarters at or below this
   */
  record Spread(double median, double q1, double q3) {
    double iqr() {
      return q3 - q1;
    }
  }

  /** How two estimators compare on one measure. */
  enum Verdict {
    FIRST_BETTER,
    SAME,
    SECOND_BETTER
  }

  /** A measure's median and quartiles over the seeds of one scenario and estimator. */
  static Spread spread(
      List<EstimatorStudy.Row> rows, String scenario, String contender, String measure) {
    double[] values =
        rows.stream()
            .filter(row -> row.scenario().equals(scenario) && row.contender().equals(contender))
            .mapToDouble(row -> row.measures().byName().get(measure))
            .toArray();
    return new Spread(
        EstimatorScores.percentile(values, 0.5),
        EstimatorScores.percentile(values, 0.25),
        EstimatorScores.percentile(values, 0.75));
  }

  /** Which of two is better on a measure where less is better, or neither. */
  static Verdict verdict(Spread first, Spread second) {
    if (Double.isNaN(first.median()) || Double.isNaN(second.median())) {
      return Verdict.SAME;
    }
    if (Double.isInfinite(first.median()) || Double.isInfinite(second.median())) {
      if (first.median() == second.median()) {
        return Verdict.SAME;
      }
      return first.median() < second.median() ? Verdict.FIRST_BETTER : Verdict.SECOND_BETTER;
    }
    double difference = first.median() - second.median();
    double seedToSeed = Math.max(finite(first.iqr()), finite(second.iqr()));
    if (Math.abs(difference) <= seedToSeed) {
      return Verdict.SAME;
    }
    return difference < 0 ? Verdict.FIRST_BETTER : Verdict.SECOND_BETTER;
  }

  private static double finite(double value) {
    return Double.isFinite(value) ? value : Double.MAX_VALUE;
  }

  /** Every estimator the rows have, in the order they first appear. */
  static List<String> contenders(List<EstimatorStudy.Row> rows) {
    Set<String> names = new LinkedHashSet<>();
    rows.forEach(row -> names.add(row.contender()));
    return List.copyOf(names);
  }

  /** Every scenario the rows have, in the order they first appear. */
  static List<String> scenarios(List<EstimatorStudy.Row> rows) {
    Set<String> names = new LinkedHashSet<>();
    rows.forEach(row -> names.add(row.scenario()));
    return List.copyOf(names);
  }

  /**
   * Of the estimators whose names start with a letter, the one with the lowest position error over
   * the clean scenarios ("clean" and "clean-match"; every scenario, if neither was run): the tuning
   * a team would settle on from ordinary driving, chosen once and held for every scenario.
   */
  static String tuned(List<EstimatorStudy.Row> rows, String letter) {
    List<String> clean =
        scenarios(rows).stream()
            .filter(name -> name.equals("clean") || name.equals("clean-match"))
            .toList();
    if (clean.isEmpty()) {
      clean = scenarios(rows);
    }
    String best = letter;
    double least = Double.POSITIVE_INFINITY;
    for (String contender : contenders(rows)) {
      if (!contender.startsWith(letter)) {
        continue;
      }
      double sum = 0;
      for (String scenario : clean) {
        sum += spread(rows, scenario, contender, "rms_m").median();
      }
      if (sum < least) {
        least = sum;
        best = contender;
      }
    }
    return best;
  }

  /** Every run: a line for each scenario, seed, and estimator. */
  static List<String> runs(List<EstimatorStudy.Row> rows) {
    List<String> lines = new ArrayList<>();
    lines.add("scenario,seed,contender," + String.join(",", EstimatorScores.names()));
    for (EstimatorStudy.Row row : rows) {
      StringBuilder line =
          new StringBuilder(row.scenario() + "," + row.seed() + "," + row.contender());
      for (double value : row.measures().byName().values()) {
        line.append(',').append(format("%.5f", value));
      }
      lines.add(line.toString());
    }
    return lines;
  }

  /** Each measure's median and quartiles across the seeds, by scenario and estimator. */
  static List<String> summary(List<EstimatorStudy.Row> rows) {
    List<String> lines = new ArrayList<>();
    lines.add("scenario,contender,measure,median,q1,q3");
    for (String scenario : scenarios(rows)) {
      for (String contender : contenders(rows)) {
        for (String measure : EstimatorScores.names()) {
          Spread spread = spread(rows, scenario, contender, measure);
          lines.add(
              format(
                  "%s,%s,%s,%.5f,%.5f,%.5f",
                  scenario, contender, measure, spread.median(), spread.q1(), spread.q3()));
        }
      }
    }
    return lines;
  }

  /** One scenario's line for the console: each estimator's position and heading error. */
  static String headline(String scenario, List<EstimatorStudy.Row> rows) {
    StringBuilder line = new StringBuilder(format("%-22s", scenario));
    for (String contender : contenders(rows)) {
      Spread meters = spread(rows, scenario, contender, "rms_m");
      Spread degrees = spread(rows, scenario, contender, "rms_deg");
      line.append(format(" %s %.1f cm %.2f°", contender, 100 * meters.median(), degrees.median()));
    }
    return line.append(System.lineSeparator()).toString();
  }

  /** The page: the verdicts, then each scenario's table. */
  static String markdown(
      List<EstimatorRuns.Scenario> scenarios,
      List<EstimatorStudy.Row> rows,
      int seeds,
      long first) {
    String sameInputs = tuned(rows, "B");
    String team = tuned(rows, "C");
    StringBuilder md = new StringBuilder("# Estimator comparison\n\n");
    md.append(
        format(
            """
            %d %s of each scenario, seeds %d to %d. A is the robot's own estimator. B is WPILib's \
            given the measurements A's vision pipeline made; C is WPILib's behind the filtering \
            strong teams' code puts in front of it; D is WPILib's given every image unfiltered; O is its odometry alone, with no cameras. \
            The suffixes are tunings (docs/estimator-comparison.md). Of each, the tuning \
            with the lowest position error in clean driving stands for it below: %s and %s.

            Each number is the median across the seeds; in brackets, the interquartile range. A \
            verdict of "same" means the medians differ by less than the wider interquartile range.

            ## Verdicts: A against %s

            | Scenario | Position RMS, A | %s | Verdict | Heading RMS, A | %s | Verdict \
            | Back within 5 cm and 1°, A | %s | Largest jump, A | %s |
            |---|---|---|---|---|---|---|---|---|---|---|
            """,
            seeds,
            seeds == 1 ? "seed" : "seeds",
            first,
            first + seeds - 1,
            sameInputs,
            team,
            team,
            team,
            team,
            team,
            team));
    for (String scenario : scenarios(rows)) {
      Spread ourMeters = spread(rows, scenario, EstimatorRuns.OURS, "rms_m");
      Spread theirMeters = spread(rows, scenario, team, "rms_m");
      Spread ourDegrees = spread(rows, scenario, EstimatorRuns.OURS, "rms_deg");
      Spread theirDegrees = spread(rows, scenario, team, "rms_deg");
      md.append(
          format(
              "| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |%n",
              scenario,
              centimeters(ourMeters),
              centimeters(theirMeters),
              words(verdict(ourMeters, theirMeters), ourMeters, theirMeters, 100, "cm"),
              degrees(ourDegrees),
              degrees(theirDegrees),
              words(verdict(ourDegrees, theirDegrees), ourDegrees, theirDegrees, 1, "°"),
              seconds(spread(rows, scenario, EstimatorRuns.OURS, "recovery_s")),
              seconds(spread(rows, scenario, team, "recovery_s")),
              centimeters(spread(rows, scenario, EstimatorRuns.OURS, "jump_m")),
              centimeters(spread(rows, scenario, team, "jump_m"))));
    }
    List<String> shown = new ArrayList<>(List.of(EstimatorRuns.OURS, sameInputs, "C"));
    if (!team.equals("C")) {
      shown.add(team);
    }
    shown.add("D");
    shown.add("O");
    for (String scenario : scenarios(rows)) {
      md.append(format("%n## %s%n%n", scenario));
      scenarios.stream()
          .filter(each -> each.name().equals(scenario))
          .forEach(each -> md.append(each.description()).append("\n\n"));
      md.append(
          """
          | | RMS | 95th | Worst | Heading RMS | Heading worst | 2 s after: worst \
          | 2 s after: heading | Back within 5 cm, 1° | Largest jump | At rest near a tag \
          | At rest far |
          |---|---|---|---|---|---|---|---|---|---|---|---|
          """);
      for (String contender : shown) {
        if (!contenders(rows).contains(contender)) {
          continue;
        }
        String of = contender;
        Function<String, Spread> each = measure -> spread(rows, scenario, of, measure);
        md.append(
            format(
                "| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |%n",
                contender,
                centimeters(each.apply("rms_m")),
                centimeters(each.apply("p95_m")),
                centimeters(each.apply("worst_m")),
                degrees(each.apply("rms_deg")),
                degrees(each.apply("worst_deg")),
                centimeters(each.apply("after_worst_m")),
                degrees(each.apply("after_worst_deg")),
                seconds(each.apply("recovery_s")),
                centimeters(each.apply("jump_m")),
                centimeters(each.apply("rest_near_m")),
                centimeters(each.apply("rest_far_m"))));
      }
    }
    return md.toString();
  }

  /** A verdict in words, with the size of the difference. */
  static String words(Verdict verdict, Spread ours, Spread theirs, double scale, String unit) {
    String by =
        format(
            scale == 1 ? "%.2f%s" : "%.1f %s",
            scale * Math.abs(ours.median() - theirs.median()),
            unit);
    return switch (verdict) {
      case FIRST_BETTER -> "A better by " + by;
      case SECOND_BETTER -> "WPILib better by " + by;
      case SAME -> "same";
    };
  }

  private static String centimeters(Spread spread) {
    if (Double.isNaN(spread.median())) {
      return "";
    }
    return format("%.1f cm [%.1f]", 100 * spread.median(), 100 * spread.iqr());
  }

  private static String degrees(Spread spread) {
    if (Double.isNaN(spread.median())) {
      return "";
    }
    return format("%.2f° [%.2f]", spread.median(), spread.iqr());
  }

  private static String seconds(Spread spread) {
    if (Double.isNaN(spread.median())) {
      return "";
    }
    if (Double.isInfinite(spread.median())) {
      return "never";
    }
    return format("%.2f s [%s]", spread.median(), bound(spread.q1()) + " to " + bound(spread.q3()));
  }

  private static String bound(double seconds) {
    return Double.isInfinite(seconds) ? "never" : format("%.2f", seconds);
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }
}
