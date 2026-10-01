package frc.robot.studies;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.jspecify.annotations.Nullable;

/**
 * The study's report, written over on each run: {@code summary.md}, short, for people; {@code
 * candidates.csv}, every layout ranked with every score, for a spreadsheet; and {@code poses.csv},
 * every pose for the robot's own layout and the best one, for a closer look.
 */
final class Report {
  /** How many of the ranking the summary shows. */
  static final int TOP = 10;

  private Report() {}

  /**
   * What ran, for the summary to say.
   *
   * @param setup the run's inputs, in words: the robot, the field, the weights, the grid
   * @param own the robot's own layout
   * @param grid the positions sampled, for the maps
   * @param result the scoring in the nominal conditions
   * @param robustness how every layout held up across the robustness trials, if there were any
   */
  record Run(
      List<String> setup,
      Candidate own,
      FieldGrid grid,
      CameraScoring.Result result,
      @Nullable Robustness robustness) {
    List<Score> ranking() {
      return result.ranking();
    }

    Candidate best() {
      return ranking().get(0).candidate();
    }
  }

  static void write(Path directory, Run run) throws IOException {
    Files.createDirectories(directory);
    Robustness robustness = run.robustness();
    Files.write(
        directory.resolve("candidates.csv"),
        robustness == null ? candidates(run.ranking()) : candidates(robustness));
    Files.write(directory.resolve("poses.csv"), poses(run));
    Files.writeString(directory.resolve("summary.md"), summary(run));
  }

  static String summary(Run run) {
    List<Score> ranking = run.ranking();
    StringBuilder md = new StringBuilder("# Camera placement\n\n");
    for (String line : run.setup()) {
      md.append(line).append("\n");
    }
    Robustness robustness = run.robustness();
    if (robustness != null) {
      robust(md, robustness, run.own());
      md.append("\n## In the nominal conditions\n");
    }
    md.append(
        "\n*Ranked by* the position error 95% of the field is within, from one set of images with"
            + " the robot standing still: smaller is better, and a tie to the millimeter goes to"
            + " fewer cameras. *None* means more than 5% of the field gets no measurement at all."
            + " Every score is explained in docs/camera-placement.md.\n");

    md.append("\n## The best\n\n").append(HEADER);
    for (int i = 0; i < Math.min(TOP, ranking.size()); i++) {
      md.append(row(i + 1, ranking.get(i)));
    }

    md.append("\n## The best at each camera count\n\n").append(HEADER);
    int most = ranking.stream().mapToInt(Score::cameras).max().orElse(0);
    for (int cameras = 1; cameras <= most; cameras++) {
      for (int i = 0; i < ranking.size(); i++) {
        if (ranking.get(i).cameras() == cameras) {
          md.append(row(i + 1, ranking.get(i)));
          break;
        }
      }
    }

    int ownRank = rank(ranking, run.own());
    md.append(
        format("%n## The robot's own layout%n%nRanks %d of %d.%n%n", ownRank, ranking.size()));
    md.append(HEADER).append(row(ownRank, ranking.get(ownRank - 1)));

    md.append(
        format(
            "%n## Maps%n%nAt each spot, the share of headings where the robot is placed within 5"
                + " cm: `.` none, `1` to `9` tenths, `#` all, blank where the weights leave it out."
                + " Blue's alliance wall is on the left; each character is %.2f m.%n",
            run.grid().spacingMeters()));
    md.append("\n### ").append(run.own().name()).append("\n\n").append(map(run, run.own()));
    if (!run.best().equals(run.own())) {
      md.append("\n### The best: ").append(run.best().name()).append("\n\n");
      md.append(map(run, run.best()));
    }
    return md.toString();
  }

  /** The robust ranking's tables, and how far it moved from the nominal one. */
  private static void robust(StringBuilder md, Robustness robustness, Candidate own) {
    List<Robustness.Layout> ranking = robustness.ranking();
    md.append(
        format(
            "%n*Ranked by* the worst of %d trials' 95%% error, each trial's conditions drawn from"
                + " the plausible ranges in docs/camera-placement.md: smaller is better, a tie to"
                + " the millimeter goes to fewer cameras. *None* means more than 5%% of the field"
                + " went unmeasured in some trial.%n%n",
            robustness.trials().size()));
    for (int t = 0; t < robustness.trials().size(); t++) {
      md.append(format("- Trial %d: %s.%n", t + 1, robustness.trials().get(t).described()));
    }
    int nominalBest = 0;
    for (int i = 0; i < ranking.size(); i++) {
      if (ranking.get(i).nominalRank() == 1) {
        nominalBest = i + 1;
      }
    }
    md.append(
        format(
            "%nAgainst the nominal ranking: the nominal best ranks %d here, and the two rankings'"
                + " rank correlation is %.2f (1 is the same order).%n",
            nominalBest, robustness.rankCorrelation()));

    md.append("\n## The most robust\n\n").append(ROBUST_HEADER);
    for (int i = 0; i < Math.min(TOP, ranking.size()); i++) {
      md.append(robustRow(i + 1, ranking.get(i)));
    }
    md.append("\n## The most robust at each camera count\n\n").append(ROBUST_HEADER);
    int most = ranking.stream().mapToInt(Robustness.Layout::cameras).max().orElse(0);
    for (int cameras = 1; cameras <= most; cameras++) {
      for (int i = 0; i < ranking.size(); i++) {
        if (ranking.get(i).cameras() == cameras) {
          md.append(robustRow(i + 1, ranking.get(i)));
          break;
        }
      }
    }
    for (int i = 0; i < ranking.size(); i++) {
      if (ranking.get(i).candidate().equals(own)) {
        md.append(
            format(
                "%n## The robot's own layout, across the trials%n%nRanks %d of %d.%n%n",
                i + 1, ranking.size()));
        md.append(ROBUST_HEADER).append(robustRow(i + 1, ranking.get(i)));
      }
    }
  }

  private static final String ROBUST_HEADER =
      "| Rank | Layout | Worst trial 95% | Median 95% | Nominal 95% | Nominal rank"
          + " | Worst within 5 cm | Two cameras |\n"
          + "|---:|---|---:|---:|---:|---:|---:|---:|\n";

  private static String robustRow(int rank, Robustness.Layout layout) {
    return format(
        "| %d | %s | %s | %s | %s | %d | %s | %s |%n",
        rank,
        layout.candidate().name(),
        centimeters(layout.worstP95Meters()),
        centimeters(layout.medianP95Meters()),
        centimeters(layout.nominal().positionP95Meters()),
        layout.nominalRank(),
        percent(layout.worstWithinFiveCentimeters()),
        percent(layout.nominal().twoCameras()));
  }

  private static int rank(List<Score> ranking, Candidate candidate) {
    for (int i = 0; i < ranking.size(); i++) {
      if (ranking.get(i).candidate().equals(candidate)) {
        return i + 1;
      }
    }
    throw new IllegalArgumentException("not ranked: " + candidate.name());
  }

  private static final String HEADER =
      "| Rank | Layout | 95% within | Within 2 cm | Within 5 cm | One tag | Two tags | Two cameras"
          + " | Measured | Heading | Position RMS | Heading RMS |\n"
          + "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";

  private static String row(int rank, Score score) {
    return format(
        "| %d | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |%n",
        rank,
        score.candidate().name(),
        centimeters(score.positionP95Meters()),
        percent(score.withinTwoCentimeters()),
        percent(score.withinFiveCentimeters()),
        percent(score.oneTag()),
        percent(score.twoTags()),
        percent(score.twoCameras()),
        percent(score.measured()),
        percent(score.heading()),
        centimeters(score.positionRmsMeters()),
        Double.isNaN(score.headingRmsDegrees())
            ? "none"
            : format("%.2f°", score.headingRmsDegrees()));
  }

  /**
   * A map of the field: one character per position, the share of its headings (and images) where
   * the layout places the robot within 5 cm. The top row is the left side as blue's drivers see it.
   */
  static String map(Run run, Candidate candidate) {
    FieldGrid grid = run.grid();
    List<Sample> samples = run.result().samples();
    Outcome[][] outcomes = run.result().outcomes(candidate);
    double[][] localized = new double[grid.rows()][grid.columns()];
    double[][] total = new double[grid.rows()][grid.columns()];
    boolean[][] counted = new boolean[grid.rows()][grid.columns()];
    for (int i = 0; i < samples.size(); i++) {
      Sample sample = samples.get(i);
      counted[sample.row()][sample.column()] |= sample.weight() > 0;
      for (Outcome outcome : outcomes[i]) {
        total[sample.row()][sample.column()] += 1;
        localized[sample.row()][sample.column()] +=
            outcome.localized(Score.FIVE_CENTIMETERS) ? 1 : 0;
      }
    }
    StringBuilder map = new StringBuilder("```\n");
    for (int row = grid.rows() - 1; row >= 0; row--) {
      for (int column = 0; column < grid.columns(); column++) {
        if (!counted[row][column]) {
          map.append(' ');
          continue;
        }
        int tenths = (int) Math.floor(10 * localized[row][column] / total[row][column]);
        map.append(tenths <= 0 ? '.' : tenths >= 10 ? '#' : (char) ('0' + tenths));
      }
      map.append('\n');
    }
    return map.append("```\n").toString();
  }

  private static final String SCORE_COLUMNS =
      "position_p95_m,within_2cm,within_5cm,one_tag,two_tags,two_cameras,measured,heading,"
          + "position_rms_m,heading_rms_deg";

  static List<String> candidates(List<Score> ranking) {
    List<String> lines = new ArrayList<>();
    lines.add("rank,layout,cameras," + SCORE_COLUMNS);
    for (int i = 0; i < ranking.size(); i++) {
      Score score = ranking.get(i);
      lines.add(
          format(
              "%d,%s,%d,%s",
              i + 1, quoted(score.candidate().name()), score.cameras(), scoreColumns(score)));
    }
    return lines;
  }

  /** Every layout, the most robust first: its robust scores, then its nominal ones. */
  static List<String> candidates(Robustness robustness) {
    List<String> lines = new ArrayList<>();
    StringBuilder header =
        new StringBuilder("rank,layout,cameras,worst_p95_m,median_p95_m,worst_within_5cm");
    for (int t = 1; t <= robustness.trials().size(); t++) {
      header.append(",trial_").append(t).append("_p95_m");
    }
    header.append(",nominal_rank,");
    lines.add(header + SCORE_COLUMNS.replaceAll("([a-z0-9_]+)", "nominal_$1"));
    List<Robustness.Layout> ranking = robustness.ranking();
    for (int i = 0; i < ranking.size(); i++) {
      Robustness.Layout layout = ranking.get(i);
      StringBuilder line =
          new StringBuilder(
              format(
                  "%d,%s,%d,%s,%s,%.4f",
                  i + 1,
                  quoted(layout.candidate().name()),
                  layout.cameras(),
                  number(layout.worstP95Meters(), 4),
                  number(layout.medianP95Meters(), 4),
                  layout.worstWithinFiveCentimeters()));
      for (double p95 : layout.trialP95Meters()) {
        line.append(',').append(number(p95, 4));
      }
      line.append(',').append(layout.nominalRank()).append(',');
      lines.add(line + scoreColumns(layout.nominal()));
    }
    return lines;
  }

  private static String scoreColumns(Score score) {
    return format(
        "%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%s,%s",
        number(score.positionP95Meters(), 4),
        score.withinTwoCentimeters(),
        score.withinFiveCentimeters(),
        score.oneTag(),
        score.twoTags(),
        score.twoCameras(),
        score.measured(),
        score.heading(),
        number(score.positionRmsMeters(), 4),
        number(score.headingRmsDegrees(), 3));
  }

  static List<String> poses(Run run) {
    List<String> lines = new ArrayList<>();
    lines.add(
        "layout,x_m,y_m,heading_deg,weight,image,most_tags_in_one_image,cameras_seeing_tags,"
            + "measured,position_error_m,heading_error_deg");
    List<Candidate> shown = new ArrayList<>(List.of(run.own()));
    if (!run.best().equals(run.own())) {
      shown.add(run.best());
    }
    List<Sample> samples = run.result().samples();
    for (Candidate candidate : shown) {
      Outcome[][] outcomes = run.result().outcomes(candidate);
      for (int i = 0; i < samples.size(); i++) {
        Sample sample = samples.get(i);
        for (int image = 0; image < outcomes[i].length; image++) {
          Outcome outcome = outcomes[i][image];
          lines.add(
              format(
                  "%s,%.3f,%.3f,%.1f,%.3f,%d,%d,%d,%b,%s,%s",
                  quoted(candidate.name()),
                  sample.pose().getX(),
                  sample.pose().getY(),
                  sample.pose().getRotation().getDegrees(),
                  sample.weight(),
                  image + 1,
                  outcome.mostTagsInOneImage(),
                  outcome.camerasSeeingTags(),
                  outcome.measured(),
                  number(outcome.positionErrorMeters(), 4),
                  number(Math.toDegrees(outcome.headingErrorRadians()), 3)));
        }
      }
    }
    return lines;
  }

  /** A number for a spreadsheet, empty when there's none. */
  private static String number(double value, int digits) {
    return Double.isFinite(value) ? format("%." + digits + "f", value) : "";
  }

  private static String quoted(String text) {
    return "\"" + text.replace("\"", "\"\"") + "\"";
  }

  private static String percent(double share) {
    return format("%.1f%%", 100 * share);
  }

  private static String centimeters(double meters) {
    return Double.isFinite(meters) ? format("%.1f cm", 100 * meters) : "none";
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }
}
