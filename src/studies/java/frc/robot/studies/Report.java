package frc.robot.studies;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

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
   */
  record Run(
      List<String> setup,
      Candidate own,
      FieldGrid grid,
      CameraScoring.Result result,
      List<Score> ranking) {}

  static void write(Path directory, Run run) throws IOException {
    Files.createDirectories(directory);
    Files.write(directory.resolve("candidates.csv"), candidates(run.ranking()));
    Files.write(directory.resolve("poses.csv"), poses(run));
    Files.writeString(directory.resolve("summary.md"), summary(run));
  }

  static String summary(Run run) {
    List<Score> ranking = run.ranking();
    double target = run.result().targetMeters();
    StringBuilder md = new StringBuilder("# Camera placement\n\n");
    for (String line : run.setup()) {
      md.append(line).append("\n");
    }
    md.append(
        format(
            "%n*Localized:* the cameras' combined measurement within %s of where the robot really"
                + " is, from one set of images with the robot standing still. Every score is"
                + " explained in docs/camera-placement.md.%n",
            centimeters(target)));

    md.append("\n## The best\n\n").append(header("Rank"));
    for (int i = 0; i < Math.min(TOP, ranking.size()); i++) {
      md.append(row(Integer.toString(i + 1), ranking.get(i)));
    }

    md.append("\n## The best at each camera count\n\n").append(header("Rank"));
    int most = ranking.stream().mapToInt(Score::cameras).max().orElse(0);
    for (int cameras = 1; cameras <= most; cameras++) {
      for (int i = 0; i < ranking.size(); i++) {
        if (ranking.get(i).cameras() == cameras) {
          md.append(row(Integer.toString(i + 1), ranking.get(i)));
          break;
        }
      }
    }

    int ownRank = rank(ranking, run.own());
    md.append(
        format("%n## The robot's own layout%n%nRanks %d of %d.%n%n", ownRank, ranking.size()));
    md.append(header("Rank")).append(row(Integer.toString(ownRank), ranking.get(ownRank - 1)));

    md.append(
        format(
            "%n## Maps%n%nAt each spot, the share of headings where the robot is localized: `.`"
                + " none, `1` to `9` tenths, `#` all, blank where the weights leave it out. Blue's"
                + " alliance wall is on the left; each character is %.2f m.%n",
            run.grid().spacingMeters()));
    Candidate best = ranking.get(0).candidate();
    md.append("\n### ").append(run.own().name()).append("\n\n");
    md.append(map(run, run.own()));
    if (!best.equals(run.own())) {
      md.append("\n### The best: ").append(best.name()).append("\n\n");
      md.append(map(run, best));
    }
    return md.toString();
  }

  private static int rank(List<Score> ranking, Candidate candidate) {
    for (int i = 0; i < ranking.size(); i++) {
      if (ranking.get(i).candidate().equals(candidate)) {
        return i + 1;
      }
    }
    throw new IllegalArgumentException("not ranked: " + candidate.name());
  }

  private static String header(String first) {
    return "| "
        + first
        + " | Layout | Localized | One tag | Two tags | Two cameras | Measured | Heading"
        + " | Position RMS | Position 95% | Heading RMS |\n"
        + "|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  }

  private static String row(String first, Score score) {
    return format(
        "| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |%n",
        first,
        score.candidate().name(),
        percent(score.localized()),
        percent(score.oneTag()),
        percent(score.twoTags()),
        percent(score.twoCameras()),
        percent(score.measured()),
        percent(score.heading()),
        centimeters(score.positionRmsMeters()),
        centimeters(score.positionP95Meters()),
        Double.isNaN(score.headingRmsDegrees())
            ? "none"
            : format("%.2f°", score.headingRmsDegrees()));
  }

  /**
   * A map of the field: one character per position, the share of its headings (and images) where
   * the layout localizes the robot. The top row is the left side as blue's drivers see it.
   */
  static String map(Run run, Candidate candidate) {
    FieldGrid grid = run.grid();
    List<Sample> samples = run.result().samples();
    Outcome[][] outcomes = run.result().of(candidate);
    double[][] localized = new double[grid.rows()][grid.columns()];
    double[][] total = new double[grid.rows()][grid.columns()];
    boolean[][] counted = new boolean[grid.rows()][grid.columns()];
    for (int i = 0; i < samples.size(); i++) {
      Sample sample = samples.get(i);
      counted[sample.row()][sample.column()] |= sample.weight() > 0;
      for (Outcome outcome : outcomes[i]) {
        total[sample.row()][sample.column()] += 1;
        boolean close =
            outcome.measured() && outcome.positionErrorMeters() <= run.result().targetMeters();
        localized[sample.row()][sample.column()] += close ? 1 : 0;
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

  static List<String> candidates(List<Score> ranking) {
    List<String> lines = new ArrayList<>();
    lines.add(
        "rank,layout,cameras,localized,one_tag,two_tags,two_cameras,measured,heading,"
            + "position_rms_m,position_p95_m,heading_rms_deg");
    for (int i = 0; i < ranking.size(); i++) {
      Score score = ranking.get(i);
      lines.add(
          format(
              "%d,%s,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f",
              i + 1,
              quoted(score.candidate().name()),
              score.cameras(),
              score.localized(),
              score.oneTag(),
              score.twoTags(),
              score.twoCameras(),
              score.measured(),
              score.heading(),
              score.positionRmsMeters(),
              score.positionP95Meters(),
              score.headingRmsDegrees()));
    }
    return lines;
  }

  static List<String> poses(Run run) {
    List<String> lines = new ArrayList<>();
    lines.add(
        "layout,x_m,y_m,heading_deg,weight,image,most_tags_in_one_image,cameras_seeing_tags,"
            + "measured,position_error_m,heading_error_deg");
    List<Candidate> shown = new ArrayList<>(List.of(run.own()));
    Candidate best = run.ranking().get(0).candidate();
    if (!best.equals(run.own())) {
      shown.add(best);
    }
    List<Sample> samples = run.result().samples();
    for (Candidate candidate : shown) {
      Outcome[][] outcomes = run.result().of(candidate);
      for (int i = 0; i < samples.size(); i++) {
        Sample sample = samples.get(i);
        for (int image = 0; image < outcomes[i].length; image++) {
          Outcome outcome = outcomes[i][image];
          lines.add(
              format(
                  "%s,%.3f,%.3f,%.1f,%.3f,%d,%d,%d,%b,%.4f,%.3f",
                  quoted(candidate.name()),
                  sample.pose().getX(),
                  sample.pose().getY(),
                  sample.pose().getRotation().getDegrees(),
                  sample.weight(),
                  image + 1,
                  outcome.mostTagsInOneImage(),
                  outcome.camerasSeeingTags(),
                  outcome.measured(),
                  outcome.positionErrorMeters(),
                  Math.toDegrees(outcome.headingErrorRadians())));
        }
      }
    }
    return lines;
  }

  private static String quoted(String text) {
    return "\"" + text.replace("\"", "\"\"") + "\"";
  }

  private static String percent(double share) {
    return format("%.1f%%", 100 * share);
  }

  private static String centimeters(double meters) {
    return Double.isNaN(meters) ? "none" : format("%.1f cm", 100 * meters);
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }
}
