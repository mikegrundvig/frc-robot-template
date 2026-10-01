package frc.robot.studies;

import frc.robot.RobotConfig;
import frc.robot.Robots;
import frc.robot.lib.field.FieldModel;
import java.io.IOException;
import java.io.PrintStream;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;
import org.jspecify.annotations.Nullable;
import org.wpilib.fields.Fields;

/**
 * The camera placement study: where to mount the robot's AprilTag cameras, and how many, so it
 * knows where it is anywhere on the field. Every layout is scored all over the field, and the
 * report ranks them (see docs/camera-placement.md).
 *
 * <pre>
 *   ./gradlew cameraStudy --args="--weights=weights.csv"
 * </pre>
 */
public final class CameraStudy {
  static final String USAGE =
      """
      Usage: ./gradlew cameraStudy --args="[options]"   (see docs/camera-placement.md)
        --robot=NAME              which of Robots.all() to place cameras on (default: Robots.current())
        --field=NAME              one of WPILib's fields (default: the season's, Fields.DEFAULT_FIELD)
        --layout=FILE             a tag layout file (JSON), for a field WPILib doesn't have yet
        --weights=FILE            regions of the field that count more, or not at all (CSV)
        --grid-meters=M           how far apart the positions are (default 0.5)
        --headings=N              headings at each position (default 8)
        --frames=N                images each camera takes at each pose (default 2)
        --gyro-error-degrees=D    the gyro heading's typical error (default 0.25)
        --seed=N                  where every random draw starts (default 1)
        --threads=N               how many threads to run on (default 2)
        --out=DIR                 where the report goes (default build/studies/camera-placement)
      """;

  private static final Set<String> OPTIONS =
      Set.of(
          "robot",
          "field",
          "layout",
          "weights",
          "grid-meters",
          "headings",
          "frames",
          "gyro-error-degrees",
          "seed",
          "threads",
          "out");

  private CameraStudy() {}

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
   * Runs the study as the options say, and writes its report.
   *
   * @param out where progress and the headline go
   * @throws IllegalArgumentException for an option that isn't one, or a value that doesn't work,
   *     saying which
   */
  static void run(String[] args, PrintStream out) throws IOException {
    Map<String, String> options = options(args);
    if (options.containsKey("help")) {
      out.print(USAGE);
      return;
    }
    CameraStudyConfig config = config(options);
    RobotConfig robot = robot(options.get("robot"));
    String layout = options.get("layout");
    FieldModel field =
        layout != null
            ? TagLayouts.read(Path.of(layout))
            : TagLayouts.named(options.getOrDefault("field", Fields.DEFAULT_FIELD.name()));
    String weightsFile = options.get("weights");
    Weights weights = weightsFile == null ? Weights.NONE : Weights.read(Path.of(weightsFile));
    FieldGrid grid =
        new FieldGrid(
            field, robot.body(), config.gridMeters(), config.headings(), weights, config.seed());
    List<Sample> samples = grid.samples();
    List<Candidate> candidates = Candidates.of(robot);
    CameraScoring scoring = new CameraScoring(robot, field, candidates, config);

    List<String> setup = new ArrayList<>();
    setup.add(
        format(
            "Robot `%s`, on %s (%s): %d tags, %s.",
            robot.name(),
            field.name(),
            layout == null ? "WPILib's layout" : "from `" + layout + "`",
            field.tags().size(),
            field.symmetry().toString().toLowerCase(Locale.ROOT)));
    setup.add(
        weightsFile == null
            ? "Every spot counts the same: no weights."
            : format(
                "Weights from `%s`: %s.",
                weightsFile,
                weights.regions().stream()
                    .map(region -> region.name() + " " + region.weight())
                    .collect(Collectors.joining(", "))));
    setup.add(
        format(
            "%d poses: %d x %d positions %.2f m apart, at %d headings; %d %s per camera at"
                + " each; the gyro off by %.2f° typically; seed %d.",
            samples.size(),
            grid.columns(),
            grid.rows(),
            grid.spacingMeters(),
            config.headings(),
            config.frames(),
            config.frames() == 1 ? "image" : "images",
            config.gyroErrorDegrees(),
            config.seed()));
    out.println(setup.get(0));
    out.printf(
        Locale.ROOT,
        "Scoring %d layouts (%d distinct cameras) at %d poses on %d threads...%n",
        candidates.size(),
        scoring.distinctCameras(),
        samples.size(),
        config.threads());

    long start = System.nanoTime();
    AtomicInteger shown = new AtomicInteger();
    CameraScoring.Result result =
        scoring.score(
            samples,
            done -> {
              int tenths = 10 * done / samples.size();
              int before = shown.get();
              if (tenths > before && shown.compareAndSet(before, tenths)) {
                out.printf(Locale.ROOT, "  %d%%%n", 10 * tenths);
              }
            });
    double seconds = (System.nanoTime() - start) / 1e9;
    setup.add(
        format(
            "%d layouts, %d distinct cameras; %.0f s on %d threads.",
            candidates.size(), scoring.distinctCameras(), seconds, config.threads()));

    Path directory = Path.of(options.getOrDefault("out", "build/studies/camera-placement"));
    Report.write(directory, new Report.Run(setup, candidates.get(0), grid, result));
    Score best = result.ranking().get(0);
    out.printf(
        Locale.ROOT,
        "Done in %.0f s. Best: %s, 95%% of the field within %.1f cm.%n",
        seconds,
        best.candidate().name(),
        100 * best.positionP95Meters());
    out.println("Report: " + directory.toAbsolutePath().resolve("summary.md"));
  }

  /** The options, by name: {@code --grid-meters=0.25} is {@code grid-meters} = {@code 0.25}. */
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

  static CameraStudyConfig config(Map<String, String> options) {
    CameraStudyConfig d = CameraStudyConfig.DEFAULTS;
    return CameraStudyConfigBuilder.builder(d)
        .gridMeters(number(options, "grid-meters", d.gridMeters()))
        .headings((int) whole(options, "headings", d.headings()))
        .frames((int) whole(options, "frames", d.frames()))
        .gyroErrorDegrees(number(options, "gyro-error-degrees", d.gyroErrorDegrees()))
        .seed(whole(options, "seed", d.seed()))
        .threads((int) whole(options, "threads", d.threads()))
        .build();
  }

  private static RobotConfig robot(@Nullable String name) {
    if (name == null) {
      return Robots.current();
    }
    for (RobotConfig robot : Robots.all()) {
      if (robot.name().equals(name)) {
        return robot;
      }
    }
    throw new IllegalArgumentException(
        "no robot named "
            + name
            + " in Robots.all(): "
            + Robots.all().stream().map(RobotConfig::name).collect(Collectors.joining(", ")));
  }

  private static double number(Map<String, String> options, String name, double otherwise) {
    String text = options.get(name);
    try {
      return text == null ? otherwise : Double.parseDouble(text);
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException("--" + name + " must be a number: " + text, e);
    }
  }

  private static long whole(Map<String, String> options, String name, long otherwise) {
    String text = options.get(name);
    try {
      return text == null ? otherwise : Long.parseLong(text);
    } catch (NumberFormatException e) {
      throw new IllegalArgumentException("--" + name + " must be a whole number: " + text, e);
    }
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }
}
