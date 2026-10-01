package frc.robot.studies;

import static frc.robot.lib.config.Rules.between;
import static frc.robot.lib.config.Rules.each;
import static frc.robot.lib.config.Rules.finite;
import static frc.robot.lib.config.Rules.notBlank;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import frc.robot.RobotConfig;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.Checks;
import frc.robot.lib.config.ConfigException;
import frc.robot.lib.config.Rule;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * Where the CAD team could put a camera on the robot being designed: each place, and the ways a
 * camera there could face. The study tries every layout of one camera to a few at these places,
 * each camera facing any of its place's ways, and ranks them.
 *
 * <p>The file is a CSV, one place a line, in inches and degrees from the robot's origin (the
 * frame's center, on the floor) as {@code CameraConfig} measures: {@code name, forward, left, up,
 * yaws, tilts}, the yaws and tilts each a list separated by spaces. Blank lines and lines starting
 * with {@code #} are skipped.
 *
 * <pre>
 * # name,      forward, left, up, yaws (left of ahead),  tilts (up)
 * front-left,  12.25,   12.25, 20, 0 30 45,              10 20
 * back-right,  -12.25, -12.25, 20, 180 -150 -135,        10 20
 * shooter-top, 0,       0,     28, 0,                    15
 * </pre>
 */
public final class MountOptions {
  /**
   * The most layouts one run scores: more takes long, and says little more than a narrower set of
   * options would.
   */
  static final int MOST_LAYOUTS = 5000;

  private final List<Option> options;

  /**
   * A place on the robot a camera could go, and the ways it could face there.
   *
   * @param name what CAD calls the place: the camera's name in every layout it's in
   * @param forwardInches how far ahead of the frame's center the lens would sit
   * @param leftInches how far left of the center
   * @param upInches how far above the floor
   * @param yawsDegrees which ways it could face: 0 is ahead, positive turns it left
   * @param tiltsDegrees how far up it could tilt (negative: down)
   */
  public record Option(
      String name,
      double forwardInches,
      double leftInches,
      double upInches,
      List<Double> yawsDegrees,
      List<Double> tiltsDegrees) {
    public Option {
      Checks.on(Option.class, name)
          .field("name", name, notBlank())
          .field("forwardInches", forwardInches, finite())
          .field("leftInches", leftInches, finite())
          .field("upInches", upInches, finite())
          .field("yawsDegrees", yawsDegrees, choices())
          .field("yawsDegrees", yawsDegrees, each(between(-360.0, 360.0)))
          .field("tiltsDegrees", tiltsDegrees, choices())
          .field("tiltsDegrees", tiltsDegrees, each(between(-90.0, 90.0)))
          .orThrow();
      yawsDegrees = List.copyOf(yawsDegrees);
      tiltsDegrees = List.copyOf(tiltsDegrees);
    }

    private static Rule<List<Double>> choices() {
      return list -> list.isEmpty() ? "must have at least one" : null;
    }

    /** How many ways a camera here could face: every yaw with every tilt. */
    int ways() {
      return yawsDegrees.size() * tiltsDegrees.size();
    }

    /** A camera here, facing one of its ways. */
    CameraConfig camera(int way, CameraConfig.Sensor sensor) {
      return CameraConfig.builder()
          .name(name)
          .forward(Inches.of(forwardInches))
          .left(Inches.of(leftInches))
          .up(Inches.of(upInches))
          .roll(Degrees.of(0))
          .tiltUp(Degrees.of(tiltsDegrees.get(way % tiltsDegrees.size())))
          .yaw(Degrees.of(yawsDegrees.get(way / tiltsDegrees.size())))
          .sensor(sensor)
          .noiseScale(1)
          .build();
    }

    /** This place and a way of facing, as a layout's name says it: "front-left 30°, up 10°". */
    String named(int way) {
      DecimalFormat shown = new DecimalFormat("0.#", DecimalFormatSymbols.getInstance(Locale.ROOT));
      return name
          + " "
          + shown.format(yawsDegrees.get(way / tiltsDegrees.size()))
          + "°, up "
          + shown.format(tiltsDegrees.get(way % tiltsDegrees.size()))
          + "°";
    }
  }

  /**
   * @throws IllegalArgumentException if two places share a name: a layout names its cameras by them
   */
  public MountOptions(List<Option> options) {
    Set<String> names = new HashSet<>();
    for (Option option : options) {
      if (!names.add(option.name())) {
        throw new IllegalArgumentException("two places are named " + option.name());
      }
    }
    this.options = List.copyOf(options);
  }

  public List<Option> options() {
    return options;
  }

  /**
   * Reads the places from a CSV file (see the class documentation).
   *
   * @throws IllegalArgumentException naming the line of any place that isn't one
   */
  public static MountOptions read(Path file) throws IOException {
    List<Option> options = new ArrayList<>();
    List<String> lines = Files.readAllLines(file);
    for (int i = 0; i < lines.size(); i++) {
      String line = lines.get(i).strip();
      if (line.isEmpty() || line.startsWith("#")) {
        continue;
      }
      String where = file.getFileName() + " line " + (i + 1) + ": ";
      String[] values = line.split(",", -1);
      if (values.length != 6) {
        throw new IllegalArgumentException(
            where + "a place is 6 values, name, forward, left, up, yaws, tilts: " + line);
      }
      try {
        options.add(
            new Option(
                values[0].strip(),
                number(values[1]),
                number(values[2]),
                number(values[3]),
                numbers(values[4]),
                numbers(values[5])));
      } catch (NumberFormatException | ConfigException e) {
        throw new IllegalArgumentException(where + e.getMessage(), e);
      }
    }
    if (options.isEmpty()) {
      throw new IllegalArgumentException(file.getFileName() + " has no places in it");
    }
    return new MountOptions(options);
  }

  private static double number(String text) {
    return Double.parseDouble(text.strip());
  }

  private static List<Double> numbers(String text) {
    List<Double> numbers = new ArrayList<>();
    for (String value : text.strip().split("\\s+")) {
      if (!value.isEmpty()) {
        numbers.add(number(value));
      }
    }
    return numbers;
  }

  /**
   * How many layouts of one camera up to {@code mostCameras} there are: for every set of places,
   * the product of their ways of facing.
   */
  public double layouts(int mostCameras) {
    // By the number of cameras: adding a place adds, to each count, the layouts one camera smaller
    // with a camera there, facing any of its ways.
    double[] count = new double[mostCameras + 1];
    count[0] = 1;
    for (Option option : options) {
      for (int cameras = mostCameras; cameras >= 1; cameras--) {
        count[cameras] += count[cameras - 1] * option.ways();
      }
    }
    double total = 0;
    for (int cameras = 1; cameras <= mostCameras; cameras++) {
      total += count[cameras];
    }
    return total;
  }

  /**
   * Every layout of one camera up to {@code mostCameras}, at different places, each camera facing
   * any of its place's ways. Each place is checked on the robot first, so a place outside it is
   * named before anything runs.
   *
   * @param robot the robot, whose body the cameras must be within
   * @param sensor the camera every layout uses
   * @throws IllegalArgumentException if a place is outside the robot, or the places make more
   *     layouts than one run scores, saying how many
   */
  public List<Candidate> candidates(
      RobotConfig robot, CameraConfig.Sensor sensor, int mostCameras) {
    for (Option option : options) {
      new Candidate("the place " + option.name(), List.of(option.camera(0, sensor))).checkOn(robot);
    }
    double count = layouts(mostCameras);
    if (count > MOST_LAYOUTS) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "these places make %,.0f layouts of 1 to %d cameras, more than the %,d one run"
                  + " scores: offer fewer ways of facing, fewer places, or fewer cameras"
                  + " (--cameras=%d)",
              count,
              mostCameras,
              MOST_LAYOUTS,
              Math.max(1, mostCameras - 1)));
    }
    List<Candidate> candidates = new ArrayList<>();
    add(candidates, new ArrayList<>(), 0, mostCameras, sensor);
    return candidates;
  }

  /** Every set of places that starts with the chosen ones, each place after the last chosen. */
  private void add(
      List<Candidate> candidates,
      List<Option> chosen,
      int next,
      int mostCameras,
      CameraConfig.Sensor sensor) {
    if (!chosen.isEmpty()) {
      addEveryWay(candidates, chosen, sensor);
    }
    if (chosen.size() == mostCameras) {
      return;
    }
    for (int i = next; i < options.size(); i++) {
      chosen.add(options.get(i));
      add(candidates, chosen, i + 1, mostCameras, sensor);
      chosen.remove(chosen.size() - 1);
    }
  }

  /** The chosen places, each camera facing each of its ways, counted like an odometer. */
  private static void addEveryWay(
      List<Candidate> candidates, List<Option> chosen, CameraConfig.Sensor sensor) {
    int[] way = new int[chosen.size()];
    while (true) {
      List<CameraConfig> cameras = new ArrayList<>();
      List<String> names = new ArrayList<>();
      for (int i = 0; i < chosen.size(); i++) {
        cameras.add(chosen.get(i).camera(way[i], sensor));
        names.add(chosen.get(i).named(way[i]));
      }
      candidates.add(
          new Candidate(
              cameras.size()
                  + (cameras.size() == 1 ? " camera: " : " cameras: ")
                  + String.join("; ", names),
              cameras));
      int i = chosen.size() - 1;
      while (i >= 0 && ++way[i] == chosen.get(i).ways()) {
        way[i] = 0;
        i--;
      }
      if (i < 0) {
        return;
      }
    }
  }
}
