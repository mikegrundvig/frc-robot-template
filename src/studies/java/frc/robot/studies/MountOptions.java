package frc.robot.studies;

import static com.michaelgrundvig.frc.world.config.Rules.atLeast;
import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.each;
import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Centimeters;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Millimeters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigException;
import com.michaelgrundvig.frc.world.config.Rule;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.io.IOException;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * Where the CAD team could put a camera on the robot being designed: each place, and the ways a
 * camera there could face. The study tries every layout of one camera to a few at these places,
 * each camera facing any of its place's ways, and ranks them.
 *
 * <p>The file is a CSV, one place a line, measured from the robot's origin (the frame's center, on
 * the floor) as {@code CameraConfig} measures: the place's name, how far forward, left, and up, and
 * the yaws and tilts a camera there could take, each a list separated by spaces. Its first line
 * names the columns and their units, so a file in another unit can't be read as inches: lengths in
 * {@code in}, {@code mm}, {@code cm}, or {@code m}, angles in {@code deg} or {@code rad}. Blank
 * lines and lines starting with {@code #} are skipped.
 *
 * <pre>
 * name,        forward (in), left (in), up (in), yaws (deg),      tilts (deg)
 * front-left,  12.25,        12.25,     20,      0 30 45,         10 20
 * back-right,  -12.25,       -12.25,    20,      180 -150 -135,   10 20
 * shooter-top, 0,            0,         28,      0,               15
 * </pre>
 */
public final class MountOptions {
  /**
   * The most layouts one run scores: more takes long, and says little more than a narrower set of
   * options would.
   */
  static final int MOST_LAYOUTS = 5000;

  /** The columns, in order: the place's name, then each value, with its unit in the header. */
  static final List<String> COLUMNS = List.of("name", "forward", "left", "up", "yaws", "tilts");

  /** No lens sits on the floor: a place lower than this was measured in another unit. */
  static final Distance LOWEST = Inches.of(1);

  private static final Map<String, DistanceUnit> LENGTHS =
      Map.of("in", Inches, "mm", Millimeters, "cm", Centimeters, "m", Meters);
  private static final Map<String, AngleUnit> ANGLES = Map.of("deg", Degrees, "rad", Radians);
  private static final Pattern HEADING = Pattern.compile("(\\w+)\\s*\\(\\s*(\\w+)\\s*\\)");

  private final List<Option> options;

  /**
   * A place on the robot a camera could go, and the ways it could face there.
   *
   * @param name what CAD calls the place: the camera's name in every layout it's in
   * @param forward how far ahead of the frame's center the lens would sit
   * @param left how far left of the center
   * @param up how far above the floor: an inch at least
   * @param yaws which ways it could face: 0 is ahead, positive turns it left; each once
   * @param tilts how far up it could tilt (negative: down); each once
   */
  public record Option(
      String name,
      Distance forward,
      Distance left,
      Distance up,
      List<Angle> yaws,
      List<Angle> tilts) {
    public Option {
      Checks.on(Option.class, name)
          .field("name", name, notBlank())
          .field("forward", forward, finite())
          .field("left", left, finite())
          .field("up", up, atLeast(LOWEST))
          .field("yaws", yaws, choices())
          .field("yaws", yaws, each(between(Degrees.of(-360), Degrees.of(360))))
          .field("tilts", tilts, choices())
          .field("tilts", tilts, each(between(Degrees.of(-90), Degrees.of(90))))
          .orThrow();
      yaws = List.copyOf(yaws);
      tilts = List.copyOf(tilts);
    }

    /** At least one, and none twice. */
    private static Rule<List<Angle>> choices() {
      return list -> {
        if (list.isEmpty()) {
          return "must have at least one";
        }
        Set<Double> seen = new HashSet<>();
        for (Angle angle : list) {
          if (!seen.add(angle.in(Degrees))) {
            return shown(angle) + " is there twice";
          }
        }
        return null;
      };
    }

    /** How many ways a camera here could face: every yaw with every tilt. */
    int ways() {
      return yaws.size() * tilts.size();
    }

    private Angle yaw(int way) {
      return yaws.get(way / tilts.size());
    }

    private Angle tilt(int way) {
      return tilts.get(way % tilts.size());
    }

    /** A camera here, facing one of its ways. */
    CameraConfig camera(int way, CameraConfig.Sensor sensor) {
      return CameraConfig.builder()
          .name(name)
          .forward(forward)
          .left(left)
          .up(up)
          .roll(Degrees.of(0))
          .tiltUp(tilt(way))
          .yaw(yaw(way))
          .sensor(sensor)
          .noiseScale(1)
          .build();
    }

    /** This place and a way of facing, as a layout's name says it: "front-left 30°, up 10°". */
    String named(int way) {
      DecimalFormat shown = new DecimalFormat("0.#", DecimalFormatSymbols.getInstance(Locale.ROOT));
      return name
          + " "
          + shown.format(yaw(way).in(Degrees))
          + "°, up "
          + shown.format(tilt(way).in(Degrees))
          + "°";
    }

    /**
     * Why the place is outside the robot, in its own unit, or null if it's within: inside the
     * bumpers, and between the floor and the robot's height, as {@code Body.contains} says.
     */
    @Nullable String outside(Body body) {
      if (body.contains(new Translation3d(forward, left, up))) {
        return null;
      }
      DistanceUnit unit = forward.unit();
      return String.format(
          Locale.ROOT,
          "the place %s, %s ahead, %s left, and %s up, is outside the robot: within %s ahead or"
              + " behind its center, %s to either side, and up to %s",
          name,
          shown(forward),
          shown(left),
          shown(up),
          shown(unit.of(body.bumperLength().div(2).in(unit))),
          shown(unit.of(body.bumperWidth().div(2).in(unit))),
          shown(unit.of(body.height().in(unit))));
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
   * Reads the places from a CSV file (see the class documentation): its first line names the
   * columns and their units, and each line after it is a place.
   *
   * @throws IllegalArgumentException naming the line, and the column, of anything that isn't right
   */
  public static MountOptions read(Path file) throws IOException {
    List<CsvFile.Line> lines = CsvFile.read(file, COLUMNS);
    if (lines.isEmpty()) {
      throw new IllegalArgumentException(file.getFileName() + " has no places in it");
    }
    CsvFile.Line header = lines.get(0);
    if (!header.values().get(0).equals(COLUMNS.get(0))) {
      throw header.wrong(
          "the first line names the columns, with their units: name, forward (in), left (in), up"
              + " (in), yaws (deg), tilts (deg); lengths in in, mm, cm, or m, angles in deg or rad");
    }
    String lengthUnits = "(in), (mm), (cm), or (m)";
    DistanceUnit forward = unit(header, 1, LENGTHS, lengthUnits);
    DistanceUnit left = unit(header, 2, LENGTHS, lengthUnits);
    DistanceUnit up = unit(header, 3, LENGTHS, lengthUnits);
    AngleUnit yaws = unit(header, 4, ANGLES, "(deg) or (rad)");
    AngleUnit tilts = unit(header, 5, ANGLES, "(deg) or (rad)");
    List<Option> options = new ArrayList<>();
    for (CsvFile.Line line : lines.subList(1, lines.size())) {
      try {
        options.add(
            new Option(
                line.values().get(0),
                forward.of(line.number(1, "forward")),
                left.of(line.number(2, "left")),
                up.of(line.number(3, "up")),
                line.numbers(4, "yaws").stream().map(yaws::of).toList(),
                line.numbers(5, "tilts").stream().map(tilts::of).toList()));
      } catch (ConfigException e) {
        throw line.wrong(String.valueOf(e.getMessage()));
      }
    }
    if (options.isEmpty()) {
      throw new IllegalArgumentException(file.getFileName() + " has no places in it");
    }
    return new MountOptions(options);
  }

  /** The unit the header names for a column: {@code forward (in)}. */
  private static <U> U unit(CsvFile.Line header, int at, Map<String, U> units, String choices) {
    String column = COLUMNS.get(at);
    Matcher heading = HEADING.matcher(header.values().get(at));
    U unit =
        heading.matches() && heading.group(1).equals(column) ? units.get(heading.group(2)) : null;
    if (unit == null) {
      throw header.wrong(
          column, "\"" + header.values().get(at) + "\" should be " + column + " " + choices);
    }
    return unit;
  }

  /**
   * How many layouts of one camera up to {@code mostCameras} there are: for every set of places,
   * the product of their ways of facing.
   */
  public double layouts(int mostCameras) {
    int most = Math.min(mostCameras, options.size());
    // By the number of cameras: adding a place adds, to each count, the layouts one camera smaller
    // with a camera there, facing any of its ways.
    double[] count = new double[most + 1];
    count[0] = 1;
    for (Option option : options) {
      for (int cameras = most; cameras >= 1; cameras--) {
        count[cameras] += count[cameras - 1] * option.ways();
      }
    }
    double total = 0;
    for (int cameras = 1; cameras <= most; cameras++) {
      total += count[cameras];
    }
    return total;
  }

  /**
   * Every layout of one camera up to {@code mostCameras} (no more than there are places), at
   * different places, each camera facing any of its place's ways. Each place is checked on the
   * robot first, so a place outside it is named before anything runs.
   *
   * @param robot the robot, whose body the cameras must be within
   * @param sensor the camera every layout uses
   * @throws IllegalArgumentException if a place is outside the robot, or the places make more
   *     layouts than one run scores, saying how many and what would help
   */
  public List<Candidate> candidates(
      RobotConfig robot, CameraConfig.Sensor sensor, int mostCameras) {
    for (Option option : options) {
      String outside = option.outside(robot.definition().body());
      if (outside != null) {
        throw new IllegalArgumentException(outside);
      }
    }
    int most = Math.min(mostCameras, options.size());
    double count = layouts(most);
    if (count > MOST_LAYOUTS) {
      throw new IllegalArgumentException(tooMany(count, most));
    }
    List<Candidate> candidates = new ArrayList<>();
    add(candidates, new ArrayList<>(), 0, most, sensor);
    return candidates;
  }

  /** Why the places make too many layouts, and the most cameras that would make few enough. */
  private String tooMany(double count, int most) {
    int fits = 0;
    for (int cameras = 1; cameras < most; cameras++) {
      if (layouts(cameras) <= MOST_LAYOUTS) {
        fits = cameras;
      }
    }
    String help =
        fits > 0
            ? String.format(
                Locale.ROOT,
                "offer fewer ways of facing, fewer places, or fewer cameras: --cameras=%d makes"
                    + " %,.0f",
                fits,
                layouts(fits))
            : "even one camera at a time makes too many: offer fewer ways of facing, or fewer"
                + " places";
    return String.format(
        Locale.ROOT,
        "these places make %,.0f layouts of 1 to %d cameras, more than the %,d one run scores: %s",
        count,
        most,
        MOST_LAYOUTS,
        help);
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
