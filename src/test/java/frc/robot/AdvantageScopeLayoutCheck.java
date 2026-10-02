package frc.robot;

import io.avaje.jsonb.Jsonb;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.datalog.DataLogReader;
import org.wpilib.datalog.DataLogRecord;

/**
 * Checks an AdvantageScope layout against what the robot logs: every source a tab reads must be a
 * value the robot logs, of the type the layout says, in a kind of source that tab can show, and
 * every robot model it names must be one the build generates. A source that fails sits empty in
 * AdvantageScope, which looks the same as "nothing happened"; a kind of source the tab doesn't know
 * stops AdvantageScope reading the rest of the tab.
 *
 * <p>The layout format and the kinds of source are AdvantageScope v27.0.0-alpha-6's, read at that
 * release: the exported state ({@code src/main/electron/StateTracker.ts}, {@code
 * src/shared/HubState.ts}), the tabs' types ({@code src/shared/TabType.ts}), and each tab's sources
 * ({@code src/hub/controllers/*Controller_Config.ts}). A source's {@code logType} is how
 * AdvantageScope names a logged value's type: a struct by its name ({@code Pose3d[]}), anything
 * else by its kind ({@code Number}, {@code String}, {@code StringArray}).
 */
final class AdvantageScopeLayoutCheck {
  /** The tabs that read sources, by AdvantageScope's number for each. */
  private static final Map<Integer, String> TABS =
      Map.of(1, "Line Graph", 2, "2D Field", 3, "3D Field", 9, "Swerve");

  private static final List<String> POSES_2D_3D =
      List.of(
          "Pose2d",
          "Pose3d",
          "Pose2d[]",
          "Pose3d[]",
          "Transform2d",
          "Transform3d",
          "Transform2d[]",
          "Transform3d[]");

  private static final List<String> POSES_AND_TRANSLATIONS =
      concat(
          POSES_2D_3D,
          List.of("Translation2d", "Translation3d", "Translation2d[]", "Translation3d[]"));

  private static final List<String> POSES_3D =
      List.of("Pose3d", "Pose3d[]", "Transform3d", "Transform3d[]");

  private static final List<String> PATHS =
      List.of(
          "Pose2d[]",
          "Pose3d[]",
          "Transform2d[]",
          "Transform3d[]",
          "Translation2d[]",
          "Translation3d[]",
          "SwerveSample[]",
          "DifferentialSample[]",
          "Trajectory");

  private static final List<String> MODULE_VELOCITIES =
      List.of("SwerveModuleVelocity[]", "SwerveModuleState[]");

  private static final List<String> ROTATIONS = List.of("Rotation2d", "Rotation3d");

  private static final List<String> DISCRETE =
      List.of("Raw", "Boolean", "Number", "String", "BooleanArray", "NumberArray", "StringArray");

  /**
   * One kind of source in a tab: the value types it shows; for a parent, the key its children name
   * (a ghost's children are a robot's); for a child, the parent it hangs from.
   */
  private record Kind(List<String> types, @Nullable String parentKey, @Nullable String childOf) {
    static Kind of(List<String> types) {
      return new Kind(types, null, null);
    }

    static Kind parent(List<String> types, String key) {
      return new Kind(types, key, null);
    }

    static Kind child(List<String> types, String of) {
      return new Kind(types, null, of);
    }
  }

  /** The kinds of source each tab's lists know, from AdvantageScope's controller configs. */
  private static final Map<String, Map<String, Kind>> KINDS =
      Map.of(
          "3D Field",
          kinds(
              "robot", Kind.parent(POSES_2D_3D, "robot"),
              "ghost", Kind.parent(POSES_2D_3D, "robot"),
              "component", Kind.child(POSES_3D, "robot"),
              "mechanism", Kind.child(List.of("Mechanism2d"), "robot"),
              "vision", Kind.child(POSES_AND_TRANSLATIONS, "robot"),
              "swerveModuleVelocities", Kind.child(MODULE_VELOCITIES, "robot"),
              "rotationOverride", Kind.child(ROTATIONS, "robot"),
              "rotationOverrideLegacy", Kind.child(List.of("Number"), "robot"),
              "gamePiece", Kind.of(concat(POSES_3D, List.of("Translation3d", "Translation3d[]"))),
              "trajectory", Kind.of(PATHS),
              "heatmap", Kind.of(POSES_AND_TRANSLATIONS),
              "aprilTag", Kind.parent(concat(POSES_3D, List.of("Trajectory")), "aprilTag"),
              "aprilTagIDs", Kind.child(List.of("NumberArray"), "aprilTag"),
              "axes", Kind.of(concat(POSES_3D, List.of("Trajectory"))),
              "cone", Kind.of(concat(POSES_2D_3D, List.of("Trajectory"))),
              "cameraOverride", Kind.of(List.of("Pose3d", "Transform3d"))),
          "2D Field",
          kinds(
              "robot", Kind.parent(POSES_2D_3D, "robot"),
              "ghost", Kind.parent(POSES_2D_3D, "robot"),
              "vision", Kind.child(POSES_AND_TRANSLATIONS, "robot"),
              "swerveModuleVelocities", Kind.child(MODULE_VELOCITIES, "robot"),
              "rotationOverride", Kind.child(ROTATIONS, "robot"),
              "rotationOverrideLegacy", Kind.child(List.of("Number"), "robot"),
              "trajectory", Kind.of(PATHS),
              "heatmap", Kind.of(POSES_AND_TRANSLATIONS),
              "arrow", Kind.of(POSES_2D_3D)),
          "Swerve",
          kinds(
              "moduleVelocities", Kind.of(MODULE_VELOCITIES),
              "robotVelocities", Kind.of(List.of("ChassisVelocities", "ChassisSpeeds")),
              "rotation", Kind.of(ROTATIONS),
              "rotationLegacy", Kind.of(List.of("Number"))),
          "Line Graph",
          kinds(
              "stepped", Kind.of(List.of("Number")),
              "smooth", Kind.of(List.of("Number")),
              "points", Kind.of(List.of("Number")),
              "stripes", Kind.of(DISCRETE),
              "graph", Kind.of(DISCRETE),
              "alerts", Kind.of(List.of("Alerts"))));

  /** The robot or ghost kinds, which name a model. */
  private static final Set<String> WITH_MODELS = Set.of("robot", "ghost");

  private AdvantageScopeLayoutCheck() {}

  /**
   * One source a tab reads: its tab and list, its kind, the value, and the options it's shown with.
   */
  record Source(
      String tab,
      String kind,
      String key,
      String type,
      Map<String, Object> options,
      String tabType) {
    @Override
    public String toString() {
      return tab + ", " + kind + " " + key;
    }
  }

  /** The layout's AdvantageScope version, as its export records it. */
  @SuppressWarnings("unchecked")
  static String version(String layoutJson) {
    Map<String, Object> layout = (Map<String, Object>) parse(layoutJson);
    return String.valueOf(layout.get("version"));
  }

  /** Every source in a layout's windows, tab by tab, in order. */
  @SuppressWarnings("unchecked")
  static List<Source> sources(String layoutJson) {
    Map<String, Object> layout = (Map<String, Object>) parse(layoutJson);
    List<Source> sources = new ArrayList<>();
    for (Object hub : list(layout.get("hubs"))) {
      Map<String, Object> tabs = map(map(map(hub).get("state")).get("tabs"));
      for (Object tabValue : list(tabs.get("tabs"))) {
        Map<String, Object> tab = map(tabValue);
        Object type = tab.get("type");
        String tabType =
            TABS.getOrDefault(type instanceof Number number ? number.intValue() : -1, "");
        if (tabType.isEmpty()) {
          continue;
        }
        Map<String, Object> controller = map(tab.get("controller"));
        for (String list : List.of("sources", "leftSources", "rightSources", "discreteSources")) {
          for (Object sourceValue : list(controller.get(list))) {
            Map<String, Object> source = map(sourceValue);
            sources.add(
                new Source(
                    String.valueOf(tab.get("title")),
                    String.valueOf(source.get("type")),
                    String.valueOf(source.get("logKey")),
                    String.valueOf(source.get("logType")),
                    map(source.get("options")),
                    tabType));
          }
        }
      }
    }
    return sources;
  }

  /**
   * What's wrong with a layout's sources, each in a sentence naming the tab and the source.
   *
   * @param logged each value the robot logged, by its key as AdvantageScope shows a log's, and its
   *     type as AdvantageScope names it ({@link #logged})
   * @param models the robot models the layout may name: those the build generates
   */
  static List<String> problems(
      List<Source> sources, Map<String, String> logged, Set<String> models) {
    List<String> problems = new ArrayList<>();
    String lastTab = "";
    String parent = "";
    for (Source source : sources) {
      if (!source.tab().equals(lastTab)) {
        lastTab = source.tab();
        parent = "";
      }
      Kind kind = KINDS.getOrDefault(source.tabType(), Map.of()).get(source.kind());
      if (kind == null) {
        problems.add(
            source + ": a " + source.tabType() + " tab has no source of kind " + source.kind());
        continue;
      }
      String childOf = kind.childOf();
      if (childOf == null) {
        String key = kind.parentKey();
        parent = key == null ? "" : key;
      } else if (!childOf.equals(parent)) {
        problems.add(
            source + ": hangs from a " + childOf + ", but follows none, so nothing shows it");
      }
      if (!kind.types().contains(source.type())) {
        problems.add(source + ": a " + source.kind() + " can't show a " + source.type());
      }
      String actual = logged.get(source.key());
      if (actual == null) {
        problems.add(source + ": " + source.key() + " is never logged");
      } else if (!actual.equals(source.type())) {
        problems.add(
            source + ": " + source.key() + " is logged as " + actual + ", not " + source.type());
      }
      Object model = source.options().get("model");
      if ("3D Field".equals(source.tabType())
          && WITH_MODELS.contains(source.kind())
          && !models.contains(String.valueOf(model))) {
        problems.add(
            source
                + ": the model "
                + model
                + " isn't one the build generates ("
                + String.join(", ", models.stream().sorted().toList())
                + ")");
      }
    }
    return problems;
  }

  /**
   * Every value a log holds, by its key as AdvantageScope shows it ({@code /RealOutputs/...}), and
   * its type as AdvantageScope names it: a struct by its name, anything else by its kind.
   */
  static Map<String, String> logged(Path log) throws IOException {
    DataLogReader reader = new DataLogReader(ByteBuffer.wrap(Files.readAllBytes(log)));
    if (!reader.isValid()) {
      throw new IOException("not a valid log: " + log);
    }
    Map<String, String> logged = new LinkedHashMap<>();
    for (DataLogRecord record : reader) {
      if (record.isStart()) {
        DataLogRecord.StartRecordData start = record.getStartData();
        String key = start.name.startsWith("/") ? start.name : "/" + start.name;
        logged.put(key, advantageScopeType(start.type));
      }
    }
    return logged;
  }

  /** A log's type, as AdvantageScope's log reader names it (src/hub/dataSources/wpilog). */
  static String advantageScopeType(String type) {
    if (type.startsWith("struct:")) {
      return type.substring("struct:".length());
    }
    return switch (type) {
      case "boolean" -> "Boolean";
      case "int", "int64", "float", "double" -> "Number";
      case "string", "json" -> "String";
      case "boolean[]" -> "BooleanArray";
      case "int[]", "int64[]", "float[]", "double[]" -> "NumberArray";
      case "string[]" -> "StringArray";
      default -> "Raw";
    };
  }

  private static Map<String, Kind> kinds(Object... namesAndKinds) {
    Map<String, Kind> kinds = new LinkedHashMap<>();
    for (int i = 0; i < namesAndKinds.length; i += 2) {
      kinds.put((String) namesAndKinds[i], (Kind) namesAndKinds[i + 1]);
    }
    return kinds;
  }

  private static List<String> concat(List<String> first, List<String> second) {
    List<String> both = new ArrayList<>(first);
    both.addAll(second);
    return List.copyOf(both);
  }

  private static Object parse(String json) {
    return Objects.requireNonNull(Jsonb.instance().type(Object.class).fromJson(json), "a layout");
  }

  @SuppressWarnings("unchecked")
  private static Map<String, Object> map(@Nullable Object value) {
    return value instanceof Map<?, ?> map ? (Map<String, Object>) map : Map.of();
  }

  @SuppressWarnings("unchecked")
  private static List<Object> list(@Nullable Object value) {
    return value instanceof List<?> list ? (List<Object>) list : List.of();
  }
}
