package com.michaelgrundvig.frc.world.field;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Reads a venue file: a small JSON document of ours that embeds WPILib's layout unchanged.
 *
 * <pre>{@code
 * {
 *   "format": 1,
 *   "name": "Practice field, October",
 *   "kind": "practice",                       // practice, event or official
 *   "base": "2026 FRC Rebuilt Welded",        // the official layout it was measured against
 *   "layout": { ... },                        // WPILib's layout, either format, as WPILib writes it
 *   "moved": {"14": "the panel was rehung after the survey"},   // optional
 *   "method": "hand",                                            // optional
 *   "date": "2026-10-12"                                         // optional
 * }
 * }</pre>
 *
 * <p>The embedded layout is either WPILib's field format ({@code field-tags}, {@code
 * field-dimensions}) or its earlier AprilTag layout ({@code tags}, {@code field}), which WPIcal and
 * PhotonVision use. A key the format doesn't have is a problem, so a misspelling can't pass
 * silently. Every problem is reported at once. Whether it fits the game's field is {@link
 * VenueCheck}'s to say.
 */
public final class VenueFile {
  private static final Set<String> KEYS =
      Set.of("format", "name", "kind", "base", "layout", "moved", "method", "date");
  private static final Set<String> SIGMA_KEYS =
      Set.of("alongWallMeters", "heightMeters", "outOfWallMeters", "yawDegrees", "tiltDegrees");

  private final List<String> problems = new ArrayList<>();

  private VenueFile() {}

  /**
   * The venue a file describes. Whether it fits a field is {@link VenueCheck}'s to say.
   *
   * @throws IllegalArgumentException listing every problem, if the file isn't a venue file
   */
  public static Venue read(String text) {
    VenueFile reader = new VenueFile();
    Object parsed = JsonText.parse(text);
    if (!(parsed instanceof Map<?, ?> top)) {
      throw new IllegalArgumentException("venue file: not a JSON object");
    }
    if (!top.containsKey("format") && (top.containsKey("field-tags") || top.containsKey("tags"))) {
      throw new IllegalArgumentException(
          "venue file: this is a bare WPILib layout, not a venue file. Wrap it with a name, kind"
              + " and base first (Pitwall's upload does)");
    }
    Venue venue = reader.venue(top, text);
    if (venue == null || !reader.problems.isEmpty()) {
      throw new IllegalArgumentException("venue file: " + String.join("; ", reader.problems));
    }
    return venue;
  }

  /**
   * A venue file around a bare WPILib layout (WPIcal's output, say), with method "unknown". The
   * layout goes in as it is.
   *
   * @param name what to call it; blank for the layout's own name, or "Uploaded layout"
   * @throws IllegalArgumentException if it isn't a WPILib layout, or doesn't make a venue file
   */
  public static Venue wrap(String layout, String name, Venue.Kind kind, String base) {
    Object parsed = JsonText.parse(layout);
    if (!(parsed instanceof Map<?, ?> top)
        || !(top.containsKey("field-tags") || top.containsKey("tags"))) {
      throw new IllegalArgumentException(
          "not a WPILib layout: it has no \"field-tags\" or \"tags\"");
    }
    if (name.isBlank()) {
      name = top.get("name") instanceof String own && !own.isBlank() ? own : "Uploaded layout";
    }
    String text =
        "{\n  \"format\": "
            + Venue.FORMAT
            + ",\n  \"name\": "
            + quoted(name)
            + ",\n  \"kind\": "
            + quoted(kind.name().toLowerCase(Locale.ROOT))
            + ",\n  \"base\": "
            + quoted(base)
            + ",\n  \"method\": \"unknown\",\n  \"layout\": "
            + layout.strip()
            + "\n}\n";
    Venue venue = read(text);
    return new Venue(
        venue.name(),
        venue.kind(),
        venue.base(),
        venue.layout(),
        venue.moved(),
        venue.method(),
        venue.date(),
        true,
        venue.text());
  }

  /**
   * Whether a file is a bare WPILib layout, which {@link #wrap} makes a venue of, rather than a
   * venue file.
   */
  public static boolean bare(String text) {
    try {
      return JsonText.parse(text) instanceof Map<?, ?> top
          && !top.containsKey("format")
          && (top.containsKey("field-tags") || top.containsKey("tags"));
    } catch (IllegalArgumentException e) {
      return false;
    }
  }

  /** Text as a JSON string. */
  private static String quoted(String text) {
    StringBuilder out = new StringBuilder("\"");
    for (char c : text.toCharArray()) {
      switch (c) {
        case '"' -> out.append("\\\"");
        case '\\' -> out.append("\\\\");
        case '\n' -> out.append("\\n");
        case '\r' -> out.append("\\r");
        case '\t' -> out.append("\\t");
        default -> {
          if (c < 0x20) {
            out.append(String.format(Locale.ROOT, "\\u%04x", (int) c));
          } else {
            out.append(c);
          }
        }
      }
    }
    return out.append('"').toString();
  }

  private @Nullable Venue venue(Map<?, ?> top, String text) {
    for (Object key : top.keySet()) {
      if (!KEYS.contains(key)) {
        problems.add("unknown key \"" + key + "\"");
      }
    }
    double format = number(top, "format", "format", Double.NaN);
    if (!Double.isNaN(format) && format != Venue.FORMAT) {
      problems.add("format " + format + " (this code reads format " + Venue.FORMAT + ")");
    }
    String name = text(top, "name", "name", null);
    if (name != null && name.isBlank()) {
      problems.add("name is blank");
    }
    Venue.Kind kind = kind(top);
    String base = text(top, "base", "base", null);
    Field layout = layout(top.get("layout"), name == null ? "" : name);
    Map<Integer, String> moved = new LinkedHashMap<>();
    for (Map.Entry<?, ?> entry : entries(top, "moved")) {
      Integer id = tagId(entry.getKey(), "moved");
      if (!(entry.getValue() instanceof String reason) || reason.isBlank()) {
        problems.add("moved: tag " + entry.getKey() + " needs a reason in words");
      } else if (id != null) {
        moved.put(id, reason);
      }
    }
    String method = text(top, "method", "method", "unknown");
    String date = text(top, "date", "date", "");
    if (name == null || kind == null || base == null || layout == null || method == null) {
      return null;
    }
    return new Venue(
        name, kind, base, layout, moved, method, date == null ? "" : date, false, text);
  }

  private Venue.@Nullable Kind kind(Map<?, ?> top) {
    String kind = text(top, "kind", "kind", null);
    if (kind == null) {
      return null;
    }
    try {
      return Venue.Kind.valueOf(kind.toUpperCase(Locale.ROOT));
    } catch (IllegalArgumentException e) {
      problems.add("kind \"" + kind + "\" (practice, event or official)");
      return null;
    }
  }

  /** The tags of either WPILib layout format, as a WPILib field named for the venue. */
  private @Nullable Field layout(@Nullable Object value, String name) {
    if (value == null) {
      problems.add("no layout");
      return null;
    }
    Map<?, ?> layout = map(value, "layout");
    if (layout == null) {
      return null;
    }
    boolean fieldFormat = layout.containsKey("field-tags");
    Object tags = layout.get(fieldFormat ? "field-tags" : "tags");
    Map<?, ?> size = map(layout.get(fieldFormat ? "field-dimensions" : "field"), "layout's size");
    if (!(tags instanceof List<?> tagList)) {
      problems.add("the layout has no list of tags (\"field-tags\" or \"tags\")");
      return null;
    }
    List<FieldTag> fieldTags = new ArrayList<>();
    Set<Integer> seen = new HashSet<>();
    for (Object item : tagList) {
      FieldTag tag = tag(item);
      if (tag != null) {
        if (!seen.add(tag.getID())) {
          problems.add("the layout has tag " + tag.getID() + " twice");
        }
        fieldTags.add(tag);
      }
    }
    if (size == null) {
      return null;
    }
    double length = number(size, "length", "the layout's length", Double.NaN);
    double width = number(size, "width", "the layout's width", Double.NaN);
    if (Double.isNaN(length) || Double.isNaN(width)) {
      return null;
    }
    String season = layout.get("season") instanceof String s ? s : "";
    String game = layout.get("game") instanceof String g ? g : "";
    String program = layout.get("program") instanceof String p ? p : "frc";
    return new Field(name, season, game, null, length, width, program, fieldTags);
  }

  private @Nullable FieldTag tag(Object item) {
    Map<?, ?> tag = map(item, "a tag in the layout");
    if (tag == null) {
      return null;
    }
    double id = number(tag, "ID", "a tag's ID", Double.NaN);
    Map<?, ?> pose = map(tag.get("pose"), "tag " + (Double.isNaN(id) ? "?" : (int) id) + "'s pose");
    if (Double.isNaN(id) || pose == null) {
      return null;
    }
    String which = "tag " + (int) id;
    Map<?, ?> translation = map(pose.get("translation"), which + "'s translation");
    Map<?, ?> rotation = map(pose.get("rotation"), which + "'s rotation");
    Map<?, ?> quaternion =
        rotation == null ? null : map(rotation.get("quaternion"), which + "'s quaternion");
    if (translation == null || quaternion == null) {
      return null;
    }
    double x = number(translation, "x", which + "'s x", Double.NaN);
    double y = number(translation, "y", which + "'s y", Double.NaN);
    double z = number(translation, "z", which + "'s z", Double.NaN);
    double w = number(quaternion, "W", which + "'s quaternion W", Double.NaN);
    double qx = number(quaternion, "X", which + "'s quaternion X", Double.NaN);
    double qy = number(quaternion, "Y", which + "'s quaternion Y", Double.NaN);
    double qz = number(quaternion, "Z", which + "'s quaternion Z", Double.NaN);
    if (Double.isNaN(x + y + z + w + qx + qy + qz)) {
      return null;
    }
    if (id != Math.rint(id) || id < 0) {
      problems.add("a tag's ID " + id + " isn't a whole number");
      return null;
    }
    double norm = Math.sqrt(w * w + qx * qx + qy * qy + qz * qz);
    if (Math.abs(norm - 1) > 1e-3) {
      problems.add(which + "'s quaternion isn't a rotation (length " + norm + ")");
      return null;
    }
    return new FieldTag(
        (int) id,
        new Pose3d(new Translation3d(x, y, z), new Rotation3d(new Quaternion(w, qx, qy, qz))));
  }

  private Iterable<? extends Map.Entry<?, ?>> entries(Map<?, ?> top, String key) {
    if (top.get(key) == null) {
      return List.of();
    }
    Map<?, ?> map = map(top.get(key), key);
    return map == null ? List.of() : map.entrySet();
  }

  private @Nullable Integer tagId(Object key, String where) {
    try {
      int id = Integer.parseInt(String.valueOf(key));
      if (id >= 0) {
        return id;
      }
    } catch (NumberFormatException e) {
      // reported below
    }
    problems.add(where + ": \"" + key + "\" isn't a tag number");
    return null;
  }

  private @Nullable Map<?, ?> map(@Nullable Object value, String what) {
    if (value instanceof Map<?, ?> map) {
      return map;
    }
    problems.add(what + (value == null ? " is missing" : " isn't an object"));
    return null;
  }

  private double number(Map<?, ?> map, String key, String what, double ifMissing) {
    Object value = map.get(key);
    if (value == null && !Double.isNaN(ifMissing)) {
      return ifMissing;
    }
    if (value instanceof Double number) {
      return number;
    }
    problems.add(
        (key.equals(what) ? key : what + "'s " + key)
            + (value == null ? " is missing" : " isn't a number"));
    return Double.NaN;
  }

  private @Nullable String text(
      Map<?, ?> map, String key, String what, @Nullable String ifMissing) {
    Object value = map.get(key);
    if (value == null && ifMissing != null) {
      return ifMissing;
    }
    if (value instanceof String text) {
      return text;
    }
    problems.add(what + (value == null ? " is missing" : " isn't text"));
    return null;
  }
}
