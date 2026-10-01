package frc.robot.studies;

import frc.robot.lib.field.FieldModel;
import frc.robot.lib.field.Symmetry;
import io.avaje.json.JsonException;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.jspecify.annotations.Nullable;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The field the study runs on: only its size and AprilTags, since the study knows nothing about any
 * game. Either one of WPILib's own layouts ({@link Fields}; its default is the season's, so
 * updating WPILib for a new season moves the study to the new field), or a layout file, for the day
 * a new field is published, before WPILib has it: WPILib's form ({@code field-tags} and {@code
 * field-dimensions}), or the one FIRST publishes, as WPILib's layouts were written before 2027
 * ({@code tags} and {@code field}). The field's symmetry is whichever of the two its tags keep
 * best.
 */
public final class TagLayouts {
  private static final JsonType<Object> JSON = Jsonb.builder().build().type(Object.class);

  private TagLayouts() {}

  /**
   * One of WPILib's layouts, by its name in {@link Fields}: {@code FRC_2026_REBUILT_ANDY_MARK}.
   *
   * @throws IllegalArgumentException if WPILib has no layout by that name, listing those it has
   */
  public static FieldModel named(String name) {
    for (Fields field : Fields.values()) {
      if (field.name().equals(name)) {
        return of(field.loadField());
      }
    }
    throw new IllegalArgumentException(
        "WPILib has no field named "
            + name
            + ": "
            + Arrays.stream(Fields.values()).map(Fields::name).collect(Collectors.joining(", ")));
  }

  /**
   * @throws IllegalArgumentException if the file isn't a tag layout in either form, saying why
   */
  public static FieldModel read(Path file) throws IOException {
    String text = Files.readString(file);
    Map<?, ?> json;
    try {
      json = object(JSON.fromJson(text), "the file");
    } catch (JsonException e) {
      throw new IllegalArgumentException(file + " isn't JSON: " + e.getMessage(), e);
    }
    Field layout;
    if (json.containsKey("field-tags")) {
      layout = Field.loadFromFile(file);
    } else if (json.containsKey("tags") && json.containsKey("field")) {
      layout = older(json, file.getFileName().toString());
    } else {
      throw new IllegalArgumentException(
          file + " isn't a tag layout: it has neither \"field-tags\" nor \"tags\" and \"field\"");
    }
    return of(layout);
  }

  /** The layout as FIRST publishes it: {@code {"tags": [...], "field": {length, width}}}. */
  private static Field older(Map<?, ?> json, String name) {
    Map<?, ?> size = object(json.get("field"), "field");
    List<FieldTag> tags = new ArrayList<>();
    if (!(json.get("tags") instanceof List<?> list)) {
      throw new IllegalArgumentException("\"tags\" must be a list");
    }
    for (Object entry : list) {
      Map<?, ?> tag = object(entry, "a tag");
      Map<?, ?> pose = object(tag.get("pose"), "a tag's pose");
      Map<?, ?> at = object(pose.get("translation"), "a tag's translation");
      Map<?, ?> turned =
          object(object(pose.get("rotation"), "a tag's rotation").get("quaternion"), "quaternion");
      tags.add(
          new FieldTag(
              (int) number(tag, "ID"),
              new Pose3d(
                  new Translation3d(number(at, "x"), number(at, "y"), number(at, "z")),
                  new Rotation3d(
                      new Quaternion(
                          number(turned, "W"),
                          number(turned, "X"),
                          number(turned, "Y"),
                          number(turned, "Z"))))));
    }
    return new Field(
        name, "", "", null, number(size, "length"), number(size, "width"), "frc", tags);
  }

  /** The field model of a layout, with whichever symmetry puts every tag nearest a partner. */
  public static FieldModel of(Field layout) {
    FieldModel best = null;
    for (Symmetry symmetry : Symmetry.values()) {
      FieldModel model = new FieldModel(layout, symmetry, List.of(), List.of(), List.of());
      if (best == null || model.symmetryError() < best.symmetryError()) {
        best = model;
      }
    }
    if (best == null) {
      throw new IllegalStateException("no symmetry to choose from");
    }
    return best;
  }

  private static Map<?, ?> object(@Nullable Object value, String what) {
    if (value instanceof Map<?, ?> map) {
      return map;
    }
    throw new IllegalArgumentException(what + " must be a JSON object");
  }

  private static double number(Map<?, ?> json, String key) {
    if (json.get(key) instanceof Number number) {
      return number.doubleValue();
    }
    throw new IllegalArgumentException("\"" + key + "\" must be a number");
  }
}
