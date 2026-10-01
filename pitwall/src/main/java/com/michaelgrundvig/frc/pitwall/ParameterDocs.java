package com.michaelgrundvig.frc.pitwall;

import io.avaje.json.JsonException;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.RecordComponent;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import org.jspecify.annotations.Nullable;

/**
 * What a configuration record's Javadoc says, for the page to show beside its values: the record's
 * own text, and what each value means, its {@code @param}. Pitwall's documentation processor
 * ({@code pitwall/docs-processor}) writes it into the robot program as it compiles, one resource
 * per record, so the page needs no source on the robot:
 *
 * <pre>{@code
 * ParameterDocs.of(SwerveConfig.Gains.class).summary() // "Drive: velocity control on the ..."
 * ParameterDocs.param(SwerveConfig.class, "gains.steerKp") // "volts per radian of error"
 * }</pre>
 *
 * <p>The text is plain, laid out for a page that shows text as it is: paragraphs separated by a
 * blank line, each list item on a line of its own starting {@code "- "}, and code in backticks.
 *
 * <p>A record that has no documentation (no Javadoc, or compiled without the processor) gets {@link
 * #NONE}, never an error: the page asks for what's missing instead.
 *
 * @param summary the first sentence of the record's Javadoc
 * @param text all of its main text, the summary included
 * @param params what each value means, by component name, in the record's order; only the values
 *     with a {@code @param}
 */
public record ParameterDocs(String summary, String text, Map<String, String> params) {
  /** No documentation. */
  public static final ParameterDocs NONE = new ParameterDocs("", "", Map.of());

  /**
   * Where the processor writes each record's documentation: then its binary name and ".json"
   * ({@code SwerveConfig$Gains.json}).
   */
  static final String FOLDER = "META-INF/pitwall/docs/";

  private static final JsonType<Object> JSON = Jsonb.builder().build().type(Object.class);

  /** Each record's documentation, read the first time it's asked for. */
  private static final ClassValue<ParameterDocs> BY_RECORD =
      new ClassValue<>() {
        @Override
        protected ParameterDocs computeValue(Class<?> record) {
          return read(record);
        }
      };

  public ParameterDocs {
    params = Collections.unmodifiableMap(new LinkedHashMap<>(params));
  }

  /** A record's documentation, or {@link #NONE}. Safe to call from any thread. */
  public static ParameterDocs of(Class<?> record) {
    return BY_RECORD.get(record);
  }

  /**
   * What a value means, found by its path from a root record, as {@link LiveConfig} names values:
   * {@code param(SwerveConfig.class, "gains.steerKp")} is the {@code @param steerKp} of {@code
   * SwerveConfig.Gains}, the record that declares it.
   *
   * @return empty when the value has no {@code @param}, or the path leads to no value
   */
  public static String param(Class<?> root, String path) {
    Class<?> record = root;
    String[] steps = path.split("\\.", -1);
    for (int i = 0; i < steps.length - 1; i++) {
      Class<?> next = componentType(record, steps[i]);
      if (next == null) {
        return "";
      }
      record = next;
    }
    return of(record).params().getOrDefault(steps[steps.length - 1], "");
  }

  private static @Nullable Class<?> componentType(Class<?> record, String name) {
    RecordComponent[] components = record.getRecordComponents(); // null for a class
    if (components != null) {
      for (RecordComponent component : components) {
        if (component.getName().equals(name)) {
          return component.getType();
        }
      }
    }
    return null;
  }

  /** Reads a record's documentation through its class loader, from wherever the record came. */
  private static ParameterDocs read(Class<?> record) {
    ClassLoader loader = record.getClassLoader();
    if (loader == null) {
      return NONE; // Java's own classes
    }
    Object parsed;
    try (InputStream in = loader.getResourceAsStream(FOLDER + record.getName() + ".json")) {
      if (in == null) {
        return NONE;
      }
      parsed = JSON.fromJson(in.readAllBytes());
    } catch (IOException | JsonException e) {
      return NONE; // documentation is a help, never a reason to fail
    }
    if (!(parsed instanceof Map<?, ?> doc)) {
      return NONE;
    }
    Map<String, String> params = new LinkedHashMap<>();
    if (doc.get("params") instanceof Map<?, ?> found) {
      found.forEach(
          (name, text) -> {
            if (name instanceof String component && text instanceof String meaning) {
              params.put(component, meaning);
            }
          });
    }
    return new ParameterDocs(text(doc.get("summary")), text(doc.get("text")), params);
  }

  private static String text(@Nullable Object value) {
    return value instanceof String text ? text : "";
  }
}
