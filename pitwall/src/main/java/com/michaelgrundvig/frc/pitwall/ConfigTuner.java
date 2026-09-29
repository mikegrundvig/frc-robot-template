package com.michaelgrundvig.frc.pitwall;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.lang.reflect.RecordComponent;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.Units;

/**
 * A live copy of a configuration whose {@link Tunable} values can change while the robot runs.
 *
 * <p>It starts as the committed configuration. Each change rebuilds the configuration through the
 * records' constructors, so a tuned value is checked exactly as a committed one is, and a value the
 * records reject leaves the configuration as it was. {@link #changes()} lists what differs from the
 * committed configuration, each with the builder call that writes it in.
 *
 * <p>Values are found by path, the record component names from the root: {@code
 * drivetrain.gains.steerKp}. A measure is tuned as a number in the unit its committed value was
 * written in: amps, for {@code Amps.of(30)}.
 *
 * @param <T> the configuration record at the root
 */
public final class ConfigTuner<T extends Record> {
  /**
   * A value that can be tuned.
   *
   * @param path where it is, such as {@code drivetrain.gains.steerKp}
   * @param unit the symbol of the unit it is tuned in, such as {@code A}; empty for a plain number
   */
  public record Knob(String path, String unit) {}

  /**
   * A tuned value that differs from the committed configuration.
   *
   * @param path where it is, such as {@code drivetrain.gains.steerKp}
   * @param committed the committed value, with its unit
   * @param tuned the tuned value, with its unit
   * @param builderCall the builder call that writes the tuned value into the configuration, such as
   *     {@code .steerKp(18.5)}
   */
  public record Change(String path, String committed, String tuned, String builderCall) {}

  /**
   * Where a tunable value is: the components leading to it from the root, then its own. {@code
   * index} is its place in the knob order.
   */
  private record Slot(
      Knob knob,
      int index,
      List<RecordComponent> route,
      RecordComponent component,
      @Nullable Unit unit) {}

  private final T committed;
  private final Map<String, Slot> slots = new LinkedHashMap<>();
  private final double[] values;
  private T current;

  /**
   * @throws IllegalArgumentException if something other than a number or a measure is marked {@link
   *     Tunable}
   */
  public ConfigTuner(T committed) {
    this.committed = committed;
    this.current = committed;
    discover(committed, List.of(), "");
    this.values = new double[slots.size()];
    refreshValues();
  }

  /** The configuration as committed. */
  public T committed() {
    return committed;
  }

  /** The configuration with every tuned value applied. */
  public T current() {
    return current;
  }

  /** Every value that can be tuned, in the order the records declare them. */
  public List<Knob> knobs() {
    return slots.values().stream().map(Slot::knob).toList();
  }

  /**
   * The current value at a path, in its knob's unit.
   *
   * @throws IllegalArgumentException if no tunable value has this path
   */
  public double value(String path) {
    return values[slot(path).index()];
  }

  /**
   * The committed value at a path, in its knob's unit.
   *
   * @throws IllegalArgumentException if no tunable value has this path
   */
  public double committedValue(String path) {
    return number(slot(path), committed);
  }

  /**
   * Tunes one value, in its knob's unit.
   *
   * @throws IllegalArgumentException if the records reject the value, which leaves the
   *     configuration as it was, or if no tunable value has this path
   */
  public void set(String path, double value) {
    set(Map.of(path, value));
  }

  /**
   * Tunes several values at once, all or none, each in its knob's unit. The configuration is
   * rebuilt once with every new value in place, so the records' checks see only the final
   * combination: values that are valid only together, such as a range's two ends, can move
   * together.
   *
   * @throws IllegalArgumentException if the records reject the combination, which leaves the
   *     configuration as it was, or if no tunable value has one of the paths
   */
  public void set(Map<String, Double> values) {
    Map<String, Object> replacements = new HashMap<>();
    for (Map.Entry<String, Double> entry : values.entrySet()) {
      Unit unit = slot(entry.getKey()).unit();
      double value = entry.getValue();
      replacements.put(entry.getKey(), unit == null ? value : unit.of(value));
    }
    current = sameType(rebuild(current, "", replacements));
    refreshValues();
  }

  /** Returns every tuned value to its committed value. */
  public void reset() {
    current = committed;
    refreshValues();
  }

  /** The tuned values that differ from the committed configuration, in knob order. */
  public List<Change> changes() {
    List<Change> changes = new ArrayList<>();
    for (Slot slot : slots.values()) {
      double was = number(slot, committed);
      double now = values[slot.index()];
      if (Math.abs(now - was) > 1e-9 * Math.max(1, Math.abs(was))) {
        changes.add(
            new Change(
                slot.knob().path(), shown(slot, was), shown(slot, now), builderCall(slot, now)));
      }
    }
    return changes;
  }

  private Slot slot(String path) {
    Slot slot = slots.get(path);
    if (slot == null) {
      throw new IllegalArgumentException("no tunable value at " + path);
    }
    return slot;
  }

  private void discover(Record record, List<RecordComponent> route, String prefix) {
    for (RecordComponent component : components(record)) {
      String path = prefix + component.getName();
      Object value = read(record, component);
      if (component.isAnnotationPresent(Tunable.class)) {
        Unit unit = null;
        if (value instanceof Measure<?> measure) {
          unit = measure.unit();
        } else if (!(value instanceof Double)) {
          throw new IllegalArgumentException(
              path + " is marked @Tunable, but only numbers and measures can be tuned");
        }
        Knob knob = new Knob(path, unit == null ? "" : unit.symbol());
        slots.put(path, new Slot(knob, slots.size(), route, component, unit));
      } else if (value instanceof Record child) {
        List<RecordComponent> next = new ArrayList<>(route);
        next.add(component);
        discover(child, List.copyOf(next), path + ".");
      }
    }
  }

  private void refreshValues() {
    for (Slot slot : slots.values()) {
      values[slot.index()] = number(slot, current);
    }
  }

  /** The value a slot points at in a configuration, in the slot's unit. */
  private static double number(Slot slot, Record root) {
    Record node = root;
    for (RecordComponent step : slot.route()) {
      node = (Record) read(node, step);
    }
    Object value = read(node, slot.component());
    Unit unit = slot.unit();
    return unit == null
        ? (Double) value
        : unit.fromBaseUnits(((Measure<?>) value).baseUnitMagnitude());
  }

  /**
   * A copy of {@code record} with the replacements below {@code prefix} made. Each record that
   * changes is built once, by its canonical constructor, with all of its new values.
   */
  private static Record rebuild(Record record, String prefix, Map<String, Object> replacements) {
    RecordComponent[] components = components(record);
    Class<?>[] types = new Class<?>[components.length];
    Object[] arguments = new Object[components.length];
    boolean changed = false;
    for (int i = 0; i < components.length; i++) {
      String path = prefix + components[i].getName();
      Object value = read(record, components[i]);
      Object replacement = replacements.get(path);
      if (replacement != null) {
        value = replacement;
        changed = true;
      } else if (value instanceof Record child && isAbove(path, replacements)) {
        value = rebuild(child, path + ".", replacements);
        changed = true;
      }
      types[i] = components[i].getType();
      arguments[i] = value;
    }
    return changed ? construct(record, types, arguments) : record;
  }

  private static boolean isAbove(String path, Map<String, Object> replacements) {
    return replacements.keySet().stream().anyMatch(replaced -> replaced.startsWith(path + "."));
  }

  private static Record construct(Record record, Class<?>[] types, Object[] arguments) {
    try {
      return (Record) record.getClass().getDeclaredConstructor(types).newInstance(arguments);
    } catch (InvocationTargetException e) {
      // The record rejected a value: pass its reason on unchanged.
      if (e.getCause() instanceof RuntimeException rejected) {
        throw rejected;
      }
      throw new IllegalStateException(e);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException("can't rebuild " + record.getClass().getName(), e);
    }
  }

  private static RecordComponent[] components(Record record) {
    RecordComponent[] components = record.getClass().getRecordComponents();
    return components == null ? new RecordComponent[0] : components;
  }

  private static Object read(Record record, RecordComponent component) {
    try {
      Object value = component.getAccessor().invoke(record);
      if (value == null) {
        throw new IllegalStateException(component.getName() + " is null");
      }
      return value;
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException("can't read " + component.getName(), e);
    }
  }

  private static String shown(Slot slot, double value) {
    Unit unit = slot.unit();
    return unit == null ? Double.toString(value) : value + " " + unit.symbol();
  }

  private static String builderCall(Slot slot, double value) {
    Unit unit = slot.unit();
    String argument =
        unit == null ? Double.toString(value) : unitsConstant(unit) + ".of(" + value + ")";
    return "." + slot.component().getName() + "(" + argument + ")";
  }

  /**
   * The name of the {@link Units} constant for a unit, as a configuration writes it: {@code Amps}.
   * Where several constants name the same unit, the plural and then the shortest wins.
   */
  private static String unitsConstant(Unit unit) {
    return Arrays.stream(Units.class.getFields())
        .filter(field -> Modifier.isStatic(field.getModifiers()) && staticValue(field) == unit)
        .map(Field::getName)
        .min(
            Comparator.comparing((String name) -> !name.endsWith("s"))
                .thenComparing(String::length)
                .thenComparing(Comparator.naturalOrder()))
        .orElse(unit.name());
  }

  private static @Nullable Object staticValue(Field field) {
    try {
      return field.get(null);
    } catch (IllegalAccessException e) {
      return null;
    }
  }

  @SuppressWarnings("unchecked") // rebuilding a record keeps its type
  private T sameType(Record rebuilt) {
    return (T) rebuilt;
  }
}
