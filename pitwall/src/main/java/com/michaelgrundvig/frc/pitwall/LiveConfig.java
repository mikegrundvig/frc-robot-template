package com.michaelgrundvig.frc.pitwall;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.lang.reflect.RecordComponent;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.jspecify.annotations.Nullable;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.Units;

/**
 * A configuration record whose chosen values can be tuned while the robot runs, as WPILib tunables.
 * Publish it like any complex tunable, and each chosen value becomes a tunable number under it,
 * with its tuning {@link Limits}; read the configuration, tuned values applied, with {@link
 * #get()}:
 *
 * <pre>{@code
 * LiveConfig<SwerveConfig> drivetrain =
 *     LiveConfig.of("drivetrain", robot.drivetrain())
 *         .number("gains.steerKp", Limits.between(0, 100))
 *         .number("limits.driveStatorCurrent", Limits.between(10, 120)); // amps, as written
 * TunableRegistry.publish("Drive/config", drivetrain);
 * ...
 * drivetrain.get().gains().steerKp() // in the robot loop
 * }</pre>
 *
 * <p>A value is checked twice before it applies: against its tuning limits (what's safe to dial in
 * live), then by rebuilding the configuration through the records' own constructors (what's valid,
 * including checks across values, such as a range's ends). Several values change at once, all or
 * none, so values valid only together can move together. Pitwall's tee builds the {@link
 * #candidate} before applying anything, and gives every reason on the page; without Pitwall, a
 * refused value is quietly declined, and the dashboard shows the value in effect again.
 *
 * <p>Nothing is saved on the robot. {@link #changes()} lists what differs from the committed
 * configuration, each with the builder call that commits it.
 *
 * <p>Values are named by path, the record component names from the root: {@code gains.steerKp}. A
 * measure is tuned as a number in the unit its committed value was written in: amps, for {@code
 * Amps.of(30)}. Tune it from the robot loop's thread; {@link #get()} is safe from any thread.
 *
 * @param <T> the configuration record
 */
public final class LiveConfig<T extends Record> implements ComplexTunable {
  /**
   * A value that can be tuned.
   *
   * @param path where it is, such as {@code gains.steerKp}
   * @param unit the symbol of the unit it is tuned in, such as {@code A}; empty for a plain number
   * @param limits the range it may be tuned in
   */
  public record Knob(String path, String unit, Limits limits) {}

  /**
   * A tuned value that differs from the committed configuration.
   *
   * @param path where it is, such as {@code gains.steerKp}
   * @param committed the committed value, with its unit
   * @param tuned the tuned value, with its unit
   * @param builderCall the builder call that writes the tuned value into the configuration, such as
   *     {@code .steerKp(18.5)}
   */
  public record Change(String path, String committed, String tuned, String builderCall) {}

  /** Where a tunable value is: the components leading to it from the root, then its own. */
  private record Slot(
      Knob knob, List<RecordComponent> route, RecordComponent component, @Nullable Unit unit) {}

  private final String name;
  private final T committed;
  private final Map<String, Slot> slots = new LinkedHashMap<>();
  private final Map<TunableBase, String> children =
      Collections.synchronizedMap(new IdentityHashMap<>());
  private volatile T current;

  private LiveConfig(String name, T committed) {
    this.name = name;
    this.committed = committed;
    this.current = committed;
  }

  /**
   * A configuration to tune, starting as committed. Choose its tunable values with {@link #number}.
   *
   * @param name what it configures, for the report: {@code "drivetrain"}
   */
  public static <T extends Record> LiveConfig<T> of(String name, T committed) {
    return new LiveConfig<>(name, committed);
  }

  /**
   * Makes a value tunable, within limits. Choose them all before publishing.
   *
   * @param path the value's path from the root, such as {@code gains.steerKp}
   * @return this, to choose the next
   * @throws IllegalArgumentException if nothing is at the path, if it isn't a number or a measure,
   *     if it was chosen already, or if its committed value is outside the limits
   */
  public LiveConfig<T> number(String path, Limits limits) {
    if (slots.containsKey(path)) {
      throw new IllegalArgumentException(name + ": " + path + " is tunable already");
    }
    List<RecordComponent> route = new ArrayList<>();
    Record node = committed;
    String[] steps = path.split("\\.", -1);
    for (int i = 0; i < steps.length; i++) {
      RecordComponent component = component(node, steps[i], path);
      Object value = read(node, component);
      if (i < steps.length - 1) {
        if (!(value instanceof Record child)) {
          throw new IllegalArgumentException(name + ": " + path + " doesn't lead to a value");
        }
        route.add(component);
        node = child;
        continue;
      }
      Unit unit = null;
      if (value instanceof Measure<?> measure) {
        unit = measure.unit();
      } else if (!(value instanceof Double)) {
        throw new IllegalArgumentException(
            name + ": " + path + " isn't a number or a measure, so it can't be tuned");
      }
      Slot slot =
          new Slot(
              new Knob(path, unit == null ? "" : unit.symbol(), limits),
              List.copyOf(route),
              component,
              unit);
      String outside = limits.whyNot(number(slot, committed));
      if (outside != null) {
        throw new IllegalArgumentException(
            name + ": the committed " + path + " is outside its tuning limits: " + outside);
      }
      slots.put(path, slot);
    }
    return this;
  }

  /**
   * Makes a measure tunable, within limits given as measures, so they mean the same whatever unit
   * the committed value was written in: {@code measure("limits.driveStatorCurrent", Amps.of(10),
   * Amps.of(120))}. Choose them all before publishing.
   *
   * @return this, to choose the next
   * @throws IllegalArgumentException as {@link #number} does, or if the value isn't a measure of
   *     the limits' kind
   */
  public LiveConfig<T> measure(String path, Measure<?> min, Measure<?> max) {
    number(path, Limits.NONE); // finds and checks it
    Slot slot = slot(path);
    Unit unit = slot.unit();
    if (unit == null
        || min.unit().getBaseUnit() != unit.getBaseUnit()
        || max.unit().getBaseUnit() != unit.getBaseUnit()) {
      slots.remove(path);
      throw new IllegalArgumentException(
          name + ": " + path + " isn't a measure of the same kind as its limits");
    }
    Limits limits =
        Limits.between(
            unit.fromBaseUnits(min.baseUnitMagnitude()),
            unit.fromBaseUnits(max.baseUnitMagnitude()));
    Slot limited =
        new Slot(new Knob(path, slot.knob().unit(), limits), slot.route(), slot.component(), unit);
    String outside = limits.whyNot(number(limited, committed));
    if (outside != null) {
      slots.remove(path);
      throw new IllegalArgumentException(
          name + ": the committed " + path + " is outside its tuning limits: " + outside);
    }
    slots.put(path, limited);
    return this;
  }

  /** What it configures, as the report names it. */
  public String name() {
    return name;
  }

  /** The configuration with every tuned value applied. Safe to call from any thread. */
  public T get() {
    return current;
  }

  /** The configuration as committed. */
  public T committed() {
    return committed;
  }

  /** Every value that can be tuned, in the order they were chosen. */
  public List<Knob> knobs() {
    return slots.values().stream().map(Slot::knob).toList();
  }

  /**
   * A tunable value as it is now, in its knob's unit.
   *
   * @throws IllegalArgumentException if no tunable value has this path
   */
  public double value(String path) {
    return number(slot(path), current);
  }

  /**
   * A tunable value as committed, in its knob's unit.
   *
   * @throws IllegalArgumentException if no tunable value has this path
   */
  public double committedValue(String path) {
    return number(slot(path), committed);
  }

  /**
   * The configuration a change would make, checked but not made: every value one that's tunable, a
   * number, and within its limits; then the configuration rebuilt through its records' own checks.
   *
   * @param values new values by path, each in its knob's unit
   * @throws IllegalArgumentException if it can't be made, with every reason, one per line, each
   *     under this configuration's name: {@code "drivetrain: Gains.steerKd: 40 must be at most
   *     steerKp (16)"}
   */
  public T candidate(Map<String, Double> values) {
    List<String> problems = new ArrayList<>();
    T candidate = check(values, problems);
    if (candidate == null) {
      throw new IllegalArgumentException(String.join("\n", problems));
    }
    return candidate;
  }

  /**
   * Why a change can't be made, checking without making it: every reason {@link #candidate} gives.
   *
   * @param values new values by path, each in its knob's unit
   * @return null when it can be made
   */
  public @Nullable String whyNot(Map<String, Double> values) {
    List<String> problems = new ArrayList<>();
    return check(values, problems) == null ? String.join("\n", problems) : null;
  }

  /**
   * Makes a change, all or none, if {@link #whyNot} allows it.
   *
   * @return whether it was made
   */
  public boolean set(Map<String, Double> values) {
    T candidate = check(values, new ArrayList<>());
    if (candidate == null) {
      return false;
    }
    current = candidate;
    return true;
  }

  /**
   * Makes a configuration from {@link #candidate} the one in effect, for Pitwall's tee, which
   * checks more before it applies a change.
   */
  void accept(Record candidate) {
    if (candidate.getClass() != committed.getClass()) {
      throw new IllegalArgumentException(
          name + " is a " + committed.getClass().getSimpleName() + ", not a " + candidate);
    }
    current = sameType(candidate);
  }

  /** Returns every tuned value to its committed value. */
  public void revert() {
    current = committed;
  }

  /** The tuned values that differ from the committed configuration, in knob order. */
  public List<Change> changes() {
    T now = current;
    List<Change> changes = new ArrayList<>();
    for (Slot slot : slots.values()) {
      double was = number(slot, committed);
      double is = number(slot, now);
      if (Math.abs(is - was) > 1e-9 * Math.max(1, Math.abs(was))) {
        changes.add(
            new Change(
                slot.knob().path(), shown(slot, was), shown(slot, is), builderCall(slot, is)));
      }
    }
    return changes;
  }

  /**
   * The tuned values that differ from the committed configuration, as lines to print or log, each
   * with the builder call that commits it. Empty when nothing differs.
   */
  public List<String> report() {
    List<Change> changes = changes();
    if (changes.isEmpty()) {
      return List.of();
    }
    List<String> lines = new ArrayList<>();
    lines.add(
        "Tuned values differ from the committed "
            + name
            + " configuration. To keep them, write these builder calls into it:");
    for (Change change : changes) {
      lines.add(
          "  "
              + change.path()
              + ": "
              + change.committed()
              + " -> "
              + change.tuned()
              + "   "
              + change.builderCall());
    }
    return lines;
  }

  /**
   * Publishes each tunable value as a WPILib tunable number under the table, its path's dots as
   * slashes ({@code gains/steerKp}), with its limits and unit as properties. WPILib calls this.
   */
  @Override
  public void publishTunable(TunableTable table) {
    for (Slot slot : slots.values()) {
      String path = slot.knob().path();
      TunableConfig config = new TunableConfig();
      if (!slot.knob().unit().isEmpty()) {
        config = config.withProperty("unit", '"' + slot.knob().unit().replace("\"", "") + '"');
      }
      // Known as this configuration's before it's published, so Pitwall's tee, which sees it
      // published, can tell.
      TunableDouble tunable =
          TunableDouble.createConfig(
              () -> value(path),
              v -> set(Map.of(path, v)), // quietly declined when refused: it must not throw
              slot.knob().limits().on(config));
      children.put(tunable, path);
      table.publish(path.replace('.', '/'), tunable);
    }
  }

  /** Which tunable value a published tunable is, or null if it isn't one of these. */
  @Nullable String pathOf(TunableBase tunable) {
    return children.get(tunable);
  }

  private Slot slot(String path) {
    Slot slot = slots.get(path);
    if (slot == null) {
      throw new IllegalArgumentException(name + ": no tunable value at " + path);
    }
    return slot;
  }

  /**
   * The configuration a change would make, or null with every reason it can't be made added to
   * {@code problems}.
   */
  private @Nullable T check(Map<String, Double> values, List<String> problems) {
    for (Map.Entry<String, Double> entry : values.entrySet()) {
      String path = entry.getKey();
      double value = entry.getValue();
      Slot slot = slots.get(path);
      if (slot == null) {
        problems.add(name + ": " + path + " isn't tunable");
      } else if (!Double.isFinite(value)) {
        problems.add(name + ": " + path + " takes a number, got " + value);
      } else {
        String outside = slot.knob().limits().whyNot(value);
        if (outside != null) {
          problems.add(name + ": " + path + ": " + outside);
        }
      }
    }
    if (!problems.isEmpty()) {
      return null;
    }
    try {
      return rebuilt(values);
    } catch (RuntimeException refused) {
      // Whatever a record's checks throw refuses the change; a ConfigException has a line for
      // each value it rejects.
      String why = refused.getMessage();
      for (String line : (why != null ? why : refused.toString()).split("\n")) {
        problems.add(name + ": " + line);
      }
      return null;
    }
  }

  /** The configuration rebuilt with new values, each in its knob's unit; throws if rejected. */
  private T rebuilt(Map<String, Double> values) {
    Map<String, Object> replacements = new HashMap<>();
    for (Map.Entry<String, Double> entry : values.entrySet()) {
      Unit unit = slot(entry.getKey()).unit();
      double value = entry.getValue();
      replacements.put(entry.getKey(), unit == null ? value : unit.of(value));
    }
    return sameType(rebuild(current, "", replacements));
  }

  private RecordComponent component(Record record, String step, String path) {
    for (RecordComponent component : components(record)) {
      if (component.getName().equals(step)) {
        return component;
      }
    }
    throw new IllegalArgumentException(name + ": nothing is at " + path);
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
