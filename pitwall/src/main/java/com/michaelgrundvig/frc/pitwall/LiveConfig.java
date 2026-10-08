package com.michaelgrundvig.frc.pitwall;

import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
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
import java.util.Objects;
import java.util.Set;
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
 * {@snippet :
 * return LiveConfig.of("drivetrain", committed)
 *     ...
 *     .number("gains.steerKp", Limits.between(0, 100))
 *     ...
 *     .measure("limits.driveStatorCurrent", Amps.of(10), Amps.of(120)) // a measure, in its unit
 *     ...
 * }
 *
 * <p>The template's drivetrain does that ({@code Drive.tuning}), publishes it ({@code
 * TunableRegistry.publish("Drive/config", drivetrain)}), and reads it in the robot loop ({@code
 * drivetrain.get().gains().steerKp()}).
 *
 * <p>A value is checked twice before it applies: against its tuning limits (what's safe to dial in
 * live), then by rebuilding the configuration through the records' own constructors (what's valid,
 * including checks across values, such as a range's ends). Several values change at once, all or
 * none, so values valid only together can move together. Pitwall's tee builds the {@link
 * #candidate} before applying anything, and gives every reason on the page; without Pitwall, a
 * refused value is quietly declined, and the dashboard shows the value in effect again.
 *
 * <p>A record refuses a value by throwing from its constructor, with a message the page can tie to
 * the value: one line for each value it rejects, {@code "where: what"}, where names the value from
 * the record that checked it, by its simple name and its path there ({@code Gains.steerKd}, or
 * {@code DrivetrainControl.limits.maxLinearSpeed} for a check across records). A record with a name
 * of its own puts it in brackets ({@code CameraConfig[front].tiltUp}). The page shows each line
 * beside its value; a line that names no tunable value (a check of the record as a whole) shows
 * beside the change's values in that record. Any message refuses the change; this form only places
 * it.
 *
 * <p>Nothing is saved on the robot. {@link #changes()} lists what differs from the committed
 * configuration, each with the builder call that commits it: by default the record's own builder
 * step, {@code .steerKp(18.5)}, or the call a value's tuning names where the configuration is
 * written otherwise, {@code .withKs(0.25)}.
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
   *     {@code .steerKp(18.5)}, or the one its tuning names ({@link #number(String, Limits,
   *     String)}), such as {@code .withKs(0.25)}
   */
  public record Change(String path, String committed, String tuned, String builderCall) {}

  /**
   * What the records' Javadoc says about this configuration, for the page and for dashboards: read
   * once, when the configuration is published, so the robot loop never reads it.
   *
   * @param record the root record's display name, such as {@code DrivetrainControl} ({@link
   *     ParameterDocs#nameOf})
   * @param doc the root record's own Javadoc
   * @param groups the records the tunable values are in, below the root, in the order the knobs
   *     reach them
   * @param values each tunable value's, by its path
   */
  public record Documentation(
      String record, ParameterDocs doc, List<Group> groups, Map<String, Value> values) {
    /**
     * A record the tunable values are in, below the root.
     *
     * @param path where it is, such as {@code gains}
     * @param record its display name, such as {@code DrivetrainControl.Gains}
     * @param doc its own Javadoc
     * @param meaning what it is to the record that holds it: that record's {@code @param} for it,
     *     such as {@code "control gains"}; empty when there's none
     */
    public record Group(String path, String record, ParameterDocs doc, String meaning) {}

    /**
     * A tunable value.
     *
     * @param record the display name of the record that declares it, such as {@code
     *     DrivetrainControl.Gains}
     * @param group the path of that record, the group it's in, such as {@code gains}; empty at the
     *     root
     * @param meaning what it means, its {@code @param} there, such as {@code "volts per radian of
     *     error"}; empty when there's none
     */
    public record Value(String record, String group, String meaning) {}

    /**
     * A tunable value's.
     *
     * @throws IllegalArgumentException if no tunable value has this path
     */
    public Value value(String path) {
      Value value = values.get(path);
      if (value == null) {
        throw new IllegalArgumentException("no tunable value at " + path);
      }
      return value;
    }
  }

  /**
   * Where a tunable value is: the components leading to it from the root, then its own; and the
   * call that commits it, {@code %s} where its value goes.
   */
  private record Slot(
      Knob knob,
      List<RecordComponent> route,
      RecordComponent component,
      @Nullable Unit unit,
      String commit) {}

  /**
   * Where a tuned value goes in the call that commits it ({@link #number(String, Limits, String)}).
   */
  private static final String VALUE = "%s";

  /** Writes text as a JSON string, quotes and backslashes escaped, as properties carry it. */
  private static final JsonType<String> TEXT = Jsonb.builder().build().type(String.class);

  /**
   * One reason this configuration refuses a change, tied to the values it concerns.
   *
   * @param layer which check refused it: a value's own ({@code VALUE}, {@code LIMIT}), or a
   *     record's ({@code RECORD})
   * @param fields the values it concerns, by path: the one it names; for a record's check that
   *     names none, the change's values in that record
   * @param record for a record's check, the record that refused it ({@code
   *     DrivetrainControl.Gains}); empty otherwise
   * @param group for a record's check that names no value, where that record is in this
   *     configuration ({@code travel}, empty for the root); empty otherwise
   * @param text why, as the check said it
   * @param line the reason as one line, under this configuration's name: {@code "drivetrain:
   *     Gains.steerKd: 40 must be at most steerKp (16)"}
   */
  record Problem(
      Outcome.Layer layer,
      List<String> fields,
      String record,
      String group,
      String text,
      String line) {}

  /**
   * A record that refused its values as the configuration was rebuilt: which record, where it is,
   * and what it threw.
   */
  private static final class Rejected extends RuntimeException {
    private static final long serialVersionUID = 1;

    private final Class<?> record;
    private final String prefix;

    Rejected(Class<?> record, String prefix, RuntimeException cause) {
      super(cause);
      this.record = record;
      this.prefix = prefix;
    }
  }

  private final String name;
  private final T committed;
  private final Map<String, Slot> slots = new LinkedHashMap<>();
  private final Map<TunableBase, String> children =
      Collections.synchronizedMap(new IdentityHashMap<>());
  private volatile T current;

  /** What the records' Javadoc says, once it's been read; again after a value is chosen. */
  private volatile @Nullable Documentation documentation;

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
    return chosen(path, limits, null);
  }

  /**
   * Makes a value tunable, within limits, committed by a call of its own where the configuration is
   * written by more than its builder: {@code number("gains.kS", Limits.between(0, 2),
   * ".withKs(%s)")}, the value in its place as written, {@code .withKs(0.25)}, a measure with its
   * unit, {@code .withSupplyLimit(Amps.of(50.0))}.
   *
   * @param commit the call, with {@code %s} where the value goes
   * @return this, to choose the next
   * @throws IllegalArgumentException as {@link #number(String, Limits)} does, or if the call has no
   *     {@code %s}
   */
  public LiveConfig<T> number(String path, Limits limits, String commit) {
    return chosen(path, limits, commit);
  }

  /** A value made tunable, committed by its call, or by its builder step for a null one. */
  private LiveConfig<T> chosen(String path, Limits limits, @Nullable String commit) {
    if (commit != null && !commit.contains(VALUE)) {
      throw new IllegalArgumentException(
          name + ": " + path + "'s call, " + commit + ", has no " + VALUE + " for its value");
    }
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
              unit,
              commit != null ? commit : "." + component.getName() + "(" + VALUE + ")");
      String outside = limits.whyNot(number(slot, committed));
      if (outside != null) {
        throw new IllegalArgumentException(
            name + ": the committed " + path + " is outside its tuning limits: " + outside);
      }
      slots.put(path, slot);
      documentation = null;
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
    return measured(path, min, max, null);
  }

  /**
   * Makes a measure tunable, within limits given as measures, committed by a call of its own, as
   * {@link #number(String, Limits, String)} is: {@code measure("currentLimits.supply", Amps.of(5),
   * Amps.of(80), ".withSupplyLimit(%s)")}.
   *
   * @return this, to choose the next
   * @throws IllegalArgumentException as {@link #measure(String, Measure, Measure)} does, or if the
   *     call has no {@code %s}
   */
  public LiveConfig<T> measure(String path, Measure<?> min, Measure<?> max, String commit) {
    return measured(path, min, max, commit);
  }

  /** A measure made tunable, committed by its call, or by its builder step for a null one. */
  private LiveConfig<T> measured(
      String path, Measure<?> min, Measure<?> max, @Nullable String commit) {
    chosen(path, Limits.NONE, commit); // finds and checks it
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
        new Slot(
            new Knob(path, slot.knob().unit(), limits),
            slot.route(),
            slot.component(),
            unit,
            slot.commit());
    String outside = limits.whyNot(number(limited, committed), limited.knob().unit());
    if (outside != null) {
      slots.remove(path);
      throw new IllegalArgumentException(
          name + ": the committed " + path + " is outside its tuning limits: " + outside);
    }
    slots.put(path, limited);
    documentation = null;
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
   * The record that declares a tunable value, as the configuration holds it: {@code
   * DrivetrainControl.Gains} for {@code gains.steerKp}.
   *
   * @throws IllegalArgumentException if no tunable value has this path
   */
  public Class<? extends Record> recordOf(String path) {
    return slot(path).component().getDeclaringRecord().asSubclass(Record.class);
  }

  /**
   * What a tunable value means: its {@code @param} in the Javadoc of the record that declares it
   * ({@link ParameterDocs}), such as {@code "volts per radian of error"}; empty when there's none.
   *
   * @throws IllegalArgumentException if no tunable value has this path
   */
  public String meaning(String path) {
    return documentation().value(slot(path).knob().path()).meaning();
  }

  /**
   * What the records' Javadoc says about this configuration. It's read when the configuration is
   * published, or when first asked for before that. Safe to call from any thread.
   */
  public Documentation documentation() {
    Documentation known = documentation;
    if (known == null) {
      known = describe();
      documentation = known;
    }
    return known;
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
    List<Problem> problems = new ArrayList<>();
    T candidate = candidate(values, problems);
    if (candidate == null) {
      throw new IllegalArgumentException(String.join("\n", lines(problems)));
    }
    return candidate;
  }

  /**
   * The configuration a change would make, as {@link #candidate(Map)} makes it, or null with every
   * reason it can't be made added to {@code problems}, each tied to the values it concerns.
   */
  @Nullable T candidate(Map<String, Double> values, List<Problem> problems) {
    return check(values, problems);
  }

  /**
   * Why a change can't be made, checking without making it: every reason {@link #candidate} gives.
   *
   * @param values new values by path, each in its knob's unit
   * @return null when it can be made
   */
  public @Nullable String whyNot(Map<String, Double> values) {
    List<Problem> problems = new ArrayList<>();
    check(values, problems);
    return problems.isEmpty() ? null : String.join("\n", lines(problems));
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
   * slashes ({@code gains/steerKp}), with its limits, unit, and meaning ({@code "description"}) as
   * properties, for any NetworkTables tool to read. WPILib calls this.
   */
  @Override
  public void publishTunable(TunableTable table) {
    Documentation docs = documentation(); // read now, at startup, never in the robot loop
    for (Slot slot : slots.values()) {
      String path = slot.knob().path();
      TunableConfig config = new TunableConfig();
      if (!slot.knob().unit().isEmpty()) {
        config = config.withProperty("unit", TEXT.toJson(slot.knob().unit()));
      }
      String meaning = docs.value(path).meaning();
      if (!meaning.isEmpty()) {
        config = config.withProperty("description", TEXT.toJson(meaning));
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

  /** Reads what the records' Javadoc says, following each value's route as it was found. */
  private Documentation describe() {
    Class<? extends Record> root = committed.getClass();
    Map<String, Documentation.Group> groups = new LinkedHashMap<>();
    Map<String, Documentation.Value> values = new LinkedHashMap<>();
    for (Slot slot : slots.values()) {
      List<RecordComponent> route = slot.route();
      StringBuilder path = new StringBuilder();
      for (int i = 0; i < route.size(); i++) {
        RecordComponent step = route.get(i);
        path.append(i == 0 ? "" : ".").append(step.getName());
        // The record at this path declares the next step: its components came from its class.
        Class<?> record =
            (i + 1 < route.size() ? route.get(i + 1) : slot.component()).getDeclaringRecord();
        groups.computeIfAbsent(
            path.toString(),
            at ->
                new Documentation.Group(
                    at, ParameterDocs.nameOf(record), ParameterDocs.of(record), meaningOf(step)));
      }
      values.put(
          slot.knob().path(),
          new Documentation.Value(
              ParameterDocs.nameOf(slot.component().getDeclaringRecord()),
              path.toString(),
              meaningOf(slot.component())));
    }
    return new Documentation(
        ParameterDocs.nameOf(root),
        ParameterDocs.of(root),
        List.copyOf(groups.values()),
        Collections.unmodifiableMap(values));
  }

  /** What a component means: its {@code @param} in the record that declares it, or empty. */
  private static String meaningOf(RecordComponent component) {
    return ParameterDocs.of(component.getDeclaringRecord())
        .params()
        .getOrDefault(component.getName(), "");
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
  private @Nullable T check(Map<String, Double> values, List<Problem> problems) {
    int before = problems.size();
    for (Map.Entry<String, Double> entry : values.entrySet()) {
      String path = entry.getKey();
      double value = entry.getValue();
      Slot slot = slots.get(path);
      if (slot == null) {
        String why = path + " isn't tunable";
        problems.add(value(Outcome.Layer.VALUE, path, why, name + ": " + why));
      } else if (!Double.isFinite(value)) {
        String why = path + " takes a number, got " + value;
        problems.add(value(Outcome.Layer.VALUE, path, why, name + ": " + why));
      } else {
        String outside = slot.knob().limits().whyNot(value, slot.knob().unit());
        if (outside != null) {
          problems.add(
              value(Outcome.Layer.LIMIT, path, outside, name + ": " + path + ": " + outside));
        }
      }
    }
    if (problems.size() > before) {
      return null;
    }
    try {
      return rebuilt(values);
    } catch (RuntimeException refused) {
      // Whatever a record's checks throw refuses the change; a ConfigException has a line for
      // each value it rejects, each naming it from the record that checked it. Anything else
      // thrown (a record that couldn't be read or built) refuses it too, as the root's.
      Rejected rejected =
          refused instanceof Rejected known
              ? known
              : new Rejected(committed.getClass(), "", refused);
      Throwable cause = Objects.requireNonNull(rejected.getCause());
      String why = cause.getMessage();
      for (String line : (why != null ? why : cause.toString()).split("\n")) {
        problems.add(refusedBy(rejected, line, values.keySet()));
      }
      return null;
    }
  }

  private static Problem value(Outcome.Layer layer, String path, String why, String line) {
    return new Problem(layer, List.of(path), "", "", why, line);
  }

  /**
   * A record's refusal, one line of it, tied to the value it names. A record names a value from
   * itself, {@code where: what}: {@code Gains.steerKd}, {@code CameraConfig[front].tiltUp}, or a
   * value further in, {@code DrivetrainControl.limits.maxLinearSpeed}. A line naming no tunable
   * value (a whole-record check, or a value that isn't tunable) concerns the change's values in
   * that record, and the record itself.
   */
  private Problem refusedBy(Rejected rejected, String line, Set<String> changed) {
    String record = ParameterDocs.nameOf(rejected.record);
    String field = "";
    String what = line;
    int colon = line.indexOf(": ");
    if (colon > 0) {
      String rest = fromRecord(line.substring(0, colon), rejected.record.getSimpleName());
      if (rest != null && slots.containsKey(rejected.prefix + rest)) {
        field = rejected.prefix + rest;
        what = line.substring(colon + 2);
      }
    }
    if (!field.isEmpty()) {
      return new Problem(
          Outcome.Layer.RECORD, List.of(field), record, "", what, name + ": " + line);
    }
    List<String> within =
        changed.stream().filter(path -> path.startsWith(rejected.prefix)).sorted().toList();
    String group =
        rejected.prefix.isEmpty() ? "" : rejected.prefix.substring(0, rejected.prefix.length() - 1);
    return new Problem(Outcome.Layer.RECORD, within, record, group, line, name + ": " + line);
  }

  /**
   * The path a record's violation names, from the record: {@code steerKd} for {@code Gains.steerKd}
   * or {@code Gains[left].steerKd} in a record named Gains; null for one that names no value of
   * that record.
   */
  private static @Nullable String fromRecord(String where, String simpleName) {
    if (where.startsWith(simpleName + ".")) {
      return where.substring(simpleName.length() + 1);
    }
    int named = where.indexOf("].");
    if (where.startsWith(simpleName + "[") && named > 0) {
      return where.substring(named + 2);
    }
    return null;
  }

  /** The reasons as lines, as {@link #candidate(Map)} and {@link #whyNot} give them. */
  private static List<String> lines(List<Problem> problems) {
    return problems.stream().map(Problem::line).toList();
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
    if (!changed) {
      return record;
    }
    try {
      return construct(record, types, arguments);
    } catch (RuntimeException refused) {
      // Which record refused, and where it is, so its reasons can be tied to its values.
      throw new Rejected(record.getClass(), prefix, refused);
    }
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
    return slot.commit().replace(VALUE, argument);
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
