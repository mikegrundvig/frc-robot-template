package com.michaelgrundvig.frc.pitwall.simulator;

import com.michaelgrundvig.frc.pitwall.Numbers;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;
import org.jspecify.annotations.Nullable;

/**
 * The simulated robot's components that can fail, with their faults: the registry the page applies
 * faults through.
 *
 * <p>The robot's simulation declares its components ({@link #add}) as the robot starts. From then
 * on, only the robot loop applies or undoes a fault ({@link #apply}, {@link #undo}), since a fault
 * changes the simulated world, which the loop steps. Looking a name up ({@link #whyNot}) is safe
 * from any thread.
 */
public final class Components {
  private final Map<String, Component> components = new LinkedHashMap<>();
  private final Consumer<String> say;

  /** The values each lasting fault was last applied with, by component and fault. */
  private final Map<String, Map<String, Values>> applied = new LinkedHashMap<>();

  /** What was declared, for lookups from any thread: replaced whole on each addition. */
  private volatile Map<String, Component> declared = Map.of();

  /** Each component as the page shows it, and what it was built from, rebuilt when that changes. */
  private final Map<String, List<Object>> shownFrom = new LinkedHashMap<>();

  private final Map<String, Object> shown = new LinkedHashMap<>();

  /**
   * @param say told each fault applied or undone, in a sentence, for the console and the log
   */
  public Components(Consumer<String> say) {
    this.say = say;
  }

  /**
   * Declares a component that can fail. Call it as the robot starts, from the robot's thread.
   *
   * @return this, to declare the next
   * @throws IllegalArgumentException if a component already has its name
   */
  public Components add(Component component) {
    if (components.putIfAbsent(component.name(), component) != null) {
      throw new IllegalArgumentException("two components are named " + component.name());
    }
    declared = Collections.unmodifiableMap(new LinkedHashMap<>(components));
    return this;
  }

  /** Every component, in the order declared. */
  public List<Component> components() {
    return List.copyOf(components.values());
  }

  /**
   * Why a fault can't be applied: no component or fault by that name, or a parameter it doesn't
   * take or a value out of range. Safe from any thread.
   *
   * @return null when it can
   */
  public @Nullable String whyNot(Setting setting) {
    Component component = declared.get(setting.component());
    if (component == null) {
      return "There is no component named "
          + setting.component()
          + "; there are "
          + String.join(", ", declared.keySet());
    }
    Fault fault = fault(component, setting.fault());
    if (fault == null) {
      return component.name()
          + " has no fault named "
          + setting.fault()
          + "; it has "
          + String.join(", ", component.faults().stream().map(Fault::name).toList());
    }
    return fault.whyNot(setting.values());
  }

  /**
   * Why a fault can't be undone: there's none by that name, or it's an event, which nothing undoes.
   * Safe from any thread.
   *
   * @return null when it can be
   */
  public @Nullable String whyNotUndo(String component, String fault) {
    String missing = whyNot(Setting.of(component, fault));
    if (missing != null) {
      return missing;
    }
    Fault named = lookup(component, fault);
    if (named.kind() == Fault.Kind.EVENT) {
      return component + ": " + fault + " happens once, and nothing undoes it: " + named.undo();
    }
    return null;
  }

  /**
   * Applies a fault: turns a lasting one on, or makes an event happen. Call it from the robot loop.
   *
   * @return why it couldn't be applied (its name, its values, or its own code refused), or null
   *     when it was
   */
  public @Nullable String apply(Setting setting) {
    String why = whyNot(setting);
    if (why != null) {
      return why;
    }
    Fault fault = lookup(setting.component(), setting.fault());
    Values values = fault.values(setting.values());
    try {
      fault.apply(values);
    } catch (RuntimeException e) {
      return setting.component() + ": " + setting.fault() + " failed: " + describe(e);
    }
    if (fault.kind() == Fault.Kind.LASTING) {
      applied
          .computeIfAbsent(setting.component(), name -> new LinkedHashMap<>())
          .put(fault.name(), values);
    }
    say.accept(
        setting.component()
            + ": "
            + fault.name()
            + (fault.kind() == Fault.Kind.LASTING ? ", on" : "")
            + shown(fault, values));
    return null;
  }

  /**
   * Undoes a lasting fault, if it's on. Call it from the robot loop.
   *
   * @return why it couldn't be undone, or null when it was (or was off already)
   */
  public @Nullable String undo(String component, String fault) {
    String why = whyNotUndo(component, fault);
    if (why != null) {
      return why;
    }
    Fault named = lookup(component, fault);
    if (!named.isOn()) {
      return null;
    }
    try {
      named.undoIt();
    } catch (RuntimeException e) {
      return component + ": undoing " + fault + " failed: " + describe(e);
    }
    say.accept(component + ": " + fault + ", undone: " + named.undo());
    return null;
  }

  /**
   * Undoes every lasting fault that's on: the robot as it was built. Call it from the robot loop.
   *
   * @return why any couldn't be undone, or null when all were
   */
  public @Nullable String undoAll() {
    List<String> problems = new ArrayList<>();
    for (Component component : components.values()) {
      for (Fault fault : component.faults()) {
        if (fault.kind() == Fault.Kind.LASTING && fault.isOn()) {
          String why = undo(component.name(), fault.name());
          if (why != null) {
            problems.add(why);
          }
        }
      }
    }
    return problems.isEmpty() ? null : String.join("; ", problems);
  }

  /** The lasting faults on now, as "Component: Fault", in the order declared. */
  public List<String> on() {
    List<String> on = new ArrayList<>();
    for (Component component : components.values()) {
      for (Fault fault : component.faults()) {
        if (fault.isOn()) {
          on.add(component.name() + ": " + fault.name());
        }
      }
    }
    return on;
  }

  /**
   * Every component as the page shows it, by name: {@code {"kind", "faults": {name: {"does",
   * "lasting", "undo", "parameters": [{"name", "unit", "min", "max", "initial"}], "on",
   * "values"}}}}. A lasting fault that's on gives the values it was applied with. Call it from the
   * robot loop; a component's map is rebuilt only when one of its faults turns on or off.
   */
  public Map<String, Object> view() {
    for (Component component : components.values()) {
      List<Object> from = new ArrayList<>();
      for (Fault fault : component.faults()) {
        boolean on = fault.isOn();
        from.add(on);
        from.add(on ? valuesOf(component, fault) : Values.NONE);
      }
      if (!from.equals(shownFrom.get(component.name()))) {
        shownFrom.put(component.name(), from);
        shown.put(component.name(), shown(component));
      }
    }
    return new LinkedHashMap<>(shown);
  }

  private Map<String, Object> shown(Component component) {
    Map<String, Object> faults = new LinkedHashMap<>();
    for (Fault fault : component.faults()) {
      Map<String, Object> described = new LinkedHashMap<>();
      described.put("does", fault.does());
      described.put("lasting", fault.kind() == Fault.Kind.LASTING);
      described.put("undo", fault.undo());
      List<Object> parameters = new ArrayList<>();
      for (Parameter parameter : fault.parameters()) {
        Map<String, Object> shown = new LinkedHashMap<>();
        shown.put("name", parameter.name());
        shown.put("unit", parameter.unit());
        shown.put("min", parameter.limits().min());
        shown.put("max", parameter.limits().max());
        shown.put("initial", parameter.initial());
        parameters.add(shown);
      }
      described.put("parameters", parameters);
      if (fault.kind() == Fault.Kind.LASTING) {
        boolean on = fault.isOn();
        described.put("on", on);
        described.put("values", on ? valuesOf(component, fault).asMap() : Map.of());
      }
      faults.put(fault.name(), described);
    }
    Map<String, Object> shown = new LinkedHashMap<>();
    shown.put("kind", component.kind());
    shown.put("faults", faults);
    return shown;
  }

  /** A fault's values as people read them: {@code ": Scale 1.15, Drift 0.2 °/min"}. */
  private static String shown(Fault fault, Values values) {
    List<String> shown = new ArrayList<>();
    for (Parameter parameter : fault.parameters()) {
      shown.add(
          parameter.name()
              + " "
              + Numbers.shown(values.get(parameter.name()))
              + (parameter.unit().isEmpty() ? "" : " " + parameter.unit()));
    }
    return shown.isEmpty() ? "" : ": " + String.join(", ", shown);
  }

  /** The values a lasting fault was last applied with here; none if it was turned on elsewhere. */
  private Values valuesOf(Component component, Fault fault) {
    return applied.getOrDefault(component.name(), Map.of()).getOrDefault(fault.name(), Values.NONE);
  }

  /** A declared fault, once its names are known to be right. */
  private Fault lookup(String component, String fault) {
    return Objects.requireNonNull(fault(Objects.requireNonNull(declared.get(component)), fault));
  }

  private static @Nullable Fault fault(Component component, String name) {
    for (Fault fault : component.faults()) {
      if (fault.name().equals(name)) {
        return fault;
      }
    }
    return null;
  }

  private static String describe(RuntimeException e) {
    String why = e.getMessage();
    return why != null ? why : e.toString();
  }
}
