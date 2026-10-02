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
 * The simulated robot's components that can fail, with their faults, and the scenarios made of
 * them: the registry the page applies faults through, and the simulator applies a scenario through
 * when it starts in one.
 *
 * <p>The robot's simulation declares its components ({@link #add}) and scenarios ({@link
 * #scenario}) as the robot starts. From then on, only the robot loop applies or undoes a fault
 * ({@link #apply}, {@link #undo}), since a fault changes the simulated world, which the loop steps.
 * Looking a name up ({@link #whyNot}) is safe from any thread.
 *
 * <p>A scenario is a named set of faults ({@link Setting}s): applying it applies each in turn, and
 * the page shows the lasting ones as on, each undone on its own or all together ({@link #undoAll}).
 */
public final class Components {
  /**
   * A named set of faults.
   *
   * @param description what it does to the robot
   */
  private record Scenario(String name, String description, List<Setting> faults) {}

  private final Map<String, Component> components = new LinkedHashMap<>();
  private final Map<String, Scenario> scenarios = new LinkedHashMap<>();
  private final Consumer<String> say;

  /** The values each lasting fault was last applied with, by component and fault. */
  private final Map<String, Map<String, Values>> applied = new LinkedHashMap<>();

  /** What was declared, for lookups from any thread: replaced whole on each addition. */
  private volatile Map<String, Component> declared = Map.of();

  private volatile Map<String, Scenario> declaredScenarios = Map.of();

  /** Each component as the page shows it, and what it was built from, rebuilt when that changes. */
  private final Map<String, List<Object>> shownFrom = new LinkedHashMap<>();

  private final Map<String, Object> shown = new LinkedHashMap<>();

  /** {@link #shown} as last handed out, and the scenarios' view, made again only on a change. */
  private Map<String, Object> shownView = Map.of();

  private @Nullable Map<String, Scenario> scenariosShown;
  private Map<String, Object> scenarioView = Map.of();

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

  /**
   * Declares a scenario: a named set of faults on the components already declared. Call it as the
   * robot starts, from the robot's thread.
   *
   * @param description what it does to the robot
   * @return this, to declare the next
   * @throws IllegalArgumentException if a scenario already has its name, or one of its faults isn't
   *     a declared component's fault with values it takes
   */
  public Components scenario(String name, String description, List<Setting> faults) {
    if (name.isBlank() || description.isBlank()) {
      throw new IllegalArgumentException("a scenario needs a name, and to say what it does");
    }
    for (Setting setting : faults) {
      String why = whyNot(setting);
      if (why != null) {
        throw new IllegalArgumentException("the scenario " + name + ": " + why);
      }
    }
    if (scenarios.putIfAbsent(name, new Scenario(name, description, List.copyOf(faults))) != null) {
      throw new IllegalArgumentException("two scenarios are named " + name);
    }
    declaredScenarios = Collections.unmodifiableMap(new LinkedHashMap<>(scenarios));
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

  /** Whether a scenario has this name. Safe from any thread. */
  public boolean hasScenario(String name) {
    return declaredScenarios.containsKey(name);
  }

  /** What applying one fault came to. */
  private enum Result {
    APPLIED,
    /** A lasting fault applied with the part's values as built: it isn't on. */
    UNCHANGED,
    REFUSED
  }

  /** What applying one fault came to, and why, when it wasn't applied. */
  private record Applied(Result result, @Nullable String why) {}

  /**
   * Applies a fault: turns a lasting one on, or makes an event happen. Call it from the robot loop.
   * A lasting fault applied with the values the part was built with changes nothing, and is
   * refused, saying so: it would be on and off at once.
   *
   * @return why it couldn't be applied (its name, its values, that it changes nothing, or its own
   *     code refused), or null when it was
   */
  public @Nullable String apply(Setting setting) {
    return applying(setting).why();
  }

  /**
   * Applies faults in order, as a scenario does: one that changes nothing (the part is that way as
   * built) is passed over, and one refused doesn't stop the rest. Call it from the robot loop.
   *
   * @return why any couldn't be applied, or null when all were
   */
  public @Nullable String apply(List<Setting> faults) {
    List<String> problems = new ArrayList<>();
    for (Setting setting : faults) {
      Applied applied = applying(setting);
      if (applied.result() == Result.REFUSED) {
        problems.add(applied.why());
      }
    }
    return problems.isEmpty() ? null : String.join("; ", problems);
  }

  private Applied applying(Setting setting) {
    String why = whyNot(setting);
    if (why != null) {
      return new Applied(Result.REFUSED, why);
    }
    Fault fault = lookup(setting.component(), setting.fault());
    Values values = fault.values(setting.values());
    try {
      fault.apply(values);
    } catch (RuntimeException e) {
      return new Applied(
          Result.REFUSED, setting.component() + ": " + setting.fault() + " failed: " + describe(e));
    }
    if (fault.kind() == Fault.Kind.LASTING) {
      if (!fault.isOn()) {
        forget(setting.component(), fault.name());
        return new Applied(
            Result.UNCHANGED,
            setting.component() + ": " + fault.name() + " changes nothing: as built");
      }
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
    return new Applied(Result.APPLIED, null);
  }

  /** Forgets the values a lasting fault was applied with: it's off. */
  private void forget(String component, String fault) {
    Map<String, Values> faults = applied.get(component);
    if (faults != null) {
      faults.remove(fault);
    }
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
    forget(component, fault);
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

  /**
   * Applies a scenario's faults, in order, as {@link #apply(List)} does. Call it from the robot
   * loop. One that fails doesn't stop the rest; the answer names it.
   *
   * @return why the scenario, or any of its faults, couldn't be applied; null when all were
   */
  public @Nullable String applyScenario(String name) {
    Scenario scenario = scenarios.get(name);
    if (scenario == null) {
      return "There is no scenario named "
          + name
          + "; there are "
          + String.join(", ", scenarios.keySet());
    }
    say.accept("scenario " + name + ": " + scenario.description());
    String problems = apply(scenario.faults());
    return problems == null ? null : name + ": " + problems;
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
   * robot loop; a component's map is rebuilt only when one of its faults turns on or off, and the
   * same map is returned until one does, so the page's comparison stops at it.
   */
  public Map<String, Object> view() {
    boolean changed = shownView.size() != components.size();
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
        changed = true;
      }
    }
    if (changed) {
      shownView = Collections.unmodifiableMap(new LinkedHashMap<>(shown));
    }
    return shownView;
  }

  /**
   * Every scenario as the page shows it, by name: {@code {"description", "faults": [{"component",
   * "fault", "values"}]}}.
   */
  public Map<String, Object> scenarioView() {
    Map<String, Scenario> scenarios = declaredScenarios;
    if (scenarios == scenariosShown) {
      return scenarioView; // the same map until a scenario is declared
    }
    Map<String, Object> view = new LinkedHashMap<>();
    for (Scenario scenario : scenarios.values()) {
      List<Object> faults = new ArrayList<>();
      for (Setting setting : scenario.faults()) {
        Map<String, Object> fault = new LinkedHashMap<>();
        fault.put("component", setting.component());
        fault.put("fault", setting.fault());
        fault.put("values", setting.values());
        faults.add(fault);
      }
      Map<String, Object> shown = new LinkedHashMap<>();
      shown.put("description", scenario.description());
      shown.put("faults", faults);
      view.put(scenario.name(), shown);
    }
    scenariosShown = scenarios;
    scenarioView = Collections.unmodifiableMap(view);
    return scenarioView;
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
