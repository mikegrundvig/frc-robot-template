package com.michaelgrundvig.frc.pitwall.simulator;

import com.michaelgrundvig.frc.pitwall.Limits;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.jspecify.annotations.Nullable;

/**
 * Something that can go wrong with a part of the simulated robot, on demand: a camera unplugged, a
 * gyro reading 15% high, the robot hit. It says in plain words what it does and how it's undone,
 * and takes its numbers as {@link Parameter}s, each with its range.
 *
 * <p>A fault either lasts until it's undone (a camera unplugged, {@link #lasting}) or happens once
 * (a hit, {@link #event}). A lasting fault says whether it's on now, read from the simulated world
 * itself, so it's right however it was turned on or off.
 *
 * {@snippet :
 * Fault.lasting("Out")
 *     .does("Dark.")
 *     .isOn(() -> lampOut)
 *     .apply(values -> lampOut = true)
 *     .undo("Lit.", () -> lampOut = false)
 *     .build()
 * }
 *
 * <p>Its code runs on the robot loop's thread ({@link Components}), so it may change the simulated
 * world freely.
 */
public final class Fault {
  /** Whether a fault lasts until it's undone, or happens once. */
  public enum Kind {
    /** On until undone: a camera unplugged, a gyro frozen. */
    LASTING,
    /** Happens once, and is over: a hit, a wheel slipping half a turn. */
    EVENT
  }

  private final String name;
  private final String does;
  private final Kind kind;
  private final List<Parameter> parameters;
  private final String undo;
  private final Consumer<Values> apply;
  private final Runnable undoAction;
  private final BooleanSupplier on;

  private Fault(Builder builder) {
    if (builder.name.isBlank()) {
      throw new IllegalArgumentException("a fault needs a name");
    }
    name = builder.name;
    kind = builder.kind;
    does = required(builder.does, "what it does");
    apply = required(builder.apply, "what applying it does");
    undo = required(builder.undo, kind == Kind.LASTING ? "how it's undone" : "how to recover");
    if (kind == Kind.LASTING) {
      on = required(builder.on, "whether it's on now");
      undoAction = required(builder.undoAction, "how it's undone");
    } else {
      if (builder.on != null || builder.undoAction != null) {
        throw new IllegalArgumentException(
            "the event " + name + " happens once: it isn't on or off, and nothing undoes it");
      }
      on = () -> false;
      undoAction = () -> {};
    }
    parameters = List.copyOf(builder.parameters);
  }

  /** A fault that lasts until it's undone, such as a gyro unplugged. */
  public static Builder lasting(String name) {
    return new Builder(name, Kind.LASTING);
  }

  /** Something that happens to the robot once, such as a hit. */
  public static Builder event(String name) {
    return new Builder(name, Kind.EVENT);
  }

  /** What it's called on the page: {@code "Unplugged"}. */
  public String name() {
    return name;
  }

  /** What it does, in plain words. */
  public String does() {
    return does;
  }

  public Kind kind() {
    return kind;
  }

  /** The numbers it takes, in order. */
  public List<Parameter> parameters() {
    return parameters;
  }

  /**
   * How it's undone, in plain words: for a lasting fault, what undoing it does; for an event, how
   * the robot recovers, since nothing undoes what happened.
   */
  public String undo() {
    return undo;
  }

  /** Whether it's on now: a lasting fault asks the simulated world; an event never is. */
  public boolean isOn() {
    return on.getAsBoolean();
  }

  /**
   * The values it would be applied with: each given one, the initial value for the rest.
   *
   * @throws IllegalArgumentException naming a parameter it doesn't have, or a value out of range
   */
  public Values values(Map<String, Double> given) {
    String why = whyNot(given);
    if (why != null) {
      throw new IllegalArgumentException(why);
    }
    Map<String, Double> values = new LinkedHashMap<>();
    for (Parameter parameter : parameters) {
      Double value = given.get(parameter.name());
      values.put(parameter.name(), value != null ? value : parameter.initial());
    }
    return new Values(values);
  }

  /**
   * Why it can't be applied with these values: a parameter it doesn't have, or a value out of its
   * range.
   *
   * @return null when it can
   */
  public @Nullable String whyNot(Map<String, Double> given) {
    Set<String> known = new HashSet<>();
    List<String> problems = new ArrayList<>();
    for (Parameter parameter : parameters) {
      known.add(parameter.name());
      Double value = given.get(parameter.name());
      String why = value == null ? null : parameter.whyNot(value);
      if (why != null) {
        problems.add(why);
      }
    }
    for (String asked : given.keySet()) {
      if (!known.contains(asked)) {
        problems.add(
            name
                + " has no parameter named "
                + asked
                + (known.isEmpty() ? "; it takes none" : "; it takes " + names()));
      }
    }
    return problems.isEmpty() ? null : String.join("; ", problems);
  }

  /** Makes it happen, or turns it on: its code, run on the robot loop's thread. */
  void apply(Values values) {
    apply.accept(values);
  }

  /** Turns a lasting fault off. */
  void undoIt() {
    undoAction.run();
  }

  private String names() {
    return String.join(", ", parameters.stream().map(Parameter::name).toList());
  }

  private <T> T required(@Nullable T value, String what) {
    if (value == null) {
      throw new IllegalArgumentException("the fault " + name + " needs " + what);
    }
    return value;
  }

  /** Builds a fault, one named part at a time. */
  public static final class Builder {
    private final String name;
    private final Kind kind;
    private @Nullable String does;
    private final List<Parameter> parameters = new ArrayList<>();
    private @Nullable String undo;
    private @Nullable Consumer<Values> apply;
    private @Nullable Runnable undoAction;
    private @Nullable BooleanSupplier on;

    private Builder(String name, Kind kind) {
      this.name = name;
      this.kind = kind;
    }

    /** What it does, in plain words: the page shows it beside the fault. */
    public Builder does(String does) {
      this.does = does;
      return this;
    }

    /**
     * A number it takes.
     *
     * @param unit empty for a plain number
     * @param limits the range the page may set it in
     * @param initial what it is when not given
     * @throws IllegalArgumentException if the fault has a parameter by this name already
     */
    public Builder parameter(String name, String unit, Limits limits, double initial) {
      if (parameters.stream().anyMatch(p -> p.name().equals(name))) {
        throw new IllegalArgumentException(this.name + " has two parameters named " + name);
      }
      parameters.add(new Parameter(name, unit, limits, initial));
      return this;
    }

    /** For a lasting fault: whether it's on now, read from the simulated world. */
    public Builder isOn(BooleanSupplier on) {
      this.on = on;
      return this;
    }

    /** What applying it does: turns a lasting fault on, or makes an event happen. */
    public Builder apply(Consumer<Values> apply) {
      this.apply = apply;
      return this;
    }

    /**
     * For a lasting fault: how it's undone.
     *
     * @param what what undoing it does, in plain words: {@code "Plugs it back in"}
     * @param undo the code that does it
     */
    public Builder undo(String what, Runnable undo) {
      this.undo = what;
      this.undoAction = undo;
      return this;
    }

    /**
     * For an event, which nothing undoes: how the robot recovers, in plain words, such as {@code
     * "Place it again, or let its cameras find it"}.
     */
    public Builder undoneBy(String how) {
      this.undo = how;
      return this;
    }

    /**
     * The fault.
     *
     * @throws IllegalArgumentException if a part it needs is missing
     */
    public Fault build() {
      return new Fault(this);
    }
  }
}
