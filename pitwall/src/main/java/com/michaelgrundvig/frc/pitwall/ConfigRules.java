package com.michaelgrundvig.frc.pitwall;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.stream.Collectors;
import org.jspecify.annotations.Nullable;
import org.wpilib.tunable.TunableRegistry;

/**
 * Rules across configurations: checks no single configuration can make, because they read more than
 * one mechanism's. Two arms whose swings can collide, but only if both reach into each other's
 * space, are the example:
 *
 * <pre>{@code
 * rules.when(intake, "swing.from", "swing.to") // its inputs: a live configuration and its values
 *     .and(hood, "swing.from", "swing.to")
 *     .check("the intake and the hood", (i, h) -> Arms.clear(i, h, Inches.of(1)));
 *
 * rules.when(intake).check("...", i -> ...);    // no values named: any of the intake's
 * }</pre>
 *
 * <p>A rule runs only when a change touches one of its inputs, against the configurations as the
 * change would leave them: the candidates of those the change touches, the others as they are now.
 * So lowering the hood and swinging the intake further, in one change, is valid where either alone
 * would be refused. {@link #checkCommitted()} runs every rule once, against the committed
 * configurations: at startup, and in the tests.
 *
 * <p>Every value a rule names is checked when it registers: it must be one of that configuration's
 * tunable values, so a misspelled input fails at startup. A rule may read values that aren't
 * tunable (an arm's length) without naming them: they never change while the robot runs.
 *
 * <p>A rule returns why it's broken, or null. It must be quick and must not change anything; one
 * that throws refuses the change, and says so, but never stops the robot loop. A registered rule
 * travels with the configurations it reads: Pitwall's tee runs it for a change from the page or a
 * dashboard, and a configuration's own {@link LiveConfig#whyNot} and {@link LiveConfig#set} run it
 * too. A replay doesn't: the rules judged each change when it was made, and the log says what was
 * applied.
 */
public final class ConfigRules {
  /**
   * The configurations a rule reads, as a change would leave them: the candidates of those it
   * touches, the others as they are now (or as committed, in {@link #checkCommitted()}).
   */
  public static final class Candidates {
    private final Map<LiveConfig<?>, ? extends Record> changed;
    private final boolean committed;

    private Candidates(Map<LiveConfig<?>, ? extends Record> changed, boolean committed) {
      this.changed = changed;
      this.committed = committed;
    }

    /** A configuration as the change would leave it. */
    @SuppressWarnings("unchecked") // a candidate is built from its own configuration
    public <T extends Record> T of(LiveConfig<T> config) {
      Record candidate = changed.get(config);
      if (candidate != null) {
        return (T) candidate;
      }
      return committed ? config.committed() : config.get();
    }
  }

  /**
   * A rule's input: a configuration, and the values of it the rule reads.
   *
   * @param values the paths of its values the rule reads; empty for any of them
   */
  private record Input(LiveConfig<?> config, Set<String> values) {
    boolean touchedBy(Map<LiveConfig<?>, ? extends Collection<String>> changes) {
      Collection<String> changed = changes.get(config);
      return changed != null && (values.isEmpty() || changed.stream().anyMatch(values::contains));
    }
  }

  private record Rule(
      String name, List<Input> inputs, Function<Candidates, @Nullable String> check) {}

  /**
   * A rule a change breaks, and the values it reads, so each can say so.
   *
   * @param rule the rule's name
   * @param text why it's broken, as the rule says it
   * @param line the whole reason as one line: {@code "the intake and the hood: ..."}
   * @param reads each configuration it reads, with the paths of the values it reads there; none for
   *     any of them
   */
  record Broken(String rule, String text, String line, Map<LiveConfig<?>, Set<String>> reads) {}

  private final List<Rule> rules = new ArrayList<>();

  /**
   * Starts a rule over one configuration's values, and any others {@link One#and} adds.
   *
   * @param values the paths of the values the rule reads, each one of the configuration's tunable
   *     values; none for any of them
   * @throws IllegalArgumentException if a value isn't one of the configuration's tunable values
   */
  public <A extends Record> One<A> when(LiveConfig<A> config, String... values) {
    return new One<>(config, input(config, values));
  }

  /** A rule's inputs so far: one configuration's values. */
  public final class One<A extends Record> {
    private final LiveConfig<A> a;
    private final Input input;

    private One(LiveConfig<A> a, Input input) {
      this.a = a;
      this.input = input;
    }

    /**
     * Adds a second configuration's values.
     *
     * @throws IllegalArgumentException if a value isn't one of the configuration's tunable values
     */
    public <B extends Record> Two<A, B> and(LiveConfig<B> b, String... values) {
      return new Two<>(a, b, List.of(input, input(b, values)));
    }

    /**
     * Registers the rule.
     *
     * @param name what the rule protects, for its messages: {@code "the intake and the hood"}
     * @param rule why the configuration breaks it, or null when it doesn't
     * @throws IllegalArgumentException if a rule has this name already
     */
    public void check(String name, Function<? super A, @Nullable String> rule) {
      register(name, List.of(input), candidates -> rule.apply(candidates.of(a)));
    }
  }

  /** A rule's inputs so far: two configurations' values. */
  public final class Two<A extends Record, B extends Record> {
    private final LiveConfig<A> a;
    private final LiveConfig<B> b;
    private final List<Input> inputs;

    private Two(LiveConfig<A> a, LiveConfig<B> b, List<Input> inputs) {
      this.a = a;
      this.b = b;
      this.inputs = inputs;
    }

    /**
     * Adds a third configuration's values. The rule then reads them all from {@link Candidates}.
     *
     * @throws IllegalArgumentException if a value isn't one of the configuration's tunable values
     */
    public Many and(LiveConfig<?> config, String... values) {
      List<Input> more = new ArrayList<>(inputs);
      more.add(input(config, values));
      return new Many(more);
    }

    /**
     * Registers the rule.
     *
     * @param name what the rule protects, for its messages
     * @param rule why the two configurations break it, or null when they don't
     * @throws IllegalArgumentException if a rule has this name already
     */
    public void check(String name, BiFunction<? super A, ? super B, @Nullable String> rule) {
      register(name, inputs, candidates -> rule.apply(candidates.of(a), candidates.of(b)));
    }
  }

  /** A rule's inputs so far: three or more configurations' values. */
  public final class Many {
    private final List<Input> inputs;

    private Many(List<Input> inputs) {
      this.inputs = inputs;
    }

    /**
     * Adds another configuration's values.
     *
     * @throws IllegalArgumentException if a value isn't one of the configuration's tunable values
     */
    public Many and(LiveConfig<?> config, String... values) {
      List<Input> more = new ArrayList<>(inputs);
      more.add(input(config, values));
      return new Many(more);
    }

    /**
     * Registers the rule, which reads each configuration with {@link Candidates#of}.
     *
     * @param name what the rule protects, for its messages
     * @param rule why the configurations break it, or null when they don't
     * @throws IllegalArgumentException if a rule has this name already
     */
    public void check(String name, Function<Candidates, @Nullable String> rule) {
      register(name, inputs, rule);
    }
  }

  /**
   * Runs every rule once, against every configuration as committed.
   *
   * @throws IllegalArgumentException if the committed configurations break any rule, listing every
   *     one they break, one per line
   */
  public void checkCommitted() {
    List<String> broken = new ArrayList<>();
    Candidates committed = new Candidates(Map.of(), true);
    for (Rule rule : rules) {
      Broken why = run(rule, committed);
      if (why != null) {
        broken.add(why.line());
      }
    }
    if (!broken.isEmpty()) {
      throw new IllegalArgumentException(String.join("\n", broken));
    }
  }

  /** The names of the rules registered, in order. */
  List<String> names() {
    return rules.stream().map(Rule::name).toList();
  }

  /**
   * Why a change breaks the rules it touches: every rule with an input the change sets, run against
   * the configurations as the change would leave them.
   *
   * @param changes the values a change sets, by configuration: their paths
   * @param candidates each configuration the change touches, as it would leave it; the others are
   *     read as they are now
   * @return one line for each rule broken, {@code "the intake and the hood: ..."}; empty when none
   *     is
   */
  List<String> whyNot(
      Map<LiveConfig<?>, ? extends Collection<String>> changes,
      Map<LiveConfig<?>, ? extends Record> candidates) {
    return broken(changes, candidates).stream().map(Broken::line).toList();
  }

  /**
   * The rules a change breaks, as {@link #whyNot} finds them, each with the values it reads.
   *
   * @return each rule broken; empty when none is
   */
  List<Broken> broken(
      Map<LiveConfig<?>, ? extends Collection<String>> changes,
      Map<LiveConfig<?>, ? extends Record> candidates) {
    List<Broken> broken = new ArrayList<>();
    Candidates changed = new Candidates(candidates, false);
    for (Rule rule : rules) {
      if (rule.inputs().stream().anyMatch(input -> input.touchedBy(changes))) {
        Broken why = run(rule, changed);
        if (why != null) {
          broken.add(why);
        }
      }
    }
    return broken;
  }

  private void register(
      String name, List<Input> inputs, Function<Candidates, @Nullable String> check) {
    if (names().contains(name)) {
      throw new IllegalArgumentException("a rule is named " + name + " already");
    }
    rules.add(new Rule(name, List.copyOf(inputs), check));
    for (Input input : inputs) {
      input.config().ruledBy(this);
    }
  }

  private static Input input(LiveConfig<?> config, String[] values) {
    Set<String> tunable =
        config.knobs().stream().map(LiveConfig.Knob::path).collect(Collectors.toSet());
    for (String value : values) {
      if (!tunable.contains(value)) {
        throw new IllegalArgumentException(
            config.name()
                + " has no tunable value "
                + value
                + "; its tunable values are "
                + config.knobs().stream().map(LiveConfig.Knob::path).toList());
      }
    }
    return new Input(config, Set.of(values));
  }

  /** Runs a rule: why it's broken, under its name, or null. A rule that throws is broken. */
  private static @Nullable Broken run(Rule rule, Candidates candidates) {
    try {
      String why = rule.check().apply(candidates);
      return why == null
          ? null
          : new Broken(rule.name(), why, rule.name() + ": " + why, reads(rule));
    } catch (RuntimeException e) {
      String failed = "rule '" + rule.name() + "' failed: " + e;
      TunableRegistry.reportWarning("Pitwall: " + failed);
      return new Broken(rule.name(), "failed: " + e, failed, reads(rule));
    }
  }

  /** The values a rule reads, by configuration: none for any of a configuration's. */
  private static Map<LiveConfig<?>, Set<String>> reads(Rule rule) {
    Map<LiveConfig<?>, Set<String>> reads = new LinkedHashMap<>();
    for (Input input : rule.inputs()) {
      reads.merge(input.config(), input.values(), ConfigRules::union);
    }
    return reads;
  }

  /** Two inputs' values of one configuration: none (any of them) wins. */
  private static Set<String> union(Set<String> a, Set<String> b) {
    if (a.isEmpty() || b.isEmpty()) {
      return Set.of();
    }
    Set<String> both = new LinkedHashSet<>(a);
    both.addAll(b);
    return both;
  }
}
