package frc.robot.lib.config;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * Checks a record's values when it is built, and reports everything wrong with them at once. A
 * record lists each value with its {@link Rule}, then any checks across its values, in its compact
 * constructor:
 *
 * <pre>{@code
 * public Gains {
 *   Checks.on(Gains.class)
 *       .field("steerKp", steerKp, nonNegative())
 *       .field("steerKd", steerKd, nonNegative())
 *       .field("skewLookahead", skewLookahead, between(Seconds.of(0), Seconds.of(0.2)))
 *       .check("steerKd", () -> steerKd <= steerKp ? null
 *           : shown(steerKd) + " must be at most steerKp (" + shown(steerKp) + ")")
 *       .orThrow();
 * }
 * }</pre>
 *
 * <p>Every rule runs, and {@link #orThrow()} throws one {@link ConfigException} naming every value
 * that broke one: a configuration with three bad values reports three. The checks across values run
 * last, and only once every value has passed its own rule, so they never compare a value that's
 * already reported. Values are named from the record that checks them ({@code Gains.steerKp}): a
 * nested record is built, and checked, before the record that holds it.
 */
public final class Checks {
  private record Across(String name, Supplier<@Nullable String> why) {}

  private final String where;
  private final List<Violation> violations = new ArrayList<>();
  private final List<Across> checks = new ArrayList<>();

  private Checks(String where) {
    this.where = where;
  }

  /** Checks for a record, whose violations are named after it: {@code Gains.steerKp}. */
  public static Checks on(Class<? extends Record> record) {
    return new Checks(record.getSimpleName());
  }

  /**
   * Checks whose violations are named after {@code where}, for a record that has a name of its own:
   * {@code Checks.on("camera " + name)}.
   */
  public static Checks on(String where) {
    return new Checks(where);
  }

  /** Checks a number, or a whole number, against a rule. */
  public Checks field(String name, double value, Rule<Double> rule) {
    return run(name, () -> rule.whyNot(value));
  }

  /** Checks a value against a rule. */
  public <V> Checks field(String name, V value, Rule<? super V> rule) {
    return run(name, () -> rule.whyNot(value));
  }

  /**
   * A check across values, reported under {@code name}. It runs in {@link #orThrow()}, only if
   * every value passed its own rule.
   *
   * @param why what's wrong, written to follow the name, or null when it's fine
   */
  public Checks check(String name, Supplier<@Nullable String> why) {
    checks.add(new Across(name, why));
    return this;
  }

  /**
   * Runs the checks across values, if every value passed its own rule, then throws if anything was
   * found wrong.
   *
   * @throws ConfigException listing every violation
   */
  public void orThrow() {
    if (violations.isEmpty()) {
      for (Across check : checks) {
        run(check.name(), check.why());
      }
    }
    if (!violations.isEmpty()) {
      throw new ConfigException(violations);
    }
  }

  private Checks run(String name, Supplier<@Nullable String> why) {
    String problem;
    try {
      problem = why.get();
    } catch (RuntimeException e) {
      // A rule that throws is a bug in the rule; report it with the rest rather than lose them.
      problem = "couldn't be checked: " + e;
    }
    if (problem != null) {
      violations.add(new Violation(where + "." + name, problem));
    }
    return this;
  }
}
