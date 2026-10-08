package com.michaelgrundvig.frc.world.config;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * Checks a record's values when it is built, and reports everything wrong with them at once. A
 * record lists each value with its {@link Rule}, then any checks across its values, in its compact
 * constructor, as a port encoder's wiring does:
 *
 * {@snippet class=com.michaelgrundvig.frc.world.robot.MechanismWiring region=checks}
 *
 * <p>Every value is checked, and {@link #orThrow()} throws one {@link ConfigException} naming every
 * value that broke a rule: a configuration with three bad values reports three. A value may be
 * listed more than once, with a rule each time; once it breaks one, its later rules are skipped.
 * The checks across values run last, and only once every value has passed its own rule, so they
 * never compare a value that's already reported. Values are named from the record that checks them
 * ({@code PortEncoder.port}): a nested record is built, and checked, before the record that holds
 * it.
 */
public final class Checks {
  /** A check across values: what's wrong, each a refusal or not. */
  private record Across(String name, Supplier<List<Finding>> found) {}

  private final String where;
  private final List<Violation> violations = new ArrayList<>();
  private final List<Across> checks = new ArrayList<>();
  private final Set<String> broken = new HashSet<>();

  private Checks(String where) {
    this.where = where;
  }

  /**
   * Checks for a record, whose violations are named after it: {@code Gains.steerKp}, or for a check
   * across its values, by the path from it, {@code Arm.limits.top} (an arm's record holding its
   * limits). Keep that form: Pitwall reads each line's name back to the value it names, to show the
   * reason beside it on its page (see its {@code LiveConfig}).
   */
  public static Checks on(Class<? extends Record> record) {
    return new Checks(record.getSimpleName());
  }

  /**
   * Checks for a record with a name of its own, which names its violations too: {@code
   * CameraConfig[front].tiltUp}.
   */
  public static Checks on(Class<? extends Record> record, String name) {
    return new Checks(record.getSimpleName() + "[" + name + "]");
  }

  /** Checks a number, or a whole number, against a rule. */
  public Checks field(String name, double value, Rule<Double> rule) {
    return broken.contains(name) ? this : run(name, () -> rule.whyNot(value));
  }

  /** Checks a value against a rule. */
  public <V> Checks field(String name, V value, Rule<? super V> rule) {
    return broken.contains(name) ? this : run(name, () -> rule.whyNot(value));
  }

  /**
   * A check across values, reported under {@code name}. It runs in {@link #orThrow()}, only if
   * every value passed its own rule.
   *
   * @param why what's wrong, written to follow the name, or null when it's fine
   */
  public Checks check(String name, Supplier<@Nullable String> why) {
    checks.add(
        new Across(
            name,
            () -> {
              String problem = why.get();
              return problem == null ? List.of() : List.of(Finding.refused(where, problem));
            }));
    return this;
  }

  /**
   * A check across values that says what it finds ({@link Finding}), reported under {@code name}:
   * each refusal a violation of its own. Doubts and notes aren't refusals: the program says them at
   * startup. It runs in {@link #orThrow()}, only if every value passed its own rule.
   */
  public Checks findings(String name, Supplier<List<Finding>> found) {
    checks.add(new Across(name, found));
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
        List<Finding> found;
        try {
          found = check.found().get();
        } catch (RuntimeException e) {
          // A check that throws is a bug in the check; report it with the rest rather than lose
          // them.
          found = List.of(Finding.refused(where, "couldn't be checked: " + e));
        }
        for (Finding finding : found) {
          if (finding.refused()) {
            violations.add(new Violation(where + "." + check.name(), finding.message()));
          }
        }
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
      broken.add(name);
    }
    return this;
  }
}
