package com.michaelgrundvig.frc.world.config;

import java.util.List;
import java.util.stream.Collectors;

/**
 * A configuration that breaks its rules, with every violation, not just the first. Its message is
 * one violation per line. It is an {@link IllegalArgumentException}, so code that catches those
 * catches it too.
 */
public final class ConfigException extends IllegalArgumentException {
  private static final long serialVersionUID = 1;

  private final List<Violation> violations;

  /**
   * @param violations at least one
   */
  public ConfigException(List<Violation> violations) {
    super(message(violations));
    this.violations = List.copyOf(violations);
  }

  /** Every violation, in the order they were found. */
  public List<Violation> violations() {
    return violations;
  }

  private static String message(List<Violation> violations) {
    if (violations.isEmpty()) {
      throw new IllegalArgumentException("a ConfigException needs at least one violation");
    }
    return violations.stream().map(Violation::toString).collect(Collectors.joining("\n"));
  }
}
