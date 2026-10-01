package com.michaelgrundvig.frc.pitwall.simulator;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * A fault's parameters as it's applied, by name, each within its range: what the page or a scenario
 * gave, and the parameter's initial value for the rest.
 */
public final class Values {
  /** For a fault without parameters. */
  public static final Values NONE = new Values(Map.of());

  private final Map<String, Double> values;

  /**
   * @param values each parameter's value, in the fault's order
   */
  Values(Map<String, Double> values) {
    this.values = Collections.unmodifiableMap(new LinkedHashMap<>(values));
  }

  /**
   * A parameter's value.
   *
   * @throws IllegalArgumentException if the fault has no parameter by that name
   */
  public double get(String name) {
    Double value = values.get(name);
    if (value == null) {
      throw new IllegalArgumentException(
          "no parameter is named " + name + "; there are " + values.keySet());
    }
    return value;
  }

  /** Every parameter's value, by name, in the fault's order. */
  public Map<String, Double> asMap() {
    return values;
  }

  @Override
  public String toString() {
    return values.toString();
  }
}
