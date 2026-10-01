package com.michaelgrundvig.frc.pitwall.simulator;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * A fault to apply, by name, with its parameters' values: what a scenario is made of, and what the
 * page sends. A parameter left out takes its initial value.
 *
 * @param component the component's name: {@code "Gyro"}
 * @param fault the fault's name: {@code "Unplugged"}
 * @param values the parameters given, by name
 */
public record Setting(String component, String fault, Map<String, Double> values) {
  public Setting {
    values = Collections.unmodifiableMap(new LinkedHashMap<>(values));
  }

  /** A fault applied with its parameters' initial values, or a fault that takes none. */
  public static Setting of(String component, String fault) {
    return new Setting(component, fault, Map.of());
  }

  /** A fault applied with these values; any parameter left out takes its initial value. */
  public static Setting of(String component, String fault, Map<String, Double> values) {
    return new Setting(component, fault, values);
  }

  /** As the console and the log say it: {@code Gyro: Errors (Scale 1.15)}. */
  @Override
  public String toString() {
    return component + ": " + fault + (values.isEmpty() ? "" : " " + values);
  }
}
