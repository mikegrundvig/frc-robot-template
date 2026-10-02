package com.michaelgrundvig.frc.world.config;

/**
 * One value that breaks a rule.
 *
 * @param where the value, named from the record that checked it: {@code Gains.steerKp}
 * @param what what's wrong with it: {@code -3 must be zero or more}
 */
public record Violation(String where, String what) {
  /** The violation as one line of a message: {@code Gains.steerKp: -3 must be zero or more}. */
  @Override
  public String toString() {
    return where + ": " + what;
  }
}
