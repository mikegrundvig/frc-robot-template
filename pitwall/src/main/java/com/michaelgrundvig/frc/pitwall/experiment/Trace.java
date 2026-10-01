package com.michaelgrundvig.frc.pitwall.experiment;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.ToIntFunction;
import org.wpilib.units.Unit;

/**
 * What a run recorded: one {@link Series} per signal, such as the setpoint, the measured angle, and
 * the motor's voltage. Each signal keeps its own sample times, so one read at 250 Hz and one read
 * each loop sit side by side without either being resampled.
 */
public final class Trace {
  /**
   * A signal a run records.
   *
   * @param name what it's called on the page, such as {@code Angle}
   * @param unit the unit its values are in
   */
  public record Signal(String name, Unit unit) {
    public Signal {
      if (name.isBlank()) {
        throw new IllegalArgumentException("a signal needs a name");
      }
    }
  }

  private final List<Signal> signals;
  private final Map<String, Series> series = new HashMap<>();

  /**
   * @param signals what the run records, in the order the page shows them
   * @param capacity how many samples each signal can hold
   * @throws IllegalArgumentException if there are no signals, two share a name, or the capacity
   *     isn't positive
   */
  public Trace(List<Signal> signals, int capacity) {
    this(signals, signal -> capacity);
  }

  /**
   * @param signals what the run records, in the order the page shows them
   * @param capacity how many samples a signal can hold: a signal read once a loop needs less room
   *     than a device's own readings
   * @throws IllegalArgumentException if there are no signals, two share a name, or a capacity isn't
   *     positive
   */
  public Trace(List<Signal> signals, ToIntFunction<Signal> capacity) {
    if (signals.isEmpty()) {
      throw new IllegalArgumentException("a trace records at least one signal");
    }
    this.signals = List.copyOf(signals);
    for (Signal signal : this.signals) {
      if (series.put(signal.name(), new Series(capacity.applyAsInt(signal))) != null) {
        throw new IllegalArgumentException("two signals are named " + signal.name());
      }
    }
  }

  /** The signals, in the order they were given. */
  public List<Signal> signals() {
    return signals;
  }

  /**
   * One signal's samples.
   *
   * @throws IllegalArgumentException if the trace has no signal by that name
   */
  public Series series(String name) {
    Series found = series.get(name);
    if (found == null) {
      throw new IllegalArgumentException("no signal named " + name + " in " + names());
    }
    return found;
  }

  /** Whether any signal ran out of room before the run ended. */
  public boolean full() {
    return series.values().stream().anyMatch(Series::full);
  }

  private List<String> names() {
    List<String> names = new ArrayList<>();
    signals.forEach(signal -> names.add(signal.name()));
    return names;
  }
}
