package com.michaelgrundvig.frc.pitwall.experiment;

import java.util.List;
import org.wpilib.units.Unit;

/**
 * A test motion: the setpoint an experiment commands at each moment of a run, the same on every run
 * so that runs can be compared. Setpoints are in the profile's base unit (radians for an angle,
 * meters for a distance), whatever unit the profile was written in. Built by {@link Profiles}.
 */
public interface Profile {
  /**
   * A jump in the setpoint. A run is scored on how the mechanism answers each one.
   *
   * @param atSeconds when it happens, from the start of the run
   * @param from the setpoint before, in the profile's base unit
   * @param to the setpoint after, in the profile's base unit
   */
  record Edge(double atSeconds, double from, double to) {}

  /** The unit setpoints are in: the base unit of whatever the profile was written in. */
  Unit baseUnit();

  /** The setpoint {@code seconds} into the run; after the end, the last one. */
  double setpoint(double seconds);

  /** How long a run lasts. */
  double durationSeconds();

  /** The jumps in the setpoint, in order. */
  List<Edge> edges();
}
