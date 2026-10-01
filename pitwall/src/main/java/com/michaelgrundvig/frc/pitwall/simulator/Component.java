package com.michaelgrundvig.frc.pitwall.simulator;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * A part of the simulated robot that can fail, and how: a camera, the gyro, a module, the robot
 * itself. The robot's simulation declares each one to the simulator's controls ({@link
 * Components#add}), and the page lists them with their faults.
 *
 * @param name what it's called on the page, unique among the robot's: {@code "Gyro"}, {@code
 *     "Camera front-left"}
 * @param kind what it is, in a word or two, for the page to group by: {@code "camera"}
 * @param faults what can go wrong with it, in the order the page lists them
 */
public record Component(String name, String kind, List<Fault> faults) {
  public Component {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a component needs a name");
    }
    if (kind.isBlank()) {
      throw new IllegalArgumentException(name + " needs to say what it is");
    }
    faults = List.copyOf(faults);
    Set<String> names = new HashSet<>();
    for (Fault fault : faults) {
      if (!names.add(fault.name())) {
        throw new IllegalArgumentException(name + " has two faults named " + fault.name());
      }
    }
  }

  /** A component and its faults. */
  public static Component of(String name, String kind, Fault... faults) {
    return new Component(name, kind, List.of(faults));
  }
}
