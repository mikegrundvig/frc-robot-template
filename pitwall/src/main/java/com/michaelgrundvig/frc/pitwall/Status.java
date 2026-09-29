package com.michaelgrundvig.frc.pitwall;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * The robot's status, as the page shows it: named items, each read in the robot loop. It answers
 * "is the robot ready?" at a glance, so keep it to what someone checks before running the robot:
 * which robot and code, the mode, the battery, whether a mechanism is homed or a camera connected.
 * Graphs and logs belong in AdvantageScope and the dashboards.
 *
 * <p>An item is one line: {@code status.add("Arm homed", arm::isHomed)}. The page shows {@code
 * true} and {@code false} as a check and a cross, anything else as text. Every item is read in
 * {@link #periodic}, on the robot loop's thread, so it may read robot state freely; keep each one
 * cheap. An item that throws shows what it threw rather than stopping the robot.
 */
public final class Status {
  /**
   * One item, as of the last loop.
   *
   * @param name what it is
   * @param value a Boolean, a Number, or text
   */
  public record Item(String name, Object value) {}

  private final Map<String, Supplier<?>> items = new LinkedHashMap<>();
  private volatile List<Item> view = List.of();

  /**
   * Adds an item, shown in the order added.
   *
   * @return this, to add the next item
   * @throws IllegalArgumentException if the name is blank or already used
   */
  public Status add(String name, Supplier<?> value) {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a status item needs a name");
    }
    if (items.putIfAbsent(name, value) != null) {
      throw new IllegalArgumentException("two status items are named " + name);
    }
    return this;
  }

  /** Reads every item. Call it once per loop. */
  public void periodic() {
    List<Item> read = new ArrayList<>(items.size());
    for (Map.Entry<String, Supplier<?>> item : items.entrySet()) {
      read.add(new Item(item.getKey(), read(item.getValue())));
    }
    view = List.copyOf(read);
  }

  /** Every item as of the last loop, in the order added. Safe to call from any thread. */
  public List<Item> view() {
    return view;
  }

  private static Object read(Supplier<?> item) {
    try {
      Object value = item.get();
      return value instanceof Boolean || value instanceof Number ? value : String.valueOf(value);
    } catch (RuntimeException e) {
      String why = e.getMessage();
      return "Error: " + (why != null ? why : e.toString());
    }
  }
}
