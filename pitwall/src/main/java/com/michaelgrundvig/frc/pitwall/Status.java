package com.michaelgrundvig.frc.pitwall;

import com.michaelgrundvig.frc.pitwall.push.Reading;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * The robot's status, as the page shows it: named items, each read in the robot loop. It answers
 * "is the robot ready?" at a glance, so keep it to what someone checks before running the robot:
 * which robot and code, the mode, the battery, whether a mechanism is homed or a camera connected.
 * Graphs and logs belong in AdvantageScope and the dashboards.
 *
 * <p>An item is one line: {@code status.add("Arm homed", arm::isHomed)}. The page shows {@code
 * true} and {@code false} as a check and a cross, anything else as text. A number that wanders (the
 * battery) is added with its unit and the resolution worth showing, {@code status.add("Battery",
 * RobotController::getBatteryVoltage, "V", 0.1)}, so the page hears of it only once it has really
 * moved.
 *
 * <p>Items are read in {@link #periodic}, on the robot loop's thread, so they may read robot state
 * freely; keep each one cheap. They're read only while someone is looking: a page has its event
 * stream open, or a request asked for them. An item that throws shows what it threw rather than
 * stopping the robot.
 */
public final class Status {
  /**
   * One item, as of the last loop it was read.
   *
   * @param name what it is
   * @param value a Boolean, a Number, or text; a number with a unit is text, such as "12.3 V"
   */
  public record Item(String name, Object value) {}

  /**
   * An item's source.
   *
   * @param unit for a number with a resolution, its unit; empty for the rest
   * @param resolution for a number, the smallest change worth showing; 0 for the rest
   */
  private record Source(Supplier<?> value, String unit, double resolution) {}

  private final Map<String, Source> items = new LinkedHashMap<>();
  private final ConcurrentLinkedQueue<CompletableFuture<List<Item>>> asked =
      new ConcurrentLinkedQueue<>();
  private volatile List<Item> view = List.of();
  private volatile Map<String, Object> read = Map.of();

  /**
   * Adds an item, shown in the order added.
   *
   * @return this, to add the next item
   * @throws IllegalArgumentException if the name is blank or already used
   */
  public Status add(String name, Supplier<?> value) {
    return add(name, new Source(value, "", 0));
  }

  /**
   * Adds a number, shown with its unit to its resolution. The page is sent it again only once it
   * moves by the resolution, so noise at a rounding edge doesn't make it flicker.
   *
   * @param unit its unit, such as {@code "V"}
   * @param resolution the smallest change worth showing, such as 0.1
   * @return this, to add the next item
   * @throws IllegalArgumentException if the name is blank or already used, or the resolution isn't
   *     a positive number
   */
  public Status add(String name, DoubleSupplier value, String unit, double resolution) {
    Reading.of(0).toNearest(resolution); // checks the resolution
    return add(name, new Source(value::getAsDouble, unit, resolution));
  }

  private Status add(String name, Source source) {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a status item needs a name");
    }
    if (items.putIfAbsent(name, source) != null) {
      throw new IllegalArgumentException("two status items are named " + name);
    }
    return this;
  }

  /**
   * Reads every item, if anyone is looking. Call it once per loop.
   *
   * @param watched whether a page is watching (its event stream is open); a request that asked for
   *     the items has them read too
   */
  public void periodic(boolean watched) {
    if (!watched && asked.isEmpty()) {
      return;
    }
    List<Item> shown = new ArrayList<>(items.size());
    Map<String, Object> values = new LinkedHashMap<>();
    for (Map.Entry<String, Source> item : items.entrySet()) {
      Source source = item.getValue();
      Object value = read(source.value());
      if (source.resolution() > 0 && value instanceof Number number) {
        values.put(item.getKey(), Reading.of(number.doubleValue()).toNearest(source.resolution()));
        value = shown(number.doubleValue(), source);
      } else {
        values.put(item.getKey(), value);
      }
      shown.add(new Item(item.getKey(), value));
    }
    List<Item> now = List.copyOf(shown);
    view = now;
    read = values;
    for (var request = asked.poll(); request != null; request = asked.poll()) {
      request.complete(now);
    }
  }

  /** Every item as of the last loop it was read, in the order added. Safe from any thread. */
  public List<Item> view() {
    return view;
  }

  /**
   * Asks the robot loop to read every item. Safe from any thread.
   *
   * @return completes, in the robot loop, with every item as read
   */
  public CompletableFuture<List<Item>> requestRead() {
    CompletableFuture<List<Item>> items = new CompletableFuture<>();
    asked.add(items);
    return items;
  }

  /**
   * Every item as of the last loop it was read, for the event stream: a number with a resolution as
   * a {@link Reading}, so it's sent only once it has moved by that much.
   */
  Map<String, Object> values() {
    return read;
  }

  /** The unit and resolution of each number that has them, for the page to show it with. */
  Map<String, Object> units() {
    Map<String, Object> units = new LinkedHashMap<>();
    items.forEach(
        (name, source) -> {
          if (source.resolution() > 0) {
            Map<String, Object> unit = new LinkedHashMap<>();
            unit.put("unit", source.unit());
            unit.put("resolution", source.resolution());
            units.put(name, unit);
          }
        });
    return units;
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

  /** A number to its resolution, with its unit: {@code 12.3 V}. */
  private static String shown(double value, Source source) {
    int decimals = (int) Math.max(0, Math.ceil(-Math.log10(source.resolution()) - 1e-9));
    String number = String.format(Locale.ROOT, "%." + decimals + "f", value);
    return source.unit().isEmpty() ? number : number + " " + source.unit();
  }
}
