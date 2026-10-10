package com.michaelgrundvig.frc.pitwall;

import com.michaelgrundvig.frc.pitwall.push.Reading;
import java.time.Duration;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.function.DoubleSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * The robot's status, as the page shows it: named items, each read in the robot loop. It answers
 * "is the robot ready?" at a glance, so keep it to what someone checks before running the robot:
 * the code, the battery, whether a mechanism is homed or a camera connected. Which robot it is and
 * its mode are in the page's bar, so they aren't items. Graphs and logs belong in AdvantageScope
 * and the dashboards.
 *
 * <p>An item is one line: {@code status.add("Arm homed", arm::isHomed)}. The page shows {@code
 * true} and {@code false} as a check and a cross, anything else as text. A number that wanders (the
 * battery) is added with its unit and the resolution worth showing, {@code status.add("Battery",
 * RobotController::getBatteryVoltage, "V", 0.1)}, so the page hears of it only once it has really
 * moved. Each change goes to every open page, so write a line to change only when something worth
 * reading does; one that still moves faster than anyone reads it (a running count) can say how
 * often it's read: {@code status.add("Loop", loop::describe, Duration.ofSeconds(1))}, which also
 * spares the loop making its text in between.
 *
 * <p>A name may say its group, as {@code "Group/Name"}: the page shows every item of a group under
 * one heading, in the order first added, so a robot program with many items can organize them
 * ({@code status.add("Mechanisms/Arm homed", arm::isHomed)}). A name with no {@code /} stands
 * alone, under no heading; that's the common case for a program with few items.
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
   * @param intervalNanos the shortest time between changes shown; 0 for none
   */
  private record Source(Supplier<?> value, String unit, double resolution, long intervalNanos) {}

  /** What an item held to an interval last read, and when. */
  private static final class Shown {
    @Nullable Object value;
    long readNanos;
  }

  private final LongSupplier clock;
  private final Map<String, Source> items = new LinkedHashMap<>();
  private final Map<String, Shown> shown = new LinkedHashMap<>();

  /** The units section, made once the items are known; null until then, or after an addition. */
  private @Nullable Map<String, Object> units;

  /** The read a request asked for, shared by every request until the loop reads; null for none. */
  private @Nullable CompletableFuture<List<Item>> asked;

  private volatile List<Item> view = List.of();
  private volatile Map<String, Object> read = Map.of();

  /** Status on the computer's own clock. */
  public Status() {
    this(System::nanoTime);
  }

  /**
   * @param clock the time in nanoseconds, for how often an item may change: the robot's clock, so
   *     the simulator's tests step it
   */
  public Status(LongSupplier clock) {
    this.clock = clock;
  }

  /**
   * Adds an item, shown in the order added.
   *
   * @return this, to add the next item
   * @throws IllegalArgumentException if the name is blank or already used
   */
  public Status add(String name, Supplier<?> value) {
    return add(name, new Source(value, "", 0, 0));
  }

  /**
   * Adds an item read at most once an interval: between reads, the page shows the value last read,
   * and its supplier isn't called. For a line that moves faster than anyone reads it, or costs the
   * loop something to make.
   *
   * @return this, to add the next item
   * @throws IllegalArgumentException if the name is blank or already used, or the interval is
   *     negative
   */
  public Status add(String name, Supplier<?> value, Duration interval) {
    if (interval.isNegative()) {
      throw new IllegalArgumentException("an interval can't be negative: " + interval);
    }
    return add(name, new Source(value, "", 0, interval.toNanos()));
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
    return add(name, new Source(value::getAsDouble, unit, resolution, 0));
  }

  private Status add(String name, Source source) {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a status item needs a name");
    }
    if (items.putIfAbsent(name, source) != null) {
      throw new IllegalArgumentException("two status items are named " + name);
    }
    units = null;
    return this;
  }

  /**
   * Reads every item, if anyone is looking. Call it once per loop.
   *
   * @param watched whether a page is watching (its event stream is open); a request that asked for
   *     the items has them read too
   */
  public void periodic(boolean watched) {
    CompletableFuture<List<Item>> request = take();
    if (!watched && request == null) {
      return;
    }
    long nowNanos = clock.getAsLong();
    List<Item> lines = new ArrayList<>(items.size());
    Map<String, Object> values = new LinkedHashMap<>();
    for (Map.Entry<String, Source> item : items.entrySet()) {
      Source source = item.getValue();
      Object value;
      if (source.intervalNanos() > 0) {
        value = everyInterval(item.getKey(), source, nowNanos);
        values.put(item.getKey(), value);
      } else {
        value = read(source.value());
        if (source.resolution() > 0 && value instanceof Number number) {
          values.put(
              item.getKey(), Reading.of(number.doubleValue()).toNearest(source.resolution()));
          value = shown(number.doubleValue(), source);
        } else {
          values.put(item.getKey(), value);
        }
      }
      lines.add(new Item(item.getKey(), value));
    }
    List<Item> now = List.copyOf(lines);
    view = now;
    read = values;
    if (request != null) {
      request.complete(now);
    }
  }

  private synchronized @Nullable CompletableFuture<List<Item>> take() {
    CompletableFuture<List<Item>> request = asked;
    asked = null;
    return request;
  }

  /** Every item as of the last loop it was read, in the order added. Safe from any thread. */
  public List<Item> view() {
    return view;
  }

  /**
   * Asks the robot loop to read every item. Safe from any thread. However many ask before the next
   * loop, it reads once, and they share the answer.
   *
   * @return completes, in the robot loop, with every item as read
   */
  public synchronized CompletableFuture<List<Item>> requestRead() {
    CompletableFuture<List<Item>> items = asked;
    if (items == null) {
      items = new CompletableFuture<>();
      asked = items;
    }
    return items;
  }

  /**
   * Every item as of the last loop it was read, for the event stream: a number with a resolution as
   * a {@link Reading}, so it's sent only once it has moved by that much.
   */
  Map<String, Object> values() {
    return read;
  }

  /**
   * The unit and resolution of each number that has them, for the page to show it with: the same
   * map until an item is added.
   */
  Map<String, Object> units() {
    Map<String, Object> made = units;
    if (made == null) {
      made = makeUnits();
      units = made;
    }
    return made;
  }

  private Map<String, Object> makeUnits() {
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

  /** An item held to an interval: read once it's due, else as last read. */
  private Object everyInterval(String name, Source source, long nowNanos) {
    Shown last = shown.computeIfAbsent(name, n -> new Shown());
    Object was = last.value;
    if (was != null && nowNanos - last.readNanos < source.intervalNanos()) {
      return was;
    }
    Object value = read(source.value());
    last.value = value;
    last.readNanos = nowNanos;
    return value;
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
