package com.michaelgrundvig.frc.pitwall;

import io.avaje.json.JsonException;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.util.Map;
import org.jspecify.annotations.Nullable;
import org.wpilib.tunable.TunableConfig;

/**
 * The range a number may be tuned in while the robot runs: a tuning limit. It says what's safe to
 * dial in live, which is usually narrower than what's valid (a configuration's own checks say
 * that): a steer gain of 500 is a valid number and a robot shaking itself apart.
 *
 * <p>Limits ride on the tunable itself, as WPILib tunable properties ({@code "min"} and {@code
 * "max"}), so anything that reads NetworkTables topic properties sees them. Pitwall enforces them
 * on every numeric tunable, from the page and from dashboards alike, before a value is applied:
 *
 * {@snippet :
 * TunableDouble.createConfig(0.0, Limits.between(0, MAX_HOLD_SECONDS).config());
 * }
 */
public final class Limits {
  /** No limits: anything that's a number. */
  public static final Limits NONE = new Limits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

  static final String MIN = "min";
  static final String MAX = "max";

  private static final JsonType<Object> JSON = Jsonb.builder().build().type(Object.class);

  private final double min;
  private final double max;

  private Limits(double min, double max) {
    if (Double.isNaN(min) || Double.isNaN(max) || !(min <= max)) {
      throw new IllegalArgumentException(
          "a tuning limit's minimum must be at most its maximum, got " + min + " and " + max);
    }
    this.min = min;
    this.max = max;
  }

  /** From {@code min} to {@code max}, both included. */
  public static Limits between(double min, double max) {
    return new Limits(min, max);
  }

  /** {@code min} or more. */
  public static Limits atLeast(double min) {
    return new Limits(min, Double.POSITIVE_INFINITY);
  }

  /** {@code max} or less. */
  public static Limits atMost(double max) {
    return new Limits(Double.NEGATIVE_INFINITY, max);
  }

  public double min() {
    return min;
  }

  public double max() {
    return max;
  }

  /**
   * Why a value is outside these limits.
   *
   * @return null when it's within them
   */
  public @Nullable String whyNot(double value) {
    return whyNot(value, "");
  }

  /**
   * Why a value of a measure is outside these limits, each number with its unit: "130 A is above
   * its limit of 120 A".
   *
   * @param unit the measure's unit symbol, as the value is given in: empty for a plain number
   * @return null when it's within them
   */
  public @Nullable String whyNot(double value, String unit) {
    String in = unit.isEmpty() ? "" : " " + unit;
    if (value < min) {
      return shown(value) + in + " is below its limit of " + shown(min) + in;
    }
    if (value > max) {
      return shown(value) + in + " is above its limit of " + shown(max) + in;
    }
    return null;
  }

  /** A new WPILib tunable configuration carrying these limits. */
  public TunableConfig config() {
    return on(new TunableConfig());
  }

  /** A WPILib tunable configuration with these limits added. */
  public TunableConfig on(TunableConfig config) {
    TunableConfig limited = config;
    if (Double.isFinite(min)) {
      limited = limited.withProperty(MIN, json(min));
    }
    if (Double.isFinite(max)) {
      limited = limited.withProperty(MAX, json(max));
    }
    return limited;
  }

  /**
   * The limits a tunable's configuration carries: its {@code "min"} and {@code "max"} properties. A
   * property that isn't a number is ignored.
   */
  public static Limits of(@Nullable TunableConfig config) {
    if (config == null) {
      return NONE;
    }
    Object parsed;
    try {
      parsed = JSON.fromJson(config.getProperties());
    } catch (JsonException e) {
      return NONE;
    }
    if (!(parsed instanceof Map<?, ?> properties)) {
      return NONE;
    }
    double low = number(properties.get(MIN), Double.NEGATIVE_INFINITY);
    double high = number(properties.get(MAX), Double.POSITIVE_INFINITY);
    return low <= high ? new Limits(low, high) : NONE;
  }

  private static double number(@Nullable Object property, double otherwise) {
    return property instanceof Number number && Double.isFinite(number.doubleValue())
        ? number.doubleValue()
        : otherwise;
  }

  /** A number as JSON writes it: plain where it can be, exponent form otherwise. */
  private static String json(double value) {
    return value == Math.rint(value) && Math.abs(value) < 1e15
        ? Long.toString((long) value)
        : Double.toString(value);
  }

  private static String shown(double value) {
    return Numbers.shown(value);
  }

  @Override
  public boolean equals(@Nullable Object other) {
    return other instanceof Limits limits
        && Double.compare(min, limits.min) == 0
        && Double.compare(max, limits.max) == 0;
  }

  @Override
  public int hashCode() {
    return Double.hashCode(min) * 31 + Double.hashCode(max);
  }

  @Override
  public String toString() {
    if (min == Double.NEGATIVE_INFINITY && max == Double.POSITIVE_INFINITY) {
      return "any number";
    }
    if (min == Double.NEGATIVE_INFINITY) {
      return "at most " + shown(max);
    }
    if (max == Double.POSITIVE_INFINITY) {
      return "at least " + shown(min);
    }
    return shown(min) + " to " + shown(max);
  }
}
