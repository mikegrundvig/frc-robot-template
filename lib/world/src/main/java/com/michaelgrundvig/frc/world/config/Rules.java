package com.michaelgrundvig.frc.world.config;

import java.math.BigDecimal;
import java.math.MathContext;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoublePredicate;
import java.util.stream.Collectors;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.Measure;

/**
 * The common {@link Rule}s. Every one that takes a number fails NaN and infinity, and says what it
 * found the way people write it: {@code "-3 A must be positive"}.
 *
 * <p>{@link #positive}, {@link #nonNegative}, and {@link #finite} take a plain number or a measure.
 * A range takes plain numbers ({@code between(-1, 1)}) or measures ({@code between(Degrees.of(-90),
 * Degrees.of(90))}): a measure's range is in real units, so a value written in radians is still
 * checked against degrees correctly.
 */
public final class Rules {
  private Rules() {}

  /** Greater than zero. */
  public static <V> Rule<V> positive() {
    return zeroRule(magnitude -> magnitude > 0, "must be positive");
  }

  /** Zero or more. */
  public static <V> Rule<V> nonNegative() {
    return zeroRule(magnitude -> magnitude >= 0, "must be zero or more");
  }

  /** Any number but NaN and infinity. */
  public static <V> Rule<V> finite() {
    return zeroRule(magnitude -> true, "");
  }

  /** At least {@code min}. */
  public static Rule<Double> atLeast(double min) {
    return value -> checked(value, value >= min, "must be at least " + shown(min));
  }

  /** At most {@code max}. */
  public static Rule<Double> atMost(double max) {
    return value -> checked(value, value <= max, "must be at most " + shown(max));
  }

  /** Less than {@code limit}. */
  public static Rule<Double> below(double limit) {
    return value -> checked(value, value < limit, "must be below " + shown(limit));
  }

  /** From {@code min} to {@code max}, both included. */
  public static Rule<Double> between(double min, double max) {
    return value ->
        checked(
            value,
            value >= min && value <= max,
            "must be from " + shown(min) + " to " + shown(max));
  }

  /** At least {@code min}, in real units. */
  public static <M extends Measure<?>> Rule<M> atLeast(M min) {
    return value ->
        checked(
            value,
            value.baseUnitMagnitude() >= min.baseUnitMagnitude(),
            "must be at least " + shown(min));
  }

  /** At most {@code max}, in real units. */
  public static <M extends Measure<?>> Rule<M> atMost(M max) {
    return value ->
        checked(
            value,
            value.baseUnitMagnitude() <= max.baseUnitMagnitude(),
            "must be at most " + shown(max));
  }

  /** Less than {@code limit}, in real units. */
  public static <M extends Measure<?>> Rule<M> below(M limit) {
    return value ->
        checked(
            value,
            value.baseUnitMagnitude() < limit.baseUnitMagnitude(),
            "must be below " + shown(limit));
  }

  /** From {@code min} to {@code max}, both included, in real units. */
  public static <M extends Measure<?>> Rule<M> between(M min, M max) {
    return value -> {
      double magnitude = value.baseUnitMagnitude();
      return checked(
          value,
          magnitude >= min.baseUnitMagnitude() && magnitude <= max.baseUnitMagnitude(),
          "must be from " + shown(min) + " to " + shown(max));
    };
  }

  /** Text with something in it besides spaces. */
  public static Rule<String> notBlank() {
    return value -> value.isBlank() ? "must not be blank" : null;
  }

  /** One of the values listed. */
  @SafeVarargs
  public static <V> Rule<V> oneOf(V... allowed) {
    List<V> choices = List.of(allowed);
    return value ->
        choices.contains(value)
            ? null
            : value
                + " must be one of "
                + choices.stream().map(String::valueOf).collect(Collectors.joining(", "));
  }

  /** Exactly {@code count} items. */
  public static <V> Rule<List<V>> size(int count) {
    return list -> list.size() == count ? null : "must have " + count + ", has " + list.size();
  }

  /** Every item in a list passes the rule. Each one that doesn't is named by its position. */
  public static <V> Rule<List<V>> each(Rule<? super V> rule) {
    return list -> {
      List<String> problems = new ArrayList<>();
      for (int i = 0; i < list.size(); i++) {
        String why = rule.whyNot(list.get(i));
        if (why != null) {
          problems.add("[" + i + "] " + why);
        }
      }
      return problems.isEmpty() ? null : String.join("; ", problems);
    };
  }

  /** An optional value: none passes, and one given passes the rule. */
  public static <V> Rule<Optional<V>> given(Rule<? super V> rule) {
    return value -> value.isPresent() ? rule.whyNot(value.get()) : null;
  }

  /** A measure as people write it: "4.729 m/s" rather than "4.728761e+00 m/s". */
  public static String shown(Measure<?> value) {
    return shown(value.magnitude()) + " " + value.unit().symbol();
  }

  /**
   * A name's possessive, as a message says it: "pivot's", and "rollers'" for a name that ends in s,
   * never "rollers's".
   */
  public static String possessive(String name) {
    return name.endsWith("s") ? name + "'" : name + "'s";
  }

  /** A number to four significant digits, without an exponent: "0.08", "1235000". */
  public static String shown(double value) {
    if (!Double.isFinite(value)) {
      return Double.toString(value);
    }
    return new BigDecimal(value).round(new MathContext(4)).stripTrailingZeros().toPlainString();
  }

  /** A rule comparing a number, or a measure in its base unit, with zero. */
  private static <V> Rule<V> zeroRule(DoublePredicate ok, String what) {
    return value -> {
      if (value instanceof Number number) {
        return checked(number.doubleValue(), ok.test(number.doubleValue()), what);
      }
      if (value instanceof Measure<?> measure) {
        return checked(measure, ok.test(measure.baseUnitMagnitude()), what);
      }
      return "can't be checked as a number: it's a " + value.getClass().getSimpleName();
    };
  }

  private static @Nullable String checked(double value, boolean ok, String what) {
    if (!Double.isFinite(value)) {
      return shown(value) + " must be a finite number";
    }
    return ok ? null : shown(value) + " " + what;
  }

  private static @Nullable String checked(Measure<?> value, boolean ok, String what) {
    if (!Double.isFinite(value.baseUnitMagnitude())) {
      return shown(value) + " must be a finite number";
    }
    return ok ? null : shown(value) + " " + what;
  }
}
