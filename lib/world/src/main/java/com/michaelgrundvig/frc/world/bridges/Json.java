package com.michaelgrundvig.frc.world.bridges;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Writes JSON from maps (in their own order), lists, text, numbers and true or false: the little
 * the writers need, with no library to add. The same values always give the same text.
 */
final class Json {
  private Json() {}

  /**
   * Readable JSON, two spaces to a level, a list of plain values on one line: one value to a line
   * otherwise, so a changed value shows on a line of its own. Numbers to the micrometer and the
   * millionth of a degree.
   */
  static String pretty(Object value) {
    StringBuilder out = new StringBuilder();
    write(out, value, 0, true);
    return out.append('\n').toString();
  }

  /** JSON on one line, every number exactly as Java gives it (a float's own digits survive). */
  static String compact(Object value) {
    StringBuilder out = new StringBuilder();
    write(out, value, -1, false);
    return out.toString();
  }

  private static void write(StringBuilder out, Object value, int level, boolean rounded) {
    switch (value) {
      case Map<?, ?> map -> {
        out.append('{');
        boolean first = true;
        for (Map.Entry<?, ?> entry : map.entrySet()) {
          out.append(first ? "" : ",");
          first = false;
          newline(out, level < 0 ? -1 : level + 1);
          text(out, String.valueOf(entry.getKey()));
          out.append(level < 0 ? ":" : ": ");
          write(out, entry.getValue(), level < 0 ? -1 : level + 1, rounded);
        }
        if (!map.isEmpty()) {
          newline(out, level);
        }
        out.append('}');
      }
      case List<?> list -> {
        boolean plain =
            list.stream().noneMatch(item -> item instanceof Map || item instanceof List);
        out.append('[');
        for (int i = 0; i < list.size(); i++) {
          out.append(i == 0 ? "" : plain && level >= 0 ? ", " : ",");
          if (!plain) {
            newline(out, level < 0 ? -1 : level + 1);
          }
          write(out, list.get(i), level < 0 ? -1 : level + 1, rounded);
        }
        if (!plain && !list.isEmpty()) {
          newline(out, level);
        }
        out.append(']');
      }
      case String string -> text(out, string);
      case Boolean bool -> out.append(bool);
      case Integer integer -> out.append(integer);
      case Long whole -> out.append(whole);
      case Double number -> out.append(rounded ? rounded(number) : exact(number));
      default ->
          throw new IllegalArgumentException("no JSON for a " + value.getClass().getSimpleName());
    }
  }

  private static void newline(StringBuilder out, int level) {
    if (level >= 0) {
      out.append('\n').append("  ".repeat(level));
    }
  }

  /** To six places, without trailing zeros: 0.69850 is 0.6985, and -0 is 0. */
  private static String rounded(double number) {
    checkFinite(number);
    BigDecimal value = BigDecimal.valueOf(number).setScale(6, RoundingMode.HALF_EVEN);
    return value.signum() == 0 ? "0" : value.stripTrailingZeros().toPlainString();
  }

  /** Java's shortest text that reads back as exactly this number. */
  private static String exact(double number) {
    checkFinite(number);
    return number == Math.rint(number) && Math.abs(number) < 1e15
        ? Long.toString((long) number)
        : Double.toString(number);
  }

  private static void checkFinite(double number) {
    if (!Double.isFinite(number)) {
      throw new IllegalArgumentException("JSON has no " + number);
    }
  }

  private static void text(StringBuilder out, String text) {
    out.append('"');
    for (char c : text.toCharArray()) {
      switch (c) {
        case '"' -> out.append("\\\"");
        case '\\' -> out.append("\\\\");
        case '\n' -> out.append("\\n");
        case '\r' -> out.append("\\r");
        case '\t' -> out.append("\\t");
        default -> {
          if (c < 0x20) {
            out.append(String.format(Locale.ROOT, "\\u%04x", (int) c));
          } else {
            out.append(c);
          }
        }
      }
    }
    out.append('"');
  }
}
