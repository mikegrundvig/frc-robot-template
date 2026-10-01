package com.michaelgrundvig.frc.pitwall;

import java.util.Locale;

/** Numbers as Pitwall writes them for people: one way, wherever they appear. */
public final class Numbers {
  private Numbers() {}

  /**
   * A number as people write it: a whole number plainly ({@code 90}, not {@code 90.0}), any other
   * to six significant figures without trailing zeros ({@code 0.25}, {@code 1.5708}).
   */
  public static String shown(double value) {
    if (value == Math.rint(value) && Math.abs(value) < 1e15) {
      return Long.toString((long) value);
    }
    return String.format(Locale.ROOT, "%.6g", value).replaceAll("\\.?0+(e|$)", "$1");
  }
}
