package com.michaelgrundvig.frc.tuning;

/**
 * The tuning requests one loop applies, for a log to record, and in a replay to supply instead, so
 * a replay makes the same changes at the same moments. Plain fields, for any logging library to
 * read and write: with AdvantageKit, wrap it in a {@code LoggableInputs}.
 */
public final class TuningInputs {
  /** What a request asks for. */
  enum Kind {
    SET,
    RESET,
    ENABLE,
    DISABLE
  }

  private static final String[] NO_TEXT = new String[0];
  private static final double[] NO_NUMBERS = new double[0];

  /** Whether tuning was switched on as the loop began. */
  public boolean enabled;

  /** Each request's kind (SET, RESET, ENABLE, or DISABLE), with its path and value alongside. */
  public String[] kinds = NO_TEXT;

  /** The value each request is for; empty for the kinds that don't name one. */
  public String[] paths = NO_TEXT;

  /** The value each SET request asks for. */
  public double[] values = NO_NUMBERS;

  void clearRequests() {
    kinds = NO_TEXT;
    paths = NO_TEXT;
    values = NO_NUMBERS;
  }
}
