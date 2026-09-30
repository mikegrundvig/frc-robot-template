package com.michaelgrundvig.frc.pitwall;

/**
 * The tuning requests one loop applies, for a log to record, and in a replay to supply instead, so
 * a replay makes the same changes at the same moments. Plain fields, for any logging library to
 * read and write: with AdvantageKit, wrap it in a {@code LoggableInputs}. (Changes to values are
 * tunables' changes, which the tee logs.)
 */
public final class TuningInputs {
  /** What a request asks for. */
  enum Kind {
    ENABLE,
    DISABLE,
    RESET
  }

  private static final String[] NO_TEXT = new String[0];

  /** Whether tuning was switched on as the loop began. */
  public boolean enabled;

  /** Each request's kind, in the order they arrived: ENABLE, DISABLE, or RESET. */
  public String[] kinds = NO_TEXT;

  void clearRequests() {
    kinds = NO_TEXT;
  }
}
