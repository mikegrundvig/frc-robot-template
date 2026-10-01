package com.michaelgrundvig.frc.pitwall.experiment;

/**
 * The experiment requests one loop applies, for a log to record, and in a replay to supply instead,
 * so a replay starts and cancels the same runs at the same moments. Plain fields, for any logging
 * library to read and write.
 */
public final class ExperimentInputs {
  /** What a request asks for. */
  enum Kind {
    START,
    CANCEL
  }

  private static final String[] NO_TEXT = new String[0];
  private static final long[] NO_RUNS = new long[0];

  /** Each request's kind: START or CANCEL. */
  public String[] kinds = NO_TEXT;

  /** The experiment each request is for. */
  public String[] names = NO_TEXT;

  /**
   * The run each cancel is for, so a cancel meant for a run that has ended never stops the next; 0
   * for a start.
   */
  public long[] runs = NO_RUNS;

  void clearRequests() {
    kinds = NO_TEXT;
    names = NO_TEXT;
    runs = NO_RUNS;
  }
}
