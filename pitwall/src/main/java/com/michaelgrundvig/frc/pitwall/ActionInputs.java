package com.michaelgrundvig.frc.pitwall;

/**
 * The action requests one loop applies, for a log to record, and in a replay to supply instead, so
 * a replay starts and cancels the same actions at the same moments. Plain fields, for any logging
 * library to read and write.
 */
public final class ActionInputs {
  /** What a request asks for. */
  enum Kind {
    START,
    CANCEL
  }

  private static final String[] NO_TEXT = new String[0];

  /** Each request's kind: START or CANCEL. */
  public String[] kinds = NO_TEXT;

  /** The action each request is for. */
  public String[] names = NO_TEXT;

  void clearRequests() {
    kinds = NO_TEXT;
    names = NO_TEXT;
  }
}
