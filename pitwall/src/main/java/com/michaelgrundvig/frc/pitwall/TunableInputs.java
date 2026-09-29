package com.michaelgrundvig.frc.pitwall;

/**
 * The changes to WPILib tunables one update applied, from dashboards or the page, for a log to
 * record, and in a replay to supply instead, so a replay makes the same changes at the same
 * moments. Plain fields, for any logging library to read and write.
 */
public final class TunableInputs {
  private static final String[] NO_TEXT = new String[0];

  /** The tunable each change was to. */
  public String[] paths = NO_TEXT;

  /** Each change's value, as text: "18.5", "true", "42". */
  public String[] values = NO_TEXT;
}
