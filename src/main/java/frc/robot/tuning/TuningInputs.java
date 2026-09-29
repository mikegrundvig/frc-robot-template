package frc.robot.tuning;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * The tuning requests one loop applies: logged as they happen, and read back from the log in
 * replay, so a replay makes the same changes at the same moments.
 */
public final class TuningInputs implements LoggableInputs {
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
  boolean enabled;

  /** Each request's {@link Kind}, with its path and value (for {@link Kind#SET}) alongside. */
  String[] kinds = NO_TEXT;

  String[] paths = NO_TEXT;
  double[] values = NO_NUMBERS;

  void clearRequests() {
    kinds = NO_TEXT;
    paths = NO_TEXT;
    values = NO_NUMBERS;
  }

  @Override
  public void toLog(LogTable table) {
    table.put("Enabled", enabled);
    table.put("Kinds", kinds);
    table.put("Paths", paths);
    table.put("Values", values);
  }

  @Override
  public void fromLog(LogTable table) {
    enabled = table.get("Enabled", enabled);
    kinds = table.get("Kinds", kinds);
    paths = table.get("Paths", paths);
    values = table.get("Values", values);
  }
}
