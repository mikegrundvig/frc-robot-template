package frc.robot;

import com.michaelgrundvig.frc.tuning.TuningInputs;
import java.util.function.Consumer;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * The tuning requests each loop applies, logged as AdvantageKit inputs: recorded as they happen,
 * and in replay read back from the log in their place, so a replay makes the same changes at the
 * same moments.
 */
final class TuningLog implements Consumer<TuningInputs>, LoggableInputs {
  private TuningInputs inputs = new TuningInputs();

  /** Logs this loop's requests, or in replay, replaces them with the logged ones. */
  @Override
  public void accept(TuningInputs inputs) {
    this.inputs = inputs;
    Logger.processInputs("Tuning", this);
  }

  @Override
  public void toLog(LogTable table) {
    table.put("Enabled", inputs.enabled);
    table.put("Kinds", inputs.kinds);
    table.put("Paths", inputs.paths);
    table.put("Values", inputs.values);
    table.put("Requests", inputs.requests);
  }

  @Override
  public void fromLog(LogTable table) {
    inputs.enabled = table.get("Enabled", inputs.enabled);
    inputs.kinds = table.get("Kinds", inputs.kinds);
    inputs.paths = table.get("Paths", inputs.paths);
    inputs.values = table.get("Values", inputs.values);
    inputs.requests = table.get("Requests", inputs.requests);
  }
}
