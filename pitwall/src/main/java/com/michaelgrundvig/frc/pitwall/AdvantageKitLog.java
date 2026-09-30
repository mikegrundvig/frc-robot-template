package com.michaelgrundvig.frc.pitwall;

import java.util.function.Consumer;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Pitwall's requests as AdvantageKit inputs, recorded as they happen and read back in their place
 * during a replay, so the replay makes the same requests at the same moments. Loaded only when the
 * robot program has AdvantageKit ({@link Pitwall} checks first).
 */
final class AdvantageKitLog {
  private AdvantageKitLog() {}

  /** Whether AdvantageKit is replaying a log, when the page must not start. */
  static boolean replaying() {
    return Logger.hasReplaySource();
  }

  static void recordOutput(String key, String[] value) {
    Logger.recordOutput(key, value);
  }

  static Consumer<TuningInputs> tuning() {
    return new TuningLog();
  }

  static Consumer<ActionInputs> actions() {
    return new ActionLog();
  }

  static Consumer<TunableInputs> tunables() {
    return new TunableLog();
  }

  static final class TuningLog implements Consumer<TuningInputs>, LoggableInputs {
    private TuningInputs inputs = new TuningInputs();

    @Override
    public void accept(TuningInputs inputs) {
      this.inputs = inputs;
      Logger.processInputs("Pitwall/Tuning", this);
    }

    @Override
    public void toLog(LogTable table) {
      table.put("Enabled", inputs.enabled);
      table.put("Kinds", inputs.kinds);
    }

    @Override
    public void fromLog(LogTable table) {
      inputs.enabled = table.get("Enabled", inputs.enabled);
      inputs.kinds = table.get("Kinds", inputs.kinds);
    }
  }

  static final class TunableLog implements Consumer<TunableInputs>, LoggableInputs {
    private TunableInputs inputs = new TunableInputs();

    @Override
    public void accept(TunableInputs inputs) {
      this.inputs = inputs;
      Logger.processInputs("Pitwall/Tunables", this);
    }

    @Override
    public void toLog(LogTable table) {
      table.put("Paths", inputs.paths);
      table.put("Values", inputs.values);
    }

    @Override
    public void fromLog(LogTable table) {
      inputs.paths = table.get("Paths", inputs.paths);
      inputs.values = table.get("Values", inputs.values);
    }
  }

  static final class ActionLog implements Consumer<ActionInputs>, LoggableInputs {
    private ActionInputs inputs = new ActionInputs();

    @Override
    public void accept(ActionInputs inputs) {
      this.inputs = inputs;
      Logger.processInputs("Pitwall/Actions", this);
    }

    @Override
    public void toLog(LogTable table) {
      table.put("Kinds", inputs.kinds);
      table.put("Names", inputs.names);
    }

    @Override
    public void fromLog(LogTable table) {
      inputs.kinds = table.get("Kinds", inputs.kinds);
      inputs.names = table.get("Names", inputs.names);
    }
  }
}
