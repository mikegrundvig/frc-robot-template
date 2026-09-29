package frc.robot.tuning;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class TuningInputsTest {
  @Test
  void whatIsLoggedIsWhatReplayReads() {
    TuningInputs logged = new TuningInputs();
    logged.enabled = true;
    logged.kinds = new String[] {"SET", "RESET"};
    logged.paths = new String[] {"drivetrain.gains.steerKp", ""};
    logged.values = new double[] {18.5, 0};
    LogTable table = new LogTable(0);

    logged.toLog(table);
    TuningInputs replayed = new TuningInputs();
    replayed.fromLog(table);

    assertThat(replayed.enabled).isTrue();
    assertThat(replayed.kinds).containsExactly("SET", "RESET");
    assertThat(replayed.paths).containsExactly("drivetrain.gains.steerKp", "");
    assertThat(replayed.values).containsExactly(18.5, 0);
  }

  @Test
  void aQuietLoopLogsNoRequests() {
    TuningInputs inputs = new TuningInputs();
    inputs.kinds = new String[] {"SET"};

    inputs.clearRequests();

    assertThat(inputs.kinds).isEmpty();
    assertThat(inputs.paths).isEmpty();
    assertThat(inputs.values).isEmpty();
  }
}
