package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.TuningInputs;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.LogTable;

class TuningLogTest {
  @Test
  void whatIsLoggedIsWhatReplayReads() {
    TuningInputs live = new TuningInputs();
    live.enabled = true;
    live.kinds = new String[] {"SET", "RESET"};
    live.paths = new String[] {"drivetrain.gains.steerKp", ""};
    live.values = new double[] {18.5, 0};
    live.requests = new int[] {0, 1};
    LogTable table = new LogTable(0);
    TuningLog log = new TuningLog();
    log.accept(live); // AdvantageKit isn't running here, so this only takes the inputs in hand

    log.toLog(table);
    TuningInputs replayed = new TuningInputs();
    TuningLog replay = new TuningLog();
    replay.accept(replayed);
    replay.fromLog(table);

    assertThat(replayed.enabled).isTrue();
    assertThat(replayed.kinds).containsExactly("SET", "RESET");
    assertThat(replayed.paths).containsExactly("drivetrain.gains.steerKp", "");
    assertThat(replayed.values).containsExactly(18.5, 0);
    assertThat(replayed.requests).containsExactly(0, 1);
  }
}
