package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.EndToEndTest;
import com.michaelgrundvig.frc.runtime.SessionRecording;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.io.CleanupMode;
import org.junit.jupiter.api.io.TempDir;

/**
 * The AdvantageScope layout (layouts/advantagescope.json) reads only what the robot logs, as the
 * types it says. The robot program records a session in the simulator, and the log is what the
 * robot logs, by name and type: every value the layout reads must be there. A source that reads a
 * value the robot doesn't log sits empty, and looks like "nothing happened". The failure names the
 * tab and the source; point it at a value the robot logs, or log the value it needs.
 */
@EndToEndTest
class AdvantageScopeLayoutEndToEndTest {
  @TempDir(cleanup = CleanupMode.ON_SUCCESS) // a failed run keeps its logs to look at
  Path dir;

  @Test
  @Timeout(value = 2, unit = TimeUnit.MINUTES) // one run of the program, allowed 90 s
  void theAdvantageScopeLayoutReadsOnlyWhatTheRobotLogs() throws Exception {
    Path recorded = SessionRecording.record(dir);

    List<AdvantageScopeLayoutCheck.Source> sources =
        AdvantageScopeLayoutCheck.sources(readLayout());
    Map<String, String> logged = AdvantageScopeLayoutCheck.logged(recorded);
    List<String> problems =
        AdvantageScopeLayoutCheck.problems(sources, logged, AdvantageScopeLayoutTest.models());

    assertThat(sources).hasSizeGreaterThan(20);
    assertThat(problems).isEmpty();
    assertThat(factsMissingFrom(logged)).isEmpty();
  }

  /**
   * What a joint and the controls did is logged as facts, each a record AdvantageKit writes as a
   * struct. A record it cannot write is skipped as it is logged, so its key would be missing from
   * the log, and whatever reads it would sit empty. These are the facts every session logs, each by
   * the end of its key and the struct it is; the ones that are missing are handed back. The robot
   * this session runs has no gate, so a gate's terms are checked where they are made (the runtime's
   * {@code RecordedFactsTest} and its gates' tests).
   */
  private static List<String> factsMissingFrom(Map<String, String> logged) {
    Map<String, String> structByKeysEnd =
        Map.of(
            "/State", "JointState",
            "/At", "JointPlace",
            "/HeldBy", "Holder[]",
            "/Controls/Fired", "BindingFired[]");
    List<String> missing = new ArrayList<>();
    structByKeysEnd.forEach(
        (keysEnd, struct) -> {
          boolean isLogged =
              logged.entrySet().stream()
                  .anyMatch(
                      each ->
                          each.getKey().startsWith("/RealOutputs/")
                              && each.getKey().endsWith(keysEnd)
                              && each.getValue().equals(struct));
          if (!isLogged) {
            missing.add(keysEnd + " as " + struct);
          }
        });
    if (!logged.containsKey("/RealOutputs/Robot/Names/Interlocks")) {
      missing.add("/Robot/Names/Interlocks");
    }
    return missing;
  }

  private static String readLayout() throws IOException {
    return Files.readString(AdvantageScopeLayoutTest.LAYOUT);
  }
}
