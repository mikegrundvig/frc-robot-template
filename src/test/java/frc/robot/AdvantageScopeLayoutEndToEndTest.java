package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.EndToEndTest;
import com.michaelgrundvig.frc.runtime.SessionRecording;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
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
    List<String> problems =
        AdvantageScopeLayoutCheck.problems(
            sources, AdvantageScopeLayoutCheck.logged(recorded), AdvantageScopeLayoutTest.models());

    assertThat(sources).hasSizeGreaterThan(20);
    assertThat(problems).isEmpty();
  }

  private static String readLayout() throws IOException {
    return Files.readString(AdvantageScopeLayoutTest.LAYOUT);
  }
}
