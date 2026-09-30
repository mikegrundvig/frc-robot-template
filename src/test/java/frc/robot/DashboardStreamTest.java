package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

/** What reaches the dashboards, and whether a session is being logged at all. */
class DashboardStreamTest {
  /** A dashboard that remembers which loops reached it. */
  private static final class Dashboard implements LogDataReceiver {
    final List<Long> received = new ArrayList<>();
    boolean started;
    boolean ended;

    @Override
    public void start() {
      started = true;
    }

    @Override
    public void end() {
      ended = true;
    }

    @Override
    public void putTable(LogTable table) {
      received.add(table.getTimestamp());
    }
  }

  private static LogTable loop(long timestamp, boolean fieldAttached) {
    LogTable table = new LogTable(timestamp);
    table.put("DriverStation/FMSAttached", fieldAttached);
    return table;
  }

  @Test
  void inThePitEveryLoopStreamsAndOnTheFieldOneInFive() throws InterruptedException {
    Dashboard dashboard = new Dashboard();
    DashboardStream stream = new DashboardStream(dashboard);

    stream.start();
    for (long t = 0; t < 3; t++) {
      stream.putTable(loop(t, false));
    }
    for (long t = 3; t < 13; t++) {
      stream.putTable(loop(t, true));
    }
    stream.end();

    assertThat(dashboard.received).containsExactly(0L, 1L, 2L, 5L, 10L);
    assertThat(dashboard.started).isTrue();
    assertThat(dashboard.ended).isTrue();
  }

  @Test
  void aSessionWithoutADriveSaysItIsntLogged(@TempDir Path folder) {
    File drive = folder.toFile();
    File missing = folder.resolve("U").toFile();

    assertThat(Robot.logDriveProblem(drive)).isNull();
    assertThat(Robot.logDriveProblem(missing))
        .isEqualTo("No USB drive at " + missing + ": this session isn't being logged");
  }

  @Test
  void aSessionOnAReadOnlyDriveSaysItIsntLogged(@TempDir Path folder) throws Exception {
    File readOnly = Files.createDirectory(folder.resolve("locked")).toFile();
    // Linux, where the robot runs, can lock a folder against writing. Windows ignores a folder's
    // read-only flag, and root can write anywhere, so there this case can't be set up: skipped.
    assumeTrue(
        readOnly.setWritable(false) && !readOnly.canWrite(),
        "this system can't make a folder read-only");

    assertThat(Robot.logDriveProblem(readOnly)).contains("can't be written");
  }
}
