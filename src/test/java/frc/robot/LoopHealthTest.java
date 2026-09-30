package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/** Whether the loop keeps up, and its log with it: the loop's times given by hand. */
class LoopHealthTest {
  private static final long MS = 1_000_000L;

  private final LoopHealth health = new LoopHealth();

  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
  }

  @AfterEach
  void tearDown() {
    health.close();
  }

  /** One loop, starting at a time and taking so long, as the robot runs it. */
  private void loop(long startMs, long tookMs, boolean logDropped) {
    health.start(startMs * MS);
    health.monitor().mark("Commands", (startMs + tookMs) * MS);
    health.finish((startMs + tookMs) * MS, logDropped);
  }

  @Test
  void anOverrunRaisesAnAlertNamingWhereTheTimeWent() {
    loop(0, 5, false);
    String quick = health.describe();
    boolean alertAfterQuick = health.isOverrunAlertActive();

    loop(20, 35, false);

    assertThat(quick).isEqualTo("5.0 ms, worst 5.0 ms lately");
    assertThat(alertAfterQuick).isFalse();
    assertThat(health.isOverrunAlertActive()).isTrue();
    assertThat(health.overrunAlertText())
        .isEqualTo(
            "The loop overran 20 ms 1 time in the last 10 s; the last took 35 ms, 35 of them in"
                + " Commands");
    assertThat(health.describe()).isEqualTo("Overran 1 time lately, worst 35.0 ms");
  }

  /** Time lost between loops (writing the log, garbage collection) shows as a late start. */
  @Test
  void aLateStartRaisesTheAlertThoughTheRobotsCodeWasQuick() {
    loop(0, 5, false);
    loop(35, 5, false); // 15 ms after its time

    assertThat(health.isOverrunAlertActive()).isTrue();
    assertThat(health.overrunAlertText())
        .startsWith("1 loop started late in the last 10 s: time lost between loops");
  }

  /** A log with a gap keeps its gap: the alert stays raised once data was dropped. */
  @Test
  void droppedLogDataStaysRaisedForTheSession() {
    loop(0, 5, true);
    loop(20, 5, false);

    assertThat(health.isLoggingBehindAlertActive()).isTrue();
  }

  @Test
  void aSessionWithoutADriveSaysItIsntLogged(@TempDir Path folder) {
    File drive = folder.toFile();
    File missing = folder.resolve("U").toFile();

    assertThat(LoopHealth.logDriveProblem(drive, 0)).isNull();
    assertThat(LoopHealth.logDriveProblem(missing, 0))
        .isEqualTo("No USB drive at " + missing + ": this session isn't being logged");
  }

  @Test
  void aNearlyFullDriveSaysLoggingWillStop(@TempDir Path folder) {
    assertThat(LoopHealth.logDriveProblem(folder.toFile(), Long.MAX_VALUE))
        .contains("MB free: logging stops when it's full");
  }

  @Test
  void aSessionOnAReadOnlyDriveSaysItIsntLogged(@TempDir Path folder) throws Exception {
    File readOnly = Files.createDirectory(folder.resolve("locked")).toFile();
    // Linux, where the robot runs, can lock a folder against writing. Windows ignores a folder's
    // read-only flag, and root can write anywhere, so there this case can't be set up: skipped.
    assumeTrue(
        readOnly.setWritable(false) && !readOnly.canWrite(),
        "this system can't make a folder read-only");

    assertThat(LoopHealth.logDriveProblem(readOnly, 0)).contains("can't be written");
  }
}
