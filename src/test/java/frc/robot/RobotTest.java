package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;
import java.util.OptionalInt;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.simulation.DriverStationSim;

/**
 * The robot program itself. {@code ./gradlew run} starts the simulator enabled in teleop so a
 * first-time driver can drive straight away; only a physics simulation that asked for it may enable
 * itself, since replay gets its enable state from the log. And a session without a writable log
 * drive says so.
 */
class RobotTest {
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

  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
  }

  @BeforeEach
  void setUp() {
    SimTestSupport.pauseTimeAndDisable();
  }

  @AfterEach
  void tearDown() {
    SimTestSupport.setEnabled(false);
    SimTestSupport.resumeTime();
  }

  @Test
  void simulatorStartsEnabledWhenAsked() {
    Robot.startEnabledIfRequested(Constants.Mode.SIM, true);

    assertTrue(DriverStationSim.getEnabled(), "enabled");
    assertTrue(DriverStationSim.getDsAttached(), "driver station attached");
    assertEquals(Optional.of(Alliance.BLUE), MatchState.getAlliance(), "on an alliance");
    assertEquals(OptionalInt.of(2), MatchState.getLocation(), "at the center start's station");
  }

  @Test
  void aStationChosenInTheSimulatorIsKept() {
    DriverStationSim.setAllianceStationId(AllianceStationID.RED_3);
    DriverStationSim.notifyNewData();

    Robot.startEnabledIfRequested(Constants.Mode.SIM, true);

    assertEquals(Optional.of(Alliance.RED), MatchState.getAlliance());
    assertEquals(OptionalInt.of(3), MatchState.getLocation());
  }

  @Test
  void simulatorStartsDisabledUnlessAsked() {
    Robot.startEnabledIfRequested(Constants.Mode.SIM, false);

    assertFalse(DriverStationSim.getEnabled());
  }

  @Test
  void replayNeverEnablesItself() {
    Robot.startEnabledIfRequested(Constants.Mode.REPLAY, true);

    assertFalse(DriverStationSim.getEnabled());
  }
}
