package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.simulation.DriverStationSim;

/**
 * {@code ./gradlew run} starts the simulator enabled in teleop so a first-time driver can drive
 * straight away. Only a physics simulation that asked for it may enable itself: replay gets its
 * enable state from the log.
 */
class RobotSimLaunchTest {
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
