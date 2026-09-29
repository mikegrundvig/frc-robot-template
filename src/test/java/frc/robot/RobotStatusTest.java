package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.Status;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;

/** Pitwall's status shows which robot and code are running, the mode, and the battery. */
class RobotStatusTest {
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
  void showsTheRobotItsCodeItsModeAndTheBattery() {
    Status status = RobotStatus.of(Robots.SIM);

    status.periodic();

    assertThat(status.view())
        .extracting(Status.Item::name)
        .containsExactly("Robot", "Code", "Mode", "Battery");
    assertThat(status.view().get(0).value()).isEqualTo("sim");
    assertThat(status.view().get(1).value().toString())
        .contains(" on " + BuildConstants.GIT_BRANCH);
    assertThat(status.view().get(3).value().toString()).matches("\\d+\\.\\d V");
  }

  @Test
  void theModeSaysWhetherTheRobotIsEnabledAndOnTheField() {
    assertThat(RobotStatus.mode()).isEqualTo("Teleop, disabled");

    SimTestSupport.setEnabled(true);
    assertThat(RobotStatus.mode()).isEqualTo("Teleop, enabled");

    mode(RobotMode.AUTONOMOUS);
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.notifyNewData();
    assertThat(RobotStatus.mode()).isEqualTo("Autonomous, enabled, on the field");

    DriverStationSim.setFmsAttached(false);
    mode(RobotMode.UTILITY);
    assertThat(RobotStatus.mode()).isEqualTo("Utility, enabled");

    mode(RobotMode.UNKNOWN);
    assertThat(RobotStatus.mode()).startsWith("No mode");
  }

  private static void mode(RobotMode mode) {
    DriverStationSim.setRobotMode(mode);
    DriverStationSim.notifyNewData();
  }
}
