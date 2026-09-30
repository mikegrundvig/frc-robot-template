package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import java.util.OptionalInt;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.simulation.DriverStationSim;

/**
 * The robot program itself. {@code ./gradlew run} starts the simulator enabled in teleop so a
 * first-time driver can drive straight away; only a physics simulation that asked for it may enable
 * itself, since replay gets its enable state from the log.
 */
@ExtendWith(RobotSim.class)
class RobotTest {
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
