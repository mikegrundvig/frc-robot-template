package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Setting;
import frc.robot.sim.Scenarios;
import frc.robot.sim.SensorErrors;
import frc.robot.sim.SimulatedWorld;
import java.util.List;
import java.util.Optional;
import java.util.OptionalInt;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.math.geometry.Pose2d;
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

  @Test
  void theSimulatorStartsInTheScenarioAskedFor() {
    assertSame(
        Scenarios.named("collision"), Robot.scenarioAskedFor(Constants.Mode.SIM, "collision"));
    assertNull(Robot.scenarioAskedFor(Constants.Mode.SIM, null));
  }

  @Test
  void aReplayRunsWhatItsLogRecordedNeverAScenario() {
    assertNull(Robot.scenarioAskedFor(Constants.Mode.REPLAY, "collision"));
    assertNull(Robot.scenarioAskedFor(Constants.Mode.REAL, "collision"));
  }

  @Test
  void aMisspelledScenarioStopsTheProgramWithTheList() {
    IllegalArgumentException refused =
        assertThrows(
            IllegalArgumentException.class,
            () -> Robot.scenarioAskedFor(Constants.Mode.SIM, "colision"));

    String message = String.valueOf(refused.getMessage());
    assertTrue(message.contains("camera-low, camera-turned"), message);
  }

  /** As ./gradlew run starts it: on the first loop the robot is enabled, then never again. */
  @Test
  void aScenarioStartsOnceTheRobotIsEnabled() {
    try (SimulatedWorld world =
        new SimulatedWorld(
            RobotSim.FIELD,
            Robots.SIM.drivetrain(),
            Robots.SIM.body(),
            List.of(),
            SensorErrors.NONE)) {
      Components components = new Components(said -> {});
      world.addTo(components);
      List<Setting> collision = Scenarios.named("collision").eventsIn(world);
      Pose2d start = world.truePose();

      assertSame(
          collision, Robot.startWhenEnabled(collision, components, false), "disabled: waits");
      assertEquals(start, world.truePose());
      assertSame(collision, Robot.startWhenEnabled(collision, null, true), "no simulator: waits");
      assertNull(Robot.startWhenEnabled(collision, components, true), "enabled: started");
      assertEquals(0.6, world.truePose().getY() - start.getY(), 1e-9);
      assertNull(Robot.startWhenEnabled(null, components, true), "nothing more to start");
    }
  }

  /** A scenario asked for by name that can't happen stops the program, saying why. */
  @Test
  void aScenarioThatCantHappenStopsTheProgram() {
    Components components = new Components(said -> {});

    IllegalStateException refused =
        assertThrows(
            IllegalStateException.class,
            () -> Robot.apply(List.of(Setting.of("Gyro", "Unplugged")), components));

    assertTrue(
        String.valueOf(refused.getMessage()).contains("no component named Gyro"),
        refused.getMessage());
  }

  @Test
  void aDebugDeployIsKnownByItsDebuggerArgument() {
    assertTrue(
        Robot.startedForDebugger(
            List.of(
                "-Xmx100M", "-agentlib:jdwp=transport=dt_socket,address=0.0.0.0:8349,server=y")));
    assertFalse(Robot.startedForDebugger(List.of("-Xmx100M", "-XX:+UseSerialGC")));
  }
}
