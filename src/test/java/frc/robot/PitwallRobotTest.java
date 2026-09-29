package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Outcome;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.Status;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;

/** Pitwall in the simulated robot program: its standard status, and actions that run commands. */
class PitwallRobotTest {
  private Pitwall<RobotConfig> pitwall;

  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
  }

  @BeforeEach
  void setUp() {
    SimTestSupport.pauseTimeAndDisable();
    TelemetryRegistry.registerBackend("", new MockTelemetryBackend()); // the simulator's field view
    pitwall = Pitwall.start(Robots.SIM.name(), Robots.SIM, 0);
  }

  @AfterEach
  void tearDown() {
    pitwall.close();
    Scheduler.getDefault().cancelAll();
    SimTestSupport.setEnabled(false);
    SimTestSupport.resumeTime();
    TelemetryRegistry.reset();
  }

  @Test
  void theStatusShowsTheRobotItsCodeItsModeAndTheBattery() {
    pitwall.periodic();

    assertThat(pitwall.status().view())
        .extracting(Status.Item::name)
        .containsExactly("Robot", "Code", "Mode", "Battery");
    assertThat(value("Robot")).isEqualTo("sim");
    assertThat(value("Code")).contains(" on " + BuildConstants.GIT_BRANCH);
    assertThat(value("Battery")).matches("\\d+\\.\\d V");
    assertThat(pitwall.port()).as("the page is served").isPositive();
  }

  @Test
  void theModeSaysWhetherTheRobotIsEnabledAndOnTheField() {
    assertThat(modeAfterALoop()).isEqualTo("Teleop, disabled");

    SimTestSupport.setEnabled(true);
    assertThat(modeAfterALoop()).isEqualTo("Teleop, enabled");

    DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.notifyNewData();
    assertThat(modeAfterALoop()).isEqualTo("Autonomous, enabled, on the field");

    DriverStationSim.setFmsAttached(false);
    setMode(RobotMode.UTILITY);
    assertThat(modeAfterALoop()).isEqualTo("Utility, enabled");

    setMode(RobotMode.UNKNOWN);
    assertThat(modeAfterALoop()).startsWith("No mode");
  }

  @Test
  void anActionRunsItsCommandOnlyInUtilityModeAndShowsWhenItEnds() {
    pitwall
        .actions()
        .add(
            "Wait a moment",
            () -> Command.noRequirements(co -> co.wait(Seconds.of(0.1))).named("Wait"));

    var locked = pitwall.actions().requestStart("Wait a moment");
    loops(1);
    assertThat(locked.getNow(null).result()).as("teleop").isEqualTo(Outcome.Result.LOCKED);

    SimTestSupport.setEnabled(true);
    setMode(RobotMode.UTILITY);
    var started = pitwall.actions().requestStart("Wait a moment");
    loops(1);
    assertThat(started.getNow(null)).isEqualTo(new Outcome(Outcome.Result.APPLIED, ""));
    assertThat(phase()).isEqualTo(Actions.Phase.RUNNING);

    loops(10);
    assertThat(phase()).as("its 0.1 s wait is over").isEqualTo(Actions.Phase.ENDED);
  }

  @Test
  void theDriveCheckIsAnActionOnThePage() {
    try (var container =
        new RobotContainer(
            Constants.Mode.SIM,
            Scheduler.getDefault(),
            Robots.SIM.body(),
            () -> pitwall.config().drivetrain())) {
      pitwall.actions().add("Drive forward slowly (2 s)", container::driveCheck);

      SimTestSupport.setEnabled(true);
      setMode(RobotMode.UTILITY);
      pitwall.actions().requestStart("Drive forward slowly (2 s)");
      for (int i = 0; i < 25; i++) {
        SimTestSupport.stepLoop();
        pitwall.periodic();
        container.periodic();
        Scheduler.getDefault().run();
      }

      assertThat(phase()).isEqualTo(Actions.Phase.RUNNING);
      assertThat(container.drive().getRobotRelativeVelocities().vx)
          .as("driving forward slowly")
          .isBetween(0.4, 0.6);
    }
  }

  private String value(String name) {
    return pitwall.status().view().stream()
        .filter(item -> item.name().equals(name))
        .findFirst()
        .orElseThrow()
        .value()
        .toString();
  }

  private String modeAfterALoop() {
    pitwall.periodic();
    return value("Mode");
  }

  private Actions.Phase phase() {
    return pitwall.actions().view().get(0).phase();
  }

  private void loops(int count) {
    for (int i = 0; i < count; i++) {
      SimTestSupport.stepLoop();
      pitwall.periodic();
      Scheduler.getDefault().run();
    }
  }

  private static void setMode(RobotMode mode) {
    DriverStationSim.setRobotMode(mode);
    DriverStationSim.notifyNewData();
  }
}
