package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.Outcome;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.Status;
import com.michaelgrundvig.frc.pitwall.TunableTee;
import frc.robot.game.Rebuilt;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;

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
    TunableRegistry.reset();
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
  void everyWpilibTunableGoesThroughPitwallAndTheConfigurationIsOnNetworkTables() {
    TunableDouble gain = TunableDouble.create(1.0);
    TunableRegistry.publish("Shooter/gain", gain);
    TunableRegistry.update();

    assertThat(pitwall.tunables().view())
        .extracting(TunableTee.Listed::path)
        .containsExactly("/Shooter/gain");
    try (var steerKp =
        NetworkTableInstance.getDefault()
            .getDoubleTopic("/Tunables/Config/drivetrain/gains/steerKp")
            .subscribe(-1)) {
      assertThat(steerKp.get()).as("dashboards see the configuration").isEqualTo(16.0);
    }
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
            Robots.SIM,
            () -> pitwall.config().drivetrain(),
            Rebuilt.field(Robots.SIM.body()))) {
      container.addTo(pitwall);

      SimTestSupport.setEnabled(true);
      setMode(RobotMode.UTILITY);
      pitwall.actions().requestStart(RobotContainer.DRIVE_CHECK);
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

  /**
   * The field lock on the running robot program: with the field system connected, no tunable
   * changes, whether the page or a dashboard asks. Pitwall's headline safety promise.
   */
  @Test
  void theFieldLocksEveryTunableFromThePageAndFromDashboards() throws Exception {
    TunableDouble gain = TunableDouble.create(1.0);
    TunableRegistry.publish("Shooter/gain", gain);
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.notifyNewData();
    pitwall.periodic();

    var page = patch("/api/wpilib-tunables", "{\"Shooter/gain\": 2.0}");
    var config = patch("/api/values", "{\"drivetrain.gains.steerKp\": 18}");
    try (var dashboard =
        NetworkTableInstance.getDefault().getDoubleTopic("/Tunables/Shooter/gain").publish()) {
      dashboard.set(3.0);
      TunableRegistry.update();

      assertThat(page.statusCode()).isEqualTo(409);
      assertThat(page.body()).contains("the field system is connected");
      assertThat(config.statusCode()).isEqualTo(409);
      assertThat(gain.get()).as("neither the page nor a dashboard").isEqualTo(1.0);
      assertThat(pitwall.config().drivetrain().gains().steerKp()).isEqualTo(16.0);

      // The same dashboard write, off the field with tuning switched back on (the field switched it
      // off): it gets through, so the lock is what held it.
      DriverStationSim.setFmsAttached(false);
      DriverStationSim.notifyNewData();
      assertThat(patch("/api/tuning", "{\"enabled\": true}").statusCode()).isEqualTo(200);
      dashboard.set(4.0);
      TunableRegistry.update();
      assertThat(gain.get()).isEqualTo(4.0);
    }
  }

  @Test
  void tuningSwitchedOffLocksEveryTunable() throws Exception {
    TunableDouble gain = TunableDouble.create(1.0);
    TunableRegistry.publish("Shooter/gain", gain);

    var off = patch("/api/tuning", "{\"enabled\": false}");
    var page = patch("/api/wpilib-tunables", "{\"Shooter/gain\": 2.0}");

    assertThat(off.body()).contains("\"enabled\":false");
    assertThat(page.statusCode()).isEqualTo(409);
    assertThat(page.body()).contains("tuning is off");
    assertThat(gain.get()).isEqualTo(1.0);
  }

  /**
   * When the robot disables, what tuning changed is printed as the builder calls that commit it.
   */
  @Test
  void disablingPrintsWhatTuningChanged() throws Exception {
    SimTestSupport.setEnabled(true);
    pitwall.periodic();
    patch("/api/values", "{\"drivetrain.gains.steerKp\": 18}");
    PrintStream console = System.out;
    ByteArrayOutputStream printed = new ByteArrayOutputStream();
    System.setOut(new PrintStream(printed, true, StandardCharsets.UTF_8));
    try {
      SimTestSupport.setEnabled(false);
      pitwall.periodic();
    } finally {
      System.setOut(console);
    }

    assertThat(printed.toString(StandardCharsets.UTF_8))
        .contains("drivetrain.gains.steerKp: 16.0 -> 18.0")
        .contains(".steerKp(18.0)");
  }

  /**
   * Sends the page's request and runs the robot loop until it's answered, as the robot does: the
   * server waits for the loop to apply a change.
   */
  private HttpResponse<String> patch(String path, String json) throws Exception {
    try (HttpClient client = HttpClient.newHttpClient()) {
      var answer =
          client.sendAsync(
              HttpRequest.newBuilder(URI.create("http://localhost:" + pitwall.port() + path))
                  .header("Content-Type", "application/merge-patch+json")
                  .method("PATCH", HttpRequest.BodyPublishers.ofString(json))
                  .build(),
              HttpResponse.BodyHandlers.ofString());
      for (int i = 0; i < 400 && !answer.isDone(); i++) {
        pitwall.periodic();
        TunableRegistry.update();
        Thread.sleep(5);
      }
      return answer.get(1, TimeUnit.SECONDS);
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
