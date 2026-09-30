package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.ElasticLayoutCheck.Widget;
import frc.robot.game.Rebuilt;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.telemetry.MockTelemetryBackend;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/**
 * The drive team's dashboard layout (`layouts/elastic-layout.json`, edited in Elastic) against the
 * robot code: every widget reads a value the robot publishes, of a kind it can show. When this
 * fails, a widget would sit blank on the Driver Station: point it at a value in {@link Dashboard},
 * or publish the value it needs there.
 */
class DashboardLayoutTest {
  private static final Path LAYOUT = Path.of("layouts/elastic-layout.json");

  private final Scheduler scheduler = Scheduler.createIndependentScheduler();
  private final MockTelemetryBackend telemetry = new MockTelemetryBackend();
  private @Nullable RobotContainer container;

  @BeforeAll
  static void initHal() {
    SimTestSupport.initHal();
  }

  @BeforeEach
  void setUp() {
    SimTestSupport.pauseTimeAndDisable();
    DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_2);
    DriverStationSim.notifyNewData();
    TelemetryRegistry.registerBackend("", telemetry);
    TunableRegistry.registerBackend("", new MockTunableBackend());
  }

  @AfterEach
  void tearDown() {
    if (container != null) {
      container.close();
    }
    TelemetryRegistry.reset();
    TunableRegistry.reset();
    SimTestSupport.resumeTime();
  }

  /** The whole robot in the simulator for a while, disabled and then enabled. */
  private Map<String, Object> runTheRobot() {
    container =
        new RobotContainer(
            Constants.Mode.SIM,
            scheduler,
            Robots.SIM,
            Robots.SIM::drivetrain,
            Rebuilt.field(Robots.SIM.body()));
    RobotContainer robot = Objects.requireNonNull(container);
    for (int i = 0; i < SimTestSupport.loops(1.5); i++) {
      if (i == SimTestSupport.loops(1.0)) {
        SimTestSupport.setEnabled(true);
      }
      SimTestSupport.stepLoop();
      robot.periodic();
      scheduler.run();
    }
    return ElasticLayoutCheck.published(telemetry);
  }

  @Test
  void everyWidgetReadsAValueTheRobotPublishes() throws IOException {
    List<Widget> widgets = ElasticLayoutCheck.widgets(Files.readString(LAYOUT));

    List<String> problems = ElasticLayoutCheck.problems(widgets, runTheRobot());

    assertThat(widgets).as("widgets found in the layout").hasSizeGreaterThan(5);
    assertThat(problems).isEmpty();
  }

  @Test
  void theCheckNamesWhatIsWrong() {
    List<Widget> widgets =
        List.of(
            new Widget("Match", "Ready", "Boolean Box", "/Telemetry/Dashboard/Renamed"),
            new Widget("Match", "Time", "Match Time", "/Telemetry/Dashboard/Scoring"),
            new Widget("Match", "Pose", "Text Display", "/AdvantageKit/RealOutputs/Pose"),
            new Widget("Match", "Alerts", "Alerts", "/Telemetry/Dashboard/Alerts"));
    Map<String, Object> published =
        Map.of(
            "/Dashboard/Scoring",
            new MockTelemetryBackend.LogStringValue("Ready to score", "string"),
            "/Dashboard/Alerts/errors",
            new String[0],
            "/Dashboard/Alerts/warnings",
            new String[0]);

    List<String> problems = ElasticLayoutCheck.problems(widgets, published);

    assertThat(problems)
        .containsExactly(
            "Match, \"Ready\" (Boolean Box): /Telemetry/Dashboard/Renamed is never published",
            "Match, \"Time\" (Match Time): /Telemetry/Dashboard/Scoring is text, not a number as"
                + " it shows",
            "Match, \"Pose\" (Text Display): /AdvantageKit/RealOutputs/Pose is outside the robot's"
                + " telemetry (publish it through Dashboard, under /Telemetry)",
            "Match, \"Alerts\" (Alerts): /Telemetry/Dashboard/Alerts/infos is never published");
  }

  @Test
  void itFindsWidgetsInsideListLayouts() {
    String layout =
        """
        {"version": 1.0, "grid_size": 128, "tabs": [{"name": "Pit", "grid_layout": {
          "containers": [],
          "layouts": [{"title": "Checks", "type": "List Layout", "children": [
            {"title": "Homed", "type": "Boolean Box",
             "properties": {"topic": "/Telemetry/Dashboard/Homed"}}]}]}}]}
        """;

    assertThat(ElasticLayoutCheck.widgets(layout))
        .containsExactly(new Widget("Pit", "Homed", "Boolean Box", "/Telemetry/Dashboard/Homed"));
  }
}
