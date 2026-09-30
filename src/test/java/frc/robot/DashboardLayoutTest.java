package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.ElasticLayoutCheck.Widget;
import frc.robot.sim.SensorErrors;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.wpilib.telemetry.MockTelemetryBackend;

/**
 * The drive team's dashboard layout (`layouts/elastic-layout.json`, edited in Elastic) against the
 * robot code: every widget reads a value the robot publishes, of a kind it can show. When this
 * fails, a widget would sit blank on the Driver Station: point it at a value in {@link
 * frc.robot.dashboard.Dashboard}, or publish the value it needs there.
 */
@SimulationTest
class DashboardLayoutTest {
  private static final Path LAYOUT =
      Path.of(System.getProperty("frc.projectDir", "."), "layouts", "elastic-layout.json");

  @RegisterExtension final RobotSim sim = new RobotSim();

  /** The whole robot in the simulator for a while, disabled and then enabled. */
  private Map<String, Object> runTheRobot() {
    sim.build(SensorErrors.TYPICAL);
    sim.run(1.0);
    RobotSim.setEnabled(true);
    sim.run(0.5);
    return ElasticLayoutCheck.published(sim.telemetry());
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
