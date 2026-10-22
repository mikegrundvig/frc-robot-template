package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import com.michaelgrundvig.frc.runtime.autos.Autos;
import com.michaelgrundvig.frc.runtime.behavior.Binding;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command3.Command;

/**
 * The team's robot program, put together as its {@code Main} hands it to the runtime: its robot,
 * this season's field, and its controls. It starts, and drives its loop.
 */
@SimulationTest
class TeamProgramSimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  @Test
  void theTeamsRobotRunsWithItsSeasonAndItsControls() {
    sim.build(Robots.current(), Rebuilt.GAME, Robots::controls, SensorErrors.TYPICAL);
    sim.run(1);

    // The robot starts with the choice it always has, plus whatever the team's Controls declares.
    assertThat(sim.autos().names()).contains(Autos.DO_NOTHING);
  }

  /**
   * The program a reader starts with {@code ./gradlew run} does what runtime/docs/guide.md says:
   * the guide's robot has the guide's controls, so the operator's right trigger intakes and the
   * driver's right trigger shoots.
   */
  @Test
  void theGuidesButtonsWorkInTheProgramATeamStartsWith() {
    sim.build(Robots.current(), Rebuilt.GAME, Robots::controls, SensorErrors.TYPICAL);
    RobotSim.setEnabled(true);
    sim.run(0.1);

    sim.operator().setRightTriggerAxis(1);
    sim.operator().notifyNewData();
    sim.run(0.1);
    List<String> afterTheOperatorsTrigger = running();

    sim.driver().setRightTriggerAxis(1);
    sim.driver().notifyNewData();
    sim.run(0.1);
    List<String> afterTheDriversTrigger = running();

    assertThat(afterTheOperatorsTrigger).contains("Intake").doesNotContain("Shoot");
    assertThat(afterTheDriversTrigger).contains("Intake", "Shoot");
    assertThat(sim.autos().names()).contains("Grab one", "Grab and shoot");
  }

  /** The practice robot is the same design, so it has the same buttons. */
  @Test
  void theGuidesPracticeRobotHasTheGuidesControlsToo() {
    sim.build(ExampleRobot.PRACTICE, Rebuilt.GAME, Robots::controls, SensorErrors.TYPICAL);

    assertThat(sim.bindings().all().stream().map(Binding::toString))
        .contains(
            "operator right trigger: while held: Intake",
            "driver right trigger: while held: Shoot");
  }

  /**
   * A robot the team adds is none of the guide's, so it gets the team's own {@code Controls}, which
   * binds nothing and declares no auto until the team writes them.
   */
  @Test
  void aRobotTheTeamAddsGetsTheTeamsOwnControls() {
    RobotConfig ours = ExampleRobot.COMPETITION.named("ours");

    sim.build(ours, Rebuilt.GAME, Robots::controls, SensorErrors.TYPICAL);

    assertThat(sim.bindings().all().stream().map(Binding::toString))
        .as("the runtime's own binding, and none of the guide's")
        .containsExactly("driver back: on press: Reset heading");
    assertThat(sim.autos().names()).containsExactly(Autos.DO_NOTHING);
  }

  /** The names of the commands running now. */
  private List<String> running() {
    return sim.scheduler().getRunningCommands().stream().map(Command::name).toList();
  }

  @ParameterizedTest
  @MethodSource("robotsWithControlsOfTheirOwn")
  void aRobotAskedForByNameRunsWithItsOwnControls(RobotConfig robot, String anAutoOfItsOwn) {
    sim.build(robot, Rebuilt.GAME, Robots::controls, SensorErrors.TYPICAL);
    sim.run(1);

    assertThat(sim.autos().names()).contains(Autos.DO_NOTHING, anAutoOfItsOwn);
  }

  static Stream<Arguments> robotsWithControlsOfTheirOwn() {
    // Beta's controls declare its autos. Delta's declare none, so "Do nothing" is all it has.
    return Stream.of(
        Arguments.of(BetaRobot.CONFIG, "Left Single"),
        Arguments.of(DeltaRobot.CONFIG, Autos.DO_NOTHING));
  }
}
