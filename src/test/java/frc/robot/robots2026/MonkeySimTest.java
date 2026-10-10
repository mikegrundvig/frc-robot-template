package frc.robot.robots2026;

import com.michaelgrundvig.frc.runtime.Monkey;
import com.michaelgrundvig.frc.runtime.MonkeyMatches;
import com.michaelgrundvig.frc.runtime.RobotSim;
import com.michaelgrundvig.frc.runtime.SimulationTest;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * The monkey ({@link Monkey}) on a team's two 2026 robots, Beta and Delta, each with its own
 * controls: a match of random presses, sticks, commands and faults for each of a few seeds, every
 * loop checked against what must always hold. {@code -PmonkeySeeds=<count>} runs more seeds, and
 * {@code -PmonkeySeed=<seed>} runs one alone, the one a failure names (docs/testing.md).
 */
@SimulationTest
class MonkeySimTest {
  @RegisterExtension final RobotSim sim = new RobotSim();

  static final List<Monkey.Robot> ROBOTS = List.of(BetaMonkey.ROBOT, DeltaMonkey.ROBOT);

  static Stream<Arguments> runs() {
    return ROBOTS.stream()
        .flatMap(robot -> MonkeyMatches.seeds().mapToObj(seed -> Arguments.of(robot, seed)));
  }

  @ParameterizedTest(name = "{0}, seed {1}")
  @MethodSource("runs")
  @Timeout(600)
  void aMatchOfMonkeyBreaksNothing(Monkey.Robot robot, long seed) {
    MonkeyMatches.play(sim, robot, seed);
  }
}
