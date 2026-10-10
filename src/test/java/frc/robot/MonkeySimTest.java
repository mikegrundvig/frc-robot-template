package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.examples.ExampleMonkey;
import frc.robot.robots2026.BetaMonkey;
import frc.robot.robots2026.DeltaMonkey;
import java.util.List;
import java.util.stream.LongStream;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.extension.RegisterExtension;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * The monkey ({@link Monkey}) on the guide's robot and on a team's two 2026 robots, Beta and Delta,
 * each with its own controls: a match of random presses, sticks, commands and faults for each of a
 * few seeds, every loop checked against what must always hold. {@code -PmonkeySeeds=<count>} runs
 * more seeds, and {@code -PmonkeySeed=<seed>} runs one alone, the one a failure names
 * (docs/testing.md).
 */
@SimulationTest
class MonkeySimTest {
  /** How many seeds each robot runs, unless asked for more or fewer. */
  static final int SEEDS = 3;

  /** A match, in seconds: disabled a second, 15 s of autonomous, disabled 2 s, 135 s of teleop. */
  static final double MATCH = 153;

  @RegisterExtension final RobotSim sim = new RobotSim();

  static final List<Monkey.Robot> ROBOTS =
      List.of(ExampleMonkey.ROBOT, BetaMonkey.ROBOT, DeltaMonkey.ROBOT);

  /** The seeds asked for: {@code -PmonkeySeed} alone, or the first {@code -PmonkeySeeds}. */
  static LongStream seeds() {
    String one = System.getProperty("frc.monkeySeed");
    if (one != null) {
      return LongStream.of(Long.parseLong(one));
    }
    return LongStream.rangeClosed(
        1, Long.parseLong(System.getProperty("frc.monkeySeeds", "" + SEEDS)));
  }

  static Stream<Arguments> runs() {
    return ROBOTS.stream().flatMap(robot -> seeds().mapToObj(seed -> Arguments.of(robot, seed)));
  }

  /**
   * A match of the monkey at the controls, things failing as it plays: nothing breaks, and each
   * check is counted, the obvious ones having judged something on the guide's robot.
   */
  @ParameterizedTest(name = "{0}, seed {1}")
  @MethodSource("runs")
  @Timeout(600)
  void aMatchOfMonkeyBreaksNothing(Monkey.Robot robot, long seed) {
    long started = System.nanoTime();

    Monkey.Report report = new Monkey(sim, robot, seed, MATCH, true).run();

    System.out.printf(
        "Monkey on %s, seed %d: %d loops, %d actions, %d faults, in %.1f s%n",
        robot,
        seed,
        report.loops(),
        report.actions(),
        report.faults(),
        (System.nanoTime() - started) / 1e9);
    assertThat(report.faults()).isPositive();
    assertThat(report.judged())
        .containsKeys(
            "numbers only",
            "known before driven",
            "limit switches stop it",
            "moves end",
            "jams stall",
            "dead channels flagged",
            "drive moves end");
    if (robot == ExampleMonkey.ROBOT) {
      assertThat(report.judged())
          .hasEntrySatisfying("numbers only", loops -> assertThat(loops).isPositive())
          .hasEntrySatisfying("moves end", loops -> assertThat(loops).isPositive())
          .hasEntrySatisfying("drive moves end", loops -> assertThat(loops).isPositive());
    }
  }

  /** A seed runs the same every time: the same doings, and every joint where it was. */
  @Test
  void aSeedRunsTheSameEveryTime() {
    Monkey.Report first = new Monkey(sim, ExampleMonkey.ROBOT, 7, 20, true).run();
    Monkey.Report again = new Monkey(sim, ExampleMonkey.ROBOT, 7, 20, true).run();

    assertThat(again).isEqualTo(first);
  }
}
