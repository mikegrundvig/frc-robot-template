package frc.robot.subsystems.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

/** Two boards taking turns at one address, told from one board replaced by another. */
class MacWatchTest {
  private static final long SECOND = 1_000_000_000L;
  private final MacWatch watch = new MacWatch(60);

  @Test
  void oneBoardIsJustOne() {
    for (int i = 0; i < 10; i++) {
      watch.heard(i * SECOND, "aa");
    }

    assertThat(watch.takingTurns()).isEmpty();
  }

  @Test
  void aReplacementIsntTwoBoards() {
    for (int i = 0; i < 5; i++) {
      watch.heard(i * SECOND, "aa");
    }
    for (int i = 5; i < 10; i++) {
      watch.heard(i * SECOND, "bb");
    }

    assertThat(watch.takingTurns()).isEmpty();
  }

  @Test
  void twoBoardsTakeTurns() {
    watch.heard(0, "aa");
    watch.heard(SECOND, "bb");
    assertThat(watch.takingTurns()).isEmpty();

    watch.heard(2 * SECOND, "aa");

    assertThat(watch.takingTurns()).containsExactly("aa", "bb");
  }

  @Test
  void turnsOutsideTheWindowAreForgotten() {
    watch.heard(0, "aa");
    watch.heard(SECOND, "bb");
    watch.heard(2 * SECOND, "aa");

    watch.heard(70 * SECOND, "aa");

    assertThat(watch.takingTurns()).isEmpty();
  }

  @Test
  void anAnswerWithoutAMacCountsForNothing() {
    watch.heard(0, "aa");
    watch.heard(SECOND, "");
    watch.heard(2 * SECOND, "aa");

    assertThat(watch.takingTurns()).isEmpty();
  }
}
