package frc.robot.lib.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Crossing raised terrain, from synthetic IMU traces at 50 Hz. */
class TerrainCrossingTest {
  private static final double DEG = Math.toRadians(1);
  private static final long LOOP = 20_000_000L;

  private final TerrainCrossing crossing = new TerrainCrossing(10 * DEG, 5 * DEG, 0.1);
  private final List<TerrainCrossing.Phase> phases = new ArrayList<>();
  private long time;
  private int completed;

  /** Loops with the ground this steep along the way the robot moves (the tilt is its size). */
  private TerrainCrossingTest ride(double seconds, double climbDegrees) {
    for (int i = 0; i < Math.round(seconds / 0.02); i++) {
      time += LOOP;
      if (crossing.update(time, climbDegrees * DEG, Math.abs(climbDegrees) * DEG)) {
        completed++;
      }
      if (phases.isEmpty() || phases.get(phases.size() - 1) != crossing.phase()) {
        phases.add(crossing.phase());
      }
    }
    return this;
  }

  @Test
  void theSlopeIsAlongTheWayTheRobotMoves() {
    double noseUp = -12 * DEG; // WPILib: positive pitch is nose down
    double leftUp = 12 * DEG; // positive roll lifts the left side

    assertThat(TerrainCrossing.climb(noseUp, 0, 0)).isCloseTo(12 * DEG, within(1e-12));
    assertThat(TerrainCrossing.climb(noseUp, 0, Math.PI)).isCloseTo(-12 * DEG, within(1e-12));
    assertThat(TerrainCrossing.climb(0, leftUp, Math.PI / 2)).isCloseTo(12 * DEG, within(1e-12));
    assertThat(TerrainCrossing.climb(noseUp, 0, Math.PI / 2)).isCloseTo(0, within(1e-12));
  }

  @Test
  void upOverDownAndLevelIsACrossing() {
    ride(0.2, 0).ride(0.3, 15).ride(0.3, -15).ride(0.3, 0);

    assertThat(completed).isEqualTo(1);
    assertThat(crossing.crossings()).isEqualTo(1);
    assertThat(phases)
        .containsExactly(
            TerrainCrossing.Phase.LEVEL,
            TerrainCrossing.Phase.CLIMBING,
            TerrainCrossing.Phase.DESCENDING,
            TerrainCrossing.Phase.LEVEL);
  }

  @Test
  void itCountsOnlyOnceTheRobotHasStayedLevel() {
    ride(0.3, 15).ride(0.3, -15).ride(0.06, 0);

    assertThat(crossing.crossings()).as("level for less than 0.1 s").isZero();
    ride(0.1, 0);
    assertThat(crossing.crossings()).isEqualTo(1);
  }

  @Test
  void backingOffTheRiseIsNoCrossing() {
    ride(0.3, 15).ride(0.3, 0);

    assertThat(crossing.crossings()).isZero();
    assertThat(crossing.phase()).isEqualTo(TerrainCrossing.Phase.LEVEL);
  }

  @Test
  void comingDownOffAPlatformIsNoCrossing() {
    ride(0.3, -15).ride(0.3, 0);

    assertThat(crossing.crossings()).isZero();
  }

  @Test
  void twoRisesBeforeLevelCountAsOneCrossing() {
    ride(0.2, 15).ride(0.2, -15).ride(0.2, 15).ride(0.2, -15).ride(0.3, 0);

    assertThat(crossing.crossings()).isEqualTo(1);
  }

  @Test
  void steepMustBeSteeperThanLevel() {
    assertThatThrownBy(() -> new TerrainCrossing(5 * DEG, 5 * DEG, 0.1))
        .isInstanceOf(IllegalArgumentException.class);
  }
}
