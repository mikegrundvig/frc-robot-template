package com.michaelgrundvig.frc.simulation;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.Set;
import org.junit.jupiter.api.Test;

/**
 * Each of a world's random streams starts where {@link Seeds} says, as recorded runs had them. The
 * truth suite holds them to being apart (the robot program's {@code SeedsTruthSimTest}, beside its
 * cameras').
 */
class SeedsTest {
  @Test
  void eachStreamStartsAsRecordedRunsHadIt() {
    assertThat(Seeds.gyro(7)).isEqualTo(7);
    assertThat(Seeds.gyroAccelerometer(7)).isEqualTo(8);
    assertThat(Seeds.backupGyro(7)).isEqualTo(7 + (1L << 32));
    assertThat(Seeds.backupAccelerometer(7)).isEqualTo(8 + (1L << 32));
    assertThat(Seeds.upAxis(8)).isEqualTo(~8L);
  }

  @Test
  void oneWorldsStreamsAreApart() {
    long seed = 3;
    assertThat(
            Set.of(
                Seeds.gyro(seed),
                Seeds.gyroAccelerometer(seed),
                Seeds.upAxis(Seeds.gyroAccelerometer(seed)),
                Seeds.backupGyro(seed),
                Seeds.backupAccelerometer(seed),
                Seeds.upAxis(Seeds.backupAccelerometer(seed))))
        .hasSize(6);
  }
}
