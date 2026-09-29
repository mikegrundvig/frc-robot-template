package frc.robot.lib.config;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.MetersPerSecond;

import org.junit.jupiter.api.Test;

class RequireTest {
  @Test
  void messagesShowNumbersAsPeopleWriteThem() {
    assertThat(Require.shown(MetersPerSecond.of(4.728761))).isEqualTo("4.729 m/s");
    assertThat(Require.shown(Amps.of(10))).isEqualTo("10 A");
    assertThat(Require.shown(0.08)).isEqualTo("0.08");
    assertThat(Require.shown(0)).isEqualTo("0");
    assertThat(Require.shown(1234567)).isEqualTo("1235000");
    assertThat(Require.shown(Double.NaN)).isEqualTo("NaN");
  }
}
