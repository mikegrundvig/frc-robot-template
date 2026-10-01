package com.michaelgrundvig.frc.pitwall.experiment;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Volts;

import java.util.List;
import org.junit.jupiter.api.Test;

class TraceTest {
  private static final Trace.Signal ANGLE = new Trace.Signal("Angle", Radians);
  private static final Trace.Signal VOLTAGE = new Trace.Signal("Voltage", Volts);

  @Test
  void eachSignalKeepsItsOwnSamples() {
    var trace = new Trace(List.of(ANGLE, VOLTAGE), 100);

    for (int i = 0; i < 5; i++) {
      trace.series("Angle").add(i * 4_000_000L, i * 0.1); // 250 Hz
    }
    trace.series("Voltage").add(0, 6.0); // each loop

    assertThat(trace.signals()).containsExactly(ANGLE, VOLTAGE);
    assertThat(trace.series("Angle").count()).isEqualTo(5);
    assertThat(trace.series("Voltage").count()).isEqualTo(1);
  }

  @Test
  void saysWhenAnySignalRanOutOfRoom() {
    var trace = new Trace(List.of(ANGLE, VOLTAGE), 1);
    trace.series("Voltage").add(0, 6.0);

    assertThat(trace.full()).isFalse();

    trace.series("Voltage").add(1, 6.0);

    assertThat(trace.full()).isTrue();
  }

  @Test
  void anUnknownSignalIsNamedWithTheOnesThereAre() {
    var trace = new Trace(List.of(ANGLE, VOLTAGE), 10);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> trace.series("Current"))
        .withMessageContaining("Current")
        .withMessageContaining("[Angle, Voltage]");
  }

  @Test
  void refusesNoSignalsTwoOfOneNameAndBlankNames() {
    assertThatIllegalArgumentException().isThrownBy(() -> new Trace(List.of(), 10));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new Trace(List.of(ANGLE, new Trace.Signal("Angle", Volts)), 10))
        .withMessageContaining("two signals are named Angle");
    assertThatIllegalArgumentException().isThrownBy(() -> new Trace.Signal(" ", Volts));
  }
}
