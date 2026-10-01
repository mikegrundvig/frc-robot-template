package com.michaelgrundvig.frc.pitwall.experiment;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatIllegalStateException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class RigTest {
  private final Lag lag = new Lag();

  /** A rig with everything but what a test leaves out or changes. */
  private Rig.Builder rig() {
    return Rig.builder()
        .name("Arm")
        .gains(lag.config, "gains.kP")
        .setpoint(Radians, radians -> lag.setpoint = radians)
        .limits(Degrees.of(-100), Degrees.of(100))
        .measurement("Angle", Degrees, () -> Math.toDegrees(lag.angle));
  }

  @Test
  void aRigIsBuiltByNameAndReadsItsGainsWhereTheyAreTuned() {
    Rig rig = lag.rig();

    assertThat(rig.name()).isEqualTo("Arm");
    assertThat(rig.config()).isSameAs(lag.config);
    assertThat(rig.gains()).containsExactly("gains.kP", "gains.kD");
    assertThat(rig.gainValues()).isEqualTo(Map.of("gains.kP", 10.0, "gains.kD", 0.0));
    assertThat(rig.limits()).isEqualTo("-100 ° to 100 °");
    assertThat(rig.measurementUnit()).isEqualTo(Degrees);
    assertThat(rig.requirements()).isEmpty();

    lag.config.set(Map.of("gains.kP", 12.5));

    assertThat(rig.gainValues()).containsEntry("gains.kP", 12.5);
  }

  @Test
  void aRunRecordsTheSetpointThenTheMeasurementsThenTheRest() {
    assertThat(lag.rig().traced())
        .containsExactly(
            new Trace.Signal("Setpoint", Degrees),
            new Trace.Signal("Angle", Degrees),
            new Trace.Signal("Voltage", Volts));
  }

  @Test
  void itsGainsMustBeTunableValuesOfTheConfiguration() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Rig.builder().gains(lag.config, "gains.kI"))
        .withMessage("arm: no tunable value at gains.kI");
  }

  @Test
  void everyPartButItsSignalsAndRequirementsIsNeeded() {
    assertThatIllegalStateException()
        .isThrownBy(() -> rig().name(" ").build())
        .withMessage("a rig needs a name");
    assertThatIllegalStateException()
        .isThrownBy(() -> Rig.builder().name("Arm").build())
        .withMessage("the rig Arm needs its gains");
    assertThatIllegalStateException()
        .isThrownBy(() -> Rig.builder().name("Arm").gains(lag.config, "gains.kP").build())
        .withMessage("the rig Arm needs something to measure");
    assertThatIllegalStateException()
        .isThrownBy(
            () ->
                Rig.builder()
                    .name("Arm")
                    .gains(lag.config, "gains.kP")
                    .measurement("Angle", Degrees, () -> 0)
                    .build())
        .withMessage("the rig Arm needs how to command its setpoint");
    assertThatIllegalStateException()
        .isThrownBy(
            () ->
                Rig.builder()
                    .name("Arm")
                    .gains(lag.config, "gains.kP")
                    .setpoint(Radians, radians -> {})
                    .measurement("Angle", Degrees, () -> 0)
                    .build())
        .withMessage("the rig Arm needs its limits");
  }

  @Test
  void itsLimitsMeasurementsAndSetpointAreOneKindOfQuantity() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> rig().limits(Meters.of(0), Meters.of(1)).build())
        .withMessage("Arm's limits aren't the same kind of quantity as its setpoint: 0 m to 1 m");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> rig().limits(Degrees.of(10), Degrees.of(10)).build())
        .withMessage("Arm's limits go nowhere: 10 ° to 10 °");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Rig.builder()
                    .name("Arm")
                    .gains(lag.config, "gains.kP")
                    .setpoint(Radians, radians -> {})
                    .limits(Degrees.of(-100), Degrees.of(100))
                    .measurement("Height", Meters, () -> 0)
                    .build())
        .withMessageContaining("a different kind of quantity from its setpoint");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> rig().measurement("Other angle", Radians, () -> 0).build())
        .withMessage("Arm's measurements share one unit: Other angle isn't in Degree");
  }

  @Test
  void everySignalHasItsOwnName() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> rig().signal("Angle", Volts, () -> 0).build())
        .withMessage("Arm has two signals named Angle (the setpoint is Setpoint)");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> rig().signal("Setpoint", Volts, () -> 0).build())
        .withMessageStartingWith("Arm has two signals named Setpoint");
  }

  @Test
  void aProfileOfAnotherKindOrBeyondTheLimitsCantRunOnIt() {
    Rig rig = lag.rig();

    assertThat(rig.whyNot(Lag.STEP_AND_BACK)).isNull();
    assertThat(rig.whyNot(Profiles.step().from(Meters.of(0)).to(Meters.of(1)).hold(Seconds.of(1))))
        .isEqualTo("its setpoints are in Meter, but Arm takes its setpoint in Radian");
    assertThat(
            rig.whyNot(
                Profiles.stepAndBack().from(Degrees.of(0)).to(Degrees.of(120)).hold(Seconds.of(1))))
        .isEqualTo("120 ° is outside Arm's limits, -100 ° to 100 °");
  }

  @Test
  void aSetpointIsCommandedInTheRigsOwnUnitAndHeldWithinItsLimits() {
    List<Double> sent = new ArrayList<>();
    Rig rig = rig().setpoint(Degrees, sent::add).build();

    rig.command(Math.PI / 2);
    rig.command(rig.clamped(Math.PI));

    assertThat(sent.get(0)).isCloseTo(90, within(1e-9));
    assertThat(sent.get(1)).isCloseTo(100, within(1e-9));
  }

  /** A signal read once a loop needs room only for its loops; a device's readings, for 1 kHz. */
  @Test
  void aRunMakesRoomForEachSignalAtItsOwnRate() {
    Rig rig = lag.rig();
    List<Trace.Signal> traced = rig.traced();

    assertThat(traced)
        .extracting(signal -> rig.capacity(signal, 2.1))
        .as("the setpoint and the voltage once a loop; the angle at 250 Hz")
        .containsExactly(260, 2600, 260);
  }

  @Test
  void aSignalThatThrowsLosesOnlyItsOwnSamples() {
    Rig rig =
        rig()
            .signal(
                "Voltage",
                Volts,
                () -> {
                  throw new IllegalStateException("no reply");
                })
            .signal(
                "Current",
                Volts,
                (now, series) -> {
                  throw new UnsupportedOperationException();
                })
            .build();
    Trace trace = new Trace(rig.traced(), 10);

    String failed = rig.sample(1, trace);

    assertThat(failed).isEqualTo("reading Voltage failed: no reply");
    assertThat(trace.series("Angle").count()).isEqualTo(1);
    assertThat(rig.sample(2, new Trace(rig.traced(), 10))).isEqualTo(failed);
  }
}
