package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;

/** Where a switch or a rangefinder is connected, and what the wiring refuses. */
class SensorWiringTest {
  private static final CanBus BUS = CanBus.SYSTEMCORE;

  @Test
  void aSwitchIsOnAnInputThatReadsIt() {
    SensorWiring.OfSwitch beam =
        SensorWiring.OfSwitch.builder()
            .sensor("claw beam")
            .input(new SensorWiring.Candi(BUS, 30, 1))
            .polarity(SensorWiring.Polarity.NORMALLY_OPEN)
            .build();
    SensorWiring.OfSwitch limit =
        SensorWiring.OfSwitch.builder()
            .sensor("pivot down")
            .input(new SensorWiring.ControllerLimit("pivot", SensorWiring.Direction.REVERSE))
            .polarity(SensorWiring.Polarity.NORMALLY_CLOSED)
            .build();

    assertThat(beam.input()).isEqualTo(new SensorWiring.Candi(BUS, 30, 1));
    assertThat(limit.sensor()).isEqualTo("pivot down");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                SensorWiring.OfSwitch.builder()
                    .sensor("claw beam")
                    .input(new SensorWiring.CanRange(BUS, 31))
                    .polarity(SensorWiring.Polarity.NORMALLY_OPEN)
                    .build())
        .withMessageContaining("a switch isn't on a CANrange");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                SensorWiring.OfSwitch.builder()
                    .sensor(" ")
                    .input(new SensorWiring.Port(0))
                    .polarity(SensorWiring.Polarity.NORMALLY_OPEN)
                    .build());
  }

  @Test
  void aRangefinderIsOnACanRange() {
    SensorWiring.OfRangefinder hopper =
        SensorWiring.OfRangefinder.builder()
            .sensor("hopper")
            .input(new SensorWiring.CanRange(BUS, 31))
            .build();

    assertThat(hopper.input()).isEqualTo(new SensorWiring.CanRange(BUS, 31));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                SensorWiring.OfRangefinder.builder()
                    .sensor("hopper")
                    .input(new SensorWiring.Port(2))
                    .build())
        .withMessageContaining("a rangefinder is on a CANrange");
  }

  @Test
  void inputsAreWhereTheHardwareHasThem() {
    assertThatIllegalArgumentException().isThrownBy(() -> new SensorWiring.Port(6));
    assertThatIllegalArgumentException().isThrownBy(() -> new SensorWiring.Candi(BUS, 63, 1));
    assertThatIllegalArgumentException().isThrownBy(() -> new SensorWiring.Candi(BUS, 30, 3));
    assertThatIllegalArgumentException().isThrownBy(() -> new SensorWiring.CanRange(BUS, -1));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new SensorWiring.ControllerLimit("", SensorWiring.Direction.FORWARD));
  }

  /**
   * A limit input on a joint's controller is named by the joint itself, and a rangefinder written
   * in one call.
   */
  @Test
  void aJointsPinAndARangefinderAreWrittenByWhatTheyAre() {
    Joint.Revolute roller = Joint.roller("roller", Part.CHASSIS, new Translation3d());
    Sensor.Rangefinder hopper =
        Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27))
            .at(Pose3d.ZERO)
            .on(new SensorWiring.CanRange(BUS, 31));

    assertThat(SensorWiring.ControllerLimit.of(roller, SensorWiring.Direction.FORWARD))
        .isEqualTo(new SensorWiring.ControllerLimit("roller", SensorWiring.Direction.FORWARD));
    assertThat(hopper.pose()).isPresent();
    assertThat(hopper.wiring()).contains(new SensorWiring.CanRange(BUS, 31));
  }
}
