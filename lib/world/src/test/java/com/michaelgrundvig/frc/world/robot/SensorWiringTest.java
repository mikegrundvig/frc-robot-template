package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANRANGE;
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

  /**
   * A switch carries where it's connected, an input a switch can be on, and a rangefinder its
   * CANrange: a switch can't be given a CANrange, nor a rangefinder a switch's input, as its types
   * say.
   */
  @Test
  void aSensorCarriesTheInputItsOn() {
    Sensor.Switch beam =
        Sensor.Switch.beam("claw beam", Part.CHASSIS).on(CANDI.can(BUS, 30).s1().normallyOpen());
    Sensor.Rangefinder hopper =
        Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27))
            .on(CANRANGE.can(BUS, 31));

    assertThat(beam.wiring())
        .contains(
            new SensorWiring.Connection(
                CANDI.can(BUS, 30).s1(), SensorWiring.Polarity.NORMALLY_OPEN));
    assertThat(hopper.wiring()).contains(CANRANGE.can(BUS, 31));
  }

  @Test
  void inputsAreWhereTheHardwareHasThem() {
    assertThatIllegalArgumentException().isThrownBy(() -> SensorWiring.port(6));
    assertThatIllegalArgumentException().isThrownBy(() -> CANDI.can(BUS, 63).s1());
    assertThatIllegalArgumentException().isThrownBy(() -> new SensorWiring.Candi(BUS, 30, 3));
    assertThatIllegalArgumentException().isThrownBy(() -> CANRANGE.can(BUS, -1));
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
            .on(CANRANGE.can(BUS, 31));

    assertThat(SensorWiring.ControllerLimit.of(roller, SensorWiring.Direction.FORWARD))
        .isEqualTo(new SensorWiring.ControllerLimit("roller", SensorWiring.Direction.FORWARD));
    assertThat(hopper.pose()).isPresent();
    assertThat(hopper.wiring()).contains(CANRANGE.can(BUS, 31));
  }

  /** An input is named as a motor's controller is: a CANdi's or a CANrange's by bus and ID. */
  @Test
  void anInputIsNamedAsAMotorsControllerIs() {
    CanBus canivore = CanBus.canivore("canivore");

    assertThat(CANDI.can(canivore, 30).s1())
        .isEqualTo(SensorWiringCandiBuilder.builder().bus(canivore).id(30).input(1).build());
    assertThat(CANDI.can(canivore, 30).s2().input()).isEqualTo(2);
    assertThat(CANDI.can(30).s1().bus()).isEqualTo(CanBus.SYSTEMCORE);
    assertThat(CANRANGE.can(canivore, 31))
        .isEqualTo(SensorWiringCanRangeBuilder.builder().bus(canivore).id(31).build());
    assertThat(CANRANGE.can(31).bus()).isEqualTo(CanBus.SYSTEMCORE);
    assertThat(SensorWiring.port(1).port()).isEqualTo(1);
  }

  /**
   * A switch's polarity is said after its input; a joint's own limit switch's after its pin, which
   * its block puts on the joint's leading controller.
   */
  @Test
  void aSwitchsPolarityFollowsItsInput() {
    Joint.Revolute roller = Joint.roller("roller", Part.CHASSIS, new Translation3d());

    assertThat(CANDI.can(BUS, 30).s1().normallyOpen())
        .isEqualTo(
            new SensorWiring.Connection(
                CANDI.can(BUS, 30).s1(), SensorWiring.Polarity.NORMALLY_OPEN));
    assertThat(SensorWiring.port(1).normallyClosed().polarity())
        .isEqualTo(SensorWiring.Polarity.NORMALLY_CLOSED);
    assertThat(SensorWiring.Direction.REVERSE.normallyClosed())
        .isEqualTo(
            new SensorWiring.Pin(
                SensorWiring.Direction.REVERSE, SensorWiring.Polarity.NORMALLY_CLOSED));
    assertThat(SensorWiring.Direction.FORWARD.normallyOpen().of(roller))
        .isEqualTo(
            SensorWiring.ControllerLimit.of(roller, SensorWiring.Direction.FORWARD).normallyOpen());
  }
}
