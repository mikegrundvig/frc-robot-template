package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.gearing;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatNoException;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

class ActuatorTest {
  /** A gearing that loses nothing changes nothing about the motors' model. */
  @Test
  void motorsThroughALosslessGearingAreTheMotorsModel() {
    DCMotor one = Motor.KRAKEN_X60.model();
    DCMotor model = motors(1, gearing(6.75)).model();

    assertThat(model.stallTorque).isEqualTo(one.stallTorque);
    assertThat(model.stallCurrent).isEqualTo(one.stallCurrent);
    assertThat(model.freeCurrent).isEqualTo(one.freeCurrent);
    assertThat(model.freeSpeed).isEqualTo(one.freeSpeed);
    assertThat(model.Kt).isEqualTo(one.Kt);
    assertThat(model.Kv).isEqualTo(one.Kv);
    assertThat(model.R).isEqualTo(one.R);
  }

  /** Two motors together: twice the torque and the current, at the same speed. */
  @Test
  void twoMotorsPushTwiceAsHard() {
    DCMotor one = Motor.KRAKEN_X60.model();
    DCMotor two = motors(2, gearing(60)).model();

    assertThat(two.stallTorque).isCloseTo(2 * one.stallTorque, within(1e-9));
    assertThat(two.stallCurrent).isCloseTo(2 * one.stallCurrent, within(1e-9));
    assertThat(two.freeSpeed).isEqualTo(one.freeSpeed);
  }

  /** What the gearing loses comes off the torque, not the speed or the current. */
  @Test
  void aGearingsLossesComeOffTheTorque() {
    DCMotor one = Motor.KRAKEN_X60.model();
    Gearing lossy = Gearing.builder().ratio(60).efficiency(0.9).build();
    DCMotor model = motors(1, lossy).model();

    assertThat(model.stallTorque).isCloseTo(0.9 * one.stallTorque, within(1e-9));
    assertThat(model.Kt).isCloseTo(0.9 * one.Kt, within(1e-9));
    assertThat(model.stallCurrent).isEqualTo(one.stallCurrent);
    assertThat(model.freeSpeed).isEqualTo(one.freeSpeed);
  }

  @Test
  void aGearingLosesSomethingButNotEverything() {
    assertThatNoException().isThrownBy(() -> Gearing.builder().ratio(0.5).efficiency(1).build());
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Gearing.builder().ratio(0).efficiency(1).build())
        .withMessageStartingWith("Gearing.ratio");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Gearing.builder().ratio(60).efficiency(0).build())
        .withMessageStartingWith("Gearing.efficiency");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Gearing.builder().ratio(60).efficiency(1.1).build())
        .withMessage("Gearing.efficiency: 1.1 must be at most 1");
  }

  @Test
  void atLeastOneMotorDrivesIt() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> motors(0, gearing(60)))
        .withMessageStartingWith("Motors.count");
  }

  private static Actuator.Motors motors(int count, Gearing gearing) {
    return Actuator.Motors.builder().motor(Motor.KRAKEN_X60).count(count).gearing(gearing).build();
  }
}
