package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatNoException;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

/** What makes a mechanism's force, each kind as a law of its coordinate. */
class ActuatorTest {
  @Test
  void oneMotorIsTheMotorsModel() {
    DCMotor one = Motor.KRAKEN_X60.model();
    DCMotor model = motors(1).model();

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
    DCMotor two = motors(2).model();

    assertThat(two.stallTorque).isCloseTo(2 * one.stallTorque, within(1e-9));
    assertThat(two.stallCurrent).isCloseTo(2 * one.stallCurrent, within(1e-9));
    assertThat(two.freeSpeed).isEqualTo(one.freeSpeed);
  }

  /** What a transmission loses comes off the torque, not the speed or the current. */
  @Test
  void aTransmissionsLossesComeOffTheTorque() {
    DCMotor one = Motor.KRAKEN_X60.model();
    DCMotor model = motors(1).model(0.9);

    assertThat(model.stallTorque).isCloseTo(0.9 * one.stallTorque, within(1e-9));
    assertThat(model.Kt).isCloseTo(0.9 * one.Kt, within(1e-9));
    assertThat(model.stallCurrent).isEqualTo(one.stallCurrent);
    assertThat(model.freeSpeed).isEqualTo(one.freeSpeed);
  }

  /**
   * At full voltage, stalled: the stall torque; spinning free: only what overcomes their own
   * friction, the free current's.
   */
  @Test
  void motorsPushByVoltageLessTheirSpeed() {
    DCMotor one = Motor.KRAKEN_X60.model();
    Actuator.Motors two = motors(2);

    assertThat(two.forceOrTorque(12, 0, 0)).isCloseTo(2 * one.stallTorque, within(1e-9));
    assertThat(two.forceOrTorque(12, 0, one.freeSpeed))
        .isCloseTo(2 * one.Kt * one.freeCurrent, within(1e-9));
    assertThat(two.forceOrTorque(6, 0, 0)).isCloseTo(one.stallTorque, within(1e-9));
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
        .isThrownBy(() -> motors(0))
        .withMessageStartingWith("Motors.count");
  }

  private static Actuator.Motors motors(int count) {
    return Actuator.Motors.builder().motor(Motor.KRAKEN_X60).count(count).build();
  }

  /**
   * A WCP-0408 linear servo, 168 mm eye to eye closed, 100 mm of stroke, 50 N and 32 mm/s: told a
   * share of its travel, it pushes toward it as hard as it can until a fiftieth of its travel away,
   * and less as it nears its top speed. What to tell it for a length is that length's share.
   */
  @Test
  void aServoPushesTowardWhereItsTold() {
    Actuator.Servo servo =
        Actuator.Servo.builder().from(0.168).to(0.268).force(50).speed(0.032).build();

    assertThat(servo.command(0.168)).isZero();
    assertThat(servo.command(0.233)).isCloseTo(0.65, within(1e-12));
    assertThat(servo.forceOrTorque(1, 0.2, 0)).as("far short: all it has").isEqualTo(50);
    assertThat(servo.forceOrTorque(0, 0.2, 0)).isEqualTo(-50);
    assertThat(servo.forceOrTorque(0.5, 0.218, 0)).as("there: nothing").isCloseTo(0, within(1e-9));
    assertThat(servo.forceOrTorque(0.5, 0.217, 0))
        .as("a hundredth of its travel short")
        .isCloseTo(25, within(1e-9));
    assertThat(servo.forceOrTorque(1, 0.2, 0.016))
        .as("at half its top speed")
        .isCloseTo(25, within(1e-9));
    assertThat(servo.forceOrTorque(2, 0.268, 0))
        .as("told past its travel, held to it")
        .isCloseTo(0, within(1e-9));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Actuator.Servo.builder().from(0.2).to(0.2).force(50).speed(0.032).build())
        .withMessageContaining("must differ from where it starts");
  }
}
