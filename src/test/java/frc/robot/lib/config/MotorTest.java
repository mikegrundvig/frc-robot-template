package frc.robot.lib.config;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.wpilib.math.system.DCMotor;

class MotorTest {
  @ParameterizedTest
  @EnumSource(Motor.class)
  void everyMotorHasAPhysicalModel(Motor motor) {
    DCMotor model = motor.model();

    assertThat(model.nominalVoltage).isPositive();
    assertThat(model.freeSpeed).isPositive();
    assertThat(model.stallTorque).isPositive();
    assertThat(model.stallCurrent).isPositive();
  }

  @Test
  void focTradesTopSpeedForTorque() {
    DCMotor plain = Motor.KRAKEN_X60.model();
    DCMotor foc = Motor.KRAKEN_X60_FOC.model();

    assertThat(foc.stallTorque).isGreaterThan(plain.stallTorque);
    assertThat(foc.freeSpeed).isLessThan(plain.freeSpeed);
  }
}
