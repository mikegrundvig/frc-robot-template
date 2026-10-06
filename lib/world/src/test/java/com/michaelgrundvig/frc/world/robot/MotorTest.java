package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.NewtonMeters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Volts;

import java.util.List;
import java.util.function.IntFunction;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.system.DCMotor;

class MotorTest {
  static List<Motor> presets() {
    return Motor.presets();
  }

  @ParameterizedTest
  @MethodSource("presets")
  void everyMotorHasAPhysicalModel(Motor motor) {
    DCMotor model = motor.model();

    assertThat(model.nominalVoltage).isPositive();
    assertThat(model.freeSpeed).isPositive();
    assertThat(model.stallTorque).isPositive();
    assertThat(model.stallCurrent).isPositive();
  }

  /** WPILib's model of each motor, which the presets take their numbers from. */
  static Stream<Arguments> wpilibsModels() {
    return Stream.of(
        Arguments.of(Motor.KRAKEN_X60, (IntFunction<DCMotor>) DCMotor::getKrakenX60),
        Arguments.of(Motor.KRAKEN_X60_FOC, (IntFunction<DCMotor>) DCMotor::getKrakenX60Foc),
        Arguments.of(Motor.KRAKEN_X44, (IntFunction<DCMotor>) DCMotor::getKrakenX44),
        Arguments.of(Motor.KRAKEN_X44_FOC, (IntFunction<DCMotor>) DCMotor::getKrakenX44Foc),
        Arguments.of(Motor.FALCON_500, (IntFunction<DCMotor>) DCMotor::getFalcon500),
        Arguments.of(Motor.FALCON_500_FOC, (IntFunction<DCMotor>) DCMotor::getFalcon500Foc),
        Arguments.of(Motor.MINION, (IntFunction<DCMotor>) DCMotor::getMinion),
        Arguments.of(Motor.NEO, (IntFunction<DCMotor>) DCMotor::getNEO),
        Arguments.of(Motor.NEO_VORTEX, (IntFunction<DCMotor>) DCMotor::getNeoVortex),
        Arguments.of(Motor.NEO_550, (IntFunction<DCMotor>) DCMotor::getNeo550));
  }

  /**
   * A preset's model is WPILib's to the last bit, so nothing that simulates a motor moved when the
   * motors became records.
   */
  @ParameterizedTest
  @MethodSource("wpilibsModels")
  void aPresetsModelIsWpilibsExactly(Motor motor, IntFunction<DCMotor> wpilib) {
    DCMotor ours = motor.model();
    DCMotor theirs = wpilib.apply(1);

    assertThat(ours.nominalVoltage).isEqualTo(theirs.nominalVoltage);
    assertThat(ours.stallTorque).isEqualTo(theirs.stallTorque);
    assertThat(ours.stallCurrent).isEqualTo(theirs.stallCurrent);
    assertThat(ours.freeCurrent).isEqualTo(theirs.freeCurrent);
    assertThat(ours.freeSpeed).isEqualTo(theirs.freeSpeed);
    assertThat(ours.R).isEqualTo(theirs.R);
    assertThat(ours.Kv).isEqualTo(theirs.Kv);
    assertThat(ours.Kt).isEqualTo(theirs.Kt);
  }

  @Test
  void everyPresetIsListedOnceUnderItsOwnName() {
    assertThat(Motor.presets()).hasSize(10).doesNotHaveDuplicates();
    assertThat(Motor.presets()).extracting(Motor::name).doesNotHaveDuplicates();
  }

  @Test
  void focTradesTopSpeedForTorque() {
    DCMotor plain = Motor.KRAKEN_X60.model();
    DCMotor foc = Motor.KRAKEN_X60_FOC.model();

    assertThat(foc.stallTorque).isGreaterThan(plain.stallTorque);
    assertThat(foc.freeSpeed).isLessThan(plain.freeSpeed);
  }

  /** Any DC motor, from the numbers on its datasheet. */
  @Test
  void anyMotorFromItsDatasheet() {
    DCMotor model = datasheet().model();

    assertThat(model.nominalVoltage).isEqualTo(12);
    assertThat(model.stallTorque).isEqualTo(1.5);
    assertThat(model.stallCurrent).isEqualTo(100);
    assertThat(model.freeCurrent).isEqualTo(2);
    assertThat(model.freeSpeed).isCloseTo(10000 * 2 * Math.PI / 60, within(1e-9));
  }

  @Test
  void aMotorsNumbersMustBeAMotors() {
    assertRejected("Motor[ ].name", () -> change().name(" ").build());
    assertRejected(
        "Motor[datasheet].nominalVoltage", () -> change().nominalVoltage(Volts.of(0)).build());
    assertRejected(
        "Motor[datasheet].stallTorque", () -> change().stallTorque(NewtonMeters.of(0)).build());
    assertRejected(
        "Motor[datasheet].stallCurrent", () -> change().stallCurrent(Amps.of(-1)).build());
    assertRejected("Motor[datasheet].freeCurrent", () -> change().freeCurrent(Amps.of(-1)).build());
    assertRejected("Motor[datasheet].freeSpeed", () -> change().freeSpeed(RPM.of(0)).build());
  }

  /** Free and stall current the other way round were each written in the other's place. */
  @Test
  void aMotorDrawsLessRunningFreeThanHeldStill() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> change().stallCurrent(Amps.of(2)).freeCurrent(Amps.of(100)).build())
        .withMessage(
            "Motor[datasheet].freeCurrent: 100 A must be less than the stall current (2 A)");
  }

  private static Motor datasheet() {
    return Motor.fromDatasheet()
        .name("datasheet")
        .nominalVoltage(Volts.of(12))
        .stallTorque(NewtonMeters.of(1.5))
        .stallCurrent(Amps.of(100))
        .freeCurrent(Amps.of(2))
        .freeSpeed(RPM.of(10000))
        .rotorInertia(KilogramSquareMeters.of(1e-5))
        .build();
  }

  private static MotorBuilder change() {
    return MotorBuilder.builder(datasheet());
  }

  private static void assertRejected(String field, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageStartingWith(field);
  }
}
