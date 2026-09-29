package frc.robot.lib.config;

import org.wpilib.math.system.DCMotor;

/**
 * The motors FRC robots commonly use. {@code _FOC}: driven with field-oriented control, which CTRE
 * sells as part of Phoenix Pro. It draws more current at the same voltage, for more torque and
 * power at a slightly lower top speed; torque per amp is unchanged, so it adds nothing when a
 * current limit is what caps the motor.
 */
public enum Motor {
  KRAKEN_X60,
  KRAKEN_X60_FOC,
  KRAKEN_X44,
  KRAKEN_X44_FOC,
  FALCON_500,
  FALCON_500_FOC,
  MINION,
  NEO,
  NEO_VORTEX,
  NEO_550;

  /** One motor's physical model: its torque, speed, and current at the nominal voltage. */
  public DCMotor model() {
    return switch (this) {
      case KRAKEN_X60 -> DCMotor.getKrakenX60(1);
      case KRAKEN_X60_FOC -> DCMotor.getKrakenX60Foc(1);
      case KRAKEN_X44 -> DCMotor.getKrakenX44(1);
      case KRAKEN_X44_FOC -> DCMotor.getKrakenX44Foc(1);
      case FALCON_500 -> DCMotor.getFalcon500(1);
      case FALCON_500_FOC -> DCMotor.getFalcon500Foc(1);
      case MINION -> DCMotor.getMinion(1);
      case NEO -> DCMotor.getNEO(1);
      case NEO_VORTEX -> DCMotor.getNeoVortex(1);
      case NEO_550 -> DCMotor.getNeo550(1);
    };
  }
}
