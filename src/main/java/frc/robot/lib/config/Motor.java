package frc.robot.lib.config;

import org.wpilib.math.system.DCMotor;

/**
 * The motors FRC robots commonly use. {@code _FOC} is CTRE's field-oriented commutation, which
 * needs a Phoenix Pro license and gives more torque at a slightly lower top speed.
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
