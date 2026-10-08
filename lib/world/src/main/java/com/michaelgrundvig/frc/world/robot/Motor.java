package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.NewtonMeters;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.List;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Torque;
import org.wpilib.units.measure.Voltage;

/**
 * A DC motor, as its datasheet gives it: what it does at its nominal voltage, stalled and running
 * free, and how hard its rotor is to spin. The first five numbers are WPILib's model of a motor
 * ({@link DCMotor}), so any DC motor can be described from its datasheet, not only the motors named
 * here.
 *
 * <p>The motors FRC robots commonly use are here by name, with WPILib's numbers for each ({@link
 * #presets}). {@code _FOC}: driven with field-oriented control, which CTRE sells as part of Phoenix
 * Pro. It draws more current at the same voltage, for more torque and power at a slightly lower top
 * speed; torque per amp is unchanged, so it adds nothing when a current limit is what caps the
 * motor. Any other motor is described from its datasheet ({@link #fromDatasheet}).
 *
 * @param name what the motor is called, for messages
 * @param nominalVoltage the voltage the datasheet's numbers were measured at: 12 V for FRC motors
 * @param stallTorque the torque at the shaft with the nominal voltage applied and the shaft held
 *     still
 * @param stallCurrent the current it draws then
 * @param freeCurrent the current it draws spinning freely, with nothing on its shaft
 * @param freeSpeed how fast it spins freely, with nothing on its shaft
 * @param rotorInertia how hard its rotor is to spin, about its shaft. Through a gearing it counts
 *     the ratio squared times over: a 0.00001 kg m² rotor through 60:1 is 0.036 kg m² at the arm,
 *     often more than the arm itself. No maker publishes it for FRC's motors, so the presets' are
 *     estimates (open question M21: measure one with SysId on a motor with nothing on its shaft).
 */
@ConfigRecord
public record Motor(
    String name,
    Voltage nominalVoltage,
    Torque stallTorque,
    Current stallCurrent,
    Current freeCurrent,
    AngularVelocity freeSpeed,
    MomentOfInertia rotorInertia) {
  /**
   * An FRC battery's nominal voltage, 12 V: what the presets' numbers are measured at, the most a
   * motor is asked for, what a SPARK's loop output is a share of (its voltage compensation), and
   * what the simulator's battery holds steady.
   */
  public static final double NOMINAL_VOLTS = 12;

  // Rotor inertias are estimates, the same for a motor with and without FOC (one rotor): no maker
  // publishes them (M21). The Kraken X60's is the one the template's drive inertia already assumed
  // (0.0006 kg m² through 6.75:1); the rest scale it by each motor's size.
  public static final Motor KRAKEN_X60 = preset("Kraken X60", DCMotor.getKrakenX60(1), 1.3e-5);
  public static final Motor KRAKEN_X60_FOC =
      preset("Kraken X60 FOC", DCMotor.getKrakenX60Foc(1), 1.3e-5);
  public static final Motor KRAKEN_X44 = preset("Kraken X44", DCMotor.getKrakenX44(1), 0.6e-5);
  public static final Motor KRAKEN_X44_FOC =
      preset("Kraken X44 FOC", DCMotor.getKrakenX44Foc(1), 0.6e-5);
  public static final Motor FALCON_500 = preset("Falcon 500", DCMotor.getFalcon500(1), 1.3e-5);
  public static final Motor FALCON_500_FOC =
      preset("Falcon 500 FOC", DCMotor.getFalcon500Foc(1), 1.3e-5);
  public static final Motor MINION = preset("Minion", DCMotor.getMinion(1), 0.8e-5);
  public static final Motor NEO = preset("NEO", DCMotor.getNEO(1), 1.3e-5);
  public static final Motor NEO_VORTEX = preset("NEO Vortex", DCMotor.getNeoVortex(1), 1.0e-5);
  public static final Motor NEO_550 = preset("NEO 550", DCMotor.getNeo550(1), 0.3e-5);

  private static final List<Motor> PRESETS =
      List.of(
          KRAKEN_X60,
          KRAKEN_X60_FOC,
          KRAKEN_X44,
          KRAKEN_X44_FOC,
          FALCON_500,
          FALCON_500_FOC,
          MINION,
          NEO,
          NEO_VORTEX,
          NEO_550);

  public Motor {
    Checks.on(Motor.class, name)
        .field("name", name, notBlank())
        .field("nominalVoltage", nominalVoltage, positive())
        .field("stallTorque", stallTorque, positive())
        .field("stallCurrent", stallCurrent, positive())
        .field("freeCurrent", freeCurrent, nonNegative())
        .field("freeSpeed", freeSpeed, positive())
        .field("rotorInertia", rotorInertia, positive())
        // A motor running free draws a little; held still, the most it ever will. Two currents the
        // other way round were each written in the other's place.
        .check(
            "freeCurrent",
            () ->
                freeCurrent.lt(stallCurrent)
                    ? null
                    : shown(freeCurrent)
                        + " must be less than the stall current ("
                        + shown(stallCurrent)
                        + ")")
        .orThrow();
  }

  /**
   * Describes any DC motor from its datasheet, one named value at a time: {@code
   * Motor.fromDatasheet().name("...").nominalVoltage(Volts.of(12))...build()}. Datasheets often
   * give the free speed in rpm: {@code RPM.of(6000)}.
   */
  public static MotorBuilder.NameStage fromDatasheet() {
    return MotorBuilder.builder();
  }

  /** The motors named here, with WPILib's numbers for each. */
  public static List<Motor> presets() {
    return PRESETS;
  }

  /** One motor's physical model: its torque, speed, and current at the nominal voltage. */
  public DCMotor model() {
    return new DCMotor(
        nominalVoltage.in(Volts),
        stallTorque.in(NewtonMeters),
        stallCurrent.in(Amps),
        freeCurrent.in(Amps),
        freeSpeed.in(RadiansPerSecond),
        1);
  }

  /**
   * A motor WPILib models, under the name it's sold by, with an estimate of its rotor's inertia.
   */
  private static Motor preset(String name, DCMotor model, double rotorKilogramSquareMeters) {
    return fromDatasheet()
        .name(name)
        .nominalVoltage(Volts.of(model.nominalVoltage))
        .stallTorque(NewtonMeters.of(model.stallTorque))
        .stallCurrent(Amps.of(model.stallCurrent))
        .freeCurrent(Amps.of(model.freeCurrent))
        .freeSpeed(RadiansPerSecond.of(model.freeSpeed))
        .rotorInertia(KilogramSquareMeters.of(rotorKilogramSquareMeters))
        .build();
  }
}
