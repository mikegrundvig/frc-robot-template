package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.Optional;
import java.util.Set;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Current;

/**
 * How a joint's devices are wired, as its block names them: each motor's controller ({@link
 * Device}: {@code TALON_FX.can(32).clockwise().pdh(4, Amps.of(40))}), and its absolute encoder's
 * CANcoder ({@link #CANCODER}), its SPARK's data port ({@link #DATA_PORT}) or a Systemcore port
 * ({@link PortEncoder}). What the mechanism is (its gearing, its range, where the encoder sits in
 * its gear train) is the definition's; this is only how it's connected, and differs robot to robot
 * even with identical mechanisms. The block holds it ({@link Driven}), and the program reads it
 * there.
 *
 * <p>Directions are against the joint's positive direction in the definition, never a motor's own
 * idea of forward: each motor says which way it turns as the joint moves forward, and the encoder
 * which way it counts. Phoenix's ratios are then plain magnitudes, and its followers' alignment
 * follows from the motors' directions ({@code PhoenixMechanism}).
 */
public final class MechanismWiring {
  private MechanismWiring() {}

  /**
   * A CANcoder at its bus and ID, named in a joint's block as a motor's controller is, with the
   * offset it's measured to need: {@code CANCODER.can(CANIVORE, 34).offset(Rotations.of(0.82))}. It
   * counts counterclockwise, seen facing its LED, as the joint moves forward, until {@code
   * .clockwise()} after its offset says otherwise, as a data port's or a Systemcore port's {@code
   * .inverted()} does. It's read once at startup, to tell its joint's TalonFX where the joint is
   * ({@link Feedback#ROTOR}), until {@code .remote()}, {@code .fused()} or {@code .sync()} after
   * its offset has the TalonFX's loop run on it.
   */
  public static final CanDeviceKind<CancoderAt> CANCODER = new CanDeviceKind<>(CancoderAt::new);

  /**
   * An absolute encoder on the leading SPARK's data port, as REV's Through Bore Encoder plugs in,
   * with the offset it's measured to need: {@code DATA_PORT.offset(Rotations.of(0.82))}. Then
   * {@code .inverted()} for one whose reading falls as the joint moves forward, and {@code
   * .closesLoop()} for the SPARK's loop to run on it.
   */
  public static final DataPort DATA_PORT = new DataPort();

  /**
   * The kind of motor controller a mechanism's motors are on, and the motors each can drive, as its
   * maker says.
   */
  public enum Controller {
    /** CTRE's TalonFX, built into a Kraken or a Falcon. */
    TALON_FX(
        "a TalonFX",
        Set.of(
            Motor.KRAKEN_X60,
            Motor.KRAKEN_X60_FOC,
            Motor.KRAKEN_X44,
            Motor.KRAKEN_X44_FOC,
            Motor.FALCON_500,
            Motor.FALCON_500_FOC)),
    /**
     * CTRE's TalonFXS: a Minion, or REV's motors by their JST connectors (Phoenix's {@code
     * MotorArrangementValue}).
     */
    TALON_FXS("a TalonFXS", Set.of(Motor.MINION, Motor.NEO, Motor.NEO_550, Motor.NEO_VORTEX)),
    /** REV's SPARK MAX: a NEO or a NEO 550, or a NEO Vortex on its Solo Adapter. */
    SPARK_MAX("a SPARK MAX", Set.of(Motor.NEO, Motor.NEO_550, Motor.NEO_VORTEX)),
    /** REV's SPARK Flex: a NEO Vortex docked to it, or a NEO or NEO 550 through its adapter. */
    SPARK_FLEX("a SPARK Flex", Set.of(Motor.NEO_VORTEX, Motor.NEO, Motor.NEO_550));

    private final String shown;
    private final Set<Motor> drives;

    Controller(String shown, Set<Motor> drives) {
      this.shown = shown;
      this.drives = drives;
    }

    /** Whether it drives a motor of the definition's catalog. */
    public boolean drives(Motor motor) {
      return drives.contains(motor);
    }

    /**
     * The controller a motor runs on where its block wires none, which the simulator runs it as and
     * its settings are checked on: a NEO's or a NEO 550's a SPARK MAX, a NEO Vortex's a SPARK Flex,
     * a Minion's a TalonFXS, and a Kraken's, a Falcon's or a motor described from its datasheet a
     * TalonFX.
     */
    public static Controller standIn(Motor motor) {
      if (motor.equals(Motor.NEO_VORTEX)) {
        return SPARK_FLEX;
      }
      if (SPARK_MAX.drives(motor)) {
        return SPARK_MAX;
      }
      if (TALON_FXS.drives(motor)) {
        return TALON_FXS;
      }
      return TALON_FX;
    }

    /** As a sentence names it: "a SPARK MAX". */
    public String shown() {
      return shown;
    }

    /**
     * This kind of controller on Systemcore's first CAN port, at an ID: {@code TALON_FX.can(32)},
     * its motor turning counterclockwise for positive, seen facing its shaft, until {@link
     * Device#clockwise} says otherwise.
     */
    public Device can(int id) {
      return can(CanBus.SYSTEMCORE, id);
    }

    /** This kind of controller on a CAN bus, at an ID: {@code TALON_FX.can(CANIVORE, 23)}. */
    public Device can(CanBus bus, int id) {
      return new Device(this, bus, id, false, Optional.empty());
    }

    /**
     * The kind of device it is on the bus, which two can't share an ID within: REV's two SPARKs are
     * one.
     */
    public CanDevice.Kind kind() {
      return switch (this) {
        case TALON_FX -> CanDevice.Kind.TALON_FX;
        case TALON_FXS -> CanDevice.Kind.TALON_FXS;
        case SPARK_MAX, SPARK_FLEX -> CanDevice.Kind.SPARK;
      };
    }
  }

  /**
   * One motor's controller, as a joint's block names it: its kind, its bus and ID, which way its
   * motor turns for positive, and the PDH channel that feeds it. {@code
   * TALON_FX.can(32).clockwise().pdh(4, Amps.of(40))}.
   *
   * @param controller what it is
   * @param bus the CAN bus it's on
   * @param id its CAN ID
   * @param turnsClockwise whether its motor turns clockwise for positive, seen facing its shaft
   * @param feed the PDH channel it's powered from, and its breaker, if its block says
   */
  public record Device(
      Controller controller, CanBus bus, int id, boolean turnsClockwise, Optional<Pdh.Feed> feed) {
    public Device {
      Checks.on(Device.class).field("id", id, between(0, 62)).orThrow();
    }

    /** The same controller, its motor turning clockwise for positive, seen facing its shaft. */
    public Device clockwise() {
      return new Device(controller, bus, id, true, feed);
    }

    /**
     * The same controller, powered from one of the PDH's channels through a breaker: {@code .pdh(4,
     * Amps.of(40))}, channel 4 as labeled on the PDH, through a 40 A breaker. The robot reads its
     * current there, and flags it drawing nothing while its joint works.
     */
    public Device pdh(int channel, Current breaker) {
      return new Device(
          controller, bus, id, turnsClockwise, Optional.of(Pdh.feed(channel, breaker)));
    }
  }

  /**
   * A CANcoder at its bus and ID, its offset still to say ({@link #offset}): what {@link #CANCODER}
   * names.
   *
   * @param bus the bus it's on
   * @param id its CAN ID
   */
  public record CancoderAt(CanBus bus, int id) {
    /**
     * The CANcoder, its reading offset so it reads zero with the joint at zero, counting
     * counterclockwise as the joint moves forward, read once at startup to tell the joint's TalonFX
     * where the joint is ({@link Feedback#ROTOR}).
     */
    public Cancoder offset(Angle offset) {
      return MechanismWiringCancoderBuilder.builder()
          .bus(bus)
          .id(id)
          .offset(offset)
          .turnsClockwise(false)
          .feedback(Feedback.ROTOR)
          .build();
    }
  }

  /**
   * The mechanism's CANcoder: {@code CANCODER.can(CANIVORE, 34).offset(Rotations.of(0.82))}.
   *
   * @param bus the bus it's on: its joint's motors', where their TalonFX reads it
   * @param id its CAN ID
   * @param offset added to its reading so it reads zero with the joint at zero: measured per
   *     install, and again whenever the encoder or its magnet is moved
   * @param turnsClockwise whether it turns clockwise, seen facing its LED, as the joint moves
   *     forward: Phoenix's {@code Clockwise_Positive}, so its reading grows with the joint
   * @param feedback what the motor controller reads its position from
   */
  @ConfigRecord
  public record Cancoder(
      CanBus bus, int id, Angle offset, boolean turnsClockwise, Feedback feedback) {
    public Cancoder {
      Checks.on(Cancoder.class)
          .field("id", id, between(0, 62))
          .field("offset", offset, between(Rotations.of(-1), Rotations.of(1)))
          .orThrow();
    }

    /** Starts describing a CANcoder, one named value at a time. */
    public static MechanismWiringCancoderBuilder.BusStage builder() {
      return MechanismWiringCancoderBuilder.builder();
    }

    /**
     * The same CANcoder, turning clockwise, seen facing its LED, as the joint moves forward:
     * Phoenix's {@code Clockwise_Positive}.
     */
    public Cancoder clockwise() {
      return MechanismWiringCancoderBuilder.builder(this).turnsClockwise(true).build();
    }

    /**
     * The same CANcoder, the TalonFX's loop running on it, read over CAN at the CANcoder's own rate
     * ({@link Feedback#REMOTE}). No license needed.
     */
    public Cancoder remote() {
      return readAs(Feedback.REMOTE);
    }

    /**
     * The same CANcoder, the TalonFX's loop running on it fused with the motor's rotor: the
     * CANcoder's position, the rotor's rate ({@link Feedback#FUSED}, Phoenix Pro).
     */
    public Cancoder fused() {
      return readAs(Feedback.FUSED);
    }

    /**
     * The same CANcoder, the TalonFX's loop running on its rotor kept in step with the CANcoder
     * ({@link Feedback#SYNC}, Phoenix Pro).
     */
    public Cancoder sync() {
      return readAs(Feedback.SYNC);
    }

    private Cancoder readAs(Feedback feedback) {
      return MechanismWiringCancoderBuilder.builder(this).feedback(feedback).build();
    }
  }

  /** An absolute encoder on the leading SPARK's data port, its offset still to say. */
  public static final class DataPort {
    private DataPort() {}

    /**
     * The encoder, its reading offset so it reads zero with the joint at zero: read once at
     * startup, to tell the motor where the joint is, its reading growing as the joint moves
     * forward.
     */
    public DataPortEncoder offset(Angle offset) {
      return MechanismWiringDataPortEncoderBuilder.builder()
          .offset(offset)
          .reversed(false)
          .closedLoop(false)
          .build();
    }
  }

  /**
   * An absolute encoder plugged into the leading motor controller's data port, as REV's Through
   * Bore Encoder is into a SPARK's: read once at startup, to tell the motor where the joint is, or,
   * closing the loop on it ({@link #closesLoop}), the SPARK's own loop running on it: {@code
   * DATA_PORT.offset(Rotations.of(0.82)).closesLoop()}. The SPARK applies its direction and offset
   * itself ({@code RevMechanism}). From a team's REVLib code, whose {@code zeroOffset} is the raw
   * reading at zero before inverting: an offset of {@code -zeroOffset}, or {@code +zeroOffset}
   * inverted.
   *
   * @param offset added to its reading, in turns, so it reads zero with the joint at zero: measured
   *     per install, and again whenever the encoder is moved
   * @param reversed whether its reading falls as the joint moves forward
   * @param closedLoop whether the SPARK's loop runs on it, rather than on its motor's own encoder
   *     told once where the joint is
   */
  @ConfigRecord
  public record DataPortEncoder(Angle offset, boolean reversed, boolean closedLoop) {
    public DataPortEncoder {
      Checks.on(DataPortEncoder.class)
          .field("offset", offset, between(Rotations.of(-1), Rotations.of(1)))
          .orThrow();
    }

    /** Starts describing a data-port encoder, one named value at a time. */
    public static MechanismWiringDataPortEncoderBuilder.OffsetStage builder() {
      return MechanismWiringDataPortEncoderBuilder.builder();
    }

    /**
     * The same encoder, inverted, as REVLib's {@code inverted(true)}: its reading falls as the
     * joint moves forward.
     */
    public DataPortEncoder inverted() {
      return MechanismWiringDataPortEncoderBuilder.builder(this).reversed(true).build();
    }

    /**
     * The same encoder, the SPARK's loop running on it, at the SPARK's own rate, rather than on its
     * motor's encoder, which can slip or sit across backlash from the joint.
     */
    public DataPortEncoder closesLoop() {
      return MechanismWiringDataPortEncoderBuilder.builder(this).closedLoop(true).build();
    }

    /**
     * What the controller reads raw, from 0 to 1, where this encoder reads a number of turns, its
     * offset and direction applied: a SPARK's {@code zeroOffset}, for it to read zero there.
     */
    public double raw(double turns) {
      double raw = (reversed ? -1 : 1) * (turns - offset.in(Rotations));
      return raw - Math.floor(raw);
    }
  }

  /**
   * An absolute encoder plugged into one of Systemcore's own ports, read by the robot, not by a
   * motor controller: a REV Through Bore Encoder's duty-cycle output, or an analog encoder or
   * potentiometer. The robot tells the motor where the joint is from its first good reading, and
   * the motor's own encoder runs the joint's loop from then on; or, closing the loop on it ({@link
   * #closesLoop}), the robot runs the loop itself each loop on this encoder's reading.
   *
   * @param port the Systemcore port it's on, 0 to 5
   * @param signal how it reports where it is: a duty cycle, or a voltage
   * @param turnsPerRange how many of its turns its signal's full range covers: 1 for an encoder
   *     that wraps each turn, ten for a ten-turn potentiometer
   * @param offset added to its reading, in its turns, so it reads zero with the joint at zero:
   *     measured per install, and again whenever the encoder is moved
   * @param reversed whether its reading falls as the joint moves forward
   * @param closedLoop whether the joint's loop closes on it, in robot code each loop, rather than
   *     on its motor's own encoder told once where the joint is
   */
  @ConfigRecord
  public record PortEncoder(
      int port,
      Signal signal,
      double turnsPerRange,
      Angle offset,
      boolean reversed,
      boolean closedLoop) {
    // @start region=checks
    public PortEncoder {
      Checks.on(PortEncoder.class)
          .field("port", port, between(0, 5))
          .field("turnsPerRange", turnsPerRange, positive())
          .check(
              "offset",
              () ->
                  Math.abs(offset.in(Rotations)) <= turnsPerRange
                      ? null
                      : offset.in(Rotations)
                          + " turns is more than its range, "
                          + turnsPerRange
                          + " turns, either way")
          .orThrow();
    }

    // @end

    /** How a port encoder reports where it is. */
    public enum Signal {
      /** A pulse's share of its period, as a REV Through Bore's absolute output. */
      DUTY_CYCLE,
      /** A voltage's share of the port's range, as an analog encoder or a potentiometer. */
      ANALOG
    }

    /** A duty-cycle encoder on a Systemcore port, one turn its full range: {@code dutyCycle(1)}. */
    public static PortEncoder dutyCycle(int port) {
      return MechanismWiringPortEncoderBuilder.builder()
          .port(port)
          .signal(Signal.DUTY_CYCLE)
          .turnsPerRange(1)
          .offset(Rotations.of(0))
          .reversed(false)
          .closedLoop(false)
          .build();
    }

    /**
     * An analog encoder or potentiometer on a Systemcore port, its full voltage range this many of
     * its turns: {@code analog(2, 10)}, a ten-turn potentiometer.
     */
    public static PortEncoder analog(int port, double turnsPerRange) {
      return MechanismWiringPortEncoderBuilder.builder()
          .port(port)
          .signal(Signal.ANALOG)
          .turnsPerRange(turnsPerRange)
          .offset(Rotations.of(0))
          .reversed(false)
          .closedLoop(false)
          .build();
    }

    /** The same encoder, its reading offset so it reads zero with the joint at zero. */
    public PortEncoder offset(Angle offset) {
      return MechanismWiringPortEncoderBuilder.builder(this).offset(offset).build();
    }

    /**
     * The same encoder, inverted, as WPILib's {@code setInverted(true)}: its reading falls as the
     * joint moves forward.
     */
    public PortEncoder inverted() {
      return MechanismWiringPortEncoderBuilder.builder(this).reversed(true).build();
    }

    /**
     * The same encoder, the joint's loop closing on it: each loop, the robot works out the motor's
     * voltage from where this encoder says the joint is, rather than streaming a position to the
     * motor controller, whose own encoder can slip or sit across backlash from the joint. A loop
     * the robot runs, at its 50 Hz, is coarser than a motor controller's at 1 kHz.
     */
    public PortEncoder closesLoop() {
      return MechanismWiringPortEncoderBuilder.builder(this).closedLoop(true).build();
    }

    /**
     * Its reading as the joint's absolute encoder in the robot's definition has it, from what the
     * port reads raw, its signal's share of its range from 0 to 1: in turns of its signal, its full
     * range one turn, its direction and offset applied. A duty-cycle encoder's turn of signal is
     * its own turn; a ten-turn potentiometer's is ten of its own, so its sensor is geared a tenth
     * as much ({@code Driven.AbsoluteEncoder}).
     */
    public double read(double raw) {
      return (reversed ? -raw : raw) + offset.in(Rotations) / turnsPerRange;
    }

    /** What the port reads raw for a reading: {@link #read}'s inverse, from 0 to 1. */
    public double raw(double reading) {
      double share = (reversed ? -1 : 1) * (reading - offset.in(Rotations) / turnsPerRange);
      return share - Math.floor(share);
    }
  }

  /** What the leading motor controller reads the joint's position from. */
  public enum Feedback {
    /**
     * Its own rotor, the CANcoder read only by robot code: at boot, to set the rotor's position, as
     * a turret's coarse encoder is.
     */
    ROTOR,
    /** The CANcoder, over CAN: its position, at the CANcoder's own rate. No license needed. */
    REMOTE,
    /**
     * The CANcoder fused with its rotor (Phoenix Pro): the CANcoder's position, the rotor's rate.
     */
    FUSED,
    /** Its rotor, synchronized to the CANcoder (Phoenix Pro). */
    SYNC
  }
}
