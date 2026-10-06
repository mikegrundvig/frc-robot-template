package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.Angle;

/**
 * Where one mechanism's devices are on the CAN bus, which way each motor turns, and how its
 * CANcoder reads: the motors of one drive in the robot's definition, the first leading and the rest
 * following it, and the absolute encoder on its joint, if it has one. What the mechanism is (its
 * gearing, its range, where the encoder sits in its gear train) is the definition's; this is only
 * how it's connected, and differs robot to robot even with identical mechanisms.
 *
 * <p>Directions are against the joint's positive direction in the definition, never a motor's own
 * idea of forward: each motor says which way it turns as the joint moves forward, and the encoder
 * which way it counts. Phoenix's ratios are then plain magnitudes, and its followers' alignment
 * follows from the motors' directions ({@code PhoenixMechanism}).
 *
 * @param canBus the bus every device of the mechanism is on
 * @param controller the kind of motor controller its motors are on, every one of them
 * @param motors its motor controllers' CAN IDs and directions: the leader, then each follower, as
 *     many as its drive's motors
 * @param cancoder its CANcoder, if it has one: the definition's absolute encoder on its joint
 * @param dataPort the absolute encoder plugged into its leading motor controller's data port, if it
 *     has one instead: the definition's absolute encoder on its joint, read through a SPARK
 */
@ConfigRecord
public record MechanismWiring(
    CanBus canBus,
    Controller controller,
    List<Motor> motors,
    Optional<Cancoder> cancoder,
    Optional<DataPortEncoder> dataPort) {
  public MechanismWiring {
    motors = List.copyOf(motors);
    Checks.on(MechanismWiring.class)
        .field("motors", motors, MechanismWiring::notMotors)
        .check(
            "dataPort",
            () ->
                cancoder.isPresent() && dataPort.isPresent()
                    ? "names a CANcoder and a data-port encoder: the joint has one absolute encoder,"
                        + " read one way"
                    : null)
        .orThrow();
  }

  /** Starts describing a mechanism's wiring, one named value at a time. */
  public static MechanismWiringBuilder.CanBusStage builder() {
    return MechanismWiringBuilder.builder();
  }

  /** The motor controller that leads: the first. */
  public Motor leader() {
    return motors.get(0);
  }

  /**
   * The kind of motor controller a mechanism's motors are on, and the motors each can drive, as its
   * maker says.
   */
  public enum Controller {
    /** CTRE's TalonFX, built into a Kraken or a Falcon. */
    TALON_FX(
        "a TalonFX",
        Set.of(
            com.michaelgrundvig.frc.world.robot.Motor.KRAKEN_X60,
            com.michaelgrundvig.frc.world.robot.Motor.KRAKEN_X60_FOC,
            com.michaelgrundvig.frc.world.robot.Motor.KRAKEN_X44,
            com.michaelgrundvig.frc.world.robot.Motor.KRAKEN_X44_FOC,
            com.michaelgrundvig.frc.world.robot.Motor.FALCON_500,
            com.michaelgrundvig.frc.world.robot.Motor.FALCON_500_FOC)),
    /**
     * CTRE's TalonFXS: a Minion, or REV's motors by their JST connectors (Phoenix's {@code
     * MotorArrangementValue}).
     */
    TALON_FXS(
        "a TalonFXS",
        Set.of(
            com.michaelgrundvig.frc.world.robot.Motor.MINION,
            com.michaelgrundvig.frc.world.robot.Motor.NEO,
            com.michaelgrundvig.frc.world.robot.Motor.NEO_550,
            com.michaelgrundvig.frc.world.robot.Motor.NEO_VORTEX)),
    /** REV's SPARK MAX: a NEO or a NEO 550, or a NEO Vortex on its Solo Adapter. */
    SPARK_MAX(
        "a SPARK MAX",
        Set.of(
            com.michaelgrundvig.frc.world.robot.Motor.NEO,
            com.michaelgrundvig.frc.world.robot.Motor.NEO_550,
            com.michaelgrundvig.frc.world.robot.Motor.NEO_VORTEX)),
    /** REV's SPARK Flex: a NEO Vortex docked to it, or a NEO or NEO 550 through its adapter. */
    SPARK_FLEX(
        "a SPARK Flex",
        Set.of(
            com.michaelgrundvig.frc.world.robot.Motor.NEO_VORTEX,
            com.michaelgrundvig.frc.world.robot.Motor.NEO,
            com.michaelgrundvig.frc.world.robot.Motor.NEO_550));

    private final String shown;
    private final Set<com.michaelgrundvig.frc.world.robot.Motor> drives;

    Controller(String shown, Set<com.michaelgrundvig.frc.world.robot.Motor> drives) {
      this.shown = shown;
      this.drives = drives;
    }

    /** Whether it drives a motor of the definition's catalog. */
    public boolean drives(com.michaelgrundvig.frc.world.robot.Motor motor) {
      return drives.contains(motor);
    }

    /** As a sentence names it: "a SPARK MAX". */
    public String shown() {
      return shown;
    }

    /**
     * The kind of device it is on the bus, which two can't share an ID within: REV's two SPARKs are
     * one.
     */
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
      return new Device(this, bus, id, false);
    }

    public String kind() {
      return switch (this) {
        case TALON_FX -> "TalonFX";
        case TALON_FXS -> "TalonFXS";
        case SPARK_MAX, SPARK_FLEX -> "SPARK";
      };
    }
  }

  /**
   * One motor controller.
   *
   * @param id its CAN ID
   * @param clockwise whether the motor turns clockwise, seen facing its shaft, as the joint moves
   *     in its positive direction: Phoenix's {@code Clockwise_Positive}, a SPARK's {@code
   *     inverted}. For a differential, as the joint both motors move together (its average axis)
   *     moves forward.
   */
  /**
   * One motor's controller, as a joint's block names it: its kind, its bus and ID, and which way
   * its motor turns for positive. {@code TALON_FX.can(32).clockwise()}.
   *
   * @param controller what it is
   * @param bus the CAN bus it's on
   * @param id its CAN ID
   * @param turnsClockwise whether its motor turns clockwise for positive, seen facing its shaft
   */
  public record Device(Controller controller, CanBus bus, int id, boolean turnsClockwise) {
    public Device {
      Checks.on(Device.class).field("id", id, between(0, 62)).orThrow();
    }

    /** The same controller, its motor turning clockwise for positive, seen facing its shaft. */
    public Device clockwise() {
      return new Device(controller, bus, id, true);
    }
  }

  @ConfigRecord
  public record Motor(int id, boolean clockwise) {
    public Motor {
      Checks.on(Motor.class).field("id", id, between(0, 62)).orThrow();
    }

    /** Starts describing a motor controller, one named value at a time. */
    public static MechanismWiringMotorBuilder.IdStage builder() {
      return MechanismWiringMotorBuilder.builder();
    }
  }

  /**
   * The mechanism's CANcoder.
   *
   * @param id its CAN ID
   * @param offset added to its reading so it reads zero with the joint at zero: measured per
   *     install, and again whenever the encoder or its magnet is moved
   * @param clockwise whether it turns clockwise, seen facing its LED, as the joint moves forward:
   *     Phoenix's {@code Clockwise_Positive}, so its reading grows with the joint
   * @param feedback what the motor controller reads its position from
   */
  @ConfigRecord
  public record Cancoder(int id, Angle offset, boolean clockwise, Feedback feedback) {
    public Cancoder {
      Checks.on(Cancoder.class)
          .field("id", id, between(0, 62))
          .field("offset", offset, between(Rotations.of(-1), Rotations.of(1)))
          .orThrow();
    }

    /** Starts describing a CANcoder, one named value at a time. */
    public static MechanismWiringCancoderBuilder.IdStage builder() {
      return MechanismWiringCancoderBuilder.builder();
    }
  }

  /**
   * An absolute encoder plugged into the leading motor controller's data port, as REV's Through
   * Bore Encoder is into a SPARK's: read once at startup, to tell the motor where the joint is.
   * From a team's REVLib code, whose {@code zeroOffset} is the raw reading at zero before
   * inverting: an offset of {@code -zeroOffset}, or {@code +zeroOffset} reversed.
   *
   * @param offset added to its reading, in turns, so it reads zero with the joint at zero: measured
   *     per install, and again whenever the encoder is moved
   * @param reversed whether its reading falls as the joint moves forward
   */
  @ConfigRecord
  public record DataPortEncoder(Angle offset, boolean reversed) {
    public DataPortEncoder {
      Checks.on(DataPortEncoder.class)
          .field("offset", offset, between(Rotations.of(-1), Rotations.of(1)))
          .orThrow();
    }

    /** Starts describing a data-port encoder, one named value at a time. */
    public static MechanismWiringDataPortEncoderBuilder.OffsetStage builder() {
      return MechanismWiringDataPortEncoderBuilder.builder();
    }

    /** Its reading in turns, from what the controller reads raw, from 0 to 1. */
    public double turns(double raw) {
      return (reversed ? -raw : raw) + offset.in(Rotations);
    }
  }

  /**
   * An absolute encoder plugged into one of Systemcore's own ports, read by the robot, not by a
   * motor controller: a REV Through Bore Encoder's duty-cycle output, or an analog encoder or
   * potentiometer. The robot tells the motor where the joint is from its first good reading.
   *
   * @param port the Systemcore port it's on, 0 to 5
   * @param signal how it reports where it is: a duty cycle, or a voltage
   * @param turnsPerRange how many of its turns its signal's full range covers: 1 for an encoder
   *     that wraps each turn, ten for a ten-turn potentiometer
   * @param offset added to its reading, in its turns, so it reads zero with the joint at zero:
   *     measured per install, and again whenever the encoder is moved
   * @param reversed whether its reading falls as the joint moves forward
   */
  @ConfigRecord
  public record PortEncoder(
      int port, Signal signal, double turnsPerRange, Angle offset, boolean reversed) {
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

  /** Why a mechanism's motors aren't a leader and its followers, or null when they are. */
  private static @Nullable String notMotors(List<Motor> motors) {
    if (motors.isEmpty()) {
      return "needs a motor";
    }
    Set<Integer> seen = new HashSet<>();
    List<String> twice = new ArrayList<>();
    for (Motor motor : motors) {
      if (!seen.add(motor.id())) {
        twice.add("CAN ID " + motor.id() + " is used by two motor controllers");
      }
    }
    return twice.isEmpty() ? null : String.join("; ", twice);
  }
}
