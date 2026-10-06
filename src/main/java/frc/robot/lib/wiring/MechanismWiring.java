package frc.robot.lib.wiring;

import static com.michaelgrundvig.frc.world.config.Rules.between;
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
