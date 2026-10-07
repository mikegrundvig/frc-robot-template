package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;

/**
 * How a joint is driven, and how its devices are wired: everything its block says beyond its shape
 * and its places. Its motors or servos, its own sensors (an absolute encoder on its axle, limit
 * switches at its ends), their wiring, and how the program drives it. A joint's block writes it a
 * call at a time ({@code .motor(...)}, {@code .control(...)}); {@link #NONE} is a joint nothing
 * drives.
 *
 * <p>It isn't part of the joint's identity: a joint that differs only here is the same joint,
 * re-tuned or re-wired, so a practice robot's, or a test's, still matches everything that names it.
 *
 * @param motors its motors, through its gearing; empty if none turn it
 * @param servos its servos, each pushing between its parent and its link
 * @param absoluteEncoder an absolute encoder on its axle, if it has one
 * @param limitSwitches the switches it presses at the ends of its travel
 * @param control how the program drives it; needed for a joint with motors
 * @param interlocks what restricts it by where another joint is, how fast one runs, or what a
 *     sensor reads, written in its block ({@code .keepsOut(...).when(...)}, {@code
 *     .runsOnlyWhen(...)}); the robot's definition can list more beside them
 */
@ConfigRecord
public record Driven(
    Optional<Motors> motors,
    List<Servo> servos,
    Optional<AbsoluteEncoder> absoluteEncoder,
    List<LimitSwitch> limitSwitches,
    Optional<MechanismControl> control,
    List<Interlock> interlocks) {
  /** A joint nothing drives: a follower, a passive hinge, or one not described yet. */
  public static final Driven NONE =
      DrivenBuilder.builder()
          .motors(Optional.empty())
          .servos(List.of())
          .absoluteEncoder(Optional.empty())
          .limitSwitches(List.of())
          .control(Optional.empty())
          .interlocks(List.of())
          .build();

  public Driven {
    servos = List.copyOf(servos);
    limitSwitches = List.copyOf(limitSwitches);
    interlocks = List.copyOf(interlocks);
    Optional<Motors> turning = motors;
    Optional<AbsoluteEncoder> encoder = absoluteEncoder;
    List<LimitSwitch> switches = limitSwitches;
    Checks.on(Driven.class)
        .check(
            "absoluteEncoder",
            () ->
                encoder.isPresent()
                        && encoder.get().readByController()
                        && (turning.isEmpty() || turning.get().controllers().isEmpty())
                    ? "its absolute encoder is wired, and its motors aren't: it's read through"
                        + " their controller, so wire them too"
                    : null)
        .check("limitSwitches", () -> twice(switches))
        .orThrow();
  }

  /** Starts describing how a joint is driven, one named value at a time. */
  public static DrivenBuilder.MotorsStage builder() {
    return DrivenBuilder.builder();
  }

  /**
   * Its devices' wiring, as the program reads it: its motors' controllers, and the absolute encoder
   * read through them; empty for a joint that runs in the simulator alone.
   */
  public Optional<MechanismWiring> wiring() {
    if (motors.isEmpty() || motors.get().controllers().isEmpty()) {
      return Optional.empty();
    }
    List<MechanismWiring.Device> controllers = motors.get().controllers();
    MechanismWiring.Device leader = controllers.get(0);
    return Optional.of(
        MechanismWiring.builder()
            .canBus(leader.bus())
            .controller(leader.controller())
            .motors(
                controllers.stream()
                    .map(
                        each ->
                            MechanismWiringMotorBuilder.builder()
                                .id(each.id())
                                .clockwise(each.turnsClockwise())
                                .build())
                    .toList())
            .cancoder(absoluteEncoder.flatMap(AbsoluteEncoder::cancoder))
            .dataPort(absoluteEncoder.flatMap(AbsoluteEncoder::dataPort))
            .build());
  }

  /**
   * Motors turning a joint together through its gearing, and, for one that slides, onto a drum.
   *
   * @param motor the motor, the same for each
   * @param count how many
   * @param gearing from the motors to the joint, or to the drum
   * @param pitchRadius where the belt, chain or rack meets the drum, sprocket or pinion: a sliding
   *     joint's, empty for one that turns
   * @param controllers each motor's controller, the leader first; none for the simulator alone
   */
  @ConfigRecord
  public record Motors(
      Motor motor,
      int count,
      Gearing gearing,
      Optional<Distance> pitchRadius,
      List<MechanismWiring.Device> controllers) {
    public Motors {
      controllers = List.copyOf(controllers);
      List<MechanismWiring.Device> each = controllers;
      Checks.on(Motors.class)
          .field("count", count, positive())
          .check(
              "controllers",
              () ->
                  each.isEmpty() || each.size() == count
                      ? null
                      : "names "
                          + each.size()
                          + " controllers for "
                          + count
                          + " motors: one each, or none for the simulator")
          .check("controllers", () -> mixed(motor, each))
          .orThrow();
    }

    /** Starts describing motors, one named value at a time. */
    public static DrivenMotorsBuilder.MotorStage builder() {
      return DrivenMotorsBuilder.builder();
    }

    /** Why a joint's controllers can't be its motors', or null when they can be. */
    private static @Nullable String mixed(Motor motor, List<MechanismWiring.Device> controllers) {
      if (controllers.isEmpty()) {
        return null;
      }
      MechanismWiring.Device leader = controllers.get(0);
      for (MechanismWiring.Device device : controllers) {
        if (device.controller() != leader.controller() || !device.bus().equals(leader.bus())) {
          return "a joint's motors are on one kind of controller, on one bus: its leader is "
              + leader.controller().shown()
              + ", and CAN ID "
              + device.id()
              + " isn't the same";
        }
      }
      return leader.controller().drives(motor)
          ? null
          : leader.controller().shown() + " doesn't drive " + motor.name();
    }
  }

  /**
   * A servo pushing between the joint's parent and its link: a hood's linear servo.
   *
   * @param label what logs call it: unique among the robot's drives
   * @param servo the servo
   * @param onParent where its body is fixed, from the joint's axle, in the parent's frame
   * @param onLink where its rod meets the link, in the joint's frame
   */
  @ConfigRecord
  public record Servo(
      String label, Actuator.Servo servo, Translation3d onParent, Translation3d onLink) {
    public Servo {
      Checks.on(Servo.class, label).field("label", label, notBlank()).orThrow();
    }
  }

  /**
   * An absolute encoder on the joint, turning with it: it knows where the joint is from the moment
   * the robot starts. On its axle at 1:1, or geared ({@link #ratio}). Read by a CANcoder, through
   * the joint's TalonFX; on its SPARK's data port; or on one of Systemcore's ports, by the robot;
   * or by none of them, in the simulator alone.
   *
   * @param countsPerTurn its resolution, from its datasheet, if it's given: only a pair geared
   *     nearly alike, decoded by their difference, reads it, and needs it ({@code
   *     RobotMath.absoluteSpan})
   * @param ratio its turns for each of the joint's, positive: 1 on the axle, 92/18 on an 18-tooth
   *     gear against a 92-tooth ring, which way it turns being its wiring's; it must stay within
   *     one of its turns, or the joint's position can't be told from it alone
   * @param cancoder its CANcoder, if it's read through one
   * @param dataPort its SPARK's data port, if it's read there
   * @param port the Systemcore port it's on, if the robot reads it there
   */
  @ConfigRecord
  public record AbsoluteEncoder(
      Optional<Integer> countsPerTurn,
      double ratio,
      Optional<MechanismWiring.Cancoder> cancoder,
      Optional<MechanismWiring.DataPortEncoder> dataPort,
      Optional<MechanismWiring.PortEncoder> port) {
    public AbsoluteEncoder {
      int read =
          (cancoder.isPresent() ? 1 : 0)
              + (dataPort.isPresent() ? 1 : 0)
              + (port.isPresent() ? 1 : 0);
      Checks.on(AbsoluteEncoder.class)
          .field("countsPerTurn", countsPerTurn, given(positive()))
          .field(
              "ratio",
              ratio,
              r ->
                  Double.isFinite(r) && r > 0
                      ? null
                      : r
                          + " must be a ratio, finite and positive: which way it turns is its"
                          + " wiring's, a CANcoder's clockwise, a data port's reversed, a port's"
                          + " inverted")
          .check(
              "port",
              () ->
                  read > 1
                      ? "names more than one of a CANcoder, a data port and a Systemcore port: it's"
                          + " read one way"
                      : null)
          .orThrow();
    }

    /**
     * An absolute encoder on the joint's axle and unwired, its resolution not given: {@code
     * AbsoluteEncoder.of().ratio(0.536).on(cancoder)}.
     */
    public static AbsoluteEncoder of() {
      return DrivenAbsoluteEncoderBuilder.builder()
          .countsPerTurn(Optional.empty())
          .ratio(1)
          .cancoder(Optional.empty())
          .dataPort(Optional.empty())
          .port(Optional.empty())
          .build();
    }

    /**
     * An absolute encoder of a resolution, on the joint's axle and unwired: {@code
     * AbsoluteEncoder.of(4096).ratio(92.0 / 18).on(cancoder)}, one of a pair decoded by their
     * difference, which reads it.
     */
    public static AbsoluteEncoder of(int countsPerTurn) {
      return DrivenAbsoluteEncoderBuilder.builder()
          .countsPerTurn(Optional.of(countsPerTurn))
          .ratio(1)
          .cancoder(Optional.empty())
          .dataPort(Optional.empty())
          .port(Optional.empty())
          .build();
    }

    /** The same encoder, geared: its turns for each of the joint's. */
    public AbsoluteEncoder ratio(double ratio) {
      return DrivenAbsoluteEncoderBuilder.builder(this).ratio(ratio).build();
    }

    /** The same encoder, read by a CANcoder through the joint's TalonFX. */
    public AbsoluteEncoder on(MechanismWiring.Cancoder cancoder) {
      return DrivenAbsoluteEncoderBuilder.builder(this).cancoder(Optional.of(cancoder)).build();
    }

    /** The same encoder, on the joint's SPARK's data port. */
    public AbsoluteEncoder on(MechanismWiring.DataPortEncoder dataPort) {
      return DrivenAbsoluteEncoderBuilder.builder(this).dataPort(Optional.of(dataPort)).build();
    }

    /** The same encoder, on one of Systemcore's ports, read by the robot. */
    public AbsoluteEncoder on(MechanismWiring.PortEncoder port) {
      return DrivenAbsoluteEncoderBuilder.builder(this).port(Optional.of(port)).build();
    }

    /** Whether it's wired at all: a CANcoder, a data port or a Systemcore port. */
    public boolean wired() {
      return cancoder.isPresent() || dataPort.isPresent() || port.isPresent();
    }

    /**
     * How many of its turns one turn of its signal covers: its port's range for a potentiometer on
     * a Systemcore port, ten for a ten-turn one; 1 for every other encoder. Its sensor in the
     * robot's definition counts in turns of its signal, so a potentiometer reads as an encoder
     * geared that much slower ({@link MechanismWiring.PortEncoder#read}).
     */
    public double turnsPerSignal() {
      return port.map(MechanismWiring.PortEncoder::turnsPerRange).orElse(1.0);
    }

    /** Whether the joint's motor controller reads it: a CANcoder, or a SPARK's data port. */
    public boolean readByController() {
      return cancoder.isPresent() || dataPort.isPresent();
    }

    /**
     * Whether the joint's loop closes on it, as its wiring says, rather than on its motor's own
     * encoder told once where the joint is: a Systemcore port's, in robot code each loop, or a
     * SPARK's data port's, on the SPARK. A CANcoder's wiring says it its own way ({@link
     * MechanismWiring.Feedback}).
     */
    public boolean closedLoop() {
      return port.map(MechanismWiring.PortEncoder::closedLoop).orElse(false)
          || dataPort.map(MechanismWiring.DataPortEncoder::closedLoop).orElse(false);
    }
  }

  /**
   * A switch the joint presses at one end of its travel.
   *
   * @param end which end
   * @param position where along the joint it trips, in the joint's units; empty if that isn't
   *     known, so it can stop the joint but not home it
   * @param wiring where it's connected, or none for the simulator alone
   */
  @ConfigRecord
  public record LimitSwitch(
      Sensor.Switch.Limit.End end,
      Optional<Double> position,
      Optional<SensorWiring.Connection> wiring) {}

  /** The ends with two limit switches, or null when each has one at most. */
  private static @Nullable String twice(List<LimitSwitch> switches) {
    Set<Sensor.Switch.Limit.End> seen = new HashSet<>();
    List<String> twice = new ArrayList<>();
    for (LimitSwitch each : switches) {
      if (!seen.add(each.end())) {
        twice.add(each.end().name().toLowerCase(java.util.Locale.ROOT));
      }
    }
    return twice.isEmpty()
        ? null
        : "two limit switches at its " + String.join(", ", twice) + " end";
  }
}
