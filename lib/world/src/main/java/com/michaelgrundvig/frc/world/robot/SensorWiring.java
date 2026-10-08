package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;

/**
 * Where the robot's sensors are connected: the inputs a switch or a rangefinder is on, which each
 * sensor carries, a joint's limit switch in its block ({@code .limitSwitch(End.LOWER,
 * Degrees.of(0), SensorWiring.port(1).normallyClosed())}) and any other with {@code .on(...)}. What
 * the sensor is and where it sits are the definition's; this is only how it's connected, as a
 * joint's wiring is ({@link MechanismWiring}).
 *
 * <p>A block names an input as it names a motor's controller: {@code SensorWiring.port(1)}, {@code
 * CANDI.can(CANIVORE, 30).s1()}, {@code CANRANGE.can(CANIVORE, 31)}.
 */
public final class SensorWiring {
  private SensorWiring() {}

  /**
   * A CANdi, CTRE's CAN input board, at its bus and ID, a switch on one of its two inputs: {@code
   * CANDI.can(CANIVORE, 30).s1()}.
   */
  public static final CanDeviceKind<CandiAt> CANDI = new CanDeviceKind<>(CandiAt::new);

  /** A CANrange, CTRE's CAN rangefinder, at its bus and ID: {@code CANRANGE.can(CANIVORE, 31)}. */
  public static final CanDeviceKind<CanRange> CANRANGE =
      new CanDeviceKind<>(
          (bus, id) -> SensorWiringCanRangeBuilder.builder().bus(bus).id(id).build());

  /** One of Systemcore's own I/O ports, 0 to 5: {@code SensorWiring.port(1)}. */
  public static Port port(int port) {
    return SensorWiringPortBuilder.builder().port(port).build();
  }

  /**
   * An input as a sentence names it: "Systemcore's port 2", "CANdi 31's S1", "CANrange 30",
   * "pivot's controller's reverse limit".
   */
  public static String shown(Input input) {
    return switch (input) {
      case Port port -> "Systemcore's port " + port.port();
      case Candi candi -> "CANdi " + candi.id() + "'s S" + candi.input();
      case CanRange range -> "CANrange " + range.id();
      case ControllerLimit limit ->
          possessive(limit.joint())
              + " controller's "
              + (limit.direction() == Direction.FORWARD ? "forward" : "reverse")
              + " limit";
    };
  }

  /** Where a sensor is connected. */
  public sealed interface Input permits SwitchInput, CanRange {}

  /**
   * Where a switch can be connected: a Systemcore port, a CANdi's input, or a limit input on a
   * joint's motor controller. Its polarity says which raw signal means tripped: {@code
   * CANDI.can(30).s1().normallyOpen()}.
   */
  public sealed interface SwitchInput extends Input permits Port, Candi, ControllerLimit {
    /** A switch on it whose contact closes, or whose output turns on, when it's tripped. */
    default Connection normallyOpen() {
      return new Connection(this, Polarity.NORMALLY_OPEN);
    }

    /**
     * A switch on it whose contact opens, or whose output turns off, when it's tripped: a cut wire
     * reads tripped, the safe way to wire a limit switch.
     */
    default Connection normallyClosed() {
      return new Connection(this, Polarity.NORMALLY_CLOSED);
    }
  }

  /**
   * One of Systemcore's own I/O ports: six, each set as what the sensor needs (a digital input for
   * a switch, an analog one for a rangefinder). Alpha spec: confirm on a production unit.
   *
   * @param port 0 to 5
   */
  @ConfigRecord
  public record Port(int port) implements SwitchInput {
    public Port {
      Checks.on(Port.class).field("port", port, between(0, 5)).orThrow();
    }
  }

  /**
   * One of a CANdi's two inputs: CTRE's CAN input board, which a TalonFX can read as its hardware
   * limit. {@code CANDI.can(CANIVORE, 30).s1()}.
   *
   * @param bus the bus it's on
   * @param id its CAN ID
   * @param input 1 or 2: its S1 or S2
   */
  @ConfigRecord
  public record Candi(CanBus bus, int id, int input) implements SwitchInput {
    public Candi {
      Checks.on(Candi.class)
          .field("id", id, between(0, 62))
          .field("input", input, between(1, 2))
          .orThrow();
    }
  }

  /**
   * A CANdi at its bus and ID, which of its inputs still to say: {@link #s1} or {@link #s2}.
   *
   * @param bus the bus it's on
   * @param id its CAN ID
   */
  public record CandiAt(CanBus bus, int id) {
    /** Its first input, S1. */
    public Candi s1() {
      return input(1);
    }

    /** Its second input, S2. */
    public Candi s2() {
      return input(2);
    }

    private Candi input(int input) {
      return SensorWiringCandiBuilder.builder().bus(bus).id(id).input(input).build();
    }
  }

  /**
   * A CANrange: CTRE's CAN rangefinder. {@code CANRANGE.can(CANIVORE, 31)}.
   *
   * @param bus the bus it's on
   * @param id its CAN ID
   */
  @ConfigRecord
  public record CanRange(CanBus bus, int id) implements Input {
    public CanRange {
      Checks.on(CanRange.class).field("id", id, between(0, 62)).orThrow();
    }
  }

  /**
   * A limit input on a joint's leading motor controller, which can stop its motor by itself: a
   * SPARK's, a TalonFXS's, or a Falcon 500's TalonFX's. A Kraken's TalonFX has none.
   *
   * @param joint the joint's label, whose leading controller it's on
   * @param direction its forward or reverse limit input, as the controller labels them
   */
  @ConfigRecord
  public record ControllerLimit(String joint, Direction direction) implements SwitchInput {
    public ControllerLimit {
      Checks.on(ControllerLimit.class, joint).field("joint", joint, notBlank()).orThrow();
    }

    /**
     * A limit input on a joint's own leading controller, by the joint itself: {@code
     * ControllerLimit.of(ROLLER, Direction.FORWARD).normallyOpen()}, a beam on the roller's SPARK.
     * A joint's own limit switch is on its pin from its block: {@code Direction.REVERSE
     * .normallyClosed()}.
     */
    public static ControllerLimit of(Joint joint, Direction direction) {
      return SensorWiringControllerLimitBuilder.builder()
          .joint(joint.label())
          .direction(direction)
          .build();
    }
  }

  /** Which of a motor controller's two limit inputs. */
  public enum Direction {
    FORWARD,
    REVERSE;

    /**
     * A joint's limit switch on this pin of its own controller, normally open: {@code
     * .limitSwitch(End.UPPER, Degrees.of(135), Direction.FORWARD.normallyOpen())}.
     */
    public Pin normallyOpen() {
      return new Pin(this, Polarity.NORMALLY_OPEN);
    }

    /**
     * A joint's limit switch on this pin of its own controller, normally closed, the safe way:
     * {@code .limitSwitch(End.LOWER, Degrees.of(0), Direction.REVERSE.normallyClosed())}.
     */
    public Pin normallyClosed() {
      return new Pin(this, Polarity.NORMALLY_CLOSED);
    }
  }

  /**
   * A joint's limit switch on one of its own leading controller's limit pins, as its block names it
   * before the joint exists: {@code Direction.REVERSE.normallyClosed()}. The controller stops its
   * motor on it by itself.
   *
   * @param direction its forward or reverse limit pin, as the controller labels them
   * @param polarity which raw signal means tripped
   */
  public record Pin(Direction direction, Polarity polarity) {
    /** Where it's connected, on this joint's leading controller. */
    public Connection of(Joint joint) {
      return new Connection(ControllerLimit.of(joint, direction), polarity);
    }
  }

  /**
   * Where a switch is connected and which raw signal means tripped, as a switch carries it: {@code
   * .on(CANDI.can(CANIVORE, 30).s1().normallyOpen())}.
   *
   * @param input where it's connected: an input a switch can be on, never a CANrange
   * @param polarity which raw signal means tripped
   */
  public record Connection(SwitchInput input, Polarity polarity) {}

  /**
   * Which raw signal means tripped. A limit switch wired normally closed reads tripped with its
   * wire cut, so its motor stops: the safe one. A beam break's output is a switch too, and which it
   * is depends on its beam: a through-beam's light reaches it while it's clear, so its light-on
   * output is normally closed; a reflective one's light comes back when something's there, so its
   * light-on output is normally open. Dark-on is the other way round, each.
   */
  public enum Polarity {
    NORMALLY_OPEN,
    NORMALLY_CLOSED
  }
}
