package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;

/**
 * Where one of the robot's sensors is connected, per robot: a switch or a rangefinder from the
 * robot's definition, by its label, on an input. What the sensor is and where it sits are the
 * definition's; this is only how it's connected, as a joint's wiring is.
 */
public sealed interface SensorWiring permits SensorWiring.OfSwitch, SensorWiring.OfRangefinder {
  /** The sensor's label, as the robot's definition names it. */
  String sensor();

  /** Where it's connected. */
  Input input();

  /**
   * A switch's wiring.
   *
   * @param sensor the switch's label, as the robot's definition names it
   * @param input where it's connected: a Systemcore port, a CANdi's input, or a limit input on a
   *     joint's motor controller
   * @param polarity which raw signal means tripped: what nothing above the wiring sees
   */
  @ConfigRecord
  record OfSwitch(String sensor, Input input, Polarity polarity) implements SensorWiring {
    public OfSwitch {
      Checks.on(OfSwitch.class, sensor)
          .field("sensor", sensor, notBlank())
          .check(
              "input",
              () ->
                  input instanceof CanRange
                      ? "a switch isn't on a CANrange: a CANrange is a rangefinder"
                      : null)
          .orThrow();
    }

    /** Starts describing a switch's wiring, one named value at a time. */
    public static SensorWiringOfSwitchBuilder.SensorStage builder() {
      return SensorWiringOfSwitchBuilder.builder();
    }
  }

  /**
   * A rangefinder's wiring.
   *
   * @param sensor the rangefinder's label, as the robot's definition names it
   * @param input where it's connected: a CANrange
   */
  @ConfigRecord
  record OfRangefinder(String sensor, Input input) implements SensorWiring {
    public OfRangefinder {
      Checks.on(OfRangefinder.class, sensor)
          .field("sensor", sensor, notBlank())
          .check(
              "input",
              () ->
                  input instanceof CanRange
                      ? null
                      : "a rangefinder is on a CANrange: an analog one on a Systemcore port needs"
                          + " its volts for each meter, which the wiring doesn't hold yet")
          .orThrow();
    }

    /** Starts describing a rangefinder's wiring, one named value at a time. */
    public static SensorWiringOfRangefinderBuilder.SensorStage builder() {
      return SensorWiringOfRangefinderBuilder.builder();
    }
  }

  /** Where a sensor is connected. */
  sealed interface Input permits Port, Candi, CanRange, ControllerLimit {}

  /**
   * One of Systemcore's own I/O ports: six, each set as what the sensor needs (a digital input for
   * a switch, an analog one for a rangefinder). Alpha spec: confirm on a production unit.
   *
   * @param port 0 to 5
   */
  @ConfigRecord
  record Port(int port) implements Input {
    public Port {
      Checks.on(Port.class).field("port", port, between(0, 5)).orThrow();
    }
  }

  /**
   * One of a CANdi's two inputs: CTRE's CAN input board, which a TalonFX can read as its hardware
   * limit.
   *
   * @param bus the bus it's on
   * @param id its CAN ID
   * @param input 1 or 2: its S1 or S2
   */
  @ConfigRecord
  record Candi(CanBus bus, int id, int input) implements Input {
    public Candi {
      Checks.on(Candi.class)
          .field("id", id, between(0, 62))
          .field("input", input, between(1, 2))
          .orThrow();
    }
  }

  /**
   * A CANrange: CTRE's CAN rangefinder.
   *
   * @param bus the bus it's on
   * @param id its CAN ID
   */
  @ConfigRecord
  record CanRange(CanBus bus, int id) implements Input {
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
  record ControllerLimit(String joint, Direction direction) implements Input {
    public ControllerLimit {
      Checks.on(ControllerLimit.class, joint).field("joint", joint, notBlank()).orThrow();
    }

    /**
     * A limit input on a joint's own leading controller, by the joint itself: {@code
     * ControllerLimit.of(ROLLER, Direction.FORWARD)}, a beam on the roller's SPARK.
     */
    public static ControllerLimit of(Joint joint, Direction direction) {
      return SensorWiringControllerLimitBuilder.builder()
          .joint(joint.label())
          .direction(direction)
          .build();
    }
  }

  /** Which of a motor controller's two limit inputs. */
  enum Direction {
    FORWARD,
    REVERSE
  }

  /**
   * Where a switch is connected and which raw signal means tripped, as a switch carries it: {@code
   * CLAW_BEAM.on(new SensorWiring.Candi(CAN, 30, 1), Polarity.NORMALLY_OPEN)}.
   *
   * @param input where it's connected
   * @param polarity which raw signal means tripped
   */
  record Connection(Input input, Polarity polarity) {
    public Connection {
      Checks.on(Connection.class)
          .check(
              "input",
              () ->
                  input instanceof CanRange
                      ? "a switch isn't on a CANrange: a CANrange is a rangefinder"
                      : null)
          .orThrow();
    }
  }

  /**
   * Which raw signal means tripped. A limit switch wired normally closed reads tripped with its
   * wire cut, so its motor stops: the safe one. A beam break's output is a switch too, and which it
   * is depends on its beam: a through-beam's light reaches it while it's clear, so its light-on
   * output is normally closed; a reflective one's light comes back when something's there, so its
   * light-on output is normally open. Dark-on is the other way round, each.
   */
  enum Polarity {
    NORMALLY_OPEN,
    NORMALLY_CLOSED
  }
}
