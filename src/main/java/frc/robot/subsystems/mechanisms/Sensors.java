package frc.robot.subsystems.mechanisms;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.michaelgrundvig.frc.world.robot.SensorWiringPortBuilder;
import frc.robot.behavior.Gate;
import frc.robot.lib.wiring.ControllerPins;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;

/**
 * The robot's switches and rangefinders, as its program reads them: the definition says what each
 * is and where, its wiring how it's connected on a robot. Each loop, after the joints, it reads
 * each through its IO ({@link SensorIO}), logged under {@code Sensors/<label>}; behavior reads them
 * through the conditions the model names, as gates:
 *
 * <pre>{@code
 * Gate coralIn = sensors.gate("coral in claw", CLAW_BEAM.tripped());
 * roller.runUntil("intaking", coralIn)              // ends ARRIVED the loop it trips
 * co.waitUntil(coralIn);                             // holds a step
 * coralIn.onTrue(...)                                // runs a command as it trips
 * }</pre>
 *
 * <p>A sensor that's lost (a CAN device silent, or one with no wiring on a robot) holds no
 * condition either way: neither tripped nor clear, neither nearer nor farther. A gate on it never
 * opens, and says so ("hopper range lost"); a move that waits on it times out.
 */
public final class Sensors {
  /** One sensor, as read. */
  private static final class Reading {
    final Sensor.Placed sensor;
    final SensorIO io;
    final SensorIOInputsAutoLogged inputs = new SensorIOInputsAutoLogged();
    final String key;

    Reading(Sensor.Placed sensor, SensorIO io) {
      this.sensor = sensor;
      this.io = io;
      key = "Sensors/" + sensor.label();
    }
  }

  private final Scheduler scheduler;
  private final Reading[] readings;
  private final Map<Sensor, Reading> bySensor = new HashMap<>();

  /**
   * @param definition what the robot is: its switches and rangefinders
   * @param io each one's hardware: a port or a CAN device, the simulator's, or none
   * @param pinned binds a switch on a joint's limit input ({@link SensorIO#onPin}) to its joint
   * @param scheduler the command scheduler its gates are registered with
   */
  Sensors(
      RobotDefinition definition,
      Function<Sensor.Placed, SensorIO> io,
      Function<SensorIO, SensorIO> pinned,
      Scheduler scheduler) {
    this.scheduler = scheduler;
    List<Reading> built = new ArrayList<>();
    for (Sensor sensor : definition.math().sensors()) {
      if (sensor instanceof Sensor.Placed placed) {
        Reading reading = new Reading(placed, pinned.apply(io.apply(placed)));
        built.add(reading);
        bySensor.put(placed, reading);
      }
    }
    readings = built.toArray(Reading[]::new);
  }

  /** Reads every sensor: once a loop, after the joints and before the command scheduler. */
  void read() {
    for (Reading reading : readings) {
      reading.io.updateInputs(reading.inputs);
      Logger.processInputs(reading.key, reading.inputs);
    }
  }

  /**
   * Whether a condition on one of the robot's sensors holds this loop: never for a sensor that's
   * lost.
   *
   * @throws IllegalArgumentException for a sensor the robot doesn't have
   */
  public boolean holds(Condition.OnSensor condition) {
    SensorIO.SensorIOInputs inputs = reading(condition.sensor()).inputs;
    if (!inputs.connected) {
      return false;
    }
    return switch (condition) {
      case Condition.Tripped tripped -> inputs.tripped;
      case Condition.Clear clear -> !inputs.tripped;
      case Condition.Nearer nearer -> inputs.meters < nearer.distance().in(Meters);
      case Condition.Farther farther -> inputs.meters > farther.distance().in(Meters);
    };
  }

  /** Whether a sensor answers this loop: false for one that's lost. */
  public boolean reads(Sensor sensor) {
    return reading(sensor).inputs.connected;
  }

  /** A gate on a condition, named as the condition reads: "claw beam tripped". */
  public Gate gate(Condition.OnSensor condition) {
    return gate(condition.toString(), condition);
  }

  /**
   * A gate on a condition, named for what it means to the robot: {@code sensors.gate("coral in
   * claw", CLAW_BEAM.tripped())}, logged as {@code Gates/coral in claw}. Combine and debounce it as
   * any gate ({@link Gate#all}, {@link Gate#debounce}).
   *
   * @throws IllegalArgumentException for a sensor the robot doesn't have
   */
  public Gate gate(String name, Condition.OnSensor condition) {
    Reading reading = reading(condition.sensor());
    return new Gate(
        scheduler,
        name,
        why -> {
          boolean holds = holds(condition);
          if (!holds && !reading.inputs.connected) {
            why.add(reading.sensor.label() + " lost");
          }
          return holds;
        });
  }

  /** The sensors that are lost this loop, by label: what an alert names. */
  public List<String> lost() {
    List<String> lost = new ArrayList<>();
    for (Reading reading : readings) {
      if (!reading.inputs.connected) {
        lost.add(reading.sensor.label());
      }
    }
    return lost;
  }

  private Reading reading(Sensor sensor) {
    Reading reading = bySensor.get(sensor);
    if (reading == null) {
      throw new IllegalArgumentException(sensor.label() + " isn't one of the robot's sensors");
    }
    return reading;
  }

  /**
   * What's wrong with a robot's sensor wiring against its definition and its joints, or null when
   * nothing is: a wiring for a sensor the definition doesn't have, or of the wrong kind; a sensor
   * wired twice; two sensors on one input, a joint's absolute encoder on a Systemcore port among
   * them; a controller's limit input on a joint that isn't wired, or whose controller has none (a
   * Kraken's TalonFX), or a joint's limit switch on another joint's controller, which would stop
   * the wrong motor. Whether a limit switch's input is the one for its end isn't checked: whether a
   * SPARK's forward input follows its inversion is unconfirmed (open question M22). The robot's
   * configuration asks this as it's built. Two CANranges on one ID are the CAN IDs' check's to say
   * ({@code CanIds.sensorsTwice}).
   *
   * @param joints each wired joint's wiring, by its label
   */
  public static @Nullable String wiringProblems(
      RobotDefinition definition, Map<String, MechanismWiring> joints, List<SensorWiring> sensors) {
    Map<String, Sensor> byLabel = new HashMap<>();
    for (Sensor sensor : definition.math().sensors()) {
      byLabel.put(sensor.label(), sensor);
    }
    List<String> problems = new ArrayList<>();
    Map<String, String> wired = new HashMap<>();
    Map<Object, String> inputs = new HashMap<>();
    for (Joint joint : definition.joints()) {
      Optional<MechanismWiring.PortEncoder> port =
          joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port);
      if (port.isPresent()) {
        SensorWiring.Port input = SensorWiringPortBuilder.builder().port(port.get().port()).build();
        String label = joint.label() + " absolute encoder";
        String other = inputs.putIfAbsent(input, label);
        if (other != null) {
          problems.add(other + " and " + label + " are on one input: " + shown(input));
        }
      }
    }
    for (SensorWiring wiring : sensors) {
      String label = wiring.sensor();
      Sensor sensor = byLabel.get(label);
      if (sensor == null) {
        problems.add(label + " is wired, and isn't one of " + definition.name() + "'s sensors");
        continue;
      }
      if (wiring instanceof SensorWiring.OfSwitch && !(sensor instanceof Sensor.Switch)) {
        problems.add(label + " is wired as a switch, and the definition's isn't one");
      }
      if (wiring instanceof SensorWiring.OfRangefinder && !(sensor instanceof Sensor.Rangefinder)) {
        problems.add(label + " is wired as a rangefinder, and the definition's isn't one");
      }
      if (wired.put(label, label) != null) {
        problems.add(label + " is wired twice");
      }
      String other =
          wiring.input() instanceof SensorWiring.CanRange
              ? null
              : inputs.putIfAbsent(wiring.input(), label);
      if (other != null && !other.equals(label)) {
        problems.add(other + " and " + label + " are on one input: " + shown(wiring.input()));
      }
      if (wiring.input() instanceof SensorWiring.ControllerLimit limit) {
        if (sensor instanceof Sensor.Switch onSwitch
            && onSwitch.kind() instanceof Sensor.Switch.Limit own
            && !own.joint().label().equals(limit.joint())) {
          problems.add(
              label
                  + " is "
                  + own.joint().label()
                  + "'s limit switch, on "
                  + limit.joint()
                  + "'s controller, which would stop "
                  + limit.joint()
                  + ": wire it to "
                  + own.joint().label()
                  + "'s");
        }
        MechanismWiring on = joints.get(limit.joint());
        if (on == null) {
          problems.add(
              label
                  + " is on "
                  + limit.joint()
                  + "'s controller's limit input, and "
                  + limit.joint()
                  + " isn't wired");
        } else if (on.controller() == MechanismWiring.Controller.TALON_FX
            && !falcon(definition, limit.joint())) {
          problems.add(
              label
                  + " is on "
                  + limit.joint()
                  + "'s TalonFX's limit input, and a Kraken's TalonFX has none: put it on a"
                  + " CANdi");
        }
      }
    }
    return problems.isEmpty() ? null : String.join("; ", problems);
  }

  /**
   * Whether a joint's motors are Falcon 500s, whose TalonFX has limit inputs, as a Kraken's hasn't.
   */
  private static boolean falcon(RobotDefinition definition, String joint) {
    RobotMath math = definition.math();
    for (Joint each : definition.joints()) {
      if (each.label().equals(joint)
          && !math.drivesOf(each).isEmpty()
          && math.drivesOf(each).get(0).actuator() instanceof Actuator.Motors motors) {
        return motors.motor().equals(Motor.FALCON_500)
            || motors.motor().equals(Motor.FALCON_500_FOC);
      }
    }
    return false;
  }

  /**
   * What a joint's leading motor controller has on its limits, from the robot's sensor wiring: a
   * switch on one of its pins, stopped on if it's a limit switch; and, for a TalonFX or TalonFXS,
   * its own limit switch on a CANdi input on its bus, which it reads over CAN, as its reverse limit
   * for the switch at its lower end and its forward for the upper. A SPARK can't read a CANdi: its
   * limit switch there stops it only through the robot's code.
   */
  public static ControllerPins pins(
      RobotDefinition definition, List<SensorWiring> sensors, JointSetup setup) {
    String joint = setup.joint().label();
    ControllerPins pins = ControllerPins.NONE;
    // Its own pins first: what's wired to a pin is read through it. A CANdi's limit switch takes
    // a limit only where no pin's switch has, since a Talon reads one source for each.
    for (SensorWiring wiring : sensors) {
      Sensor.Switch sensor = switchOf(definition, wiring);
      if (sensor != null
          && wiring instanceof SensorWiring.OfSwitch onSwitch
          && onSwitch.input() instanceof SensorWiring.ControllerLimit limit
          && limit.joint().equals(joint)) {
        pins =
            pins.with(
                limit.direction(),
                new ControllerPins.OnPin(onSwitch.polarity(), ownLimit(sensor, joint)));
      }
    }
    for (SensorWiring wiring : sensors) {
      Sensor.Switch sensor = switchOf(definition, wiring);
      if (sensor != null
          && ownLimit(sensor, joint)
          && wiring.input() instanceof SensorWiring.Candi candi
          && readsCandi(setup, candi)) {
        Sensor.Switch.Limit limit = (Sensor.Switch.Limit) sensor.kind();
        boolean forward = limit.end() == Sensor.Switch.Limit.End.UPPER;
        if ((forward ? pins.forward() : pins.reverse()).isEmpty()) {
          pins =
              pins.with(
                  forward ? SensorWiring.Direction.FORWARD : SensorWiring.Direction.REVERSE,
                  new ControllerPins.OnCandi(candi.id(), candi.input()));
        }
      }
    }
    return pins;
  }

  /** The definition's switch a wiring is for, or null. */
  private static Sensor.@Nullable Switch switchOf(RobotDefinition definition, SensorWiring wiring) {
    for (Sensor each : definition.math().sensors()) {
      if (each instanceof Sensor.Switch found && found.label().equals(wiring.sensor())) {
        return found;
      }
    }
    return null;
  }

  /** Whether a switch is a joint's own limit switch. */
  private static boolean ownLimit(Sensor.Switch sensor, String joint) {
    return sensor.kind() instanceof Sensor.Switch.Limit limit
        && limit.joint().label().equals(joint);
  }

  /** Whether a joint's leading controller can read a CANdi as its limit: a Talon on its bus. */
  private static boolean readsCandi(JointSetup setup, SensorWiring.Candi candi) {
    return setup.wiring().isPresent()
        && setup.wiring().get().canBus().equals(candi.bus())
        && (setup.wiring().get().controller() == MechanismWiring.Controller.TALON_FX
            || setup.wiring().get().controller() == MechanismWiring.Controller.TALON_FXS);
  }

  /** An input as a sentence names it. */
  static String shown(SensorWiring.Input input) {
    return switch (input) {
      case SensorWiring.Port port -> "Systemcore's port " + port.port();
      case SensorWiring.Candi candi -> "CANdi " + candi.id() + "'s S" + candi.input();
      case SensorWiring.CanRange range -> "CANrange " + range.id();
      case SensorWiring.ControllerLimit limit ->
          limit.joint()
              + "'s controller's "
              + (limit.direction() == SensorWiring.Direction.FORWARD ? "forward" : "reverse")
              + " limit";
    };
  }
}
