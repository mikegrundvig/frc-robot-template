package frc.robot.subsystems.mechanisms;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import frc.robot.behavior.Gate;
import frc.robot.behavior.Refusals;
import frc.robot.lib.wiring.ControllerPins;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;

/**
 * The robot's switches and rangefinders, as its program reads them: the definition says what each
 * is and where, its wiring how it's connected on a robot. Each loop, after the joints, it reads
 * each through its IO ({@link SensorIO}), logged under {@code Sensors/<label>}; a switch on a
 * joint's motor controller's limit input is read by that joint, with the rest of it, from its
 * controller ({@link JointIO.JointIOInputs#forwardLimit}), and logged here the same. Behavior reads
 * them through the conditions the model names, as gates ({@link Joints#gate}):
 *
 * {@snippet class=frc.robot.examples.ExampleCommands region=sensor-gates}
 *
 * <p>A gate on a sensor waits, guards and binds as any gate does:
 *
 * {@snippet class=frc.robot.examples.ExampleCommands region=intake-one}
 *
 * <p>A sensor that's lost (a CAN device silent, a joint's controller silent for a switch on its
 * limit input, or one with no wiring on a robot) holds no condition either way: neither tripped nor
 * clear, neither nearer nor farther. A gate on it never opens, and says so ("hopper range lost"); a
 * move that waits on it times out.
 */
public final class Sensors {
  /**
   * One sensor, as read: through its IO, or, for a switch on a joint's limit input, by its joint.
   */
  private static final class Reading {
    final Sensor.Placed sensor;
    final @Nullable SensorIO io;
    final SensorIOInputsAutoLogged inputs = new SensorIOInputsAutoLogged();
    final String key;

    Reading(Sensor.Placed sensor, @Nullable SensorIO io) {
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
   * @param io each one's hardware: a port or a CAN device, the simulator's, or none; never asked
   *     for a switch on a joint's motor controller's limit input, which its joint reads ({@link
   *     #pinned})
   * @param scheduler the command scheduler its gates are registered with
   */
  Sensors(RobotDefinition definition, Function<Sensor.Placed, SensorIO> io, Scheduler scheduler) {
    this.scheduler = scheduler;
    List<Reading> built = new ArrayList<>();
    for (Sensor sensor : definition.math().sensors()) {
      if (sensor instanceof Sensor.Placed placed) {
        Reading reading = new Reading(placed, onPin(placed) != null ? null : io.apply(placed));
        built.add(reading);
        bySensor.put(placed, reading);
      }
    }
    readings = built.toArray(Reading[]::new);
  }

  /**
   * The limit input on a joint's motor controller a switch is on, whose joint reads it: null for a
   * sensor connected anywhere else, or nowhere.
   */
  public static SensorWiring.@Nullable ControllerLimit onPin(Sensor.Placed sensor) {
    return sensor instanceof Sensor.Switch onSwitch
            && onSwitch.wiring().isPresent()
            && onSwitch.wiring().get().input() instanceof SensorWiring.ControllerLimit limit
        ? limit
        : null;
  }

  /**
   * A switch on a joint's motor controller's limit input, as its joint read it from its controller
   * this loop: before the sensors are read, as the joints are.
   *
   * @param connected whether the controller answers: the switch is lost with it
   * @param tripped whether the controller reads it tripped, its polarity told to the controller
   */
  void pinned(Sensor.Switch sensor, boolean connected, boolean tripped) {
    SensorIO.SensorIOInputs inputs = reading(sensor).inputs;
    inputs.connected = connected;
    inputs.tripped = tripped;
  }

  /** Reads every sensor: once a loop, after the joints and before the command scheduler. */
  void read() {
    for (Reading reading : readings) {
      SensorIO io = reading.io;
      if (io != null) {
        io.updateInputs(reading.inputs);
      }
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

  /**
   * A gate on a condition, named: {@link Joints#gate(String, Condition)}'s for a sensor. One on a
   * sensor the robot doesn't have is refused as a joint's gate on a place not its own is ({@link
   * Refusals#gate}): thrown as the robot starts, and once it runs, a gate that holds false saying
   * why.
   *
   * @throws IllegalArgumentException for a sensor the robot doesn't have, as the robot starts
   */
  Gate gate(String name, Condition.OnSensor condition) {
    Reading reading = bySensor.get(condition.sensor());
    if (reading == null) {
      return Refusals.gate(scheduler, name, notOurs(condition.sensor()));
    }
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
      throw new IllegalArgumentException(notOurs(sensor));
    }
    return reading;
  }

  private static String notOurs(Sensor sensor) {
    return sensor.label() + " isn't one of the robot's sensors";
  }

  /**
   * What a joint's leading motor controller has on its limits, from the wiring its switches carry
   * ({@link #onLimit}): a switch on one of its pins, its polarity told and stopped on if it's a
   * limit switch; and, for a TalonFX or TalonFXS, its own limit switch on a CANdi input on its bus,
   * which it reads over CAN. A SPARK can't read a CANdi: its limit switch there stops it only
   * through the robot's code. The robot's IO for the joint is given these, and so is the
   * simulator's, which reads the same switches and stops where the controller would.
   */
  public static ControllerPins pins(RobotDefinition definition, Joint joint) {
    ControllerPins pins = ControllerPins.NONE;
    for (SensorWiring.Direction direction : SensorWiring.Direction.values()) {
      Sensor.Switch on = onLimit(definition, joint, direction);
      if (on == null) {
        continue;
      }
      SensorWiring.Connection wiring = on.wiring().orElseThrow();
      pins =
          pins.with(
              direction,
              wiring.input() instanceof SensorWiring.Candi candi
                  ? new ControllerPins.OnCandi(candi.id(), candi.input())
                  : new ControllerPins.OnPin(wiring.polarity(), ownLimit(on, joint.label())));
    }
    return pins;
  }

  /**
   * The switch a joint's leading motor controller reads as one of its limits, or null for none: the
   * switch on that limit pin of its own; else, for a TalonFX or TalonFXS, its own limit switch on a
   * CANdi input on its bus, as its reverse limit for the switch at its lower end and its forward
   * for the upper. What's wired to a pin is read through it, so a CANdi's switch takes a limit only
   * where no pin's switch has: a Talon reads one source for each.
   */
  public static Sensor.@Nullable Switch onLimit(
      RobotDefinition definition, Joint joint, SensorWiring.Direction direction) {
    String label = joint.label();
    for (Sensor sensor : definition.math().sensors()) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onPin(onSwitch) instanceof SensorWiring.ControllerLimit limit
          && limit.joint().equals(label)
          && limit.direction() == direction) {
        return onSwitch;
      }
    }
    Sensor.Switch.Limit.End end =
        direction == SensorWiring.Direction.FORWARD
            ? Sensor.Switch.Limit.End.UPPER
            : Sensor.Switch.Limit.End.LOWER;
    for (Sensor sensor : definition.math().sensors()) {
      if (sensor instanceof Sensor.Switch onSwitch
          && ownLimit(onSwitch, label)
          && ((Sensor.Switch.Limit) onSwitch.kind()).end() == end
          && onSwitch.wiring().isPresent()
          && onSwitch.wiring().get().input() instanceof SensorWiring.Candi candi
          && readsCandi(joint, candi)) {
        return onSwitch;
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
  private static boolean readsCandi(Joint joint, SensorWiring.Candi candi) {
    List<MechanismWiring.Device> controllers =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    return !controllers.isEmpty()
        && controllers.get(0).bus().equals(candi.bus())
        && (controllers.get(0).controller() == MechanismWiring.Controller.TALON_FX
            || controllers.get(0).controller() == MechanismWiring.Controller.TALON_FXS);
  }
}
