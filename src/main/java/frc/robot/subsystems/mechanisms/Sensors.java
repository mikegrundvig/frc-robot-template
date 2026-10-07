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
import frc.robot.lib.wiring.ControllerPins;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Scheduler;

/**
 * The robot's switches and rangefinders, as its program reads them: the definition says what each
 * is and where, its wiring how it's connected on a robot. Each loop, after the joints, it reads
 * each through its IO ({@link SensorIO}), logged under {@code Sensors/<label>}; behavior reads them
 * through the conditions the model names, as gates:
 *
 * {@snippet :
 * Gate coralIn = sensors.gate("coral in claw", CLAW_BEAM.tripped());
 * roller.runUntil(INTAKING, coralIn)                // ends ARRIVED the loop it trips
 * co.waitUntil(coralIn);                            // holds a step
 * coralIn.onTrue(countPiece());                     // runs a command as it trips
 * }
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
   * What a joint's leading motor controller has on its limits, from the wiring its switches carry:
   * a switch on one of its pins, stopped on if it's a limit switch; and, for a TalonFX or TalonFXS,
   * its own limit switch on a CANdi input on its bus, which it reads over CAN, as its reverse limit
   * for the switch at its lower end and its forward for the upper. A SPARK can't read a CANdi: its
   * limit switch there stops it only through the robot's code.
   */
  public static ControllerPins pins(RobotDefinition definition, Joint joint) {
    String label = joint.label();
    ControllerPins pins = ControllerPins.NONE;
    // Its own pins first: what's wired to a pin is read through it. A CANdi's limit switch takes
    // a limit only where no pin's switch has, since a Talon reads one source for each.
    for (Sensor sensor : definition.math().sensors()) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.wiring().isPresent()
          && onSwitch.wiring().get().input() instanceof SensorWiring.ControllerLimit limit
          && limit.joint().equals(label)) {
        pins =
            pins.with(
                limit.direction(),
                new ControllerPins.OnPin(
                    onSwitch.wiring().get().polarity(), ownLimit(onSwitch, label)));
      }
    }
    for (Sensor sensor : definition.math().sensors()) {
      if (sensor instanceof Sensor.Switch onSwitch
          && ownLimit(onSwitch, label)
          && onSwitch.wiring().isPresent()
          && onSwitch.wiring().get().input() instanceof SensorWiring.Candi candi
          && readsCandi(joint, candi)) {
        Sensor.Switch.Limit limit = (Sensor.Switch.Limit) onSwitch.kind();
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
