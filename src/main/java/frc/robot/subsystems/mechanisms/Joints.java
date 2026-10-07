package frc.robot.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.behavior.Gate;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;

/**
 * Every driven joint of the robot, each a v3 {@link Mechanism} the template builds from the robot's
 * definition and the joint's setup ({@link JointMechanism}). The composition root builds it once;
 * behavior code looks its joints up by the joints themselves, never by a label:
 *
 * <pre>{@code
 * JointMechanism pivot = joints.of(PIVOT), roller = joints.of(ROLLER);
 * joints.goTo(AMP)       // requires exactly the joints the configuration names
 * joints.run(FEEDING)    // holds it, outputs included, until canceled
 * joints.isAt(STOWED)    // a gate: every joint at its place
 * }</pre>
 *
 * <p>It enforces the definition's interlocks beneath every command, each on the joint it restricts
 * ({@link JointMechanism}); one about a joint the template doesn't drive can't be, and is refused.
 *
 * <p>Each loop it reads every joint before the command scheduler runs ({@link #read}), and applies
 * what the commands asked after it ({@link #write}), so a goal set this loop is applied this loop.
 */
public final class Joints {
  private final JointMechanism[] joints;
  private final Map<Joint, JointMechanism> byJoint = new HashMap<>();
  private final RobotDefinition definition;
  private final Sensors sensors;

  /** The command scheduler its mechanisms and gates are registered with. */
  private final Scheduler scheduler;

  /**
   * @param definition what the robot is
   * @param setups how each driven joint is driven and wired: one for every joint the definition's
   *     motors drive ({@link JointSetup#problems})
   * @param io each joint's hardware: its motor controllers, the simulator's, or none, in replay
   * @param scheduler the command scheduler its mechanisms are registered with
   * @throws IllegalArgumentException if the setups don't match the definition, or an interlock is
   *     about a joint the template doesn't drive
   */
  public Joints(
      RobotDefinition definition,
      List<JointSetup> setups,
      Function<JointSetup, JointIO> io,
      Scheduler scheduler) {
    this(definition, setups, setup -> setup::control, io, scheduler);
  }

  /**
   * The same, each joint's control as the pit tunes it ({@link JointMechanism#tuning}): read each
   * loop, its gains and current limits handed to its motor controller as they change.
   *
   * @param tuned each setup's control, as tuned
   */
  public Joints(
      RobotDefinition definition,
      List<JointSetup> setups,
      Function<JointSetup, Supplier<MechanismControl>> tuned,
      Function<JointSetup, JointIO> io,
      Scheduler scheduler) {
    this(definition, setups, tuned, io, sensor -> new SensorIO() {}, scheduler);
  }

  /**
   * The same, with the robot's switches and rangefinders ({@link #sensors}).
   *
   * @param sensorIo each sensor's hardware: a port or a CAN device, the simulator's, or none
   */
  public Joints(
      RobotDefinition definition,
      List<JointSetup> setups,
      Function<JointSetup, Supplier<MechanismControl>> tuned,
      Function<JointSetup, JointIO> io,
      Function<Sensor.Placed, SensorIO> sensorIo,
      Scheduler scheduler) {
    this(definition, setups, tuned, io, setup -> new AbsoluteEncoderIO() {}, sensorIo, scheduler);
  }

  /**
   * The same, with the absolute encoders the robot reads itself ({@link JointSetup#readsAbsolute}):
   * each joint with one waits for its first reading, and tells its motor controller where it is
   * from it.
   *
   * @param absoluteIo each such encoder's hardware: a Systemcore port, the simulator's, or none;
   *     asked for after the joint's own
   */
  public Joints(
      RobotDefinition definition,
      List<JointSetup> setups,
      Function<JointSetup, Supplier<MechanismControl>> tuned,
      Function<JointSetup, JointIO> io,
      Function<JointSetup, AbsoluteEncoderIO> absoluteIo,
      Function<Sensor.Placed, SensorIO> sensorIo,
      Scheduler scheduler) {
    String problems = JointSetup.problems(definition, setups);
    if (problems != null) {
      throw new IllegalArgumentException(problems);
    }
    String unenforceable = unenforceable(definition);
    if (unenforceable != null) {
      throw new IllegalArgumentException(unenforceable);
    }
    this.definition = definition;
    this.scheduler = scheduler;
    RobotMath math = definition.math();
    List<JointMechanism> built = new ArrayList<>();
    for (Joint joint : definition.joints()) {
      for (JointSetup setup : setups) {
        if (setup.joint().equals(joint)) {
          JointMechanism mechanism =
              new JointMechanism(math, setup, tuned.apply(setup), io.apply(setup), scheduler);
          if (JointSetup.readsAbsolute(setup) != null) {
            mechanism.seedFrom(absoluteIo.apply(setup));
          }
          built.add(mechanism);
          byJoint.put(joint, mechanism);
        }
      }
    }
    joints = built.toArray(JointMechanism[]::new);
    sensors = new Sensors(definition, sensorIo, this::pinned, scheduler);
    for (JointMechanism joint : joints) {
      List<Sensor.Switch> limits = new ArrayList<>();
      for (Sensor sensor : definition.math().sensors()) {
        if (sensor instanceof Sensor.Switch onSwitch
            && onSwitch.kind() instanceof Sensor.Switch.Limit limit
            && limit.joint().equals(joint.joint())) {
          limits.add(onSwitch);
        }
      }
      joint.stopAt(limits, sensors);
    }
    for (Interlock interlock : definition.everyInterlock()) {
      enforced(interlock.joint())
          .restrict(interlock, interlock.other().map(this::enforced).orElse(null));
    }
  }

  /**
   * Why the template can't enforce a robot's interlocks, or null when it can enforce every one: an
   * interlock about a joint it doesn't drive. A robot's configuration asks this as it's built,
   * before anything of the robot is made.
   */
  public static @Nullable String unenforceable(RobotDefinition definition) {
    RobotMath math = definition.math();
    List<String> problems = new ArrayList<>();
    for (Interlock interlock : definition.everyInterlock()) {
      List<Joint> joints = new ArrayList<>(List.of(interlock.joint()));
      interlock.other().ifPresent(joints::add);
      for (Joint joint : joints) {
        String why = JointSetup.whyNotDriven(math, joint);
        if (why != null) {
          problems.add(
              "\""
                  + interlock
                  + "\" can't be enforced: "
                  + joint.label()
                  + " has no mechanism, as "
                  + why);
          break;
        }
      }
    }
    return problems.isEmpty() ? null : String.join("; ", problems);
  }

  /** The mechanism of a joint an interlock speaks of, which has one ({@link #unenforceable}). */
  private JointMechanism enforced(Joint joint) {
    return Objects.requireNonNull(byJoint.get(joint), joint.label());
  }

  /**
   * A joint's mechanism.
   *
   * @throws IllegalArgumentException for a joint that isn't the robot's, or one the template
   *     doesn't drive, saying why
   */
  public JointMechanism of(Joint joint) {
    JointMechanism mechanism = byJoint.get(joint);
    if (mechanism != null) {
      return mechanism;
    }
    if (!definition.joints().contains(joint)) {
      throw new IllegalArgumentException(
          joint.label() + " isn't one of " + definition.name() + "'s joints");
    }
    throw new IllegalArgumentException(
        joint.label() + " has no mechanism: " + JointSetup.whyNotDriven(definition.math(), joint));
  }

  /**
   * Several joints' mechanisms, as one requirement: {@code Command.requiring(joints.all(PIVOT,
   * ROLLER))}: the joints a command drives together.
   *
   * @throws IllegalArgumentException as {@link #of(Joint)} does, for any of them
   */
  public List<Mechanism> all(Joint... joints) {
    List<Mechanism> mechanisms = new ArrayList<>();
    for (Joint joint : joints) {
      mechanisms.add(of(joint));
    }
    return List.copyOf(mechanisms);
  }

  /**
   * Moves a configuration's joints to their named places together, each on its own profile, and
   * ends when every one is there; it requires exactly the joints it names. A joint it runs at an
   * output is there at once for a voltage or a duty cycle, and within its band for a speed. One
   * that names a joint the template doesn't drive is refused whenever it would start, saying why:
   * the robot still runs, and the startup's doubts name it.
   */
  public Move goTo(Configuration configuration) {
    return Move.configuration(
        configuration, mechanisms(configuration), true, undriven(definition, configuration));
  }

  /**
   * Holds a configuration's joints at their named places and outputs until canceled: {@code
   * driver.rightTrigger().whileTrue(joints.run(FEEDING))}. Refused, as {@link #goTo(Configuration)}
   * is, if it names a joint the template doesn't drive.
   */
  public Move run(Configuration configuration) {
    return Move.configuration(
        configuration, mechanisms(configuration), false, undriven(definition, configuration));
  }

  /**
   * A gate on a configuration: every joint it names at its named place, or running at its named
   * output, "at stowed". Never true for one that names a joint the template doesn't drive.
   */
  public Gate isAt(Configuration configuration) {
    List<Gate> terms = new ArrayList<>();
    for (Condition.At target : configuration.targets()) {
      JointMechanism mechanism = byJoint.get(target.joint());
      terms.add(
          mechanism != null
              ? target.isOutput()
                  ? mechanism.isAt(target.joint().output(target.name()))
                  : mechanism.isAt(target.joint().position(target.name()))
              : Gate.of(scheduler, target.joint().label() + " driven", () -> false));
    }
    return Gate.all("at " + configuration.name(), terms.toArray(Gate[]::new));
  }

  /**
   * Why a configuration can't run, or null when it can: the joints it names that the template
   * doesn't drive, each with why ({@link JointSetup#whyNotDriven}). The startup's doubts list
   * these.
   */
  public static @Nullable String cantRun(RobotDefinition definition, Configuration configuration) {
    String undriven = undriven(definition, configuration);
    return undriven == null ? null : configuration.name() + " can't run: " + undriven;
  }

  /** The joints a configuration names that the template doesn't drive, each with why; or null. */
  private static @Nullable String undriven(
      RobotDefinition definition, Configuration configuration) {
    RobotMath math = definition.math();
    List<String> undriven = new ArrayList<>();
    for (Condition.At target : configuration.targets()) {
      String why = JointSetup.whyNotDriven(math, target.joint());
      if (why != null) {
        undriven.add(target.joint().label() + " has no mechanism: " + why);
      }
    }
    return undriven.isEmpty() ? null : String.join("; ", undriven);
  }

  /** Every joint's mechanism, in the definition's order. */
  public List<JointMechanism> all() {
    return List.of(joints);
  }

  /**
   * Where every joint is as of this loop, by joint: what localization needs of a camera riding one.
   * A joint that doesn't know where it is yet isn't in it.
   */
  public Map<Joint, Double> positions() {
    Map<Joint, Double> positions = new LinkedHashMap<>();
    for (JointMechanism joint : joints) {
      if (Double.isFinite(joint.position())) {
        positions.put(joint.joint(), joint.position());
      }
    }
    return positions;
  }

  /**
   * A switch on a joint's limit input, bound to that joint: read from its controller with the rest
   * of the joint, a loop's reading behind none. Any other sensor's hardware, as it is.
   *
   * @throws IllegalArgumentException for a joint the template doesn't drive
   */
  private SensorIO pinned(SensorIO io) {
    if (!(io instanceof SensorIO.OnPin pin)) {
      return io;
    }
    for (JointMechanism joint : joints) {
      if (joint.joint().label().equals(pin.joint())) {
        return new SensorIO() {
          @Override
          public void updateInputs(SensorIOInputs inputs) {
            joint.readPin(pin.direction(), inputs);
          }
        };
      }
    }
    throw new IllegalArgumentException(
        "a switch is on " + pin.joint() + "'s limit input, and the template doesn't drive it");
  }

  /**
   * What the robot's limit switches are doubted for this loop: each one reading tripped nowhere
   * near where it trips, or clear well past it, with its joint homed. Empty while they agree.
   */
  public List<String> switchDoubts() {
    List<String> doubts = new ArrayList<>();
    for (JointMechanism joint : joints) {
      doubts.addAll(joint.switchDoubts());
    }
    return doubts;
  }

  /** The robot's switches and rangefinders, read with the joints. */
  public Sensors sensors() {
    return sensors;
  }

  /** Reads every joint, then every sensor: once a loop, before the command scheduler runs. */
  public void read(long nowNanos, boolean enabled) {
    for (JointMechanism joint : joints) {
      joint.read(nowNanos, enabled);
    }
    sensors.read();
    for (JointMechanism joint : joints) {
      joint.sensed();
    }
  }

  /** Applies what the commands asked of each joint, or its idle: once a loop, after them. */
  public void write() {
    for (JointMechanism joint : joints) {
      joint.write();
    }
  }

  /** Each target's joint's mechanism, in order: null for one the template doesn't drive. */
  private List<@Nullable JointMechanism> mechanisms(Configuration configuration) {
    List<@Nullable JointMechanism> mechanisms = new ArrayList<>();
    for (Condition.At target : configuration.targets()) {
      mechanisms.add(byJoint.get(target.joint()));
    }
    return mechanisms;
  }
}
