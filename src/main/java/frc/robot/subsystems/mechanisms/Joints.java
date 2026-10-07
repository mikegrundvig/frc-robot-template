package frc.robot.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismRotations;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Transmission;
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
 * definition, the joint's block saying how it's driven and wired ({@link JointMechanism}). The
 * composition root builds it once; behavior code looks its joints up by the joints themselves,
 * never by a label:
 *
 * {@snippet :
 * JointMechanism pivot = joints.of(PIVOT);
 * joints.goTo(STOWED)    // requires exactly the joints the configuration names
 * joints.run(FEEDING)    // holds it, outputs included, until canceled
 * joints.isAt(STOWED)    // a gate: every joint at its place
 * }
 *
 * <p>It enforces the definition's interlocks beneath every command, each on the joint it restricts
 * ({@link JointMechanism}); one about a joint the template doesn't drive can't be, and is refused.
 *
 * <p>Each loop it reads every joint before the command scheduler runs ({@link #read}), and applies
 * what the commands asked after it ({@link #write}), so a goal set this loop is applied this loop.
 */
public final class Joints {
  /**
   * What the joints read and drive, made for the mode the program runs in: on a robot, the devices
   * each block's wiring names; in the simulator, its own; in a replay, none, since the log has
   * every reading. The joints ask for each driven joint's first, then the absolute encoders the
   * robot reads itself, then the sensors, then the PDH, each once as they're built: the simulator's
   * absolute encoders and limit switches move with its joints.
   *
   * <p>A test that needs only its joints' hardware gives it as a lambda, {@code joint -> io}, the
   * rest none. The robot's hardware ({@code frc.robot.hardware.Hardware}) gives every one.
   */
  @FunctionalInterface
  public interface Hardware {
    /** A driven joint's motor controllers: its TalonFX's or SPARK's, the simulator's, or none. */
    JointIO joint(Joint joint);

    /**
     * A joint's absolute encoder, where the robot reads it itself ({@link #readsAbsolute}): on a
     * Systemcore port, the simulator's, or none.
     */
    default AbsoluteEncoderIO absoluteEncoder(Joint joint) {
      return new AbsoluteEncoderIO() {};
    }

    /** A switch or a rangefinder: on a port or a CAN device, the simulator's, or none. */
    default SensorIO sensor(Sensor.Placed sensor) {
      return new SensorIO() {};
    }

    /** The PDH, read with the joints ({@link #power}): the robot's, the simulator's, or none. */
    default PdhIO pdh() {
      return new PdhIO() {};
    }
  }

  private final JointMechanism[] joints;
  private final Map<Joint, JointMechanism> byJoint = new HashMap<>();
  private final RobotDefinition definition;
  private final Sensors sensors;
  private final Power power;

  /** The command scheduler its mechanisms and gates are registered with. */
  private final Scheduler scheduler;

  /**
   * Every driven joint's mechanism, with the robot's switches and rangefinders ({@link #sensors})
   * and its PDH ({@link #power}). A joint whose absolute encoder the robot reads itself ({@link
   * #readsAbsolute}) waits for its first reading, and tells its motor controller where it is from
   * it.
   *
   * @param definition what the robot is, each joint's block saying how it's driven and wired
   * @param hardware what each joint, sensor and the PDH read and drive, for the mode the program
   *     runs in
   * @param tunings each joint's control, read each loop, its gains and current limits handed to its
   *     motor controller as they change: as the pit tunes it ({@link JointMechanism#tuning}), or as
   *     its block commits it ({@link #untuned})
   * @param scheduler the command scheduler its mechanisms are registered with
   * @throws IllegalArgumentException if the template can't drive the definition's joints ({@link
   *     #problems}), or an interlock is about a joint the template doesn't drive
   */
  public Joints(
      RobotDefinition definition,
      Hardware hardware,
      Function<Joint, Supplier<MechanismControl>> tunings,
      Scheduler scheduler) {
    List<Finding> refused = new ArrayList<>(problems(definition));
    refused.addAll(unenforceable(definition));
    if (!refused.isEmpty()) {
      throw new IllegalArgumentException(
          String.join("; ", refused.stream().map(Finding::message).toList()));
    }
    this.definition = definition;
    this.scheduler = scheduler;
    RobotMath math = definition.math();
    List<JointMechanism> built = new ArrayList<>();
    for (Joint joint : driven(definition)) {
      JointMechanism mechanism =
          new JointMechanism(math, joint, tunings.apply(joint), hardware.joint(joint), scheduler);
      if (readsAbsolute(joint) != null) {
        mechanism.seedFrom(hardware.absoluteEncoder(joint));
      }
      built.add(mechanism);
      byJoint.put(joint, mechanism);
    }
    joints = built.toArray(JointMechanism[]::new);
    sensors = new Sensors(definition, hardware::sensor, this::pinned, scheduler);
    power = new Power(definition, hardware.pdh());
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

  /** Each joint's control as its block commits it, never tuned: for a robot nobody tunes. */
  public static Function<Joint, Supplier<MechanismControl>> untuned() {
    return joint -> () -> ControllerSettings.control(joint);
  }

  /**
   * Each joint the template drives, in the definition's order: one with a control in its block,
   * turned by one drive of motors ({@link #whyNotDriven}). Each is a mechanism of its own.
   */
  public static List<Joint> driven(RobotDefinition definition) {
    RobotMath math = definition.math();
    List<Joint> driven = new ArrayList<>();
    for (Joint joint : definition.joints()) {
      if (joint.driven().control().isPresent() && whyNotDriven(math, joint) == null) {
        driven.add(joint);
      }
    }
    return driven;
  }

  /**
   * Why the template can't drive a robot's joints as their blocks say, each about its joint;
   * nothing when it can: a joint with a control it doesn't drive yet ({@link #whyNotDriven}), and
   * settings its motor controllers would refuse, as the robot would say it at startup ({@link
   * ControllerSettings#findings}), wired or not. A robot's configuration asks this as it's built,
   * and the definition has already refused what its blocks say of themselves.
   */
  public static List<Finding> problems(RobotDefinition definition) {
    RobotMath math = definition.math();
    List<Finding> problems = new ArrayList<>();
    for (Joint joint : definition.joints()) {
      if (joint.driven().control().isEmpty()) {
        continue;
      }
      String why = whyNotDriven(math, joint);
      if (why != null) {
        problems.add(
            Finding.refused(
                joint.label(),
                joint.label() + " has a control, and the template doesn't drive it: " + why));
      } else {
        ControllerSettings.findings(math, joint).stream()
            .filter(Finding::refused)
            .forEach(problems::add);
      }
    }
    return problems;
  }

  /**
   * Why the template doesn't drive a joint, or null when it does: one drive of motors turning a
   * shaft. Any motor of the catalog: in the simulator as it is, on a robot through the controller
   * its block names ({@link com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller}). A
   * joint that follows another is carried by its leader's moves; a joint driven by a servo, or by
   * two drives (a differential's pair, a hood's two servos), isn't driven by the template yet.
   *
   * @throws IllegalArgumentException for a joint that isn't the robot's
   */
  public static @Nullable String whyNotDriven(RobotMath math, Joint joint) {
    Joint leader = math.leaderOf(joint);
    if (leader != null) {
      return "it follows " + leader.label() + ", whose moves carry it";
    }
    List<Drive> drives = math.drivesOf(joint);
    if (drives.isEmpty()) {
      List<String> movers = new ArrayList<>();
      for (Drive drive : math.drives()) {
        if (math.leverage(drive, joint, math.references()) != 0) {
          movers.add(drive.label());
        }
      }
      return movers.isEmpty()
          ? "nothing drives it"
          : "it's moved by "
              + String.join(" and ", movers)
              + ", and the template drives only motors turning a shaft so far";
    }
    if (drives.size() > 1) {
      return "it's driven by "
          + String.join(" and ", drives.stream().map(Drive::label).toList())
          + " together, which the template doesn't drive yet";
    }
    Drive drive = drives.get(0);
    if (!(drive.actuator() instanceof Actuator.Motors)
        || drive.transmission() instanceof Transmission.Between) {
      return drive.label() + " isn't motors turning a shaft";
    }
    try {
      MechanismRotations.of(joint, drive);
    } catch (IllegalArgumentException e) {
      return String.valueOf(e.getMessage());
    }
    return null;
  }

  /**
   * A joint's absolute encoder, when the robot reads it itself and tells the motor where the joint
   * is from it: on a Systemcore port, or unwired, read only by the simulator. Null for none, or one
   * its motor controller reads, a CANcoder or a SPARK's data port, or for a joint whose control
   * names a homing recipe, which it homes by instead.
   */
  public static Driven.@Nullable AbsoluteEncoder readsAbsolute(Joint joint) {
    Driven.AbsoluteEncoder encoder = joint.driven().absoluteEncoder().orElse(null);
    return encoder == null
            || encoder.readByController()
            || joint.driven().control().flatMap(MechanismControl::homing).isPresent()
            || !joint.holdsPositions()
        ? null
        : encoder;
  }

  /**
   * Why the template can't enforce a robot's interlocks, each about the joint restricted; nothing
   * when it can enforce every one: an interlock about a joint it doesn't drive. A robot's
   * configuration asks this as it's built, before anything of the robot is made.
   */
  public static List<Finding> unenforceable(RobotDefinition definition) {
    RobotMath math = definition.math();
    List<Finding> problems = new ArrayList<>();
    for (Interlock interlock : definition.everyInterlock()) {
      List<Joint> joints = new ArrayList<>(List.of(interlock.joint()));
      interlock.other().ifPresent(joints::add);
      for (Joint joint : joints) {
        String why = whyNotDriven(math, joint);
        if (why != null) {
          problems.add(
              Finding.refused(
                  interlock.joint().label(),
                  "\""
                      + interlock
                      + "\" can't be enforced: "
                      + joint.label()
                      + " has no mechanism, as "
                      + why));
          break;
        }
      }
    }
    return problems;
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
        joint.label() + " has no mechanism: " + whyNotDriven(definition.math(), joint));
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
                  ? mechanism.isAt(mechanism.joint().output(target.name()))
                  : mechanism.isAt(mechanism.joint().position(target.name()))
              : Gate.of(scheduler, target.joint().label() + " driven", () -> false));
    }
    return Gate.all("at " + configuration.name(), terms.toArray(Gate[]::new));
  }

  /**
   * Why a configuration can't run, or null when it can: the joints it names that the template
   * doesn't drive, each with why ({@link #whyNotDriven}). The startup's doubts list these.
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
      String why = whyNotDriven(math, target.joint());
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

  /**
   * The motor controllers that read as drawing nothing on their PDH channels while their joints
   * work, as of this loop, each said with what to check: a tripped breaker, unplugged power, or a
   * channel its block names wrong. Empty while each draws as it should.
   */
  public List<String> deadChannels() {
    List<String> dead = new ArrayList<>();
    for (JointMechanism joint : joints) {
      dead.addAll(joint.deadChannels());
    }
    return dead;
  }

  /** The robot's switches and rangefinders, read with the joints. */
  public Sensors sensors() {
    return sensors;
  }

  /** The robot's PDH, read with the joints: each channel's current, and each joint's. */
  public Power power() {
    return power;
  }

  /**
   * Reads every joint, then every sensor, then the PDH: once a loop, before the command scheduler
   * runs.
   */
  public void read(long nowNanos, boolean enabled) {
    for (JointMechanism joint : joints) {
      joint.read(nowNanos, enabled);
    }
    sensors.read();
    power.read();
    for (JointMechanism joint : joints) {
      joint.sensed();
      joint.powered(power);
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
