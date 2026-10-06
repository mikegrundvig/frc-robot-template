package frc.robot.subsystems.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.behavior.Gate;
import frc.robot.behavior.Outcome;
import frc.robot.lib.phoenix.MechanismControl;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.lib.wiring.SensorWiring;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

/**
 * One driven joint, as a Commands v3 {@link Mechanism}: the template builds one for each joint the
 * robot's motors drive ({@link Joints}), from the robot's definition and the joint's setup, and
 * behavior code looks it up through the robot's parts ({@code joints.of(parts.pivot())}). Nobody
 * writes one.
 *
 * <p>Its commands are pre-built, each a {@link Move} that ends with an outcome:
 *
 * <ul>
 *   <li>{@link #goTo(String)} one of its named positions, profiled from where it is, and {@code
 *       goTo("out").at(0.9)} at a share of its cruise; {@link #goTo(Angle)} or {@link
 *       #goTo(Distance)} a computed one, inside its limits;
 *   <li>{@link #follow} a target that changes every loop, which it never stops chasing;
 *   <li>{@link #hold()} its goal, owning the joint;
 *   <li>{@link #run(String)} one of its named outputs, and {@link #runUntil} one until a condition
 *       holds;
 *   <li>{@link #home()} its homing recipe again.
 * </ul>
 *
 * <p>Its checks are {@link Gate}s, v3 triggers that log why they're false: {@link #isAt}, {@link
 * #isPast}, {@link #isBetween}, {@link #isReady}, {@link #isAtGoal}, {@link #isStalled}.
 *
 * <p><b>Idle.</b> A joint does what a command told it this loop, or else what it does idle: a joint
 * held at positions holds its goal (where its last move arrived, or where it came to a stop if it
 * was canceled or timed out), and one that only runs at outputs, a roller, stops. Idle is the
 * joint's own, so a joint a command owns but drives no longer doesn't sag; a default command
 * replaces it, as v3's do.
 *
 * <p><b>Homing.</b> A joint held at positions knows where it is from its absolute encoder, or from
 * its control's homing recipe. Pushed into a hard stop, it homes on the first enable; moves wait
 * for it, and if it finds no stop it's faulted, and its moves are refused.
 *
 * <p><b>Interlocks</b> from the robot's definition apply beneath every command, as the last thing
 * before the joint's output: no command, binding or auto skips one. While a {@code runsOnlyWhen}
 * condition doesn't hold, the joint holds where it is, or stops; while a {@code keepsOut} region is
 * closed, its target is held at the region's edge on the side it's on, so its profile stops it
 * there. A condition on a joint whose position isn't known (unhomed, faulted, silent) fails safe.
 * The command keeps its goal, and carries on when the interlock lets it.
 *
 * <p><b>Each loop</b>, the joint reads its sensors before the commands run ({@link Joints#read}),
 * and applies what they asked after ({@link Joints#write}), so a goal set this loop is applied this
 * loop. Moves are profiled here, on the robot, each from where the joint is as it starts, and
 * streamed to the motor controller as a position and velocity, with the profile's acceleration
 * times kA added; the motor controller adds its own kV, kS and gravity. Its readings, the motor
 * controller's, are AdvantageKit inputs, so a replay reproduces everything here. It logs under
 * {@code Joints/<label>}: {@code State}, {@code Command}, {@code Goal}, {@code At}, {@code HeldBy}
 * and {@code Outcome}, in the order to read them when it won't move.
 */
public final class JointMechanism implements Mechanism {
  /** How long a joint stays inside its band before a move to there has arrived, in seconds. */
  static final double DWELL_SECONDS = 0.04;

  /** A revolute joint's bands until its control says: 2° to arrive, 4° to stay. */
  static final MechanismControl.Bands ANGLE_BANDS = bands(Math.toRadians(2), Math.toRadians(4));

  /** A prismatic joint's: 1 cm to arrive, 2 cm to stay. */
  static final MechanismControl.Bands LENGTH_BANDS = bands(0.01, 0.02);

  /** A speed's, as shares of it: 3% to arrive, 6% to stay. */
  static final MechanismControl.Bands SPEED_BANDS = bands(0.03, 0.06);

  /**
   * A joint whose control gives no profile cruises at this share of its free speed: slow enough to
   * be safe on a first enable. Give it a profile to go faster.
   */
  static final double DEFAULT_CRUISE_SHARE = 0.25;

  /** ...and reaches its cruise in this long, in seconds. */
  static final double DEFAULT_RAMP_SECONDS = 0.25;

  /** Homing into a hard stop: the joint has found it once it has moved slower than this share... */
  static final double STILL_SHARE = 0.02;

  /** ...of its free speed for this long. */
  static final long STILL_NANOS = 250_000_000L;

  /** A joint run at an output is stalled once it's moved slower than this share of the speed... */
  static final double STALL_SHARE = 0.1;

  /** ...the output asks for, for this long, in seconds. */
  static final double STALL_SECONDS = 0.25;

  /** Where a joint stands, as {@code State} logs it. */
  public enum State {
    /** Held at positions, and doesn't yet know where it is: it homes on the first enable. */
    UNHOMED,
    /** Finding where it is, now. */
    HOMING,
    /** It knows where it is, or only runs at outputs. */
    READY,
    /** Its homing failed: its moves are refused until it homes again ({@link #home()}). */
    FAULTED
  }

  /** What the joint is asked to do this loop. */
  enum Request {
    NONE,
    POSITION,
    VELOCITY,
    VOLTAGE,
    DUTY_CYCLE,
    NEUTRAL,
    HOMING
  }

  /**
   * An interlock on this joint, the joint its condition reads (none for a sensor's), and for a
   * region it keeps out of, the side the joint was last on: the only way it may leave the region,
   * should it ever be inside.
   */
  static final class Restriction {
    private final Interlock interlock;
    private final @Nullable JointMechanism other;
    private final String shown;

    /** Below the region (-1) or above it (1) when last outside it; 0 before it ever was. */
    private int side;

    Restriction(Interlock interlock, @Nullable JointMechanism other, String shown) {
      this.interlock = interlock;
      this.other = other;
      this.shown = shown;
    }

    Interlock interlock() {
      return interlock;
    }

    @Nullable JointMechanism other() {
      return other;
    }

    String shown() {
      return shown;
    }
  }

  private final Joint joint;
  private final String label;
  private final Scheduler scheduler;
  private final JointIO io;
  private final JointIOInputsAutoLogged inputs = new JointIOInputsAutoLogged();

  /** Whether it holds a position when idle: it has a range or named positions. */
  private final boolean positional;

  /** Whether it turns freely, so a goal is taken the short way round. */
  private final boolean wraps;

  /** Where it's driven, in its units: its soft limits, else its hard stops, else anywhere. */
  private final double lower;

  private final double upper;

  private final double cruise;
  private final double acceleration;
  private final double kA;
  private final double kV;
  private final double freeSpeed;

  /** How near a position it must be to be there, and may stray: in its units. */
  private final MechanismControl.Bands positionBands;

  /** How near a speed: shares of it. */
  private final MechanismControl.Bands speedBands;

  /** Its homing recipe, or null for one that needs none: its absolute encoder, or a roller. */
  private final MechanismControl.@Nullable Homing homing;

  /** Its named positions, lowest first, and how {@code At} reads between and beyond them. */
  private final List<Position> places;

  private final String[] atPlace;
  private final String[] betweenPlaces;
  private final String belowPlaces;
  private final String abovePlaces;

  private final List<Restriction> restrictions = new ArrayList<>();

  private final String inputsKey;
  private final String stateKey;
  private final String commandKey;
  private final String goalKey;
  private final String goalNameKey;
  private final String atKey;
  private final String heldByKey;

  /** Where what its limit switches are doubted for is logged. */
  private final String switchDoubtsKey;

  private final String setpointKey;
  private final String setpointVelocityKey;
  private final String outcomeKey;

  /** This loop: its count, when it started, and whether the robot is enabled. */
  private long loop;

  private long nowNanos;
  private boolean enabled;
  private boolean read;

  private State state = State.READY;
  private String stateShown = State.READY.name();

  /** When its homing began, and when the joint last stopped moving, in nanoseconds; -1 for not. */
  private long homingSince = -1;

  private long stillSince = -1;

  /** Whether it found its stop this loop: its new position is read from the next. */
  private boolean justHomed;

  /** What a command asked of it this loop, if {@link #requestLoop} is this loop. */
  private Request request = Request.NONE;

  private double requestValue;
  private double requestFraction = 1;
  private long requestLoop = -1;
  private @Nullable Command requester;
  private @Nullable String requestName;

  /** What it applied last loop, the value it was asked for, and the name it was asked by. */
  private Request applied = Request.NONE;

  private double appliedValue = Double.NaN;
  private @Nullable String appliedName;

  /** The profile, at a share of the cruise, and where it has the joint now. */
  private TrapezoidProfile profile;

  private double profileFraction = 1;
  private final TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private final TrapezoidProfile.State goal = new TrapezoidProfile.State();

  /** Whether the next position it's driven to starts its profile from where it is measured. */
  private boolean reseed = true;

  /** Whether idle holds a position, and which: NaN until the joint is first read. */
  private boolean holds;

  private double holdPosition = Double.NaN;

  /** Where a {@code runsOnlyWhen} interlock holds it, while one does; NaN while none does. */
  private double heldAt = Double.NaN;

  /** The interlocks holding it this loop, as logged; empty for none. */
  private String heldBy = "";

  /** Its limit switches, and the sensors that read them: none until it's told. */
  private List<Sensor.Switch> limitSwitches = List.of();

  private @Nullable Sensors sensors;

  /** How long a limit switch must disagree with where the joint is before it's doubted. */
  private static final long DOUBT_NANOS = 250_000_000L;

  /** What each limit switch is doubted for, in their order: empty while it agrees. */
  private String[] doubts = new String[0];

  /** Since when each has disagreed, in nanoseconds: -1 while it agrees. */
  private long[] doubtedSince = new long[0];

  /** Whether homing to its switch is backing off it first, having started on it. */
  private boolean backingOff;

  /** Whether homing to its switch has yet to see the switch: it decides whether to back off. */
  private boolean homingFresh;

  /** Where it read, homing toward its switch, the loop before: NaN before it's heading in. */
  private double homingLast = Double.NaN;

  /** Where it was as a limit switch it's pushed toward last tripped: held there, NaN when not. */
  private double stoppedAt = Double.NaN;

  /** How the last move on it ended, as logged. */
  private String outcome = "";

  /**
   * Its control as the pit tunes it, read each loop; and the one its motor controller last took.
   */
  private final Supplier<MechanismControl> tuned;

  private MechanismControl retuned;

  /**
   * A joint's control as the pit tunes it: its feedback and friction gains and its motors' current
   * limits, each within what's safe to dial in on a running robot. Its feedforward, profile and
   * named positions aren't tunable yet: optional and listed values are beyond what {@link
   * LiveConfig} reaches.
   *
   * @param label the joint's, which names its tuning
   * @param committed its control as written in {@code Robots.java}
   */
  public static LiveConfig<MechanismControl> tuning(String label, MechanismControl committed) {
    MechanismControl.Gains gains = committed.gains();
    MechanismControl.CurrentLimits limits = committed.currentLimits();
    double stator = limits.stator().in(Amps);
    double supply = limits.supply().in(Amps);
    return LiveConfig.of(label, committed)
        // Volts per radian (or meter), and per radian (or meter) per second: up to ten times what's
        // committed, which tells the joint's scale.
        .number("gains.kP", Limits.between(0, Math.max(10 * gains.kP(), 1)))
        .number("gains.kD", Limits.between(0, Math.max(10 * gains.kD(), 1)))
        .number("gains.kS", Limits.between(0, Math.max(gains.kS(), 2))) // volts
        .measure(
            "currentLimits.stator", Amps.of(Math.min(5, stator)), Amps.of(Math.max(120, stator)))
        .measure(
            "currentLimits.supply", Amps.of(Math.min(5, supply)), Amps.of(Math.max(80, supply)));
  }

  JointMechanism(
      RobotMath math,
      JointSetup setup,
      Supplier<MechanismControl> tuned,
      JointIO io,
      Scheduler scheduler) {
    joint = setup.joint();
    this.tuned = tuned;
    retuned = setup.control();
    label = joint.label();
    this.scheduler = scheduler;
    this.io = io;
    MechanismControl control = setup.control();
    positional = JointSetup.holdsPositions(joint);
    holds = positional;
    wraps = joint instanceof Joint.Revolute && joint.range().isEmpty();
    if (control.softLimits().isPresent()) {
      lower = control.softLimits().get().lower();
      upper = control.softLimits().get().upper();
    } else if (joint.range().isPresent()) {
      lower = joint.range().get().lower();
      upper = joint.range().get().upper();
    } else {
      lower = Double.NEGATIVE_INFINITY;
      upper = Double.POSITIVE_INFINITY;
    }
    freeSpeed = math.freeSpeed(joint);
    cruise =
        control
            .profile()
            .map(MechanismControl.Profile::cruiseVelocity)
            .orElse(DEFAULT_CRUISE_SHARE * freeSpeed);
    acceleration =
        control
            .profile()
            .map(MechanismControl.Profile::acceleration)
            .orElse(DEFAULT_CRUISE_SHARE * freeSpeed / DEFAULT_RAMP_SECONDS);
    RobotMath.Gains seed = math.idealGains(joint);
    kA = control.gains().kA().orElse(seed.kA());
    kV = control.gains().kV().orElse(seed.kV());
    MechanismControl.Bands kindBands = joint instanceof Joint.Revolute ? ANGLE_BANDS : LENGTH_BANDS;
    positionBands = positional ? control.bands().orElse(kindBands) : kindBands;
    speedBands = positional ? SPEED_BANDS : control.bands().orElse(SPEED_BANDS);
    homing = positional ? control.homing().orElse(null) : null;
    profile = profile(1);
    places =
        joint.positions().stream().sorted(Comparator.comparingDouble(Position::value)).toList();
    atPlace = new String[places.size()];
    betweenPlaces = new String[Math.max(0, places.size() - 1)];
    for (int i = 0; i < places.size(); i++) {
      atPlace[i] = "at " + places.get(i).name();
      if (i > 0) {
        betweenPlaces[i - 1] =
            "between " + places.get(i - 1).name() + " and " + places.get(i).name();
      }
    }
    belowPlaces = places.isEmpty() ? "" : "below " + places.getFirst().name();
    abovePlaces = places.isEmpty() ? "" : "above " + places.getLast().name();
    String key = "Joints/" + label;
    inputsKey = key;
    stateKey = key + "/State";
    commandKey = key + "/Command";
    goalKey = key + "/Goal";
    goalNameKey = key + "/GoalName";
    atKey = key + "/At";
    heldByKey = key + "/HeldBy";
    switchDoubtsKey = key + "/SwitchDoubts";
    setpointKey = key + "/Setpoint";
    setpointVelocityKey = key + "/SetpointVelocity";
    outcomeKey = key + "/Outcome";
  }

  private static MechanismControl.Bands bands(double enter, double stay) {
    return MechanismControl.Bands.builder().enter(enter).stay(stay).build();
  }

  /** An interlock on this joint, whose condition reads another: set once, by {@link Joints}. */
  void restrict(Interlock interlock, @Nullable JointMechanism other) {
    restrictions.add(new Restriction(interlock, other, interlock.toString()));
  }

  /**
   * Its limit switches, read with the robot's sensors: while one is tripped, nothing drives the
   * joint on toward its end, beneath every command and interlock. Beside the controller's own stop
   * where its controller reads the switch, and the only stop where it doesn't (a Systemcore port, a
   * CANdi beside a SPARK).
   */
  void stopAt(List<Sensor.Switch> limits, Sensors sensors) {
    limitSwitches = List.copyOf(limits);
    this.sensors = sensors;
    doubts = new String[limits.size()];
    java.util.Arrays.fill(doubts, "");
    doubtedSince = new long[limits.size()];
    java.util.Arrays.fill(doubtedSince, -1);
  }

  /** The joint, as the robot's definition has it. */
  public Joint joint() {
    return joint;
  }

  @Override
  public String getName() {
    return label;
  }

  @Override
  public Scheduler getRegisteredScheduler() {
    return scheduler;
  }

  /** Where the joint is, as of this loop: radians, or meters. */
  public double position() {
    return inputs.position;
  }

  /** How fast it moves, as of this loop: radians, or meters, a second. */
  public double velocity() {
    return inputs.velocity;
  }

  /**
   * Where it's held or headed: the goal of the move driving it, or of its idle, as its interlocks
   * leave it; where it is, while it runs at an output. NaN before it's first read.
   */
  public double goal() {
    return switch (applied) {
      case POSITION -> goal.position;
      case NONE, HOMING -> holdPosition;
      case VELOCITY, VOLTAGE, DUTY_CYCLE, NEUTRAL -> clamp(inputs.position);
    };
  }

  /** Where it stands: unhomed, homing, ready or faulted. */
  public State state() {
    return state;
  }

  /** The interlocks holding it this loop, as logged; empty while none is. */
  public String heldBy() {
    return heldBy;
  }

  // ---- The pre-built commands ----

  /**
   * Moves to one of its named positions at its cruise, profiled from where it is, and ends when it
   * gets there ({@link Outcome#ARRIVED}) or its deadline passes: twice the profile's time from
   * where it starts, plus half a second, unless {@link Move#within} says otherwise.
   *
   * @throws IllegalArgumentException if it has no position by the name
   */
  public Move goTo(String position) {
    return Move.goTo(this, joint.position(position).value(), position);
  }

  /**
   * Moves to a computed angle, as {@link #goTo(String)} does: {@link Outcome#REFUSED} if it's
   * outside the joint's limits, its soft limits or else its range.
   *
   * @throws IllegalArgumentException if the joint slides
   */
  public Move goTo(Angle angle) {
    if (!(joint instanceof Joint.Revolute)) {
      throw new IllegalArgumentException(label + " slides: send it to a distance");
    }
    return Move.goTo(this, angle.in(Radians), null);
  }

  /**
   * Moves to a computed distance, as {@link #goTo(String)} does: {@link Outcome#REFUSED} if it's
   * outside the joint's limits.
   *
   * @throws IllegalArgumentException if the joint turns
   */
  public Move goTo(Distance distance) {
    if (!(joint instanceof Joint.Prismatic)) {
      throw new IllegalArgumentException(label + " turns: send it to an angle");
    }
    return Move.goTo(this, distance.in(Meters), null);
  }

  /**
   * Chases a target every loop, never ending on its own: a position (an angle or a distance),
   * profiled toward wherever it is now and held inside the joint's limits, or a speed, on the
   * velocity loop. {@code hood.follow(() -> Degrees.of(HOOD_AT_METERS.get(distance())))}, {@code
   * shooter.follow(() -> RPM.of(speedFor(distance())))}. A target of the wrong kind for the joint
   * ends it {@link Outcome#REFUSED}.
   */
  public Move follow(Supplier<? extends Measure<?>> target) {
    return Move.follow(this, target);
  }

  /** Owns the joint and holds its goal, never ending: what it does idle, as a command. */
  public Move hold() {
    return Move.hold(this);
  }

  /**
   * Runs it at one of its named outputs, a speed, a voltage or a duty cycle, until canceled: for
   * "while held" and for work in the background. {@code "stopped"} is every joint's.
   *
   * @throws IllegalArgumentException if it has no output by the name
   */
  public Move run(String output) {
    return Move.run(this, joint.output(output), null);
  }

  /**
   * Runs it at one of its named outputs until a condition holds, which it ends on: {@link
   * Outcome#ARRIVED}, or {@link Outcome#TIMED_OUT} after 3 s unless {@link Move#within} says
   * otherwise. {@code roller.runUntil("intaking", coralIn)}. A {@link Gate} names it ("roller:
   * intaking until coral in claw") and, timed out, says why it never held ("coral in claw is false:
   * claw beam lost").
   *
   * @throws IllegalArgumentException if it has no output by the name
   */
  public Move runUntil(String output, BooleanSupplier condition) {
    return Move.run(this, joint.output(output), condition);
  }

  /**
   * Homes it again by its recipe, owning it meanwhile: {@link Outcome#ARRIVED} once it knows where
   * it is, {@link Outcome#REFUSED} if its homing fails. A joint with an absolute encoder, one
   * stowed by hand, or a roller is home already.
   */
  public Move home() {
    return Move.home(this);
  }

  /**
   * Asks for a voltage across its motors, this loop only: for a command with a law of its own, such
   * as a push held to a current. The loop it isn't asked again, the joint goes back to its idle.
   */
  public void setVoltage(double volts) {
    ask(Request.VOLTAGE, volts, 1, null);
  }

  // ---- Its gates ----

  /**
   * Whether it's at one of its named positions, within its band, measured; or runs at one of its
   * named outputs, within its band for a speed: "pivot at out".
   *
   * @throws IllegalArgumentException if it has no position or output by the name
   */
  public Gate isAt(String name) {
    if (joint.positions().stream().anyMatch(p -> p.name().equals(name))) {
      double target = joint.position(name).value();
      boolean[] inside = {false};
      return new Gate(
          scheduler,
          label + " at " + name,
          why -> {
            double off = Math.abs(inputs.position - target);
            inside[0] =
                state == State.READY
                    && (inside[0] ? off <= positionBands.stay() : off <= positionBands.enter());
            return inside[0];
          });
    }
    Output output = joint.output(name);
    boolean[] inside = {false};
    return new Gate(
        scheduler,
        label + " at " + name,
        why -> {
          boolean running = enabled && name.equals(appliedName);
          inside[0] =
              running
                  && (output.kind() != Output.Kind.VELOCITY
                      || withinSpeed(output.value(), inside[0]));
          return inside[0];
        });
  }

  /**
   * Whether it's past one of its named positions, from its reference: "pivot past run". A position
   * isn't known until the joint is homed: until then, it's false.
   *
   * @throws IllegalArgumentException if it has no position by the name, or it's the reference
   */
  public Gate isPast(String position) {
    Condition.Past past = joint.past(position);
    return new Gate(
        scheduler,
        label + " past " + position,
        why -> state == State.READY && past.holdsAt(inputs.position));
  }

  /**
   * Whether it's between two of its named positions, edges included: "pivot between run and out".
   *
   * @throws IllegalArgumentException if it has no position by either name
   */
  public Gate isBetween(String from, String to) {
    Condition.Between between = joint.between(from, to);
    return new Gate(
        scheduler,
        label + " between " + from + " and " + to,
        why -> state == State.READY && between.holdsAt(inputs.position));
  }

  /** Whether it knows where it is and its motor controllers answer: "pivot ready". */
  public Gate isReady() {
    return new Gate(scheduler, label + " ready", why -> state == State.READY && inputs.connected);
  }

  /**
   * Whether it's where it's asked to be: at its goal, within its band, with its profile there, for
   * a position (its idle's included); within its band of the speed, for a speed; running, for a
   * voltage or a duty cycle; still, stopped. "shooter at goal".
   */
  public Gate isAtGoal() {
    boolean[] inside = {false};
    return new Gate(
        scheduler,
        label + " at goal",
        why -> {
          inside[0] =
              enabled
                  && state == State.READY
                  && switch (applied) {
                    case POSITION -> {
                      double off = Math.abs(inputs.position - goal.position);
                      yield setpoint.position == goal.position
                          && setpoint.velocity == 0
                          && (inside[0]
                              ? off <= positionBands.stay()
                              : off <= positionBands.enter());
                    }
                    case VELOCITY -> withinSpeed(appliedValue, inside[0]);
                    case VOLTAGE, DUTY_CYCLE -> true;
                    case NEUTRAL -> Math.abs(inputs.velocity) <= speedBands.enter() * freeSpeed;
                    case NONE, HOMING -> false;
                  };
          return inside[0];
        });
  }

  /**
   * Whether it's stalled: run at an output, it has moved slower than a tenth of the speed it asks
   * for, for a quarter of a second. A jam, for an unjam to start on: {@code
   * indexer.isStalled().onTrue(indexer.run("reverse").withTimeout(Seconds.of(0.2)))}.
   */
  public Gate isStalled() {
    Debouncer stalling = new Debouncer(STALL_SECONDS, Debouncer.DebounceType.RISING);
    return new Gate(
        scheduler,
        label + " stalled",
        why -> {
          double asked =
              switch (applied) {
                case VELOCITY -> Math.abs(appliedValue);
                case VOLTAGE -> Math.abs(appliedValue) / 12 * freeSpeed;
                case DUTY_CYCLE -> Math.abs(appliedValue) * freeSpeed;
                default -> 0;
              };
          return stalling.calculate(
              enabled && asked > 0 && Math.abs(inputs.velocity) < STALL_SHARE * asked);
        });
  }

  /** Whether it runs within its speed band of a speed: the stay band if it already was. */
  private boolean withinSpeed(double velocity, boolean was) {
    double share = was ? speedBands.stay() : speedBands.enter();
    double allowed = share * (velocity == 0 ? freeSpeed : Math.abs(velocity));
    return Math.abs(inputs.velocity - velocity) <= allowed;
  }

  // ---- What moves ask of it, each loop they run ----

  /** Asks it to be at a position this loop, profiled at a share of its cruise. */
  void askPosition(double position, double fraction, @Nullable String name) {
    ask(Request.POSITION, position, fraction, name);
  }

  /** Asks it to run at an output this loop. */
  void askOutput(Output output) {
    switch (output.kind()) {
      case VELOCITY -> ask(Request.VELOCITY, output.value(), 1, output.name());
      case VOLTAGE -> ask(Request.VOLTAGE, output.value(), 1, output.name());
      case DUTY_CYCLE ->
          ask(
              output.value() == 0 ? Request.NEUTRAL : Request.DUTY_CYCLE,
              output.value(),
              1,
              output.name());
    }
  }

  /** Asks it to run at a speed this loop, a computed one. */
  void askVelocity(double velocity) {
    ask(Request.VELOCITY, velocity, 1, null);
  }

  private void ask(Request kind, double value, double fraction, @Nullable String name) {
    request = kind;
    requestValue = value;
    requestFraction = fraction;
    requestName = name;
    requestLoop = loop;
    requester = outermost(scheduler.currentCommand());
  }

  /** The command a command runs inside, all the way out: the one bound or scheduled, "Intake". */
  private @Nullable Command outermost(@Nullable Command command) {
    Command outer = command;
    for (Command up = outer; up != null; up = scheduler.getParentOf(up)) {
      outer = up;
    }
    return outer;
  }

  /** A move begins: its profile starts from where the joint is now, not where it was headed. */
  void beginMove() {
    reseed = true;
  }

  /**
   * A move on it ended: idle holds where it arrived, or, if it didn't, where the joint comes to a
   * stop from where it is measured, so a joint that jammed holds where it jammed rather than
   * pushing on toward a target it couldn't reach.
   */
  void moveEnded(boolean arrived, double target) {
    if (applied != Request.POSITION) {
      return; // it ran at a speed, or never moved: its idle decides from what it did
    }
    holds = true;
    if (arrived) {
      holdPosition = target;
    } else {
      holdPosition = stoppingPoint();
      reseed = true;
    }
  }

  /** Logs how the last move on it ended, from now on. */
  void recordOutcome(String outcome) {
    this.outcome = outcome;
  }

  /** How the last move on it ended: "pivot → out: ARRIVED in 0.41 s"; empty before any has. */
  public String lastOutcome() {
    return outcome;
  }

  /** When this loop began, in nanoseconds on the robot's clock. */
  long now() {
    return nowNanos;
  }

  /**
   * Why moves on it can't run, or null when they can: its motor controllers aren't answering, or
   * its homing failed.
   */
  @Nullable String fault() {
    if (!inputs.connected) {
      return label + " isn't answering: its motor controllers are silent";
    }
    return state == State.FAULTED ? stateShown.substring("FAULTED: ".length()) : null;
  }

  /** Whether moves on it wait: it's homing, or will on the first enable. */
  boolean waitsForHoming() {
    return state == State.UNHOMED || state == State.HOMING;
  }

  /** Whether it homes by moving, into a hard stop or onto a switch, and so can be homed again. */
  boolean homesToAStop() {
    return homing instanceof MechanismControl.Homing.ToHardStop
        || homing instanceof MechanismControl.Homing.ToSwitch;
  }

  /** Starts homing it again by its recipe: a command asked ({@link #home()}). */
  void startHoming() {
    if (enabled) {
      homingBegins();
    } else {
      setState(State.UNHOMED, null);
    }
  }

  /** A command homing it was canceled: where it is isn't known. */
  void stopHoming() {
    if (state == State.HOMING) {
      setState(State.UNHOMED, null);
    }
  }

  /** Why it can't be sent to a position, or null when it can: outside its limits. */
  @Nullable String whyUnreachable(double position) {
    return position >= lower && position <= upper
        ? null
        : JointSetup.shown(joint, position)
            + " is outside "
            + label
            + "'s limits, "
            + JointSetup.shown(joint, lower)
            + " to "
            + JointSetup.shown(joint, upper);
  }

  /**
   * Whether the move to a position has it there: its profile has reached the position, nothing
   * holds it short, and the joint is within its band of it.
   */
  boolean settledAt(double position) {
    return applied == Request.POSITION
        && appliedValue == position
        && heldBy.isEmpty()
        && setpoint.position == goal.position
        && setpoint.velocity == 0
        && Math.abs(inputs.position - goal.position) <= positionBands.enter();
  }

  /** Whether it runs at a speed, within its band. */
  boolean atSpeed(double velocity) {
    return applied == Request.VELOCITY && appliedValue == velocity && withinSpeed(velocity, false);
  }

  /** How long its profile takes from where it is to a position, at a share of its cruise. */
  double secondsTo(double position, double fraction) {
    return profile(fraction)
        .timeLeftUntil(
            new TrapezoidProfile.State(inputs.position, inputs.velocity),
            new TrapezoidProfile.State(goalNear(position, inputs.position), 0));
  }

  /** How long its velocity loop takes to come within its band of a new speed, from the model. */
  double secondsToSpeed() {
    return kV > 0 ? kA / kV * Math.log(1 / speedBands.enter()) : 0;
  }

  /** A position inside its limits. */
  double clamp(double position) {
    return Math.max(lower, Math.min(upper, position));
  }

  /** Whether a measure is a position the joint takes: an angle if it turns, a distance if not. */
  boolean isPosition(Measure<?> measure) {
    return joint instanceof Joint.Revolute ? measure instanceof Angle : measure instanceof Distance;
  }

  /** Whether a measure is a speed the joint runs at. */
  boolean isSpeed(Measure<?> measure) {
    return joint instanceof Joint.Revolute
        ? measure instanceof AngularVelocity
        : measure instanceof LinearVelocity;
  }

  // ---- Each loop ----

  /** Reads its sensors, and moves its homing on: before the commands run. */
  /** A switch on one of its controller's limit inputs, as its controller read it this loop. */
  void readPin(SensorWiring.Direction direction, SensorIO.SensorIOInputs pin) {
    pin.connected = inputs.connected;
    pin.tripped =
        direction == SensorWiring.Direction.FORWARD ? inputs.forwardLimit : inputs.reverseLimit;
  }

  void read(long nowNanos, boolean enabled) {
    loop++;
    this.nowNanos = nowNanos;
    MechanismControl control = tuned.get();
    if (!control.equals(retuned)) {
      // The pit tuned its gains or current limits: its motor controller takes them now.
      retuned = control;
      io.retune(control);
    }
    io.updateInputs(inputs);
    Logger.processInputs(inputsKey, inputs);
    boolean enabling = enabled && !this.enabled;
    if (!read) {
      read = true;
      setState(homesToAStop() ? State.UNHOMED : State.READY, null);
    }
    if (!inputs.unconfigured.isEmpty() && state != State.FAULTED) {
      // Without its settings, a motor controller has no current limits or ratio: driving it would
      // be a guess at full power, so nothing does.
      setState(
          State.FAULTED,
          label
              + "'s "
              + inputs.unconfigured
              + " didn't take its settings: check its CAN ID and wiring, then restart the robot");
    }
    if (Double.isNaN(holdPosition) || enabling) {
      // First read, or just enabled: idle holds where the joint is, not where it was left.
      holdPosition = clamp(inputs.position);
      reseed = true;
    }
    this.enabled = enabled;
    if (!enabled && state == State.HOMING) {
      setState(State.UNHOMED, null); // it starts again on the next enable
    } else if (enabling && state == State.UNHOMED) {
      homingBegins();
    } else if (state == State.HOMING && !(homing instanceof MechanismControl.Homing.ToSwitch)) {
      homingGoesOn();
    }
  }

  /**
   * What it does with this loop's sensors, read after the joints: homing to its switch goes on, on
   * the switch's reading this loop, not the last.
   */
  void sensed() {
    if (state == State.HOMING && homing instanceof MechanismControl.Homing.ToSwitch to) {
      homingToSwitchGoesOn(to);
    }
    doubtSwitches();
    if (doubts.length > 0) {
      Logger.recordOutput(switchDoubtsKey, doubts);
    }
  }

  /**
   * How far from where a limit switch trips its joint must be, past it or short of it, before the
   * switch's reading is doubted: a tenth of the joint's range, or 15° for one that turns freely. A
   * guess at what a switch bent a little, or flex in the mechanism, never reaches.
   */
  private double doubtMargin() {
    return joint
        .range()
        .map(range -> 0.1 * (range.upper() - range.lower()))
        .orElse(Math.toRadians(15));
  }

  /**
   * Checks each limit switch against where the joint is, once it knows: tripped nowhere near where
   * it trips (a cut wire, if it's normally closed), or clear well past it (a cut wire, if it's
   * normally open), for a quarter second. Said, never acted on: a doubted switch still stops the
   * joint, the safe way to be wrong.
   */
  private void doubtSwitches() {
    Sensors read = sensors;
    for (int i = 0; i < limitSwitches.size(); i++) {
      Sensor.Switch limit = limitSwitches.get(i);
      Sensor.Switch.Limit kind = (Sensor.Switch.Limit) limit.kind();
      String doubt = null;
      if (read != null
          && kind.position().isPresent()
          && state == State.READY
          && inputs.connected
          && read.reads(limit)) {
        double at = kind.position().get();
        double toward = kind.end() == Sensor.Switch.Limit.End.LOWER ? -1 : 1;
        double past = (inputs.position - at) * toward;
        boolean tripped = read.holds(limit.tripped());
        boolean revolute = joint instanceof Joint.Revolute;
        if (tripped && past < -doubtMargin()) {
          doubt =
              limit.label()
                  + " reads tripped with "
                  + label
                  + " at "
                  + JointSetup.shown(joint, inputs.position)
                  + ", nowhere near where it trips, "
                  + JointSetup.shown(joint, at)
                  + ": a cut wire, if it's normally closed, or a switch out of place";
        } else if (!tripped && past > doubtMargin()) {
          doubt =
              limit.label()
                  + " reads clear with "
                  + label
                  + " at "
                  + JointSetup.shown(joint, inputs.position)
                  + ", past where it trips, "
                  + JointSetup.shown(joint, at)
                  + ": a cut wire, if it's normally open, or a switch out of place";
        }
      }
      if (doubt == null) {
        doubtedSince[i] = -1;
        doubts[i] = "";
      } else if (doubtedSince[i] < 0) {
        doubtedSince[i] = nowNanos;
      } else if (nowNanos - doubtedSince[i] >= DOUBT_NANOS) {
        doubts[i] = doubt;
      }
    }
  }

  /** What its limit switches are doubted for this loop, by switch: none while they agree. */
  List<String> switchDoubts() {
    List<String> doubted = new ArrayList<>();
    for (String doubt : doubts) {
      if (!doubt.isEmpty()) {
        doubted.add(doubt);
      }
    }
    return doubted;
  }

  /** Applies what the commands asked this loop, or its idle, then its interlocks. */
  void write() {
    Request asked = requestLoop == loop ? request : Request.NONE;
    double value = requestValue;
    double fraction = requestFraction;
    String byName = requestLoop == loop ? requestName : null;
    heldBy = "";
    if (!enabled || justHomed || (positional && state != State.READY)) {
      // Disabled; or where it is isn't known, so nothing is driven to a position on a guess,
      // except homing's push; or it found its stop, and reads its new position from the next loop.
      if (enabled && state == State.HOMING) {
        io.home(homingVolts());
        applied = Request.HOMING;
      } else {
        io.stop();
        applied = Request.NONE;
      }
      justHomed = false;
      reseed = true;
      appliedName = null;
      log(asked);
      return;
    }
    if (asked == Request.NONE) {
      if (holds) {
        if (applied != Request.POSITION) {
          // From an output, or a stop: hold where it is now.
          holdPosition = clamp(inputs.position);
          reseed = true;
        }
        asked = Request.POSITION;
        value = holdPosition;
        fraction = 1;
      } else {
        asked = Request.NEUTRAL;
        value = 0;
      }
    } else if (asked == Request.POSITION) {
      holds = true;
    } else {
      holds = positional;
    }
    appliedValue = value;
    appliedName = byName;
    double target = value;
    // The interlocks, beneath every command.
    boolean frozen = false;
    for (Restriction restriction : restrictions) {
      Interlock interlock = restriction.interlock();
      JointMechanism other = restriction.other();
      Sensors read = sensors;
      if (interlock instanceof Interlock.KeepsOut keeps) {
        // Which side of the region the joint is on, whether the interlock is closed or not.
        if (inputs.position <= keeps.region().lower()) {
          restriction.side = -1;
        } else if (inputs.position >= keeps.region().upper()) {
          restriction.side = 1;
        }
      }
      // Whether what it speaks of is known, and holds: a joint that knows where it is, or a
      // sensor that answers. Unknown, it restricts: kept out, or held.
      boolean known;
      boolean condition;
      switch (interlock.condition()) {
        case Condition.OnJoint onJoint -> {
          known = other != null && other.state == State.READY && other.inputs.connected;
          condition = known && other != null && onJoint.holdsAt(other.inputs.position);
        }
        case Condition.OnSensor onSensor -> {
          known = read != null && read.reads(onSensor.sensor());
          condition = known && read != null && read.holds(onSensor);
        }
      }
      switch (interlock) {
        case Interlock.RunsOnlyWhen runs -> {
          if (!condition) {
            if (positional) {
              if (Double.isNaN(heldAt)) {
                heldAt = stoppingPoint();
              }
              asked = Request.POSITION;
              target = heldAt;
              fraction = 1;
              frozen = true;
            } else {
              asked = Request.NEUTRAL;
            }
            held(restriction);
          }
        }
        case Interlock.KeepsOut keeps -> {
          if (!known || condition) {
            double kept = keptOut(keeps, restriction.side, asked, target);
            if (!Double.isNaN(kept)) {
              if (asked != Request.POSITION || kept != target) {
                held(restriction);
              }
              asked = Request.POSITION;
              target = kept;
              fraction = 1;
            }
          }
        }
      }
    }
    if (!frozen && !Double.isNaN(heldAt)) {
      heldAt = Double.NaN; // let go: the command's goal again, from where the joint is
      reseed = true;
    }
    // A tripped limit switch, beneath them all: nothing drives the joint on toward its end.
    boolean stopped = false;
    Sensors read = sensors;
    for (Sensor.Switch limit : limitSwitches) {
      if (read == null || !read.holds(limit.tripped())) {
        continue;
      }
      double toward =
          ((Sensor.Switch.Limit) limit.kind()).end() == Sensor.Switch.Limit.End.LOWER ? -1 : 1;
      if (asked == Request.POSITION) {
        if (Double.isNaN(stoppedAt)) {
          stoppedAt = inputs.position;
        }
        if ((target - stoppedAt) * toward > 0) {
          if (!stopped && target != stoppedAt) {
            reseed = true;
          }
          target = stoppedAt;
          stopped = true;
          heldBy =
              heldBy.isEmpty()
                  ? limit.label() + " tripped"
                  : heldBy + "; " + limit.label() + " tripped";
        }
      } else if ((asked == Request.VELOCITY
              || asked == Request.VOLTAGE
              || asked == Request.DUTY_CYCLE)
          && target * toward > 0) {
        asked = Request.NEUTRAL;
        stopped = true;
        heldBy =
            heldBy.isEmpty()
                ? limit.label() + " tripped"
                : heldBy + "; " + limit.label() + " tripped";
      }
    }
    if (!stopped) {
      stoppedAt = Double.NaN;
    }
    switch (asked) {
      case POSITION -> drive(target, fraction);
      case VELOCITY -> io.setVelocity(target);
      case VOLTAGE -> io.setVoltage(target);
      case DUTY_CYCLE -> io.setDutyCycle(target);
      case NEUTRAL, NONE, HOMING -> io.stop();
    }
    if (asked != Request.POSITION) {
      reseed = true;
    }
    applied = asked;
    log(requestLoop == loop ? request : Request.NONE);
  }

  /**
   * Where a {@code keepsOut} interlock that's closed holds the joint, or NaN when it doesn't: a
   * position at the region's edge on the side the joint is on, which its profile stops at. A joint
   * inside the region, one that overshot an edge or was there when it closed, may leave it only the
   * way it came, toward a goal outside on the side it was last on (the nearer side, if it was never
   * outside), and holds where it stops otherwise: never across. Run at an output, it's held at the
   * edge once its stopping point would reach the region.
   *
   * @param side the side the joint was last on: -1 below the region, 1 above, 0 never outside
   */
  private double keptOut(Interlock.KeepsOut keeps, int side, Request asked, double target) {
    double from = keeps.region().lower();
    double to = keeps.region().upper();
    double at = inputs.position;
    if (asked == Request.POSITION) {
      if (at <= from) {
        return Math.min(target, from);
      }
      if (at >= to) {
        return Math.max(target, to);
      }
      boolean below = side != 0 ? side < 0 : at - from <= to - at;
      boolean leaving = below ? target <= from : target >= to;
      return leaving ? target : stoppingPoint();
    }
    if (asked == Request.NEUTRAL) {
      return Double.NaN;
    }
    double stopping = stoppingPoint();
    if (at <= from) {
      return stopping >= from ? from : Double.NaN;
    }
    if (at >= to) {
      return stopping <= to ? to : Double.NaN;
    }
    return stopping;
  }

  /** Notes an interlock holding the joint this loop. */
  private void held(Restriction restriction) {
    heldBy = heldBy.isEmpty() ? restriction.shown() : heldBy + "; " + restriction.shown();
  }

  /** Where the joint comes to a stop from where it is and how fast, braking at its acceleration. */
  private double stoppingPoint() {
    double v = inputs.velocity;
    return clamp(inputs.position + v * Math.abs(v) / (2 * acceleration));
  }

  /** Drives it along its profile toward a position, one loop on. */
  private void drive(double position, double fraction) {
    if (fraction != profileFraction) {
      profile = profile(fraction);
      profileFraction = fraction;
    }
    if (reseed) {
      setpoint.position = inputs.position;
      setpoint.velocity = inputs.velocity;
      reseed = false;
    }
    goal.position = goalNear(position, setpoint.position);
    goal.velocity = 0;
    double before = setpoint.velocity;
    if (setpoint.position != goal.position || setpoint.velocity != 0) {
      TrapezoidProfile.State next = profile.calculate(RobotLoop.PERIOD_SECONDS, setpoint, goal);
      setpoint.position = next.position;
      setpoint.velocity = next.velocity;
    }
    double accelerating = (setpoint.velocity - before) / RobotLoop.PERIOD_SECONDS;
    io.setPosition(setpoint.position, setpoint.velocity, kA * accelerating);
  }

  /**
   * A position as a goal from a point: inside its limits, and for a joint that turns freely, the
   * nearest turn of it, the short way round.
   */
  private double goalNear(double position, double from) {
    double inside = clamp(position);
    return wraps ? from + MathUtil.angleModulus(inside - from) : inside;
  }

  private TrapezoidProfile profile(double fraction) {
    return new TrapezoidProfile(new TrapezoidProfile.Constraints(cruise * fraction, acceleration));
  }

  // ---- Homing ----

  private void homingBegins() {
    homingSince = nowNanos;
    stillSince = -1;
    // Whether it starts on its switch, and backs off first, is seen with this loop's sensors.
    homingFresh = true;
    backingOff = false;
    setState(State.HOMING, null);
  }

  /** The limit switch it homes to, or null when it doesn't home to one. */
  private Sensor.@Nullable Switch homingSwitch() {
    if (!(homing instanceof MechanismControl.Homing.ToSwitch to)) {
      return null;
    }
    Sensor.Switch.Limit.End end =
        to.end() == MechanismControl.Homing.Stop.LOWER
            ? Sensor.Switch.Limit.End.LOWER
            : Sensor.Switch.Limit.End.UPPER;
    for (Sensor.Switch limit : limitSwitches) {
      if (limit.kind() instanceof Sensor.Switch.Limit kind
          && kind.end() == end
          && kind.position().isPresent()) {
        return limit;
      }
    }
    return null;
  }

  /** Whether it has found its stop, or its switch, or run out of time: a loop on. */
  private void homingGoesOn() {
    if (homing instanceof MechanismControl.Homing.ToSwitch to) {
      homingToSwitchGoesOn(to);
      return;
    }
    if (!(homing instanceof MechanismControl.Homing.ToHardStop recipe)) {
      return;
    }
    if (Math.abs(inputs.velocity) <= STILL_SHARE * freeSpeed) {
      if (stillSince < 0) {
        stillSince = nowNanos;
      } else if (nowNanos - stillSince >= STILL_NANOS) {
        double stop = stopPosition(recipe);
        io.resetPosition(stop);
        holdPosition = clamp(stop);
        justHomed = true;
        setState(State.READY, null);
        return;
      }
    } else {
      stillSince = -1;
    }
    double seconds = recipe.timeout().in(Seconds);
    if (nowNanos - homingSince >= Math.round(seconds * 1e9)) {
      setState(
          State.FAULTED,
          String.format(
              java.util.Locale.ROOT,
              "%s's homing found no stop in %.1f s, pushing toward its %s one",
              label,
              seconds,
              recipe.stop() == MechanismControl.Homing.Stop.LOWER ? "lower" : "upper"));
    }
  }

  /**
   * Whether its switch has tripped, where it's home, or the time's run out: a loop on. One that
   * started on its switch backs off until it clears, then comes back until it trips, so it's home
   * where the switch trips, not wherever past it it rested. It's as close as the joint travels in a
   * loop at the homing's speed: approach slowly where it matters.
   */
  private void homingToSwitchGoesOn(MechanismControl.Homing.ToSwitch to) {
    Sensor.Switch limit = homingSwitch();
    Sensors read = sensors;
    if (limit == null || read == null) {
      return;
    }
    boolean tripped = read.holds(limit.tripped());
    if (homingFresh) {
      // On its switch already, past where it trips: back off it first, so it trips on the way in.
      homingFresh = false;
      backingOff = tripped;
      homingLast = Double.NaN;
    }
    if (backingOff) {
      backingOff = tripped;
    } else if (tripped) {
      // It tripped somewhere in this loop's travel: the middle is the best guess, half a loop's
      // travel at worst.
      double past = Double.isNaN(homingLast) ? 0 : (inputs.position - homingLast) / 2;
      double at = ((Sensor.Switch.Limit) limit.kind()).position().orElseThrow() + past;
      io.resetPosition(at);
      holdPosition = clamp(at);
      justHomed = true;
      setState(State.READY, null);
      return;
    }
    homingLast = backingOff ? Double.NaN : inputs.position;
    double seconds = to.timeout().in(Seconds);
    if (nowNanos - homingSince >= Math.round(seconds * 1e9)) {
      setState(
          State.FAULTED,
          String.format(
              java.util.Locale.ROOT,
              "%s's homing didn't %s %s in %.1f s%s",
              label,
              backingOff ? "back off" : "trip",
              limit.label(),
              seconds,
              read.reads(limit) ? "" : ": " + limit.label() + " is lost"));
    }
  }

  /** Homing's push, toward its stop or its switch. */
  private double homingVolts() {
    return switch (homing) {
      case MechanismControl.Homing.ToHardStop recipe ->
          recipe.stop() == MechanismControl.Homing.Stop.LOWER ? -recipe.volts() : recipe.volts();
      case MechanismControl.Homing.ToSwitch to ->
          (to.end() == MechanismControl.Homing.Stop.LOWER ? -to.volts() : to.volts())
              * (backingOff ? -1 : 1);
      case null, default -> 0;
    };
  }

  /** Where the joint is at the stop it homes to: its range's end. */
  private double stopPosition(MechanismControl.Homing.ToHardStop recipe) {
    var range = joint.range().orElseThrow();
    return recipe.stop() == MechanismControl.Homing.Stop.LOWER ? range.lower() : range.upper();
  }

  private void setState(State state, @Nullable String why) {
    this.state = state;
    stateShown = why == null ? state.name() : state.name() + ": " + why;
  }

  // ---- The log ----

  /** What it did this loop, under its fixed keys. */
  private void log(Request asked) {
    @Nullable Command by = requester;
    Logger.recordOutput(stateKey, stateShown);
    Logger.recordOutput(
        commandKey,
        asked == Request.NONE ? "idle" : by == null ? "outside any command" : by.name());
    Logger.recordOutput(goalKey, applied == Request.POSITION ? goal.position : appliedValue);
    String name = asked == Request.NONE ? null : requestName;
    Logger.recordOutput(goalNameKey, name == null ? "" : name);
    Logger.recordOutput(atKey, where());
    Logger.recordOutput(heldByKey, heldBy);
    Logger.recordOutput(setpointKey, setpoint.position);
    Logger.recordOutput(setpointVelocityKey, setpoint.velocity);
    Logger.recordOutput(outcomeKey, outcome);
  }

  /**
   * Where it is, in its names: "at out", "between run and out", "below in"; a number for a joint
   * without names; what it runs at, for a roller.
   */
  private String where() {
    if (!positional) {
      String name = appliedName;
      return name != null ? name : applied == Request.NEUTRAL ? "stopped" : "idle";
    }
    if (state != State.READY) {
      return "unknown";
    }
    if (places.isEmpty()) {
      return JointSetup.shown(joint, inputs.position);
    }
    double at = inputs.position;
    for (int i = 0; i < places.size(); i++) {
      if (Math.abs(at - places.get(i).value()) <= positionBands.enter()) {
        return atPlace[i];
      }
    }
    if (at < places.getFirst().value()) {
      return belowPlaces;
    }
    for (int i = 1; i < places.size(); i++) {
      if (at < places.get(i).value()) {
        return betweenPlaces[i - 1];
      }
    }
    return abovePlaces;
  }
}
