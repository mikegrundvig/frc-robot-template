package frc.robot.subsystems.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.world.robot.CanDevice;
import com.michaelgrundvig.frc.world.robot.Condition;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import frc.robot.behavior.Gate;
import frc.robot.behavior.Outcome;
import frc.robot.lib.timing.RobotLoop;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
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
import org.wpilib.units.measure.Time;

/**
 * One driven joint, as a Commands v3 {@link Mechanism}: the template builds one for each joint the
 * robot's motors drive ({@link Joints}), from the robot's definition and the joint's block, and
 * behavior code looks it up by its joint ({@code joints.of(PIVOT)}). Nobody writes one.
 *
 * <p>Its commands are pre-built, each a {@link Move} that ends with an outcome:
 *
 * <ul>
 *   <li>{@link #goTo(Position)} one of its named positions, profiled from where it is, and {@code
 *       goTo(PIVOT_OUT).at(0.9)} at a share of its cruise; {@link #goTo(Angle)} or {@link
 *       #goTo(Distance)} a computed one, inside its limits, and a turret's at the turn of it
 *       nearest where it is;
 *   <li>{@link #follow} a target that changes every loop, which it never stops chasing;
 *   <li>{@link #hold()} its goal, owning the joint;
 *   <li>{@link #run(Output)} one of its named outputs, and {@link #runUntil} one until a condition
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
 * for it, and if it finds no stop it's faulted, and its moves are refused. An absolute encoder its
 * motor controller reads tells it at startup; one the robot reads itself, on a Systemcore port,
 * seeds it from its first reading, and until then it waits as if homing.
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
 * times kA added; the motor controller adds its own kV, kS and gravity. A joint whose loop closes
 * on its absolute encoder on a Systemcore port closes it here instead: each loop, the volts for the
 * profile's point from where the encoder says the joint is ({@link EncoderLoop}), sent as a
 * voltage, and the encoder's word is where the joint is. Its readings, the motor controller's and
 * the encoder's, are AdvantageKit inputs, so a replay reproduces everything here. It logs under
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

  /** One whole turn, in radians. */
  private static final double TURN = 2 * Math.PI;

  /**
   * A hair, in radians: a range given in degrees as a whole turn, or a heading on a limit's turn,
   * converts a floating-point step short of it or past it.
   */
  private static final double EDGE = 1e-9;

  /** A joint run at an output is stalled once it's moved slower than this share of the speed... */
  static final double STALL_SHARE = 0.1;

  /** ...the output asks for, for this long, in seconds. */
  static final double STALL_SECONDS = 0.25;

  /**
   * A joint that knows where it is from an absolute encoder the robot reads waits this long for the
   * encoder's first reading before it's faulted, in nanoseconds: the encoder still seeds it when
   * the reading comes.
   */
  static final long ABSOLUTE_WAIT_NANOS = 2_000_000_000L;

  /** Where a joint stands, as {@code State} logs it. */
  public enum State {
    /**
     * Held at positions, and doesn't yet know where it is: it homes on the first enable, or from
     * its absolute encoder's first reading.
     */
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
   * should it ever be inside; and where it's held inside, once.
   */
  static final class Restriction {
    private final Interlock interlock;
    private final @Nullable JointMechanism other;
    private final String shown;

    /** Below the region (-1) or above it (1) when last outside it; 0 before it ever was. */
    private int side;

    /**
     * Where the joint is held inside the region, worked out once as the hold starts, or NaN when it
     * isn't: worked out again each loop from how fast it's still going, the point would run ahead
     * of a joint its loop hasn't yet slowed, and the joint after it through the region.
     */
    private double inside = Double.NaN;

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
  private final RobotMath math;

  /**
   * Its absolute encoder, where the robot reads it itself and tells the motor where the joint is
   * from it ({@link Joints#readsAbsolute}), or null.
   */
  private @Nullable AbsoluteEncoderIO absoluteIo;

  private final AbsoluteEncoderIOInputsAutoLogged absoluteInputs =
      new AbsoluteEncoderIOInputsAutoLogged();

  /** The encoder in the robot's definition, and its port, if it's on one. */
  private Sensor.@Nullable Encoder absoluteModel;

  private MechanismWiring.@Nullable PortEncoder absolutePort;

  /** Whether it's waiting for its absolute encoder's reading, and since when. */
  private boolean seeking;

  private long seekingSince = -1;

  /** Whether it's faulted for its encoder's silence, which a reading clears. */
  private boolean silentEncoder;

  /**
   * The loop it closes itself on its absolute encoder, each loop, where its port's wiring says so
   * ({@code PortEncoder#closesLoop}); null where its motor controller runs the loop on its motor's
   * own encoder.
   */
  private @Nullable EncoderLoop encoderLoop;

  /**
   * Where the joint is this loop, in its units: its motor controller's reading, or, where its loop
   * closes on its absolute encoder, the encoder's.
   */
  private double measured;

  /**
   * Where, by its motor's count, it comes to a stop once the absolute encoder its loop closes on is
   * lost: it holds there on its motor controller's own loop until the encoder answers. NaN while
   * nothing's lost.
   */
  private double heldOnMotor = Double.NaN;

  /** Whether it holds a position when idle: it has a range or named positions. */
  private final boolean positional;

  /** Whether it turns freely, so a goal is taken the short way round. */
  private final boolean wraps;

  /**
   * Whether its range spans a whole turn or more, a turret's: it can face every way, at more than
   * one position for some, so a computed angle is a heading, taken at whichever turn of it inside
   * its limits is nearest where it is ({@link #turnNear}).
   */
  private final boolean choosesTurn;

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

  /**
   * How long its homing may push it before it's faulted, in seconds: its recipe's timeout, or one
   * worked out from its travel ({@link MechanismControl.Homing#secondsAllowed}); NaN for none.
   */
  private final double homingSeconds;

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

  /** Where the current its motors draw on their PDH channels is logged. */
  private final String pdhCurrentKey;

  /** Where its motor controllers drawing nothing on their channels while it works are logged. */
  private final String deadChannelsKey;

  /** Where its absolute encoder's reading is logged, and where it says the joint is. */
  private final String absoluteKey;

  private final String absoluteAtKey;

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

  /** Its motor controllers the PDH feeds, each with its channel, as its block says them. */
  private final List<CanDevice> fed;

  /** When one of them reads as drawing nothing while the joint works, from its control. */
  private final MechanismControl.DeadChannel deadChannel;

  /** Since when each has read as drawing nothing while the joint works, in nanoseconds: -1 not. */
  private final long[] deadSince;

  /** What each is flagged for, in their order: empty while it draws as it should. */
  private final String[] dead;

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
   * limits, each within what's safe to dial in on a running robot, and each committed by the call
   * that writes it in the joint's block: its kP by the factory the control starts from, {@code
   * MechanismControl.position(31.5)}, the rest by their withers, {@code .withKs(0.25)}. Its
   * feedforward, profile and named positions aren't tunable yet: optional and listed values are
   * beyond what {@link LiveConfig} reaches.
   *
   * @param joint the joint, whose label names its tuning
   * @param committed its control as written in {@code Robots.java}
   */
  public static LiveConfig<MechanismControl> tuning(Joint joint, MechanismControl committed) {
    MechanismControl.Gains gains = committed.gains();
    MechanismControl.CurrentLimits limits = committed.currentLimits();
    double stator = limits.stator().in(Amps);
    double supply = limits.supply().in(Amps);
    String factory = joint.holdsPositions() ? "position" : "velocity";
    return LiveConfig.of(joint.label(), committed)
        // Volts per radian (or meter), and per radian (or meter) per second: up to ten times what's
        // committed, which tells the joint's scale.
        .number(
            "gains.kP",
            Limits.between(0, Math.max(10 * gains.kP(), 1)),
            "MechanismControl." + factory + "(%s)")
        .number("gains.kD", Limits.between(0, Math.max(10 * gains.kD(), 1)), ".withKd(%s)")
        .number("gains.kS", Limits.between(0, Math.max(gains.kS(), 2)), ".withKs(%s)") // volts
        .measure(
            "currentLimits.stator",
            Amps.of(Math.min(5, stator)),
            Amps.of(Math.max(120, stator)),
            ".withStatorLimit(%s)")
        .measure(
            "currentLimits.supply",
            Amps.of(Math.min(5, supply)),
            Amps.of(Math.max(80, supply)),
            ".withSupplyLimit(%s)");
  }

  /**
   * @param joint one the template drives ({@link Joints#driven}), its block saying how
   * @param tuned its control as the pit tunes it, read each loop; its block's until it's tuned
   */
  JointMechanism(
      RobotMath math,
      Joint joint,
      Supplier<MechanismControl> tuned,
      JointIO io,
      Scheduler scheduler) {
    this.joint = joint;
    this.math = math;
    this.tuned = tuned;
    MechanismControl control = ControllerSettings.control(joint);
    retuned = control;
    label = joint.label();
    this.scheduler = scheduler;
    this.io = io;
    positional = joint.holdsPositions();
    holds = positional;
    wraps = joint instanceof Joint.Revolute && joint.range().isEmpty();
    choosesTurn =
        joint instanceof Joint.Revolute
            && joint.range().map(range -> range.width() >= TURN - EDGE).orElse(false);
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
    // The plan's: tuned, else the definition's seed, as its motor controller has them.
    ControllerPlan plan = ControllerPlan.of(math, joint, control);
    kA = plan.kA();
    kV = plan.kV();
    MechanismControl.Bands kindBands = joint instanceof Joint.Revolute ? ANGLE_BANDS : LENGTH_BANDS;
    positionBands = positional ? control.bands().orElse(kindBands) : kindBands;
    speedBands = positional ? SPEED_BANDS : control.bands().orElse(SPEED_BANDS);
    homing = positional ? control.homing().orElse(null) : null;
    homingSeconds =
        switch (homing) {
          case MechanismControl.Homing.ToHardStop recipe -> recipe.secondsOn(math, joint);
          case MechanismControl.Homing.ToSwitch to -> to.secondsOn(math, joint);
          case null, default -> Double.NaN;
        };
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
    pdhCurrentKey = key + "/PdhCurrentAmps";
    deadChannelsKey = key + "/DeadChannels";
    absoluteKey = key + "/Absolute";
    absoluteAtKey = key + "/AbsoluteAt";
    setpointKey = key + "/Setpoint";
    setpointVelocityKey = key + "/SetpointVelocity";
    outcomeKey = key + "/Outcome";
    fed = joint.devices().stream().filter(device -> device.feed().isPresent()).toList();
    deadChannel = control.deadChannel();
    deadSince = new long[fed.size()];
    java.util.Arrays.fill(deadSince, -1);
    dead = new String[fed.size()];
    java.util.Arrays.fill(dead, "");
  }

  private static MechanismControl.Bands bands(double enter, double stay) {
    return MechanismControl.Bands.builder().enter(enter).stay(stay).build();
  }

  /**
   * Its absolute encoder, read by the robot: until its first reading the joint waits, unhomed, and
   * then tells its motor controller where it is from it. Where its port's wiring says the joint's
   * loop closes on it, the joint closes that loop itself, each loop, on the encoder's reading. Set
   * once, by {@link Joints}, before the first loop.
   */
  void seedFrom(AbsoluteEncoderIO encoder) {
    absoluteIo = encoder;
    absoluteModel = math.absoluteEncoder(joint);
    MechanismWiring.PortEncoder port =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port).orElse(null);
    absolutePort = port;
    if (port != null && port.closedLoop()) {
      encoderLoop = ControllerSettings.encoderLoop(math, joint, retuned);
    }
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
    return measured;
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
      case VELOCITY, VOLTAGE, DUTY_CYCLE, NEUTRAL -> clamp(measured);
    };
  }

  /** Where it stands: unhomed, homing, ready or faulted. */
  public State state() {
    return state;
  }

  /**
   * Where it stands, as {@code State} logs it: why, when it's faulted, and while it homes, toward
   * what and for how long at most ("HOMING: toward its lower stop, for at most 9.5 s").
   */
  String stateShown() {
    return stateShown;
  }

  /** The interlocks holding it this loop, as logged; empty while none is. */
  public String heldBy() {
    return heldBy;
  }

  // ---- The pre-built commands ----

  /**
   * Moves to one of its positions at its cruise, profiled from where it is, and ends when it gets
   * there ({@link Outcome#ARRIVED}) or its deadline passes: twice the profile's time from where it
   * starts, plus half a second, unless {@link Move#within} says otherwise. {@code
   * pivot.goTo(PIVOT_OUT)}.
   *
   * <p>A position is a name on the joint, and it goes where the robot running it puts it: a
   * practice robot's joint, its position re-placed ({@code replaced}), goes to its own, given the
   * competition robot's constant. A position that isn't its own ({@link Joint#whyNotOwn}) is
   * refused, saying so: a name it doesn't have, another joint's or one left out of its {@code
   * .positions(...)}, or one of its names at a value it has never had, another joint's of the same
   * name, elsewhere ({@code elevator.goTo(HOPPER_OUT)}, each with an "out"). Built as the robot
   * starts, as its bindings and the autos built with them are, before the joint's first loop and
   * outside any command, the move throws: the robot doesn't start, and a test that builds it fails.
   * Built once the robot runs, in a command or anywhere else, it ends {@link Outcome#REFUSED} as it
   * starts, its reason logged, so a robot on the field is never stopped by one.
   *
   * @throws IllegalArgumentException for a position not its own, built as the robot starts
   */
  public Move goTo(Position position) {
    String why = refusedAsBuilt(joint.whyNotOwn(position));
    return Move.goTo(
        this, (why == null ? joint.own(position) : position).value(), position.name(), why);
  }

  /**
   * Moves to a computed angle, as {@link #goTo(Position)} does: {@link Outcome#REFUSED} if it's
   * outside the joint's limits, its soft limits or else its range. A joint whose range spans a
   * whole turn or more, a turret's, takes it as a heading: it goes to whichever turn of it, the
   * angle plus or minus whole turns, lies inside its limits nearest where it is as the move starts,
   * and is refused only if none does.
   *
   * @throws IllegalArgumentException if the joint slides
   */
  public Move goTo(Angle angle) {
    if (!(joint instanceof Joint.Revolute)) {
      throw new IllegalArgumentException(label + " slides: send it to a distance");
    }
    return Move.goTo(this, angle.in(Radians), null, null);
  }

  /**
   * Moves to a computed distance, as {@link #goTo(Position)} does: {@link Outcome#REFUSED} if it's
   * outside the joint's limits.
   *
   * @throws IllegalArgumentException if the joint turns
   */
  public Move goTo(Distance distance) {
    if (!(joint instanceof Joint.Prismatic)) {
      throw new IllegalArgumentException(label + " turns: send it to an angle");
    }
    return Move.goTo(this, distance.in(Meters), null, null);
  }

  /**
   * Chases a target every loop, never ending on its own: a position (an angle or a distance),
   * profiled toward wherever it is now and held inside the joint's limits, or a speed, on the
   * velocity loop. A turret, a joint whose range spans a whole turn or more, chases an angle as a
   * heading, at the turn of it inside its limits nearest where it is each loop ({@link
   * #goTo(Angle)}). {@code hood.follow(() -> Degrees.of(HOOD_AT_METERS.get(distance())))}, {@code
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
   * Runs it at one of its outputs, a speed, a voltage or a duty cycle, until canceled: for "while
   * held" and for work in the background. {@code roller.run(INTAKING)}. {@link Output#STOPPED} is
   * every joint's. An output is a name on the joint, run as the robot running it has it; one it
   * doesn't have is refused as a position is by {@link #goTo(Position)}: thrown as the robot
   * starts, {@link Outcome#REFUSED} once it runs.
   *
   * @throws IllegalArgumentException for an output not its own, built as the robot starts
   */
  public Move run(Output output) {
    String why = refusedAsBuilt(joint.whyNotOwn(output));
    return Move.run(this, why == null ? joint.own(output) : output, null, why);
  }

  /**
   * Runs it at one of its outputs until a condition holds, which it ends on: {@link
   * Outcome#ARRIVED}, or {@link Outcome#TIMED_OUT} after 3 s unless {@link Move#within} says
   * otherwise. {@code roller.runUntil(INTAKING, coralIn)}. A {@link Gate} names it ("roller:
   * intaking until coral in claw") and, timed out, says why it never held ("coral in claw is false:
   * claw beam lost"). An output that isn't its own is refused as a position is by {@link
   * #goTo(Position)}: thrown as the robot starts, {@link Outcome#REFUSED} once it runs.
   *
   * @throws IllegalArgumentException for an output not its own, built as the robot starts
   */
  public Move runUntil(Output output, BooleanSupplier condition) {
    String why = refusedAsBuilt(joint.whyNotOwn(output));
    return Move.run(this, why == null ? joint.own(output) : output, condition, why);
  }

  /**
   * Why a move can't run, as it's built: thrown as the robot starts, before the joint's first loop
   * and outside any command, so a mistake in a binding fails at startup and in the simulator's
   * tests, not when it's pressed; kept for the move to end {@link Outcome#REFUSED} on once the
   * robot runs. Commands v3 says whether a command's body is running ({@code
   * Scheduler.currentCommand}), but outside one isn't only at startup: a team's {@code
   * autonomousInit}, a periodic method or an auto rebuilt as its chooser changes all run on the
   * field outside any command, where a throw would stop the robot. So startup is the joint's first
   * loop not yet read, and a command running never throws.
   *
   * @throws IllegalArgumentException with the reason, before the joint's first loop
   */
  private @Nullable String refusedAsBuilt(@Nullable String why) {
    if (why != null && loop == 0 && scheduler.currentCommand() == null) {
      throw new IllegalArgumentException(why);
    }
    return why;
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
   * Whether it's at one of its positions, within its band, measured: "pivot at out".
   *
   * @throws IllegalArgumentException if the position isn't one of its own
   */
  public Gate isAt(Position position) {
    Position own = joint.own(position);
    String name = own.name();
    double target = own.value();
    boolean[] inside = {false};
    return new Gate(
        scheduler,
        label + " at " + name,
        why -> {
          double off = Math.abs(measured - target);
          inside[0] =
              state == State.READY
                  && (inside[0] ? off <= positionBands.stay() : off <= positionBands.enter());
          return inside[0];
        });
  }

  /**
   * Whether it runs at one of its outputs, within its band for a speed: "shooter at shooting".
   *
   * @throws IllegalArgumentException if the output isn't one of its own
   */
  public Gate isAt(Output output) {
    Output own = joint.own(output);
    String name = own.name();
    boolean[] inside = {false};
    return new Gate(
        scheduler,
        label + " at " + name,
        why -> {
          boolean running = enabled && name.equals(appliedName);
          inside[0] =
              running
                  && (own.kind() != Output.Kind.VELOCITY || withinSpeed(own.value(), inside[0]));
          return inside[0];
        });
  }

  /**
   * Whether it's past one of its positions, from where it rests: "pivot past run". A position isn't
   * known until the joint is homed: until then, it's false.
   *
   * @throws IllegalArgumentException if the position isn't one of its own, or it's where it rests
   */
  public Gate isPast(Position position) {
    Condition.Past past = joint.past(position);
    return new Gate(
        scheduler,
        label + " past " + position.name(),
        why -> state == State.READY && past.holdsAt(measured));
  }

  /**
   * Whether it's between two of its positions, edges included: "pivot between run and out".
   *
   * @throws IllegalArgumentException if either isn't one of its own
   */
  public Gate isBetween(Position from, Position to) {
    Condition.Between between = joint.between(from, to);
    return new Gate(
        scheduler,
        label + " between " + from.name() + " and " + to.name(),
        why -> state == State.READY && between.holdsAt(measured));
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
                      double off = Math.abs(measured - goal.position);
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
   * indexer.isStalled().onTrue(indexer.run(INDEXER_UNJAMMING).withTimeout(Seconds.of(0.2)))}.
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

  /** Whether it finds where it is from an absolute encoder the robot reads, and can again. */
  boolean homesByEncoder() {
    return absoluteIo != null;
  }

  /**
   * Starts homing it again by its recipe, or from its absolute encoder's next reading: a command
   * asked ({@link #home()}).
   */
  void startHoming() {
    if (homesByEncoder()) {
      seekAgain();
    } else if (enabled) {
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

  /**
   * Why it can't be sent to a position, or null when it can: outside its limits, at every turn of
   * it for a joint whose range spans a turn or more.
   */
  @Nullable String whyUnreachable(double position) {
    if (position >= lower && position <= upper) {
      return null;
    }
    if (choosesTurn && !Double.isNaN(turnInside(position, measured))) {
      return null;
    }
    return joint.shown(position)
        + (choosesTurn ? ", at every turn of it, is" : " is")
        + " outside "
        + label
        + "'s limits, "
        + joint.shown(lower)
        + " to "
        + joint.shown(upper);
  }

  /**
   * Where a computed angle takes the joint from a point. For a joint whose range spans a whole turn
   * or more, a turret's, the angle is a heading: of the angle plus or minus whole turns, the one
   * inside its limits nearest the point, so a turret at 170° asked for -170° goes on to 190°, and
   * one asked for a heading it can reach only the long way round goes the long way. With none
   * inside, which only soft limits narrower than a turn leave, whichever limit is nearer the
   * heading, either way round: a move there is refused first ({@link #whyUnreachable}), and a chase
   * holds there. Any other joint takes the angle as it is.
   */
  double turnNear(double angle, double from) {
    if (!choosesTurn) {
      return angle;
    }
    double inside = turnInside(angle, from);
    if (!Double.isNaN(inside)) {
      return inside;
    }
    return Math.abs(MathUtil.angleModulus(angle - lower))
            <= Math.abs(MathUtil.angleModulus(angle - upper))
        ? lower
        : upper;
  }

  /** The turn of an angle inside its limits nearest a point, or NaN for none. */
  private double turnInside(double angle, double from) {
    // Its lowest turn at or above the lower limit, then one turn up at a time, as many as the
    // limits span: a count, so an angle far too big to turn by steps still ends.
    double lowest = angle + TURN * Math.ceil((lower - angle) / TURN - EDGE);
    int spanned = (int) ((upper - lower) / TURN) + 1;
    double nearest = Double.NaN;
    for (int turns = 0; turns <= spanned; turns++) {
      double turn = lowest + turns * TURN;
      if (turn >= lower - EDGE
          && turn <= upper + EDGE
          && (Double.isNaN(nearest) || Math.abs(turn - from) < Math.abs(nearest - from))) {
        nearest = turn;
      }
    }
    return Double.isNaN(nearest) ? nearest : clamp(nearest);
  }

  /**
   * A computed position as the joint chases it from where it is now: inside its limits, at the turn
   * of it nearest where it is for a joint whose range spans a turn or more ({@link #turnNear}).
   */
  double reachable(double position) {
    return clamp(turnNear(position, measured));
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
        && Math.abs(measured - goal.position) <= positionBands.enter();
  }

  /** Whether it runs at a speed, within its band. */
  boolean atSpeed(double velocity) {
    return applied == Request.VELOCITY && appliedValue == velocity && withinSpeed(velocity, false);
  }

  /** How long its profile takes from where it is to a position, at a share of its cruise. */
  double secondsTo(double position, double fraction) {
    return profile(fraction)
        .timeLeftUntil(
            new TrapezoidProfile.State(measured, inputs.velocity),
            new TrapezoidProfile.State(goalNear(position, measured), 0));
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

  /** A switch on one of its controller's limit inputs, as its controller read it this loop. */
  void readPin(SensorWiring.Direction direction, SensorIO.SensorIOInputs pin) {
    pin.connected = inputs.connected;
    pin.tripped =
        direction == SensorWiring.Direction.FORWARD ? inputs.forwardLimit : inputs.reverseLimit;
  }

  /** Reads its sensors, and moves its homing on: before the commands run. */
  void read(long nowNanos, boolean enabled) {
    loop++;
    this.nowNanos = nowNanos;
    MechanismControl control = tuned.get();
    if (!control.equals(retuned)) {
      // The pit tuned its gains or current limits: its motor controller takes them now, and so
      // does the loop it closes itself, if it does.
      retuned = control;
      io.retune(control);
      if (encoderLoop != null) {
        encoderLoop = ControllerSettings.encoderLoop(math, joint, control);
      }
    }
    io.updateInputs(inputs);
    Logger.processInputs(inputsKey, inputs);
    AbsoluteEncoderIO encoder = absoluteIo;
    if (encoder != null) {
      encoder.updateInputs(absoluteInputs);
      Logger.processInputs(absoluteKey, absoluteInputs);
    }
    Sensor.Encoder model = absoluteModel;
    // Where the joint is: its motor's word, unless its loop closes on its encoder, whose word it
    // then takes while the encoder answers.
    measured =
        encoderLoop != null && model != null && absoluteInputs.connected
            ? math.fromAbsolute(model, absoluteReading())
            : inputs.position;
    boolean enabling = enabled && !this.enabled;
    if (!read) {
      read = true;
      setState(homesToAStop() || homesByEncoder() ? State.UNHOMED : State.READY, null);
      if (homesByEncoder()) {
        seekAgain();
      }
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
      holdPosition = clamp(measured);
      reseed = true;
    }
    this.enabled = enabled;
    if (seeking) {
      seek();
    }
    if (encoderLoop != null) {
      watchEncoder();
    }
    if (!enabled && state == State.HOMING) {
      setState(State.UNHOMED, null); // it starts again on the next enable
    } else if (enabling && state == State.UNHOMED && homesToAStop()) {
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
   * What it does with this loop's PDH reading, read after its sensors: the current its motors draw
   * together on their channels, logged beside its leading controller's own ({@code
   * SupplyCurrentAmps}), NaN while the PDH isn't read; and each of its motor controllers that reads
   * as drawing nothing while the joint works, for longer than its control allows ({@link
   * MechanismControl.DeadChannel}), flagged until it draws again. Nothing for a joint whose block
   * says no channel.
   */
  void powered(Power power) {
    if (fed.isEmpty()) {
      return;
    }
    // Working: its leading controller answers, and draws from the battery, so each motor does.
    boolean working =
        power.reads()
            && inputs.connected
            && Math.abs(inputs.supplyCurrentAmps) > deadChannel.drawing().in(Amps);
    long allowed = Math.round(deadChannel.time().in(Seconds) * 1e9);
    double together = 0;
    for (int i = 0; i < fed.size(); i++) {
      int channel = fed.get(i).feed().orElseThrow().channel();
      double amps = power.amps(channel);
      together += amps;
      if (!working || amps >= deadChannel.dead().in(Amps)) {
        deadSince[i] = -1;
        dead[i] = "";
        continue;
      }
      if (deadSince[i] < 0) {
        deadSince[i] = nowNanos;
      }
      if (nowNanos - deadSince[i] >= allowed) {
        dead[i] =
            fed.get(i).name()
                + " draws nothing on channel "
                + channel
                + " while "
                + label
                + "'s motors work: its breaker tripped, its power is unplugged, or its block"
                + " names the wrong channel";
      }
    }
    Logger.recordOutput(pdhCurrentKey, together);
    Logger.recordOutput(deadChannelsKey, dead);
  }

  /**
   * Its motor controllers that read as drawing nothing on their PDH channels while it works, as of
   * this loop: each said, with what to check. Empty while each draws as it should.
   */
  List<String> deadChannels() {
    List<String> flagged = new ArrayList<>();
    for (String each : dead) {
      if (!each.isEmpty()) {
        flagged.add(each);
      }
    }
    return flagged;
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
        double past = (measured - at) * toward;
        boolean tripped = read.holds(limit.tripped());
        boolean revolute = joint instanceof Joint.Revolute;
        if (tripped && past < -doubtMargin()) {
          doubt =
              limit.label()
                  + " reads tripped with "
                  + label
                  + " at "
                  + joint.shown(measured)
                  + ", nowhere near where it trips, "
                  + joint.shown(at)
                  + ": a cut wire, if it's normally closed, or a switch out of place";
        } else if (!tripped && past > doubtMargin()) {
          doubt =
              limit.label()
                  + " reads clear with "
                  + label
                  + " at "
                  + joint.shown(measured)
                  + ", past where it trips, "
                  + joint.shown(at)
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
      } else if (enabled && !Double.isNaN(heldOnMotor)) {
        // The encoder its loop closes on is lost: it holds where its motor says, on its motor
        // controller's own loop, rather than going limp.
        io.setPosition(heldOnMotor, 0, 0);
        applied = Request.NONE;
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
          holdPosition = clamp(measured);
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
        // Which side of the region the joint is on, whether the interlock is closed or not. An
        // edge at the range's end has no side past it: a joint there is inside.
        if (!keeps.keepsOut(measured)) {
          restriction.side = keeps.keptBelow(measured, 0) ? -1 : 1;
        }
      }
      // Whether what it speaks of is known, and holds: a joint that knows where it is, or a
      // sensor that answers. Unknown, it restricts: kept out, or held.
      boolean known;
      boolean condition;
      switch (interlock.condition()) {
        case Condition.OnJoint onJoint -> {
          known = other != null && other.state == State.READY && other.inputs.connected;
          condition = known && other != null && onJoint.holdsAt(other.measured);
        }
        case Condition.UpToSpeed upToSpeed -> {
          // How fast it runs is known from its motors, homed or not.
          known = other != null && other.inputs.connected;
          condition = known && other != null && upToSpeed.holdsAt(other.inputs.velocity);
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
            double kept = keptOut(keeps, restriction, asked, target);
            if (!Double.isNaN(kept)) {
              if (asked != Request.POSITION || kept != target) {
                held(restriction);
              }
              asked = Request.POSITION;
              target = kept;
              fraction = 1;
            }
          } else {
            restriction.inside = Double.NaN;
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
          stoppedAt = measured;
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
   * outside), and otherwise holds where it would stop as the hold starts: never across. Run at an
   * output, it's held at the edge once its stopping point would reach the region.
   *
   * @param restriction the interlock's: the side the joint was last on, and where it's held inside
   */
  private double keptOut(
      Interlock.KeepsOut keeps, Restriction restriction, Request asked, double target) {
    double from = keeps.region().lower();
    double to = keeps.region().upper();
    double at = measured;
    // Out of the region, it's held on its side; inside, it may only leave, by the side it's kept
    // to.
    boolean inside = keeps.keepsOut(at);
    boolean below = keeps.keptBelow(at, restriction.side);
    boolean leaving = asked == Request.POSITION && (below ? target <= from : target >= to);
    if (!inside || leaving || asked == Request.NEUTRAL) {
      restriction.inside = Double.NaN;
    }
    if (asked == Request.POSITION) {
      if (!inside) {
        return below ? Math.min(target, from) : Math.max(target, to);
      }
      return leaving ? target : heldInside(restriction);
    }
    if (asked == Request.NEUTRAL) {
      return Double.NaN;
    }
    if (!inside) {
      double stopping = stoppingPoint();
      if (below) {
        return stopping >= from ? from : Double.NaN;
      }
      return stopping <= to ? to : Double.NaN;
    }
    return heldInside(restriction);
  }

  /**
   * Where a joint inside a region it's kept out of is held: where it would stop, worked out once as
   * the hold starts.
   */
  private double heldInside(Restriction restriction) {
    if (Double.isNaN(restriction.inside)) {
      restriction.inside = stoppingPoint();
    }
    return restriction.inside;
  }

  /** Notes an interlock holding the joint this loop. */
  private void held(Restriction restriction) {
    heldBy = heldBy.isEmpty() ? restriction.shown() : heldBy + "; " + restriction.shown();
  }

  /** Where the joint comes to a stop from where it is and how fast, braking at its acceleration. */
  private double stoppingPoint() {
    double v = inputs.velocity;
    return clamp(measured + v * Math.abs(v) / (2 * acceleration));
  }

  /** Drives it along its profile toward a position, one loop on. */
  private void drive(double position, double fraction) {
    if (fraction != profileFraction) {
      profile = profile(fraction);
      profileFraction = fraction;
    }
    if (reseed) {
      setpoint.position = measured;
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
    EncoderLoop closed = encoderLoop;
    if (closed != null) {
      // Its loop closes here, on its encoder: the volts for the profile's point, sent as they are.
      io.setVoltage(
          closed.volts(
              setpoint.position, setpoint.velocity, kA * accelerating, measured, inputs.velocity));
    } else {
      io.setPosition(setpoint.position, setpoint.velocity, kA * accelerating);
    }
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

  /** Waits for its absolute encoder's next reading, unhomed, to tell the motor where it is. */
  private void seekAgain() {
    seeking = true;
    seekingSince = nowNanos;
    if (state != State.FAULTED || silentEncoder) {
      silentEncoder = false;
      setState(State.UNHOMED, null);
    }
  }

  /**
   * Tells its motor controller where the joint is from its absolute encoder's reading, or, after a
   * wait with none, faults it: it's seeded whenever the reading comes. Each loop it seeks, and the
   * encoder's word on where the joint is is logged every loop it answers, beside where the motor
   * says it is, so drift between them shows.
   */
  private void seek() {
    Sensor.Encoder model = absoluteModel;
    if (model == null) {
      return;
    }
    if (absoluteInputs.connected) {
      double at = math.fromAbsolute(model, absoluteReading());
      io.resetPosition(at);
      holdPosition = clamp(at);
      // Its motor's new count is read from the next loop; a joint whose loop closes on its encoder
      // reads where it is from the encoder this loop already.
      justHomed = encoderLoop == null;
      heldOnMotor = Double.NaN;
      seeking = false;
      if (state == State.UNHOMED || silentEncoder) {
        silentEncoder = false;
        setState(State.READY, null);
      }
    } else if (state == State.UNHOMED && nowNanos - seekingSince >= ABSOLUTE_WAIT_NANOS) {
      silentEncoder = true;
      setState(
          State.FAULTED,
          String.format(
              java.util.Locale.ROOT,
              "%s's absolute encoder gave no reading in %.1f s%s: it's seeded when it does",
              label,
              ABSOLUTE_WAIT_NANOS / 1e9,
              absolutePort == null ? "" : ", on port " + absolutePort.port()));
    }
  }

  /**
   * Whether the absolute encoder its loop closes on was lost this loop: then the joint is faulted,
   * so its moves are refused and the interlocks that read it hold, as for a motor controller gone
   * silent, and it holds where it comes to a stop by its motor's count, on its motor controller's
   * own loop, rather than going limp; it doesn't know where it is, so it's driven nowhere else. It
   * waits for the encoder as it did at startup, and is ready again at its first reading ({@link
   * #seek}).
   */
  private void watchEncoder() {
    if (state == State.READY && !absoluteInputs.connected) {
      // Where it comes to a stop, by its motor's count, which is where it is now that the encoder
      // isn't answering.
      heldOnMotor = stoppingPoint();
      silentEncoder = true;
      seeking = true;
      seekingSince = nowNanos;
      MechanismWiring.PortEncoder port = absolutePort;
      setState(
          State.FAULTED,
          label
              + "'s absolute encoder is lost"
              + (port == null ? "" : ", on port " + port.port())
              + ": its loop closes on it, so it holds where its motor says until the encoder"
              + " answers");
    } else if (!enabled && !Double.isNaN(heldOnMotor)) {
      heldOnMotor = inputs.position; // it holds wherever it is when it's enabled
    }
  }

  /**
   * Its absolute encoder's reading, as its sensor in the definition reads: offset and direction.
   */
  private double absoluteReading() {
    MechanismWiring.PortEncoder port = absolutePort;
    return port == null ? absoluteInputs.share : port.read(absoluteInputs.share);
  }

  private void homingBegins() {
    homingSince = nowNanos;
    stillSince = -1;
    // Whether it starts on its switch, and backs off first, is seen with this loop's sensors.
    homingFresh = true;
    backingOff = false;
    Sensor.Switch limit = homingSwitch();
    String toward =
        homing instanceof MechanismControl.Homing.ToHardStop recipe
            ? "its " + end(recipe.stop()) + " stop"
            : limit != null ? limit.label() : "its switch";
    setState(
        State.HOMING,
        String.format(
            java.util.Locale.ROOT, "toward %s, for at most %.1f s", toward, homingSeconds));
  }

  private static String end(MechanismControl.Homing.Stop stop) {
    return stop == MechanismControl.Homing.Stop.LOWER ? "lower" : "upper";
  }

  /**
   * Where its homing's timeout came from, when it was worked out from the joint, for a fault to
   * say: " (its travel takes 6.4 s at 2.0 V)". Empty when its recipe gave one.
   */
  private String timedFrom(Optional<Time> timeout, double volts) {
    return timeout.isPresent()
        ? ""
        : String.format(
            java.util.Locale.ROOT,
            " (its travel takes %.1f s at %.1f V)",
            MechanismControl.Homing.travelSeconds(math, joint, volts),
            volts);
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
    if (nowNanos - homingSince >= Math.round(homingSeconds * 1e9)) {
      setState(
          State.FAULTED,
          String.format(
              java.util.Locale.ROOT,
              "%s's homing found no stop in %.1f s%s, pushing toward its %s one",
              label,
              homingSeconds,
              timedFrom(recipe.timeout(), recipe.volts()),
              end(recipe.stop())));
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
      double past = Double.isNaN(homingLast) ? 0 : (measured - homingLast) / 2;
      double at = ((Sensor.Switch.Limit) limit.kind()).position().orElseThrow() + past;
      io.resetPosition(at);
      holdPosition = clamp(at);
      justHomed = true;
      setState(State.READY, null);
      return;
    }
    homingLast = backingOff ? Double.NaN : measured;
    if (nowNanos - homingSince >= Math.round(homingSeconds * 1e9)) {
      setState(
          State.FAULTED,
          String.format(
              java.util.Locale.ROOT,
              "%s's homing didn't %s %s in %.1f s%s%s",
              label,
              backingOff ? "back off" : "trip",
              limit.label(),
              homingSeconds,
              timedFrom(to.timeout(), to.volts()),
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
    Sensor.Encoder model = absoluteModel;
    if (model != null && absoluteInputs.connected) {
      Logger.recordOutput(absoluteAtKey, math.fromAbsolute(model, absoluteReading()));
    } else if (!Double.isNaN(inputs.absolutePosition)) {
      // An encoder its motor controller reads: where it says the joint is, as the controller read.
      Logger.recordOutput(absoluteAtKey, inputs.absolutePosition);
    }
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
      return joint.shown(measured);
    }
    double at = measured;
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
