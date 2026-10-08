package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Value;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.physical.Vector3;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

/**
 * A mechanism's joint: one way a link moves on what it hangs from, turning about an axis ({@link
 * Revolute}, in radians) or sliding along one ({@link Prismatic}, in meters). Joints hang from the
 * chassis or from each other, each written after its parent, so they form a tree, and an arm on an
 * elevator is a joint whose parent is the elevator's.
 *
 * <p>Position 0 is the pose its {@link #origin} gives: an arm's zero is wherever CAD's frame put
 * it. Where its weight pulls hardest comes from its links' mass ({@code RobotMath.gravityOffset}),
 * or from its control, beside its tuned kG ({@code MechanismControl.withKg(kG, angle)}); with
 * neither, it's its zero, horizontal as WPILib takes an arm's. A joint has no state: where it is
 * now comes from its sensors, as a position the derived functions take.
 *
 * <p>It names its own places and outputs ({@link #positions}, {@link #outputs}), and from those,
 * what configurations and interlocks say about it: {@link #at}, {@link #atLeast(Position)}, {@link
 * #atMost(Position)} and {@link #between}.
 *
 * <p>Its block says each thing once. What a joint has one of (its link, its positions, its outputs,
 * its motors, its absolute encoder, a limit switch at each end, its control) is refused said again,
 * in its block or on the finished joint, saying how to change it; what it has several of (servos,
 * interlocks) adds up. Its hard stops and where it rests are each one verb, {@code range(...)} and
 * {@code rest(...)}, that says them in its block and changes them on the finished joint. A finished
 * joint is changed one thing at a time, the rest kept: its motors re-wired ({@link #motorsOn}) or
 * re-geared ({@code gearing}), its absolute encoder re-wired ({@code encoderOn}) or offset
 * otherwise ({@code encoderOffset}), a limit switch ({@link #limitSwitchOn}), its control re-tuned
 * ({@link #retuned}), one of its named positions or outputs placed otherwise, its name kept ({@code
 * replaced}), its hard stops moved, where it rests kept ({@code range(...)}), or where it rests
 * moved, its stops kept ({@code rest(...)}). That's how a practice robot is written, so a CAD
 * revision that regears the competition robot reaches it.
 *
 * <p>Its positions and outputs are names on it, and their values the robot's: code that names one,
 * a move, a configuration or an interlock, finds it where the robot running it puts it ({@link
 * #own(Position)}), so the same code drives a practice robot to its own places. A constant of one
 * of its names is its own at a value the joint has, or had before it was re-placed: the competition
 * robot's, which the team's constants hold. At any other value it's another joint's, or another
 * robot's, and refused, wherever it's given.
 */
public sealed interface Joint extends Part permits Joint.Revolute, Joint.Prismatic {
  /**
   * An axis about which a link whose zero points forward swings up as its position grows: about the
   * robot's y, the other way round.
   */
  Vector3 PITCH_UP = new Vector3(0, -1, 0);

  /** An axis about which a link whose zero points forward swings down as its position grows. */
  Vector3 PITCH_DOWN = new Vector3(0, 1, 0);

  /**
   * A pivot: an arm, an intake's swing, a wrist, turning about an axis. Its block says the rest,
   * one call at a time: {@code Joint.pivot("pivot", Part.CHASSIS, at,
   * Joint.PITCH_DOWN).range(Range.degrees(0, 135)).rest(PIVOT_IN).carries(link).positions(PIVOT_IN,
   * PIVOT_OUT).motor(Motor.KRAKEN_X60, Gearing.of(25)).control(...)}.
   *
   * @param at where its axle is, in its parent's frame
   */
  static Revolute pivot(String label, Part parent, Translation3d at, Vector3 axis) {
    return Revolute.builder()
        .label(label)
        .parent(parent)
        .origin(new Transform3d(at, Rotation3d.ZERO))
        .axis(axis)
        .link(Link.unseen(label))
        .build();
  }

  /**
   * A roller, a belt or a flywheel: a joint that turns freely, from 0, run at its outputs. {@code
   * Joint.roller("intake", PIVOT, at).outputs(INTAKING).motor(Motor.KRAKEN_X60, Gearing.of(3))}.
   *
   * @param at where its axle is, in its parent's frame
   */
  static Revolute roller(String label, Part parent, Translation3d at) {
    return pivot(label, parent, at, PITCH_DOWN);
  }

  /**
   * A roller turning about another axis: a spindexer about the vertical, {@code
   * Joint.roller("spindexer", Part.CHASSIS, at, Vector3.UP)}.
   */
  static Revolute roller(String label, Part parent, Translation3d at, Vector3 axis) {
    return pivot(label, parent, at, axis);
  }

  /**
   * An elevator: a carriage sliding straight up. {@code Joint.elevator("elevator", Part.CHASSIS,
   * at).range(Range.meters(0, 1.2)).rest(ELEVATOR_DOWN).motors(Motor.KRAKEN_X60, gearing, drum,
   * 2)}.
   *
   * @param at where the slide starts, in its parent's frame
   */
  static Prismatic elevator(String label, Part parent, Translation3d at) {
    return slide(label, parent, at, Vector3.UP);
  }

  /**
   * A joint sliding along any axis: a hopper that extends forward, {@code Joint.slide("hopper",
   * Part.CHASSIS, at, new Vector3(1, 0, 0))}.
   *
   * @param at where the slide starts, in its parent's frame
   */
  static Prismatic slide(String label, Part parent, Translation3d at, Vector3 axis) {
    return Prismatic.builder()
        .label(label)
        .parent(parent)
        .origin(new Transform3d(at, Rotation3d.ZERO))
        .axis(axis)
        .link(Link.unseen(label))
        .build();
  }

  /** What logs and writers call it: unique among the robot's joints. */
  @Override
  String label();

  /** What it hangs from: the chassis, or another joint's link. */
  Part parent();

  /** Where its frame is in its parent's frame at position 0: the pivot, or the slide's start. */
  Transform3d origin();

  /** The axis it turns about or slides along, in its own frame; any length but zero. */
  Vector3 axis();

  /** Its hard stops, unwrapped; empty for one that turns freely, a roller or a flywheel. */
  Optional<Range> range();

  /**
   * Where it rests as the robot starts, as its block says it ({@code rest(...)}); none for one
   * whose block doesn't, which starts at its zero. A practice robot's moves it ({@code rest(...)}).
   */
  Optional<Double> rest();

  /**
   * Where it starts: where the robot is weighed and modeled (stowed), and where a mechanism that
   * zeroes itself at boot is taken to be. Where it rests ({@link #rest}), or 0. Inside its range.
   */
  default double restPosition() {
    return rest().orElse(0.0);
  }

  /** What it moves. */
  Link link();

  /**
   * Its places by name, where code moves it to: an intake's "in", "run" and "out". Inside its
   * range; a joint that follows another has none, since its leader's moves carry it. Code names
   * each by its name, and each robot's joint puts it where that robot's is ({@code replaced}).
   */
  List<Position> positions();

  /**
   * What code runs it at, by name: a roller's "intaking" at a speed, its "outtaking" at a voltage,
   * each within what its motors can do. Not {@link Output#STOPPED}, which every joint has.
   */
  List<Output> outputs();

  /**
   * Where its positions were before they were re-placed ({@code replaced}), each value one had: on
   * a practice robot, the competition robot's, which the team's constants hold, so a constant at
   * one is still this joint's own. None on a joint whose places are where its block put them.
   */
  List<Position> formerPositions();

  /** What its outputs were before they were re-valued ({@code replaced}), as its positions. */
  List<Output> formerOutputs();

  /**
   * Whether it's held at positions: it has a range or named positions. One without either, a
   * roller, only runs at outputs, and stops when idle.
   */
  default boolean holdsPositions() {
    return range().isPresent() || !positions().isEmpty();
  }

  /**
   * How it's driven and wired: its motors or servos, its own sensors, their wiring, and its
   * control. Not part of its identity: joints that differ only here are equal.
   */
  Driven driven();

  /** How its frame moves from its origin's at a position: turned about, or slid along, its axis. */
  Transform3d motion(double position);

  /**
   * How the program drives it, said once in its block: {@code
   * .control(MechanismControl.position(30))}.
   *
   * @throws IllegalArgumentException if it has a control already: {@link #retuned} changes it
   */
  Joint control(MechanismControl control);

  /**
   * The same joint, its control changed and the rest of it kept: a practice robot's arm, lighter,
   * {@code ARM.retuned(control -> control.withKg(0.3))}. Equal to this one, so a robot that has it
   * takes this one in its place ({@code RobotDefinition.with}), and a change to the competition
   * robot's control reaches it.
   *
   * @throws IllegalArgumentException if it has no control to change
   */
  Joint retuned(UnaryOperator<MechanismControl> change);

  /**
   * The same joint, its motors on other controllers, the leader first, their motor, count and
   * gearing kept: a practice robot's, {@code PIVOT.motorsOn(TALON_FX.can(33).pdh(4, Amps.of(40)))}.
   * With none, they run in the simulator alone.
   *
   * @throws IllegalArgumentException if no motors turn it, or the controllers aren't one each
   */
  Joint motorsOn(MechanismWiring.Device... controllers);

  /**
   * The same joint, its limit switch at an end on another input, where it trips kept: {@code
   * ELEVATOR.limitSwitchOn(End.LOWER, SensorWiring.port(5).normallyClosed())}.
   *
   * @throws IllegalArgumentException if it has no limit switch at that end
   */
  Joint limitSwitchOn(Sensor.Switch.Limit.End end, SensorWiring.Connection connection);

  /**
   * The same joint, its limit switch at an end on its own motor controller's limit pin, where it
   * trips kept: {@code ELEVATOR.limitSwitchOn(End.LOWER, Direction.REVERSE.normallyClosed())}.
   *
   * @throws IllegalArgumentException if it has no limit switch at that end
   */
  Joint limitSwitchOn(Sensor.Switch.Limit.End end, SensorWiring.Pin pin);

  /**
   * A position on it, or a distance along it, as every message shows one: degrees for a joint that
   * turns, meters for one that slides, to four significant figures ("130°", "0.4 m"), and no finer
   * than a hundredth of a degree or a tenth of a millimeter.
   *
   * @param value in its units: radians, or meters
   */
  default String shown(double value) {
    return Places.shownAt(this instanceof Revolute, value);
  }

  /**
   * Its own drives, as its block says them: its motors, under its label, and each servo, under its
   * own. A drive between joints, a differential's, is the robot's ({@code RobotDefinition.drives}).
   */
  List<Drive> ownDrives();

  /**
   * Its own sensors, as its block says them: its absolute encoder ("arm absolute encoder"), and its
   * limit switches ("arm lower limit", "arm upper limit"), each carrying its wiring. A sliding
   * joint's absolute encoder turns with its drum, so it's one of its sensors once its motors say
   * the drum's pitch radius; without them, the robot's definition refuses it ({@code Blocks}).
   */
  default List<Sensor> ownSensors() {
    List<Sensor> sensors = new ArrayList<>();
    Optional<Distance> drum =
        this instanceof Prismatic
            ? driven().motors().flatMap(Driven.Motors::pitchRadius)
            : Optional.empty();
    driven()
        .absoluteEncoder()
        .filter(encoder -> this instanceof Revolute || drum.isPresent())
        .ifPresent(
            encoder ->
                sensors.add(
                    Sensor.Encoder.builder()
                        .label(label() + " absolute encoder")
                        .terms(
                            List.of(
                                Transmission.Term.builder()
                                    .joint(this)
                                    .ratio(encoder.ratio() / encoder.turnsPerSignal())
                                    .pitchRadius(drum)
                                    .build()))
                        .absolute(true)
                        .build()));
    for (Driven.LimitSwitch each : driven().limitSwitches()) {
      Sensor.Switch made =
          each.position().isPresent()
              ? Sensor.Switch.limit(limitLabel(each.end()), this, each.end(), each.position().get())
              : Sensor.Switch.limit(limitLabel(each.end()), this, each.end());
      sensors.add(each.wiring().map(made::on).orElse(made));
    }
    return sensors;
  }

  /**
   * Its devices on the CAN bus, as its block wires them: its motor controllers, the leader first,
   * each "pivot motor 32" with the PDH channel that feeds it where its block says ({@code
   * TALON_FX.can(32).pdh(4, Amps.of(40))}); then its CANcoder, "pivot CANcoder", on its motors'
   * bus. None for a joint that runs in the simulator alone.
   */
  default List<CanDevice> devices() {
    List<CanDevice> devices = new ArrayList<>();
    List<MechanismWiring.Device> controllers =
        driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    for (MechanismWiring.Device device : controllers) {
      devices.add(
          new CanDevice(
              label() + " motor " + device.id(),
              device.controller().kind(),
              device.bus(),
              device.id(),
              device.feed()));
    }
    Optional<MechanismWiring.Cancoder> cancoder =
        driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder);
    if (cancoder.isPresent() && !controllers.isEmpty()) {
      devices.add(
          new CanDevice(
              label() + " CANcoder",
              CanDevice.Kind.CANCODER,
              cancoder.get().bus(),
              cancoder.get().id(),
              Optional.empty()));
    }
    return devices;
  }

  /**
   * Its own interlocks, as its block says them ({@code .keepsOut(...).when(...)}, {@code
   * .runsOnlyWhen(...)}), each about this joint as the robot has it.
   */
  default List<Interlock> ownInterlocks() {
    List<Interlock> own = new ArrayList<>();
    for (Interlock interlock : driven().interlocks()) {
      own.add(
          switch (interlock) {
            case Interlock.RunsOnlyWhen runs -> Interlock.that(this).runsOnlyWhen(runs.condition());
            case Interlock.KeepsOut keeps ->
                Interlock.that(this).keepsOut(keeps.region()).when(keeps.condition());
          });
    }
    return own;
  }

  /**
   * A stretch of a joint's travel being kept out, in its block: say while what. Not an interlock
   * until it's said.
   *
   * @param joint the joint
   * @param region where it may not be
   * @param adding the joint with the interlock added
   */
  record KeepingOut<J extends Joint>(J joint, Range region, Function<Interlock, J> adding) {
    /** While the condition holds: the joint, kept out meanwhile. */
    public J when(Condition condition) {
      return adding.apply(Interlock.that(joint).keepsOut(region).when(condition));
    }
  }

  /**
   * Its limit switch at an end, from its block: what a condition on it is made from, {@code
   * PIVOT.switchAt(End.LOWER).tripped()}.
   *
   * @throws IllegalArgumentException if it has none there
   */
  default Sensor.Switch switchAt(Sensor.Switch.Limit.End end) {
    for (Sensor sensor : ownSensors()) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
          && limit.end() == end) {
        return onSwitch;
      }
    }
    throw new IllegalArgumentException(
        label() + " has no limit switch at its " + end.name().toLowerCase(Locale.ROOT) + " end");
  }

  /** A limit switch's label, after its joint and its end: "arm lower limit". */
  private String limitLabel(Sensor.Switch.Limit.End end) {
    return label() + " " + end.name().toLowerCase(Locale.ROOT) + " limit";
  }

  /**
   * Its position by a name.
   *
   * @throws IllegalArgumentException if it has none by the name, naming those it has
   */
  default Position position(String name) {
    for (Position position : positions()) {
      if (position.name().equals(name)) {
        return position;
      }
    }
    throw new IllegalArgumentException(
        label() + " has no position named \"" + name + "\": " + has(names(positions(), List.of())));
  }

  /**
   * Its output by a name: one of its own, or {@link Output#STOPPED}, which every joint has.
   *
   * @throws IllegalArgumentException if it has none by the name, naming those it has
   */
  default Output output(String name) {
    for (Output output : outputs()) {
      if (output.name().equals(name)) {
        return output;
      }
    }
    if (name.equals(Output.STOPPED_NAME)) {
      return Output.STOPPED;
    }
    throw new IllegalArgumentException(
        label()
            + " has no output named \""
            + name
            + "\": "
            + has(names(List.of(), outputs()))
            + ", and "
            + Output.STOPPED_NAME);
  }

  /**
   * It at one of its positions: what a configuration is made of ({@code Configuration.of("amp",
   * ELEVATOR.at(ELEVATOR_CLEAR), ARM.at(ARM_SCORING))}). It's exact, and a measured position never
   * is, so an interlock refuses it: its edge is {@code atLeast}, {@code atMost} or {@code between}.
   * A position not its own is refused where the condition is used ({@link Condition}).
   */
  default Condition.At at(Position position) {
    return ConditionAtBuilder.builder().joint(this).position(position).build();
  }

  /**
   * It at one of its outputs, or {@link Output#STOPPED}: what a configuration is made of ({@code
   * Configuration.of("feeding", FEEDER.at(FEEDER_FEEDING), INDEXER.at(INDEXER_FEEDING))}). An
   * output not its own is refused where the condition is used ({@link Condition}).
   */
  default Condition.At at(Output output) {
    return ConditionAtBuilder.builder().joint(this).output(output).build();
  }

  /**
   * It at one of its positions or beyond it, toward its range's upper end: {@code
   * PIVOT.atLeast(PIVOT_OUT)}, from "out" on. An edge that's a clearance rather than a place it
   * goes is written as a number ({@link Revolute#atLeast(Angle)}). A position not its own is
   * refused where the condition is used ({@link Condition}).
   *
   * @throws IllegalArgumentException for its lower end
   */
  default Condition.AtLeast atLeast(Position position) {
    return ConditionAtLeastBuilder.builder()
        .joint(this)
        .position(where(position))
        .place(position)
        .build();
  }

  /**
   * It at one of its positions or short of it, toward its range's lower end: {@code
   * ELEVATOR.atMost(ELEVATOR_CLEAR)}, up to "clear". A position not its own is refused where the
   * condition is used ({@link Condition}).
   *
   * @throws IllegalArgumentException for its upper end
   */
  default Condition.AtMost atMost(Position position) {
    return ConditionAtMostBuilder.builder()
        .joint(this)
        .position(where(position))
        .place(position)
        .build();
  }

  /**
   * It between two of its positions, both included, whichever way round they're given. A position
   * not its own is refused where the condition is used ({@link Condition}).
   *
   * @throws IllegalArgumentException if they're one
   */
  default Condition.Between between(Position from, Position to) {
    return ConditionBetweenBuilder.builder()
        .joint(this)
        .from(where(from))
        .to(where(to))
        .fromPlace(from)
        .toPlace(to)
        .build();
  }

  /**
   * Where it puts a position of its own; one not its own, where that says, for what uses the
   * condition naming it to refuse.
   */
  private double where(Position position) {
    return whyNotOwn(position) == null ? own(position).value() : position.value();
  }

  /**
   * Its own position of a position's name, where this joint puts it: the constant a team's code
   * names, as the robot running it places it. A practice robot's joint, its position re-placed
   * ({@code replaced}), gives its own.
   *
   * @throws IllegalArgumentException if it isn't its own ({@link #whyNotOwn(Position)})
   */
  default Position own(Position position) {
    String why = whyNotOwn(position);
    if (why != null) {
      throw new IllegalArgumentException(why);
    }
    return position(position.name());
  }

  /**
   * Its own output of an output's name, at this joint's value, or {@link Output#STOPPED}: a
   * practice robot's joint, its output re-valued ({@code replaced}), gives its own.
   *
   * @throws IllegalArgumentException if it isn't its own ({@link #whyNotOwn(Output)})
   */
  default Output own(Output output) {
    String why = whyNotOwn(output);
    if (why != null) {
      throw new IllegalArgumentException(why);
    }
    return output(output.name());
  }

  /**
   * Why a position isn't one of this joint's own, or null when it is. It's its own when the joint
   * has a position of its name and kind, and it's where that one is, or was before it was re-placed
   * ({@link #formerPositions}): the competition robot's constant, on its practice robot. Another
   * joint's of the same name, elsewhere, is refused, saying where this joint's is, and so is one of
   * a name the joint hasn't, naming those it has. Two joints' places of one name and value are each
   * one's: nothing tells them apart, and either drives each to its own.
   */
  default @Nullable String whyNotOwn(Position position) {
    boolean revolute = position.kind() == Position.Kind.ANGLE;
    for (Position own : positions()) {
      if (!own.name().equals(position.name())) {
        continue;
      }
      if (own.kind() != position.kind()) {
        return possessive(label())
            + " "
            + own.name()
            + " is "
            + (revolute ? "a length" : "an angle")
            + ", and this "
            + own.name()
            + " "
            + (revolute ? "an angle" : "a length")
            + ": another joint's";
      }
      if (had(own, position) || formerPositions().stream().anyMatch(was -> had(was, position))) {
        return null;
      }
      return notHad(own.name(), "is at " + shown(own.value()), shown(position.value()));
    }
    return label()
        + " has no position "
        + position.name()
        + " at "
        + Places.shownAt(revolute, position.value())
        + ": "
        + has(names(positions(), List.of()));
  }

  /**
   * Why an output isn't one of this joint's own, or null when it is: {@link Output#STOPPED}, which
   * every joint has, or one of its name at what it runs at, or ran at before it was re-valued
   * ({@link #formerOutputs}), as for a position ({@link #whyNotOwn(Position)}).
   */
  default @Nullable String whyNotOwn(Output output) {
    if (output.name().equals(Output.STOPPED_NAME)) {
      return null;
    }
    for (Output own : outputs()) {
      if (!own.name().equals(output.name())) {
        continue;
      }
      if (had(own, output) || formerOutputs().stream().anyMatch(was -> had(was, output))) {
        return null;
      }
      boolean revolute = this instanceof Revolute;
      return notHad(
          own.name(),
          "runs at " + Places.shownOutput(revolute, own),
          Places.shownOutput(revolute, output));
    }
    return label()
        + " has no output "
        + output.name()
        + ": "
        + has(names(List.of(), outputs()))
        + ", and "
        + Output.STOPPED_NAME;
  }

  /**
   * A place of one of its names, at a value it hasn't had: "elevator's L1 is at 0.5 m, and this L1,
   * at 0.25 m, isn't one it has had: another joint's or another robot's; give elevator's own".
   *
   * @param own how its own is: "is at 0.5 m"
   * @param given where the one given is: "0.25 m"
   */
  private String notHad(String name, String own, String given) {
    return possessive(label())
        + " "
        + name
        + " "
        + own
        + ", and this "
        + name
        + ", at "
        + given
        + ", isn't one it has had: another joint's or another robot's; give "
        + possessive(label())
        + " own";
  }

  /**
   * Whether a position is one a joint had, of its name and kind, there: to a nanometer, or a
   * nanoradian, so a place written in degrees and one in radians, a bit apart, are one.
   */
  private static boolean had(Position had, Position position) {
    return had.name().equals(position.name())
        && had.kind() == position.kind()
        && Math.abs(had.value() - position.value()) <= Range.EDGE;
  }

  /** Whether an output is one a joint had, of its name and kind, at its value, as a position. */
  private static boolean had(Output had, Output output) {
    return had.name().equals(output.name())
        && had.kind() == output.kind()
        && had.motion().equals(output.motion())
        && Math.abs(had.value() - output.value()) <= Range.EDGE;
  }

  /**
   * A joint that turns about its axis, in radians: an arm, a wrist, a turret, a roller.
   *
   * @param label what logs and writers call it: unique among the robot's joints
   * @param parent what it hangs from: the chassis, or another joint's link
   * @param origin where its frame is in its parent's frame at position 0: the pivot
   * @param axis what it turns about, in its own frame, positive by the right-hand rule
   * @param range its hard stops, in radians, unwrapped; empty for one that turns freely
   * @param rest where it rests as the robot starts, in radians, if its block says: stowed, inside
   *     its range; none starts it at 0
   * @param link what it moves
   * @param positions its places by name, in radians, inside its range: {@code
   *     Position.degrees("out", 130)}
   * @param outputs what code runs it at, by name: {@code Output.rpm("intaking", 2000)}, {@code
   *     Output.volts("push", 6)}
   * @param formerPositions where its positions were before they were re-placed ({@code replaced}):
   *     the competition robot's, on a practice robot, still its own
   * @param formerOutputs what its outputs were before they were re-valued, still its own
   * @param driven how it's driven and wired, its own sensors, and its control: its block's {@code
   *     .motor(...)} and the rest; not part of its identity
   */
  @ConfigRecord
  record Revolute(
      String label,
      Part parent,
      Transform3d origin,
      Vector3 axis,
      Optional<Range> range,
      Optional<Double> rest,
      Link link,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Position> positions,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Output> outputs,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Position> formerPositions,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Output> formerOutputs,
      @RecordBuilder.Initializer(value = "NONE", source = Driven.class) Driven driven)
      implements Joint {
    public Revolute {
      positions = List.copyOf(positions);
      outputs = List.copyOf(outputs);
      formerPositions = List.copyOf(formerPositions);
      formerOutputs = List.copyOf(formerOutputs);
      check(Revolute.class, label, origin, axis, range, rest, positions, outputs);
      Optional<Driven.Motors> motors = driven.motors();
      Optional<MechanismControl> control = driven.control();
      Checks.on(Revolute.class, label)
          .check(
              "driven.motors",
              () ->
                  motors.isPresent() && motors.get().pitchRadius().isPresent()
                      ? "a joint that turns has no drum: give its motors a gearing alone"
                      : null)
          .check(
              "driven.control",
              () -> control.map(c -> c.ofTheOtherKind(Position.Kind.ANGLE)).orElse(null))
          .orThrow();
    }

    /** Starts describing a revolute joint, one named value at a time. */
    public static JointRevoluteBuilder.LabelStage builder() {
      return JointRevoluteBuilder.builder();
    }

    @Override
    public Transform3d motion(double position) {
      return new Transform3d(
          Translation3d.ZERO,
          new Rotation3d(VecBuilder.fill(axis.x(), axis.y(), axis.z()), position));
    }

    @Override
    public List<Drive> ownDrives() {
      return drives(
          this,
          motors -> Transmission.Geared.builder().joint(this).gearing(motors.gearing()).build());
    }

    /**
     * Equal to another joint the same in everything but how it's driven ({@link #driven}), where
     * its hard stops are, where it rests, and where its named places are: its positions and outputs
     * are the same names, their values each robot's own ({@code replaced}), and it has stops or
     * none, and rests, each robot's own ({@code range(range)}, {@code rest(...)}).
     */
    @Override
    public boolean equals(@Nullable Object other) {
      return other instanceof Revolute joint
          && label.equals(joint.label)
          && parent.equals(joint.parent)
          && origin.equals(joint.origin)
          && axis.equals(joint.axis)
          && range.isPresent() == joint.range.isPresent()
          && link.equals(joint.link)
          && names(positions, List.of()).equals(names(joint.positions, List.of()))
          && names(List.of(), outputs).equals(names(List.of(), joint.outputs));
    }

    @Override
    public int hashCode() {
      return Objects.hash(
          label,
          parent,
          origin,
          axis,
          range.isPresent(),
          link,
          names(positions, List.of()),
          names(List.of(), outputs));
    }

    /**
     * Its hard stops: {@code .range(Range.degrees(0, 135))}. On a finished joint, its stops
     * elsewhere, where it rests kept: a practice robot's, {@code TURRET.range(Range.degrees(-355,
     * 5))}. Every check of the robot runs on them, as it's built: where it rests, its positions,
     * its soft limits, its interlocks' edges and regions inside them. So its places re-placed
     * ({@code replaced}) may come before or after.
     */
    public Revolute range(Range range) {
      return with().range(range).build();
    }

    /**
     * Where it rests as the robot starts, stowed: {@code .rest(Degrees.of(-30))}. Without one, it
     * rests at its zero, which must then be inside its stops. On a finished joint, it rests
     * elsewhere, its stops kept: a practice robot's arm, stowed lower. It must be inside its stops,
     * and every check of the robot runs on it as it's built: the interlocks as the robot starts.
     */
    public Revolute rest(Angle rest) {
      return with().rest(rest.in(Radians)).build();
    }

    /**
     * Where it rests as the robot starts, at one of its positions: {@code .rest(ARM_STOWED)}, its
     * own of that name, where this joint puts it, so after {@code replaced(ARM_STOWED, ...)}, at
     * the practice robot's. In its block before its positions are named, where the position is.
     *
     * @throws IllegalArgumentException if the position isn't its own, or isn't an angle
     */
    public Revolute rest(Position rest) {
      return with().rest(restAt(this, rest)).build();
    }

    /**
     * It at an angle or beyond it: what an interlock's edge usually is, a clearance rather than a
     * place it goes, and the same wherever it rests: {@code PIVOT.atLeast(Degrees.of(110))}.
     *
     * @throws IllegalArgumentException if it's outside its range, or its lower end
     */
    public Condition.AtLeast atLeast(Angle position) {
      return ConditionAtLeastBuilder.builder().joint(this).position(position.in(Radians)).build();
    }

    /**
     * It at an angle or short of it, as {@link #atLeast(Angle)} the other way: {@code
     * ARM.atMost(Degrees.of(-40))}.
     *
     * @throws IllegalArgumentException if it's outside its range, or its upper end
     */
    public Condition.AtMost atMost(Angle position) {
      return ConditionAtMostBuilder.builder().joint(this).position(position.in(Radians)).build();
    }

    /**
     * It between two angles, both included, whichever way round they're given.
     *
     * @throws IllegalArgumentException if either is outside its range, or they're one
     */
    public Condition.Between between(Angle from, Angle to) {
      return ConditionBetweenBuilder.builder()
          .joint(this)
          .from(from.in(Radians))
          .to(to.in(Radians))
          .build();
    }

    /**
     * It running at least a speed, forward: {@code SHOOTER.upToSpeed(RPM.of(3000))}.
     *
     * @throws IllegalArgumentException for a speed that isn't forward
     */
    public Condition.UpToSpeed upToSpeed(AngularVelocity speed) {
      return ConditionUpToSpeedBuilder.builder()
          .joint(this)
          .speed(speed.in(RadiansPerSecond))
          .build();
    }

    /** What it moves. */
    public Revolute carries(Link link) {
      onceLink(this);
      return with().link(link).build();
    }

    /** Its places by name: {@code .positions(ARM_STOWED, ARM_SCORING)}. */
    public Revolute positions(Position... positions) {
      oncePositions(this);
      return with().positions(List.of(positions)).build();
    }

    /** What it runs at, by name: {@code .outputs(INTAKING, OUTTAKING)}. */
    public Revolute outputs(Output... outputs) {
      onceOutputs(this);
      return with().outputs(List.of(outputs)).build();
    }

    /**
     * The same joint, one of its named positions at another angle, its name kept: a practice
     * robot's, {@code PIVOT.replaced(PIVOT_OUT, Degrees.of(128))}. Code naming it, {@code
     * pivot.goTo(PIVOT_OUT)}, a configuration or an interlock, finds it there on the robot that has
     * this joint, so the same code drives each robot to its own. It must be inside the joint's
     * stops, as the robot that has it checks.
     *
     * @throws IllegalArgumentException if it has no position of the name, or the one named isn't
     *     its own: another joint's of the name, elsewhere
     */
    public Revolute replaced(Position position, Angle at) {
      return placed(position, Position.of(position.name(), at));
    }

    /**
     * The same joint, one of its named outputs at another speed, its name kept: {@code
     * FEEDER.replaced(FEEDER_FEEDING, RPM.of(1200))}. Code running it, {@code
     * feeder.run(FEEDER_FEEDING)} or a configuration, runs it there on the robot that has this
     * joint.
     *
     * @throws IllegalArgumentException if it has no output of the name
     */
    public Revolute replaced(Output output, AngularVelocity speed) {
      return valued(output, Output.of(output.name(), speed));
    }

    /**
     * The same joint, one of its named outputs at another voltage, its name kept: a practice
     * robot's, {@code ROLLER.replaced(OUTTAKING, Volts.of(-3))}.
     *
     * @throws IllegalArgumentException if it has no output of the name
     */
    public Revolute replaced(Output output, Voltage voltage) {
      return valued(output, Output.of(output.name(), voltage));
    }

    /**
     * The same joint, one of its named outputs at another share of the battery, its name kept:
     * {@code ROLLER.replaced(OUTTAKING, Percent.of(-30))}.
     *
     * @throws IllegalArgumentException if it has no output of the name
     */
    public Revolute replaced(Output output, Dimensionless share) {
      return valued(output, Output.dutyCycle(output.name(), share.in(Value)));
    }

    /**
     * The same joint, its own position of a name elsewhere, where it was among its former ones. The
     * position named is its own ({@link #own(Position)}), as anywhere a place is named.
     */
    private Revolute placed(Position position, Position replacing) {
      return with()
          .positions(replacedPosition(this, replacing))
          .formerPositions(former(formerPositions, own(position), replacing))
          .build();
    }

    /** The same joint, its own output of a name otherwise, what it was among its former ones. */
    private Revolute valued(Output output, Output replacing) {
      return with()
          .outputs(replacedOutput(this, replacing))
          .formerOutputs(former(formerOutputs, own(output), replacing))
          .build();
    }

    /** A motor turning it through a gearing, in the simulator alone until it's wired. */
    public Revolute motor(Motor motor, Gearing gearing) {
      return motors(motor, gearing, 1);
    }

    /** A motor turning it through a gearing, on a controller: {@code TALON_FX.can(32)}. */
    public Revolute motor(Motor motor, Gearing gearing, MechanismWiring.Device controller) {
      return motors(motor, gearing, controller);
    }

    /** Motors turning it together through a gearing, in the simulator alone until they're wired. */
    public Revolute motors(Motor motor, Gearing gearing, int count) {
      onceMotors(this);
      return driven(
          drivenWith().motors(turning(motor, count, gearing, Optional.empty(), List.of())));
    }

    /** Motors turning it together through a gearing, each on its controller, the leader first. */
    public Revolute motors(Motor motor, Gearing gearing, MechanismWiring.Device... controllers) {
      onceMotors(this);
      return driven(
          drivenWith()
              .motors(
                  Optional.of(
                      turning(
                          motor,
                          controllers.length,
                          gearing,
                          Optional.empty(),
                          List.of(controllers)))));
    }

    @Override
    public Revolute motorsOn(MechanismWiring.Device... controllers) {
      return driven(drivenWith().motors(rewired(this, controllers)));
    }

    /**
     * The same joint, its motors through another gearing, their controllers as they are: a second
     * robot of the design, geared otherwise.
     *
     * @throws IllegalArgumentException if no motors turn it
     */
    public Revolute gearing(Gearing gearing) {
      return driven(drivenWith().motors(regeared(this, gearing)));
    }

    /**
     * A servo pushing between its parent and its link, under its own label.
     *
     * @param onParent where its body is fixed, from the joint's axle, in the parent's frame
     * @param onLink where its rod meets the link, in the joint's frame
     */
    public Revolute servo(
        String label, Actuator.Servo servo, Translation3d onParent, Translation3d onLink) {
      List<Driven.Servo> servos = new ArrayList<>(driven.servos());
      servos.add(
          DrivenServoBuilder.builder()
              .label(label)
              .servo(servo)
              .onParent(onParent)
              .onLink(onLink)
              .build());
      return driven(drivenWith().servos(servos));
    }

    /** An absolute encoder on its axle, in the simulator alone until it's wired. */
    public Revolute absoluteEncoder() {
      return absoluteEncoder(Driven.AbsoluteEncoder.of());
    }

    /** An absolute encoder on its axle, read by a CANcoder through its TalonFX. */
    public Revolute absoluteEncoder(MechanismWiring.Cancoder cancoder) {
      return absoluteEncoder(Driven.AbsoluteEncoder.of().on(cancoder));
    }

    /**
     * An absolute encoder on its axle, on its SPARK's data port: with {@code closesLoop()} on it,
     * the SPARK's loop closing on it.
     */
    public Revolute absoluteEncoder(MechanismWiring.DataPortEncoder dataPort) {
      return absoluteEncoder(Driven.AbsoluteEncoder.of().on(dataPort));
    }

    /**
     * An absolute encoder on its axle, on one of Systemcore's ports, read by the robot: {@code
     * .absoluteEncoder(PortEncoder.dutyCycle(1).offset(Rotations.of(0.82)))}, and with {@code
     * .closesLoop()} after it, the joint's loop closing on it.
     */
    public Revolute absoluteEncoder(MechanismWiring.PortEncoder port) {
      return absoluteEncoder(Driven.AbsoluteEncoder.of().on(port));
    }

    /**
     * An absolute encoder as written in full, geared or not: {@code
     * .absoluteEncoder(AbsoluteEncoder.of().ratio(0.536).on(cancoder))}. A joint has one: a second
     * is refused, a decoded pair not being supported yet.
     */
    public Revolute absoluteEncoder(Driven.AbsoluteEncoder encoder) {
      onceEncoder(this);
      return driven(drivenWith().absoluteEncoder(encoder));
    }

    /**
     * The same joint, its absolute encoder read by another CANcoder, its ratio kept, and how its
     * loop reads it, fused, remote or synced, unless the CANcoder says its own: a practice robot's,
     * {@code TURRET.encoderOn(CANCODER.can(35).offset(Rotations.of(0.11)))}.
     *
     * @throws IllegalArgumentException if it has no absolute encoder
     */
    public Revolute encoderOn(MechanismWiring.Cancoder cancoder) {
      return encoderRewired(encoder(this).on(loopKept(this, cancoder)));
    }

    /**
     * The same joint, its absolute encoder on its SPARK's data port, its ratio kept, and its loop
     * closed on it if it was closed on the data port before.
     *
     * @throws IllegalArgumentException if it has no absolute encoder
     */
    public Revolute encoderOn(MechanismWiring.DataPortEncoder dataPort) {
      return encoderRewired(encoder(this).on(loopKept(this, dataPort)));
    }

    /**
     * The same joint, its absolute encoder on one of Systemcore's ports, its ratio kept, and its
     * loop closed on it if it was closed on a port before.
     *
     * @throws IllegalArgumentException if it has no absolute encoder
     */
    public Revolute encoderOn(MechanismWiring.PortEncoder port) {
      return encoderRewired(encoder(this).on(loopKept(this, port)));
    }

    /**
     * The same joint, its absolute encoder offset otherwise, the rest of it and its wiring kept: a
     * practice robot's magnet, {@code ARM.encoderOffset(Rotations.of(0.47))}, on the CANcoder, data
     * port or Systemcore port the robot's own block reads it on, so a CANcoder moved to another ID
     * there reaches it.
     *
     * @throws IllegalArgumentException if it has no absolute encoder, or one that isn't wired
     */
    public Revolute encoderOffset(Angle offset) {
      return encoderRewired(offset(this, offset));
    }

    private Revolute encoderRewired(Driven.AbsoluteEncoder encoder) {
      return driven(drivenWith().absoluteEncoder(encoder));
    }

    /** A limit switch at an end, tripping at an angle, in the simulator alone until it's wired. */
    public Revolute limitSwitch(Sensor.Switch.Limit.End end, Angle at) {
      return limit(end, Optional.of(at.in(Radians)), Optional.empty());
    }

    /**
     * A limit switch at an end whose position isn't known: it stops the joint, but can't home it.
     * In the simulator alone until it's wired.
     */
    public Revolute limitSwitch(Sensor.Switch.Limit.End end) {
      return limit(end, Optional.empty(), Optional.empty());
    }

    /**
     * A limit switch at an end whose position isn't known, on an input: {@code
     * .limitSwitch(End.UPPER, CANDI.can(30).s2().normallyClosed())}.
     */
    public Revolute limitSwitch(Sensor.Switch.Limit.End end, SensorWiring.Connection connection) {
      return limit(end, Optional.empty(), Optional.of(connection));
    }

    /**
     * A limit switch at an end whose position isn't known, on its own motor controller's limit pin:
     * the controller stops the motor itself. {@code .limitSwitch(End.UPPER,
     * Direction.FORWARD.normallyClosed())}.
     */
    public Revolute limitSwitch(Sensor.Switch.Limit.End end, SensorWiring.Pin pin) {
      return limitSwitch(end, pin.of(this));
    }

    /**
     * A limit switch at an end, tripping at an angle, on an input, a Systemcore port or a CANdi's:
     * {@code .limitSwitch(End.LOWER, Degrees.of(0), SensorWiring.port(1).normallyClosed())}.
     */
    public Revolute limitSwitch(
        Sensor.Switch.Limit.End end, Angle at, SensorWiring.Connection connection) {
      return limit(end, Optional.of(at.in(Radians)), Optional.of(connection));
    }

    /**
     * A limit switch at an end, tripping at an angle, on its own motor controller's limit pin: the
     * controller stops the motor itself. {@code .limitSwitch(End.LOWER, Degrees.of(0),
     * Direction.REVERSE.normallyClosed())}.
     */
    public Revolute limitSwitch(Sensor.Switch.Limit.End end, Angle at, SensorWiring.Pin pin) {
      return limitSwitch(end, at, pin.of(this));
    }

    @Override
    public Revolute limitSwitchOn(Sensor.Switch.Limit.End end, SensorWiring.Connection connection) {
      return driven(drivenWith().limitSwitches(switchRewired(this, end, connection)));
    }

    @Override
    public Revolute limitSwitchOn(Sensor.Switch.Limit.End end, SensorWiring.Pin pin) {
      return limitSwitchOn(end, pin.of(this));
    }

    /**
     * It kept out of a stretch of its travel while a condition holds, enforced beneath every
     * command: {@code .keepsOut(Range.degrees(-180, -101)).when(ELEVATOR.atMost(Inches.of(32)))}.
     * Its target is held at the stretch's edge, on the side it's on, until the condition clears.
     */
    public KeepingOut<Revolute> keepsOut(Range region) {
      return new KeepingOut<>(this, region, this::interlocked);
    }

    /**
     * It moving only while a condition holds, enforced beneath every command: {@code
     * .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))}. Meanwhile it holds where it is, or stops.
     */
    public Revolute runsOnlyWhen(Condition condition) {
      return interlocked(Interlock.that(this).runsOnlyWhen(condition));
    }

    private Revolute interlocked(Interlock interlock) {
      List<Interlock> interlocks = new ArrayList<>(driven.interlocks());
      interlocks.add(interlock);
      return driven(drivenWith().interlocks(interlocks));
    }

    @Override
    public Revolute control(MechanismControl control) {
      onceControl(this);
      return driven(drivenWith().control(control));
    }

    @Override
    public Revolute retuned(UnaryOperator<MechanismControl> change) {
      return driven(drivenWith().control(change.apply(controlOf(this))));
    }

    /** The same joint, driven otherwise. */
    public Revolute driven(Driven driven) {
      return with().driven(driven).build();
    }

    private JointRevoluteBuilder with() {
      return JointRevoluteBuilder.builder(this);
    }

    private DrivenBuilder drivenWith() {
      return DrivenBuilder.builder(driven);
    }

    private Revolute limit(
        Sensor.Switch.Limit.End end,
        Optional<Double> at,
        Optional<SensorWiring.Connection> wiring) {
      return driven(drivenWith().limitSwitches(switchAdded(this, end, at, wiring)));
    }

    private Revolute driven(DrivenBuilder driven) {
      return driven(driven.build());
    }
  }

  /**
   * A joint that slides along its axis, in meters: an elevator's carriage or stage, a telescoping
   * arm, a linear deploy.
   *
   * @param label what logs and writers call it: unique among the robot's joints
   * @param parent what it hangs from: the chassis, or another joint's link
   * @param origin where its frame is in its parent's frame at position 0: the slide's start
   * @param axis what it slides along, in its own frame
   * @param range its hard stops, in meters
   * @param rest where it rests as the robot starts, in meters, if its block says: stowed, inside
   *     its range; none starts it at 0
   * @param link what it moves
   * @param positions its places by name, in meters, inside its range: {@code Position.of("clear",
   *     Inches.of(20))}
   * @param outputs what code runs it at, by name: a speed in meters per second, a voltage, or a
   *     duty cycle
   * @param formerPositions where its positions were before they were re-placed ({@code replaced}):
   *     the competition robot's, on a practice robot, still its own
   * @param formerOutputs what its outputs were before they were re-valued, still its own
   * @param driven how it's driven and wired, its own sensors, and its control: its block's {@code
   *     .motors(...)} and the rest; not part of its identity
   */
  @ConfigRecord
  record Prismatic(
      String label,
      Part parent,
      Transform3d origin,
      Vector3 axis,
      Optional<Range> range,
      Optional<Double> rest,
      Link link,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Position> positions,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Output> outputs,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Position> formerPositions,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Output> formerOutputs,
      @RecordBuilder.Initializer(value = "NONE", source = Driven.class) Driven driven)
      implements Joint {
    public Prismatic {
      positions = List.copyOf(positions);
      outputs = List.copyOf(outputs);
      formerPositions = List.copyOf(formerPositions);
      formerOutputs = List.copyOf(formerOutputs);
      check(Prismatic.class, label, origin, axis, range, rest, positions, outputs);
      Optional<Driven.Motors> motors = driven.motors();
      Optional<MechanismControl> control = driven.control();
      Checks.on(Prismatic.class, label)
          .check(
              "driven.motors",
              () ->
                  motors.isPresent() && motors.get().pitchRadius().isEmpty()
                      ? "a joint that slides needs its drum: give its motors the pitch radius"
                      : null)
          .check(
              "driven.control",
              () ->
                  control.isPresent() && control.get().gains().gravityAngle().isPresent()
                      ? "a joint that slides is held against gravity the same all along it, so"
                          + " its kG has no angle: give kG alone, withKg(kG)"
                      : null)
          .check(
              "driven.control",
              () -> control.map(c -> c.ofTheOtherKind(Position.Kind.LENGTH)).orElse(null))
          .orThrow();
    }

    /** Starts describing a prismatic joint, one named value at a time. */
    public static JointPrismaticBuilder.LabelStage builder() {
      return JointPrismaticBuilder.builder();
    }

    @Override
    public Transform3d motion(double position) {
      Vector3 along = axis.unit().times(position);
      return new Transform3d(new Translation3d(along.x(), along.y(), along.z()), Rotation3d.ZERO);
    }

    @Override
    public List<Drive> ownDrives() {
      return drives(
          this,
          motors ->
              Transmission.Drum.builder()
                  .joint(this)
                  .gearing(motors.gearing())
                  .pitchRadius(motors.pitchRadius().orElseThrow())
                  .build());
    }

    /**
     * Equal to another joint the same in everything but how it's driven ({@link #driven}), where
     * its hard stops are, where it rests, and where its named places are: its positions and outputs
     * are the same names, their values each robot's own ({@code replaced}), and it has stops or
     * none, and rests, each robot's own ({@code range(range)}, {@code rest(...)}).
     */
    @Override
    public boolean equals(@Nullable Object other) {
      return other instanceof Prismatic joint
          && label.equals(joint.label)
          && parent.equals(joint.parent)
          && origin.equals(joint.origin)
          && axis.equals(joint.axis)
          && range.isPresent() == joint.range.isPresent()
          && link.equals(joint.link)
          && names(positions, List.of()).equals(names(joint.positions, List.of()))
          && names(List.of(), outputs).equals(names(List.of(), joint.outputs));
    }

    @Override
    public int hashCode() {
      return Objects.hash(
          label,
          parent,
          origin,
          axis,
          range.isPresent(),
          link,
          names(positions, List.of()),
          names(List.of(), outputs));
    }

    /**
     * Its hard stops: {@code .range(Range.meters(0, 1.2))}. On a finished joint, its stops
     * elsewhere, where it rests kept: a practice robot's, {@code ELEVATOR.range(Range.meters(0,
     * 1.1))}. Every check of the robot runs on them, as it's built: where it rests, its positions,
     * its soft limits, its interlocks' edges and regions inside them. So its places re-placed
     * ({@code replaced}) may come before or after.
     */
    public Prismatic range(Range range) {
      return with().range(range).build();
    }

    /**
     * Where it rests as the robot starts: {@code .rest(Inches.of(1))}. Without one, it rests at its
     * zero, which must then be inside its stops. On a finished joint, it rests elsewhere, its stops
     * kept: a practice robot's elevator, resting on a spacer. It must be inside its stops, and
     * every check of the robot runs on it as it's built.
     */
    public Prismatic rest(Distance rest) {
      return with().rest(rest.in(Meters)).build();
    }

    /**
     * Where it rests as the robot starts, at one of its positions, as a revolute joint's ({@code
     * Revolute.rest(Position)}).
     *
     * @throws IllegalArgumentException if the position isn't its own, or isn't a length
     */
    public Prismatic rest(Position rest) {
      return with().rest(restAt(this, rest)).build();
    }

    /**
     * It at a distance or beyond it: what an interlock's edge usually is, a clearance rather than a
     * place it goes, and the same wherever it rests: {@code ELEVATOR.atLeast(Inches.of(32))}.
     *
     * @throws IllegalArgumentException if it's outside its range, or its lower end
     */
    public Condition.AtLeast atLeast(Distance position) {
      return ConditionAtLeastBuilder.builder().joint(this).position(position.in(Meters)).build();
    }

    /**
     * It at a distance or short of it, as {@link #atLeast(Distance)} the other way: {@code
     * ELEVATOR.atMost(Inches.of(32))}.
     *
     * @throws IllegalArgumentException if it's outside its range, or its upper end
     */
    public Condition.AtMost atMost(Distance position) {
      return ConditionAtMostBuilder.builder().joint(this).position(position.in(Meters)).build();
    }

    /**
     * It between two distances, both included, whichever way round they're given.
     *
     * @throws IllegalArgumentException if either is outside its range, or they're one
     */
    public Condition.Between between(Distance from, Distance to) {
      return ConditionBetweenBuilder.builder()
          .joint(this)
          .from(from.in(Meters))
          .to(to.in(Meters))
          .build();
    }

    /**
     * It running at least a speed, forward: {@code LIFT.upToSpeed(MetersPerSecond.of(0.5))}.
     *
     * @throws IllegalArgumentException for a speed that isn't forward
     */
    public Condition.UpToSpeed upToSpeed(LinearVelocity speed) {
      return ConditionUpToSpeedBuilder.builder()
          .joint(this)
          .speed(speed.in(MetersPerSecond))
          .build();
    }

    /** What it moves. */
    public Prismatic carries(Link link) {
      onceLink(this);
      return with().link(link).build();
    }

    /** Its places by name: {@code .positions(ELEVATOR_DOWN, ELEVATOR_CLEAR)}. */
    public Prismatic positions(Position... positions) {
      oncePositions(this);
      return with().positions(List.of(positions)).build();
    }

    /** What it runs at, by name. */
    public Prismatic outputs(Output... outputs) {
      onceOutputs(this);
      return with().outputs(List.of(outputs)).build();
    }

    /**
     * The same joint, one of its named positions at another distance, its name kept: a practice
     * robot's, {@code ELEVATOR.replaced(L1, Meters.of(0.5))}. Code naming it, {@code
     * elevator.goTo(L1)}, a configuration or an interlock, finds it there on the robot that has
     * this joint, so the same code drives each robot to its own. It must be inside the joint's
     * stops, as the robot that has it checks.
     *
     * @throws IllegalArgumentException if it has no position of the name, or the one named isn't
     *     its own: another joint's of the name, elsewhere
     */
    public Prismatic replaced(Position position, Distance at) {
      return placed(position, Position.of(position.name(), at));
    }

    /**
     * The same joint, one of its named outputs at another speed, its name kept.
     *
     * @throws IllegalArgumentException if it has no output of the name
     */
    public Prismatic replaced(Output output, LinearVelocity speed) {
      return valued(output, Output.of(output.name(), speed));
    }

    /**
     * The same joint, one of its named outputs at another voltage, its name kept.
     *
     * @throws IllegalArgumentException if it has no output of the name
     */
    public Prismatic replaced(Output output, Voltage voltage) {
      return valued(output, Output.of(output.name(), voltage));
    }

    /**
     * The same joint, one of its named outputs at another share of the battery, its name kept.
     *
     * @throws IllegalArgumentException if it has no output of the name
     */
    public Prismatic replaced(Output output, Dimensionless share) {
      return valued(output, Output.dutyCycle(output.name(), share.in(Value)));
    }

    /**
     * The same joint, its own position of a name elsewhere, where it was among its former ones. The
     * position named is its own ({@link #own(Position)}), as anywhere a place is named.
     */
    private Prismatic placed(Position position, Position replacing) {
      return with()
          .positions(replacedPosition(this, replacing))
          .formerPositions(former(formerPositions, own(position), replacing))
          .build();
    }

    /** The same joint, its own output of a name otherwise, what it was among its former ones. */
    private Prismatic valued(Output output, Output replacing) {
      return with()
          .outputs(replacedOutput(this, replacing))
          .formerOutputs(former(formerOutputs, own(output), replacing))
          .build();
    }

    /**
     * A motor driving it through a gearing onto a drum, in the simulator alone until it's wired.
     *
     * @param pitchRadius where the belt, chain or rack meets the drum, sprocket or pinion
     */
    public Prismatic motor(Motor motor, Gearing gearing, Distance pitchRadius) {
      return motors(motor, gearing, pitchRadius, 1);
    }

    /** A motor driving it through a gearing onto a drum, on a controller. */
    public Prismatic motor(
        Motor motor, Gearing gearing, Distance pitchRadius, MechanismWiring.Device controller) {
      return motors(motor, gearing, pitchRadius, controller);
    }

    /** Motors driving it together onto a drum, in the simulator alone until they're wired. */
    public Prismatic motors(Motor motor, Gearing gearing, Distance pitchRadius, int count) {
      onceMotors(this);
      return driven(
          drivenWith()
              .motors(
                  Optional.of(turning(motor, count, gearing, Optional.of(pitchRadius), List.of())))
              .build());
    }

    /** Motors driving it together onto a drum, each on its controller, the leader first. */
    public Prismatic motors(
        Motor motor, Gearing gearing, Distance pitchRadius, MechanismWiring.Device... controllers) {
      onceMotors(this);
      return driven(
          drivenWith()
              .motors(
                  Optional.of(
                      turning(
                          motor,
                          controllers.length,
                          gearing,
                          Optional.of(pitchRadius),
                          List.of(controllers))))
              .build());
    }

    @Override
    public Prismatic motorsOn(MechanismWiring.Device... controllers) {
      return driven(drivenWith().motors(rewired(this, controllers)).build());
    }

    /**
     * The same joint, its motors through another gearing onto the same drum, their controllers as
     * they are.
     *
     * @throws IllegalArgumentException if no motors drive it
     */
    public Prismatic gearing(Gearing gearing) {
      return driven(drivenWith().motors(regeared(this, gearing)).build());
    }

    /**
     * A servo pushing between its parent and its link, under its own label.
     *
     * @param onParent where its body is fixed, from the slide's start, in the parent's frame
     * @param onLink where its rod meets the link, in the joint's frame
     */
    public Prismatic servo(
        String label, Actuator.Servo servo, Translation3d onParent, Translation3d onLink) {
      List<Driven.Servo> servos = new ArrayList<>(driven.servos());
      servos.add(
          DrivenServoBuilder.builder()
              .label(label)
              .servo(servo)
              .onParent(onParent)
              .onLink(onLink)
              .build());
      return driven(drivenWith().servos(servos).build());
    }

    /** An absolute encoder on its drum's shaft, in the simulator alone until it's wired. */
    public Prismatic absoluteEncoder() {
      return absoluteEncoder(Driven.AbsoluteEncoder.of());
    }

    /**
     * An absolute encoder on its drum's shaft, read by a CANcoder through its TalonFX. One turn of
     * it must cover the joint's travel, which a drum's seldom does: one geared down from the drum
     * is written in full ({@link #absoluteEncoder(Driven.AbsoluteEncoder)}).
     */
    public Prismatic absoluteEncoder(MechanismWiring.Cancoder cancoder) {
      return absoluteEncoder(Driven.AbsoluteEncoder.of().on(cancoder));
    }

    /**
     * An absolute encoder on its drum's shaft, on its SPARK's data port: with {@code closesLoop()}
     * on it, the SPARK's loop closing on it.
     */
    public Prismatic absoluteEncoder(MechanismWiring.DataPortEncoder dataPort) {
      return absoluteEncoder(Driven.AbsoluteEncoder.of().on(dataPort));
    }

    /**
     * An absolute encoder on its drum's shaft, on one of Systemcore's ports, read by the robot: a
     * ten-turn potentiometer, {@code .absoluteEncoder(PortEncoder.analog(2, 10))}, and with {@code
     * .closesLoop()} after it, the joint's loop closing on it.
     */
    public Prismatic absoluteEncoder(MechanismWiring.PortEncoder port) {
      return absoluteEncoder(Driven.AbsoluteEncoder.of().on(port));
    }

    /**
     * An absolute encoder as written in full, geared to its drum: {@code
     * .absoluteEncoder(AbsoluteEncoder.of().ratio(1.0 / 12).on(cancoder))}, turning once for twelve
     * turns of the drum. Its ratio is its turns for each of the drum's, the one its motors' pitch
     * radius names, so one of its turns covers the joint's whole travel, which the robot's
     * definition checks. A joint has one: a second is refused.
     */
    public Prismatic absoluteEncoder(Driven.AbsoluteEncoder encoder) {
      onceEncoder(this);
      return driven(drivenWith().absoluteEncoder(encoder).build());
    }

    /**
     * The same joint, its absolute encoder read by another CANcoder, its ratio kept, and how its
     * loop reads it, fused, remote or synced, unless the CANcoder says its own: a practice robot's,
     * {@code ELEVATOR.encoderOn(CANCODER.can(25).offset(Rotations.of(0.11)))}.
     *
     * @throws IllegalArgumentException if it has no absolute encoder
     */
    public Prismatic encoderOn(MechanismWiring.Cancoder cancoder) {
      return encoderRewired(encoder(this).on(loopKept(this, cancoder)));
    }

    /**
     * The same joint, its absolute encoder on its SPARK's data port, its ratio kept, and its loop
     * closed on it if it was closed on the data port before.
     *
     * @throws IllegalArgumentException if it has no absolute encoder
     */
    public Prismatic encoderOn(MechanismWiring.DataPortEncoder dataPort) {
      return encoderRewired(encoder(this).on(loopKept(this, dataPort)));
    }

    /**
     * The same joint, its absolute encoder on one of Systemcore's ports, its ratio kept, and its
     * loop closed on it if it was closed on a port before.
     *
     * @throws IllegalArgumentException if it has no absolute encoder
     */
    public Prismatic encoderOn(MechanismWiring.PortEncoder port) {
      return encoderRewired(encoder(this).on(loopKept(this, port)));
    }

    /**
     * The same joint, its absolute encoder offset otherwise, the rest of it and its wiring kept: a
     * practice robot's, {@code ELEVATOR.encoderOffset(Rotations.of(0.47))}, on the CANcoder, data
     * port or Systemcore port the robot's own block reads it on.
     *
     * @throws IllegalArgumentException if it has no absolute encoder, or one that isn't wired
     */
    public Prismatic encoderOffset(Angle offset) {
      return encoderRewired(offset(this, offset));
    }

    private Prismatic encoderRewired(Driven.AbsoluteEncoder encoder) {
      return driven(drivenWith().absoluteEncoder(encoder).build());
    }

    /**
     * A limit switch at an end, tripping at a distance, in the simulator alone until it's wired.
     */
    public Prismatic limitSwitch(Sensor.Switch.Limit.End end, Distance at) {
      return limit(end, Optional.of(at.in(Meters)), Optional.empty());
    }

    /**
     * A limit switch at an end whose position isn't known: it stops the joint, but can't home it.
     * In the simulator alone until it's wired.
     */
    public Prismatic limitSwitch(Sensor.Switch.Limit.End end) {
      return limit(end, Optional.empty(), Optional.empty());
    }

    /** A limit switch at an end whose position isn't known, on an input. */
    public Prismatic limitSwitch(Sensor.Switch.Limit.End end, SensorWiring.Connection connection) {
      return limit(end, Optional.empty(), Optional.of(connection));
    }

    /**
     * A limit switch at an end whose position isn't known, on its own motor controller's limit pin.
     */
    public Prismatic limitSwitch(Sensor.Switch.Limit.End end, SensorWiring.Pin pin) {
      return limitSwitch(end, pin.of(this));
    }

    /**
     * A limit switch at an end, tripping at a distance, on an input: {@code .limitSwitch(End.LOWER,
     * Inches.of(0), CANDI.can(30).s1().normallyClosed())}.
     */
    public Prismatic limitSwitch(
        Sensor.Switch.Limit.End end, Distance at, SensorWiring.Connection connection) {
      return limit(end, Optional.of(at.in(Meters)), Optional.of(connection));
    }

    /** A limit switch at an end, tripping at a distance, on its own motor controller's pin. */
    public Prismatic limitSwitch(Sensor.Switch.Limit.End end, Distance at, SensorWiring.Pin pin) {
      return limitSwitch(end, at, pin.of(this));
    }

    @Override
    public Prismatic limitSwitchOn(
        Sensor.Switch.Limit.End end, SensorWiring.Connection connection) {
      return driven(drivenWith().limitSwitches(switchRewired(this, end, connection)).build());
    }

    @Override
    public Prismatic limitSwitchOn(Sensor.Switch.Limit.End end, SensorWiring.Pin pin) {
      return limitSwitchOn(end, pin.of(this));
    }

    /**
     * It kept out of a stretch of its travel while a condition holds, enforced beneath every
     * command: {@code .keepsOut(Range.meters(0, 0.25)).when(INTAKE.atMost(Degrees.of(30)))}. Its
     * target is held at the stretch's edge, on the side it's on, until the condition clears.
     */
    public KeepingOut<Prismatic> keepsOut(Range region) {
      return new KeepingOut<>(this, region, this::interlocked);
    }

    /**
     * It moving only while a condition holds, enforced beneath every command: {@code
     * .runsOnlyWhen(CLIMBER.atMost(Degrees.of(-75)))}. Meanwhile it holds where it is, or stops.
     */
    public Prismatic runsOnlyWhen(Condition condition) {
      return interlocked(Interlock.that(this).runsOnlyWhen(condition));
    }

    private Prismatic interlocked(Interlock interlock) {
      List<Interlock> interlocks = new ArrayList<>(driven.interlocks());
      interlocks.add(interlock);
      return driven(drivenWith().interlocks(interlocks).build());
    }

    @Override
    public Prismatic control(MechanismControl control) {
      onceControl(this);
      return driven(drivenWith().control(control).build());
    }

    @Override
    public Prismatic retuned(UnaryOperator<MechanismControl> change) {
      return driven(drivenWith().control(change.apply(controlOf(this))).build());
    }

    /** The same joint, driven otherwise. */
    public Prismatic driven(Driven driven) {
      return with().driven(driven).build();
    }

    private JointPrismaticBuilder with() {
      return JointPrismaticBuilder.builder(this);
    }

    private DrivenBuilder drivenWith() {
      return DrivenBuilder.builder(driven);
    }

    private Prismatic limit(
        Sensor.Switch.Limit.End end,
        Optional<Double> at,
        Optional<SensorWiring.Connection> wiring) {
      return driven(drivenWith().limitSwitches(switchAdded(this, end, at, wiring)).build());
    }
  }

  /**
   * Refuses a block saying again what a joint has one of, in its block or on the finished joint.
   *
   * @param why what it has, and how to change it
   */
  private static void once(boolean said, Supplier<String> why) {
    if (said) {
      throw new IllegalArgumentException(why.get());
    }
  }

  /** Refuses a second link. */
  private static void onceLink(Joint joint) {
    once(
        !joint.link().equals(Link.unseen(joint.label())),
        () ->
            joint.label()
                + " already carries "
                + joint.link().label()
                + ": it moves one link, so give it once, with everything it carries");
  }

  /** Refuses its positions said twice. */
  private static void oncePositions(Joint joint) {
    once(
        !joint.positions().isEmpty(),
        () ->
            joint.label()
                + " already has its positions, "
                + String.join(", ", names(joint.positions(), List.of()))
                + ": name them all in one positions(...); to put one elsewhere, replaced(...)");
  }

  /** Refuses its outputs said twice. */
  private static void onceOutputs(Joint joint) {
    once(
        !joint.outputs().isEmpty(),
        () ->
            joint.label()
                + " already has its outputs, "
                + String.join(", ", names(List.of(), joint.outputs()))
                + ": name them all in one outputs(...); to run one otherwise, replaced(...)");
  }

  /** Refuses its motors said twice. */
  private static void onceMotors(Joint joint) {
    once(
        joint.driven().motors().isPresent(),
        () ->
            joint.label()
                + " already has its motors: to re-wire them, motorsOn(...); to re-gear them,"
                + " gearing(...)");
  }

  /** Refuses its absolute encoder said twice: a pair isn't decoded yet. */
  private static void onceEncoder(Joint joint) {
    once(
        joint.driven().absoluteEncoder().isPresent(),
        () ->
            joint.label()
                + " already has an absolute encoder: to re-wire it, encoderOn(...); to offset it,"
                + " encoderOffset(...); a decoded pair isn't supported yet");
  }

  /** Refuses its control said twice. */
  private static void onceControl(Joint joint) {
    once(
        joint.driven().control().isPresent(),
        () ->
            joint.label()
                + " already has its control: to re-tune it, retuned(control ->"
                + " control.withKg(...))");
  }

  /** A joint's control, to change. */
  private static MechanismControl controlOf(Joint joint) {
    return joint
        .driven()
        .control()
        .orElseThrow(
            () ->
                new IllegalArgumentException(
                    joint.label() + " has no control to re-tune: give it one, .control(...)"));
  }

  /**
   * A CANcoder a joint's absolute encoder moves to ({@code encoderOn}), keeping how the joint's
   * loop reads the CANcoder it was on: one written as read once at startup, as a fresh {@code
   * CANCODER.can(35).offset(...)} is, runs the loop as the one before did, fused, remote or synced,
   * so a practice robot that only moved its CANcoder's ID runs as the competition robot does. One
   * written with its own way, {@code .remote()}, has it. One from another kind of input is as
   * written: a SPARK's data port or a Systemcore port says its own, {@code .closesLoop()}.
   */
  private static MechanismWiring.Cancoder loopKept(Joint joint, MechanismWiring.Cancoder cancoder) {
    if (cancoder.feedback() != MechanismWiring.Feedback.ROTOR) {
      return cancoder;
    }
    return joint
        .driven()
        .absoluteEncoder()
        .flatMap(Driven.AbsoluteEncoder::cancoder)
        .map(
            was ->
                MechanismWiringCancoderBuilder.builder(cancoder).feedback(was.feedback()).build())
        .orElse(cancoder);
  }

  /**
   * A data port a joint's absolute encoder moves to, keeping whether its loop closes on the data
   * port it was on, as a CANcoder keeps how it's read ({@link #loopKept(Joint,
   * MechanismWiring.Cancoder)}).
   */
  private static MechanismWiring.DataPortEncoder loopKept(
      Joint joint, MechanismWiring.DataPortEncoder dataPort) {
    boolean closed =
        joint
            .driven()
            .absoluteEncoder()
            .flatMap(Driven.AbsoluteEncoder::dataPort)
            .map(MechanismWiring.DataPortEncoder::closedLoop)
            .orElse(false);
    return closed && !dataPort.closedLoop() ? dataPort.closesLoop() : dataPort;
  }

  /**
   * A Systemcore port a joint's absolute encoder moves to, keeping whether its loop closes on the
   * port it was on, as a CANcoder keeps how it's read ({@link #loopKept(Joint,
   * MechanismWiring.Cancoder)}).
   */
  private static MechanismWiring.PortEncoder loopKept(
      Joint joint, MechanismWiring.PortEncoder port) {
    boolean closed =
        joint
            .driven()
            .absoluteEncoder()
            .flatMap(Driven.AbsoluteEncoder::port)
            .map(MechanismWiring.PortEncoder::closedLoop)
            .orElse(false);
    return closed && !port.closedLoop() ? port.closesLoop() : port;
  }

  /** A joint's absolute encoder, its ratio kept, unwired: to wire again. */
  private static Driven.AbsoluteEncoder encoder(Joint joint) {
    Driven.AbsoluteEncoder encoder =
        joint
            .driven()
            .absoluteEncoder()
            .orElseThrow(
                () ->
                    new IllegalArgumentException(
                        joint.label()
                            + " has no absolute encoder to re-wire: give it one,"
                            + " .absoluteEncoder(...)"));
    return DrivenAbsoluteEncoderBuilder.builder(encoder)
        .cancoder(Optional.empty())
        .dataPort(Optional.empty())
        .port(Optional.empty())
        .build();
  }

  /**
   * Where a joint rests at one of its positions: its own of the position's name, where this joint
   * puts it; or, a name it hasn't (in its block, before its positions are named), where the
   * position is. Whether that's inside its stops is the robot's definition's to judge.
   */
  private static double restAt(Joint joint, Position rest) {
    boolean named = joint.positions().stream().anyMatch(each -> each.name().equals(rest.name()));
    if (named) {
      return joint.own(rest).value();
    }
    boolean revolute = joint instanceof Revolute;
    if (rest.kind() != Position.Kind.of(revolute)) {
      throw new IllegalArgumentException(
          joint.label()
              + " can't rest at "
              + rest.name()
              + ", "
              + (revolute
                  ? "a length, since it turns: give it an angle, rest(Degrees.of(...))"
                  : "an angle, since it slides: give it a length, rest(Inches.of(...))"));
    }
    return rest.value();
  }

  /** A joint's positions, the one of a position's name in its place, there instead. */
  private static List<Position> replacedPosition(Joint joint, Position replacing) {
    List<Position> positions = new ArrayList<>();
    for (Position each : joint.positions()) {
      positions.add(each.name().equals(replacing.name()) ? replacing : each);
    }
    if (!positions.contains(replacing)) {
      throw new IllegalArgumentException(
          joint.label()
              + " has no position "
              + replacing.name()
              + " to re-place: "
              + has(names(joint.positions(), List.of()))
              + ". Its positions are what its code names, the same on every robot of the design");
    }
    return positions;
  }

  /** A joint's outputs, the one of an output's name in its place, at its value instead. */
  private static List<Output> replacedOutput(Joint joint, Output replacing) {
    List<Output> outputs = new ArrayList<>();
    for (Output each : joint.outputs()) {
      outputs.add(each.name().equals(replacing.name()) ? replacing : each);
    }
    if (!outputs.contains(replacing)) {
      throw new IllegalArgumentException(
          joint.label()
              + " has no output "
              + replacing.name()
              + " to re-value: "
              + has(names(List.of(), joint.outputs()))
              + ". Its outputs are what its code names, the same on every robot of the design");
    }
    return outputs;
  }

  /**
   * A place's former values with the one it has now, as it's re-placed elsewhere: each once, so a
   * constant at any is still the joint's own.
   */
  private static <P> List<P> former(List<P> former, P now, P replacing) {
    List<P> had = new ArrayList<>(former);
    if (!now.equals(replacing) && !had.contains(now)) {
      had.add(now);
    }
    return had;
  }

  /** A joint's absolute encoder offset otherwise, the rest of it and its wiring as they are. */
  private static Driven.AbsoluteEncoder offset(Joint joint, Angle offset) {
    Driven.AbsoluteEncoder encoder =
        joint
            .driven()
            .absoluteEncoder()
            .orElseThrow(
                () ->
                    new IllegalArgumentException(
                        joint.label()
                            + " has no absolute encoder to offset: give it one,"
                            + " .absoluteEncoder(...)"));
    DrivenAbsoluteEncoderBuilder offsetOne = DrivenAbsoluteEncoderBuilder.builder(encoder);
    if (encoder.cancoder().isPresent()) {
      offsetOne.cancoder(
          Optional.of(
              MechanismWiringCancoderBuilder.builder(encoder.cancoder().get())
                  .offset(offset)
                  .build()));
    } else if (encoder.dataPort().isPresent()) {
      offsetOne.dataPort(
          Optional.of(
              MechanismWiringDataPortEncoderBuilder.builder(encoder.dataPort().get())
                  .offset(offset)
                  .build()));
    } else if (encoder.port().isPresent()) {
      offsetOne.port(encoder.port().get().offset(offset));
    } else {
      throw new IllegalArgumentException(
          possessive(joint.label())
              + " absolute encoder isn't wired, so it has no offset: the simulator alone reads"
              + " it. Wire it, encoderOn(...)");
    }
    return offsetOne.build();
  }

  /** A joint's motors on other controllers, the rest as they are. */
  private static Driven.Motors rewired(Joint joint, MechanismWiring.Device... controllers) {
    Driven.Motors motors =
        joint
            .driven()
            .motors()
            .orElseThrow(
                () ->
                    new IllegalArgumentException(
                        joint.label() + " has no motors to re-wire: give it .motor(...) first"));
    if (controllers.length != 0 && controllers.length != motors.count()) {
      throw new IllegalArgumentException(
          joint.label()
              + " has "
              + motors.count()
              + " motors, and motorsOn(...) names "
              + controllers.length
              + ": name a controller for each, the leader first");
    }
    return DrivenMotorsBuilder.builder(motors).controllers(List.of(controllers)).build();
  }

  /** A joint's limit switches with one more, at an end it has none. */
  private static List<Driven.LimitSwitch> switchAdded(
      Joint joint,
      Sensor.Switch.Limit.End end,
      Optional<Double> at,
      Optional<SensorWiring.Connection> wiring) {
    once(
        joint.driven().limitSwitches().stream().anyMatch(each -> each.end() == end),
        () ->
            joint.label()
                + " already has a limit switch at its "
                + end.name().toLowerCase(Locale.ROOT)
                + " end: to re-wire it, limitSwitchOn(End."
                + end.name()
                + ", ...)");
    List<Driven.LimitSwitch> switches = new ArrayList<>(joint.driven().limitSwitches());
    switches.add(DrivenLimitSwitchBuilder.builder().end(end).position(at).wiring(wiring).build());
    return switches;
  }

  /** A joint's limit switches, the one at an end on another input, where it trips kept. */
  private static List<Driven.LimitSwitch> switchRewired(
      Joint joint, Sensor.Switch.Limit.End end, SensorWiring.Connection connection) {
    List<Driven.LimitSwitch> switches = new ArrayList<>();
    boolean found = false;
    for (Driven.LimitSwitch each : joint.driven().limitSwitches()) {
      found |= each.end() == end;
      switches.add(
          each.end() == end
              ? DrivenLimitSwitchBuilder.builder(each).wiring(connection).build()
              : each);
    }
    if (!found) {
      throw new IllegalArgumentException(
          joint.label()
              + " has no limit switch at its "
              + end.name().toLowerCase(Locale.ROOT)
              + " end to re-wire: give it one, .limitSwitch(End."
              + end.name()
              + ", ...)");
    }
    return switches;
  }

  /** A joint's motors through another gearing, the rest as they are. */
  private static Driven.Motors regeared(Joint joint, Gearing gearing) {
    Driven.Motors motors =
        joint
            .driven()
            .motors()
            .orElseThrow(
                () ->
                    new IllegalArgumentException(
                        joint.label() + " has no motors to gear: give it .motor(...) first"));
    return DrivenMotorsBuilder.builder(motors).gearing(gearing).build();
  }

  /** Motors as a joint's block says them. */
  private static Driven.Motors turning(
      Motor motor,
      int count,
      Gearing gearing,
      Optional<Distance> pitchRadius,
      List<MechanismWiring.Device> controllers) {
    return Driven.Motors.builder()
        .motor(motor)
        .count(count)
        .gearing(gearing)
        .controllers(controllers)
        .pitchRadius(pitchRadius)
        .build();
  }

  /**
   * A joint's own drives: its motors, under its label, through the transmission its kind gives
   * them; and each servo, between its parent and its link, under its own label.
   */
  private static List<Drive> drives(
      Joint joint, java.util.function.Function<Driven.Motors, Transmission> through) {
    List<Drive> drives = new ArrayList<>();
    joint
        .driven()
        .motors()
        .ifPresent(
            motors ->
                drives.add(
                    Drive.builder()
                        .label(joint.label())
                        .actuator(
                            Actuator.Motors.builder()
                                .motor(motors.motor())
                                .count(motors.count())
                                .build())
                        .transmission(through.apply(motors))
                        .build()));
    Transform3d origin = joint.origin();
    for (Driven.Servo servo : joint.driven().servos()) {
      drives.add(
          Drive.builder()
              .label(servo.label())
              .actuator(servo.servo())
              .transmission(
                  Transmission.Between.builder()
                      .a(joint.parent())
                      .onA(
                          origin
                              .getTranslation()
                              .plus(servo.onParent().rotateBy(origin.getRotation())))
                      .b(joint)
                      .onB(servo.onLink())
                      .build())
              .build());
    }
    return drives;
  }

  /**
   * A joint's own checks, the same for both kinds. Whether where it rests and its places are inside
   * its stops is its robot's definition's to judge, on the finished joint ({@code Blocks}), so a
   * practice robot changes them in either order.
   */
  private static void check(
      Class<? extends Record> kind,
      String label,
      Transform3d origin,
      Vector3 axis,
      Optional<Range> range,
      Optional<Double> rest,
      List<Position> positions,
      List<Output> outputs) {
    boolean revolute = kind == Revolute.class;
    Checks.on(kind, label)
        .field("label", label, notBlank())
        .field("origin", origin, Joint::notATransform)
        .field(
            "axis",
            axis,
            a ->
                Double.isFinite(a.length()) && a.length() > 0
                    ? null
                    : a + " must point some way: an axis can't be zero")
        .field("rest", rest, given(finite()))
        .check(
            "range",
            () ->
                range.isPresent() && range.get().kind() != Position.Kind.of(revolute)
                    ? wrongKind("its range is", revolute)
                    : null)
        .check("positions", () -> ofTheOtherKind(revolute, positions))
        .check("outputs", () -> twice(positions, outputs))
        .check("outputs", () -> speedsOfTheOtherKind(label, revolute, outputs))
        .orThrow();
  }

  /**
   * A stretch of a joint's travel given as the other kind, lengths for a joint that turns or angles
   * for one that slides, and how to give it: {@code "its range is lengths, and the joint turns:
   * give it in angles, Range.degrees(...)"}.
   *
   * @param what what it is, and the verb: {@code "its range is"}
   */
  static String wrongKind(String what, boolean revolute) {
    return what
        + (revolute
            ? " lengths, and the joint turns: give it in angles, Range.degrees(...)"
            : " angles, and the joint slides: give it in lengths, Range.meters(...) or"
                + " Range.of(Inches.of(...), Inches.of(...))");
  }

  /**
   * The named positions of the wrong kind, a length for a joint that turns or an angle for one that
   * slides, or null when they're all of its kind. Whether they're inside its stops is the
   * definition's to judge, on the finished joint, so a practice robot's stops and places change in
   * either order.
   */
  private static @Nullable String ofTheOtherKind(boolean revolute, List<Position> positions) {
    Position.Kind kind = revolute ? Position.Kind.ANGLE : Position.Kind.LENGTH;
    List<String> wrong = new ArrayList<>();
    for (Position position : positions) {
      if (position.kind() != kind) {
        wrong.add(
            position.name()
                + " is "
                + (revolute
                    ? "a length, and the joint turns: give it an angle, Position.degrees(...)"
                    : "an angle, and the joint slides: give it a length, Position.of(name,"
                        + " Inches.of(...))"));
      }
    }
    return wrong.isEmpty() ? null : String.join("; ", wrong);
  }

  /**
   * The named speeds of the wrong kind, of turning for a joint that slides or of sliding for one
   * that turns, or null when they're all of its kind: an rpm on an elevator would run in meters a
   * second.
   */
  private static @Nullable String speedsOfTheOtherKind(
      String label, boolean revolute, List<Output> outputs) {
    Position.Kind kind = Position.Kind.of(revolute);
    List<String> wrong = new ArrayList<>();
    for (Output output : outputs) {
      if (output.motion().isPresent() && output.motion().get() != kind) {
        wrong.add(
            output.name()
                + ", "
                + Places.shownSpeed(!revolute, output.value())
                + ", is "
                + (revolute
                    ? "a speed along a slide, and "
                        + label
                        + " turns: give it a speed of turning, Output.rpm(\""
                        + output.name()
                        + "\", ...)"
                    : "a speed of turning, and "
                        + label
                        + " slides: give it a speed along it, Output.of(\""
                        + output.name()
                        + "\", InchesPerSecond.of(...))"));
      }
    }
    return wrong.isEmpty() ? null : String.join("; ", wrong);
  }

  /**
   * The names used twice among its positions and outputs, or the built-in output's, or null when
   * none are.
   */
  private static @Nullable String twice(List<Position> positions, List<Output> outputs) {
    Set<String> seen = new HashSet<>();
    Set<String> twice = new LinkedHashSet<>();
    for (String name : names(positions, outputs)) {
      if (!seen.add(name)) {
        twice.add(name);
      }
    }
    if (seen.contains(Output.STOPPED_NAME)) {
      return "it names "
          + Output.STOPPED_NAME
          + ", which every joint has built in (its motors at neutral): name it something else";
    }
    return twice.isEmpty()
        ? null
        : "it names "
            + String.join(", ", twice)
            + " twice: each name is one position or output of the joint";
  }

  /** What a joint has by name, for a message: "it has in, run, out", or "it has none". */
  private static String has(List<String> names) {
    return "it has " + (names.isEmpty() ? "none" : String.join(", ", names));
  }

  /** Its positions' names, then its outputs'. */
  private static List<String> names(List<Position> positions, List<Output> outputs) {
    List<String> names = new ArrayList<>();
    positions.forEach(position -> names.add(position.name()));
    outputs.forEach(output -> names.add(output.name()));
    return names;
  }

  private static @Nullable String notATransform(Transform3d transform) {
    Quaternion turn = transform.getRotation().getQuaternion();
    return Double.isFinite(
            transform.getX()
                + transform.getY()
                + transform.getZ()
                + turn.getW()
                + turn.getX()
                + turn.getY()
                + turn.getZ())
        ? null
        : transform + " must be a transform, numbers all";
  }
}
