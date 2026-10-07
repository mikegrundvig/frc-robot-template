package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

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
 * what configurations and interlocks say about it: {@link #at}, {@link #past}, {@link #before} and
 * {@link #between}.
 */
public sealed interface Joint extends Part permits Joint.Revolute, Joint.Prismatic {
  /**
   * An axis about which a link whose zero points forward swings up as its position grows: about the
   * robot's y, the other way round.
   */
  Vector3 PITCH_UP = new Vector3(0, -1, 0);

  /** An axis about which a link whose zero points forward swings down as its position grows. */
  Vector3 PITCH_DOWN = new Vector3(0, 1, 0);

  /** Straight up: what an elevator slides along. */
  Vector3 UP = new Vector3(0, 0, 1);

  /**
   * A pivot: an arm, an intake's swing, a wrist, turning about an axis. Its block says the rest,
   * one call at a time: {@code Joint.pivot("pivot", Part.CHASSIS, at,
   * Joint.PITCH_DOWN).range(Range.degrees(0, 135), PIVOT_IN).carries(link).positions(PIVOT_IN,
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
        .range(Optional.empty())
        .reference(0)
        .link(Link.unseen(label))
        .positions(List.of())
        .outputs(List.of())
        .driven(Driven.NONE)
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
   * at).range(Range.meters(0, 1.2), ELEVATOR_DOWN).motors(Motor.KRAKEN_X60, gearing, drum, 2)}.
   *
   * @param at where the slide starts, in its parent's frame
   */
  static Prismatic elevator(String label, Part parent, Translation3d at) {
    return slide(label, parent, at, UP);
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
        .range(Optional.empty())
        .reference(0)
        .link(Link.unseen(label))
        .positions(List.of())
        .outputs(List.of())
        .driven(Driven.NONE)
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
   * Where it starts: where the robot is weighed and modeled (stowed), and where a mechanism that
   * zeroes itself at boot is taken to be. Inside its range.
   */
  double reference();

  /** What it moves. */
  Link link();

  /**
   * Its places by name, where code moves it to: an intake's "in", "run" and "out". Inside its
   * range; a joint that follows another has none, since its leader's moves carry it.
   */
  List<Position> positions();

  /**
   * What code runs it at, by name: a roller's "intaking" at a speed, its "outtaking" at a voltage,
   * each within what its motors can do. Not {@link Output#STOPPED}, which every joint has.
   */
  List<Output> outputs();

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
   * Its own drives, as its block says them: its motors, under its label, and each servo, under its
   * own. A drive between joints, a differential's, is the robot's ({@code RobotDefinition.drives}).
   */
  List<Drive> ownDrives();

  /**
   * Its own sensors, as its block says them: its absolute encoder ("arm absolute encoder"), and its
   * limit switches ("arm lower limit", "arm upper limit"), each carrying its wiring.
   */
  default List<Sensor> ownSensors() {
    List<Sensor> sensors = new ArrayList<>();
    driven()
        .absoluteEncoder()
        .ifPresent(
            encoder -> {
              if (this instanceof Revolute revolute) {
                sensors.add(
                    Sensor.Encoder.builder()
                        .label(label() + " absolute encoder")
                        .terms(
                            List.of(
                                Transmission.Term.builder()
                                    .joint(revolute)
                                    .ratio(encoder.ratio() / encoder.turnsPerSignal())
                                    .pitchRadius(Optional.empty())
                                    .build()))
                        .absolute(true)
                        .countsPerTurn(
                            encoder
                                .countsPerTurn()
                                .map(counts -> (int) Math.round(counts * encoder.turnsPerSignal())))
                        .build());
              }
            });
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
   * ELEVATOR.at(ELEVATOR_CLEAR), ARM.at(ARM_LEVEL))}), and an interlock's condition.
   *
   * @throws IllegalArgumentException if the position isn't one of its own
   */
  default Condition.At at(Position position) {
    return ConditionAtBuilder.builder().joint(this).name(own(position).name()).build();
  }

  /**
   * It at one of its outputs, or {@link Output#STOPPED}: what a configuration is made of ({@code
   * Configuration.of("feeding", FEEDER.at(FEEDER_FEEDING), INDEXER.at(INDEXER_FEEDING))}).
   *
   * @throws IllegalArgumentException if the output isn't one of its own
   */
  default Condition.At at(Output output) {
    return ConditionAtBuilder.builder().joint(this).name(own(output).name()).build();
  }

  /**
   * It past one of its positions: the position lies between its reference and where it is, so a
   * pivot stowed at 1.5° is past "run", at 110°, from 110° on.
   *
   * @throws IllegalArgumentException if the position isn't one of its own, or it's the reference
   */
  default Condition.Past past(Position position) {
    return ConditionPastBuilder.builder()
        .joint(this)
        .position(own(position).value())
        .place(Optional.of(position.name()))
        .build();
  }

  /**
   * It before one of its positions: not yet past it, on its reference's side.
   *
   * @throws IllegalArgumentException if the position isn't one of its own, or it's the reference
   */
  default Condition.Before before(Position position) {
    return ConditionBeforeBuilder.builder()
        .joint(this)
        .position(own(position).value())
        .place(Optional.of(position.name()))
        .build();
  }

  /**
   * It between two of its positions, both included, whichever way round they're given.
   *
   * @throws IllegalArgumentException if either isn't one of its own, or they're one
   */
  default Condition.Between between(Position from, Position to) {
    return ConditionBetweenBuilder.builder()
        .joint(this)
        .from(own(from).value())
        .to(own(to).value())
        .fromPlace(Optional.of(from.name()))
        .toPlace(Optional.of(to.name()))
        .build();
  }

  /**
   * The position, if it's one of this joint's own.
   *
   * @throws IllegalArgumentException if it isn't, naming those it has
   */
  default Position own(Position position) {
    String why = whyNotOwn(position);
    if (why != null) {
      throw new IllegalArgumentException(why);
    }
    return position;
  }

  /**
   * The output, if it's one of this joint's own, or {@link Output#STOPPED}.
   *
   * @throws IllegalArgumentException if it isn't, naming those it has
   */
  default Output own(Output output) {
    String why = whyNotOwn(output);
    if (why != null) {
      throw new IllegalArgumentException(why);
    }
    return output;
  }

  /** Why a position isn't one of this joint's own, naming those it has; or null when it is. */
  default @Nullable String whyNotOwn(Position position) {
    if (positions().contains(position)) {
      return null;
    }
    boolean revolute = position.kind() == Position.Kind.ANGLE;
    for (Position own : positions()) {
      if (own.name().equals(position.name())) {
        return label()
            + "'s "
            + own.name()
            + " is at "
            + Places.shownAt(revolute, own.value())
            + ", not "
            + Places.shownAt(revolute, position.value())
            + ": another joint's "
            + own.name()
            + ", or another robot's";
      }
    }
    return label()
        + " has no position "
        + position.name()
        + " at "
        + Places.shownAt(position.kind() == Position.Kind.ANGLE, position.value())
        + ": "
        + has(names(positions(), List.of()));
  }

  /** Why an output isn't one of this joint's own, naming those it has; or null when it is. */
  default @Nullable String whyNotOwn(Output output) {
    return outputs().contains(output) || output.equals(Output.STOPPED)
        ? null
        : label()
            + " has no output "
            + output.name()
            + ": "
            + has(names(List.of(), outputs()))
            + ", and "
            + Output.STOPPED_NAME;
  }

  /**
   * A joint that turns about its axis, in radians: an arm, a wrist, a turret, a roller.
   *
   * @param label what logs and writers call it: unique among the robot's joints
   * @param parent what it hangs from: the chassis, or another joint's link
   * @param origin where its frame is in its parent's frame at position 0: the pivot
   * @param axis what it turns about, in its own frame, positive by the right-hand rule
   * @param range its hard stops, in radians, unwrapped; empty for one that turns freely
   * @param reference where it starts, in radians: stowed, inside its range
   * @param link what it moves
   * @param positions its places by name, in radians, inside its range: {@code
   *     Position.degrees("out", 130)}
   * @param outputs what code runs it at, by name: {@code Output.rpm("intaking", 2000)}, {@code
   *     Output.volts("push", 6)}
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
      double reference,
      Link link,
      List<Position> positions,
      List<Output> outputs,
      Driven driven)
      implements Joint {
    public Revolute {
      positions = List.copyOf(positions);
      outputs = List.copyOf(outputs);
      check(Revolute.class, label, origin, axis, range, reference, positions, outputs);
      Optional<Driven.Motors> motors = driven.motors();
      Checks.on(Revolute.class, label)
          .check(
              "driven.motors",
              () ->
                  motors.isPresent() && motors.get().pitchRadius().isPresent()
                      ? "a joint that turns has no drum: give its motors a gearing alone"
                      : null)
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

    /** Equal to another joint the same in everything but how it's driven ({@link #driven}). */
    @Override
    public boolean equals(@Nullable Object other) {
      return other instanceof Revolute joint
          && label.equals(joint.label)
          && parent.equals(joint.parent)
          && origin.equals(joint.origin)
          && axis.equals(joint.axis)
          && range.equals(joint.range)
          && Double.compare(reference, joint.reference) == 0
          && link.equals(joint.link)
          && positions.equals(joint.positions)
          && outputs.equals(joint.outputs);
    }

    @Override
    public int hashCode() {
      return Objects.hash(label, parent, origin, axis, range, reference, link, positions, outputs);
    }

    /** It between hard stops, resting at one of its positions where the robot starts. */
    public Revolute range(Range range, Position rest) {
      return with().range(Optional.of(range)).reference(rest.value()).build();
    }

    /** It between hard stops, resting at an angle where the robot starts. */
    public Revolute range(Range range, Angle rest) {
      return with().range(Optional.of(range)).reference(rest.in(Radians)).build();
    }

    /** It resting at one of its positions where the robot starts, with no hard stops. */
    public Revolute rests(Position rest) {
      return with().reference(rest.value()).build();
    }

    /** It resting at an angle where the robot starts, with no hard stops. */
    public Revolute rests(Angle rest) {
      return with().reference(rest.in(Radians)).build();
    }

    /**
     * It past an angle: the angle lies between its reference and where it is. What an interlock's
     * edge usually is, a clearance rather than a place it goes: {@code
     * PIVOT.past(Degrees.of(110))}.
     *
     * @throws IllegalArgumentException if it's outside its range, or its reference
     */
    public Condition.Past past(Angle position) {
      return ConditionPastBuilder.builder()
          .joint(this)
          .position(position.in(Radians))
          .place(Optional.empty())
          .build();
    }

    /**
     * It before an angle: not yet past it, on its reference's side.
     *
     * @throws IllegalArgumentException if it's outside its range, or its reference
     */
    public Condition.Before before(Angle position) {
      return ConditionBeforeBuilder.builder()
          .joint(this)
          .position(position.in(Radians))
          .place(Optional.empty())
          .build();
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
          .fromPlace(Optional.empty())
          .toPlace(Optional.empty())
          .build();
    }

    /**
     * It running at least a speed, forward: {@code SHOOTER.atLeast(RPM.of(3000))}.
     *
     * @throws IllegalArgumentException for a speed that isn't forward
     */
    public Condition.AtLeast atLeast(AngularVelocity speed) {
      return ConditionAtLeastBuilder.builder()
          .joint(this)
          .speed(speed.in(RadiansPerSecond))
          .build();
    }

    /** What it moves. */
    public Revolute carries(Link link) {
      return with().link(link).build();
    }

    /** Its places by name: {@code .positions(ARM_STOWED, ARM_LEVEL)}. */
    public Revolute positions(Position... positions) {
      return with().positions(List.of(positions)).build();
    }

    /** What it runs at, by name: {@code .outputs(INTAKING, OUTTAKING)}. */
    public Revolute outputs(Output... outputs) {
      return with().outputs(List.of(outputs)).build();
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
      return driven(
          drivenWith()
              .motors(Optional.of(turning(motor, count, gearing, Optional.empty(), List.of()))));
    }

    /** Motors turning it together through a gearing, each on its controller, the leader first. */
    public Revolute motors(Motor motor, Gearing gearing, MechanismWiring.Device... controllers) {
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

    /**
     * The same joint, its motors through another gearing, their controllers as they are: a second
     * robot of the design, geared otherwise.
     *
     * @throws IllegalArgumentException if no motors turn it
     */
    public Revolute gearing(Gearing gearing) {
      return driven(drivenWith().motors(Optional.of(regeared(this, gearing))));
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
     * An absolute encoder on its axle, of a resolution, in the simulator alone until it's wired.
     * The resolution is optional: only a pair decoded by their difference reads it.
     */
    public Revolute absoluteEncoder(int countsPerTurn) {
      return absoluteEncoder(Driven.AbsoluteEncoder.of(countsPerTurn));
    }

    /** An absolute encoder on its axle, of a resolution, read by a CANcoder through its TalonFX. */
    public Revolute absoluteEncoder(int countsPerTurn, MechanismWiring.Cancoder cancoder) {
      return absoluteEncoder(Driven.AbsoluteEncoder.of(countsPerTurn).on(cancoder));
    }

    /** An absolute encoder on its axle, of a resolution, on its SPARK's data port. */
    public Revolute absoluteEncoder(int countsPerTurn, MechanismWiring.DataPortEncoder dataPort) {
      return absoluteEncoder(Driven.AbsoluteEncoder.of(countsPerTurn).on(dataPort));
    }

    /**
     * An absolute encoder on its axle, of a resolution, on one of Systemcore's ports, read by the
     * robot: {@code .absoluteEncoder(1024, PortEncoder.dutyCycle(1).offset(Rotations.of(0.82)))}.
     */
    public Revolute absoluteEncoder(int countsPerTurn, MechanismWiring.PortEncoder port) {
      return absoluteEncoder(Driven.AbsoluteEncoder.of(countsPerTurn).on(port));
    }

    /**
     * An absolute encoder as written in full, geared or not: {@code
     * .absoluteEncoder(AbsoluteEncoder.of(4096).ratio(0.536).on(cancoder))}.
     */
    public Revolute absoluteEncoder(Driven.AbsoluteEncoder encoder) {
      return driven(drivenWith().absoluteEncoder(Optional.of(encoder)));
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

    /** A limit switch at an end whose position isn't known, on an input. */
    public Revolute limitSwitch(
        Sensor.Switch.Limit.End end, SensorWiring.Input input, SensorWiring.Polarity polarity) {
      return limit(
          end, Optional.empty(), Optional.of(new SensorWiring.Connection(input, polarity)));
    }

    /**
     * A limit switch at an end whose position isn't known, on its own motor controller's limit pin:
     * the controller stops the motor itself.
     */
    public Revolute limitSwitch(
        Sensor.Switch.Limit.End end, SensorWiring.Direction pin, SensorWiring.Polarity polarity) {
      return limitSwitch(end, SensorWiring.ControllerLimit.of(this, pin), polarity);
    }

    /**
     * A limit switch at an end, tripping at an angle, on an input: a Systemcore port or a CANdi's.
     */
    public Revolute limitSwitch(
        Sensor.Switch.Limit.End end,
        Angle at,
        SensorWiring.Input input,
        SensorWiring.Polarity polarity) {
      return limit(
          end,
          Optional.of(at.in(Radians)),
          Optional.of(new SensorWiring.Connection(input, polarity)));
    }

    /**
     * A limit switch at an end, tripping at an angle, on its own motor controller's limit pin: the
     * controller stops the motor itself.
     */
    public Revolute limitSwitch(
        Sensor.Switch.Limit.End end,
        Angle at,
        SensorWiring.Direction pin,
        SensorWiring.Polarity polarity) {
      return limitSwitch(end, at, SensorWiring.ControllerLimit.of(this, pin), polarity);
    }

    /**
     * It kept out of a stretch of its travel while a condition holds, enforced beneath every
     * command: {@code .keepsOut(Range.degrees(-180, -101)).when(ELEVATOR.before(Inches.of(32)))}.
     * Its target is held at the stretch's edge, on the side it's on, until the condition clears.
     */
    public KeepingOut<Revolute> keepsOut(Range region) {
      return new KeepingOut<>(this, region, this::interlocked);
    }

    /**
     * It moving only while a condition holds, enforced beneath every command: {@code
     * .runsOnlyWhen(PIVOT.past(Degrees.of(110)))}. Meanwhile it holds where it is, or stops.
     */
    public Revolute runsOnlyWhen(Condition condition) {
      return interlocked(Interlock.that(this).runsOnlyWhen(condition));
    }

    private Revolute interlocked(Interlock interlock) {
      List<Interlock> interlocks = new ArrayList<>(driven.interlocks());
      interlocks.add(interlock);
      return driven(drivenWith().interlocks(interlocks));
    }

    /** How the program drives it: {@code .control(MechanismControl.position(30))}. */
    public Revolute control(MechanismControl control) {
      return driven(drivenWith().control(Optional.of(control)));
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
      List<Driven.LimitSwitch> switches = new ArrayList<>(driven.limitSwitches());
      switches.add(DrivenLimitSwitchBuilder.builder().end(end).position(at).wiring(wiring).build());
      return driven(drivenWith().limitSwitches(switches));
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
   * @param reference where it starts, in meters: stowed, inside its range
   * @param link what it moves
   * @param positions its places by name, in meters, inside its range: {@code Position.of("clear",
   *     Inches.of(20))}
   * @param outputs what code runs it at, by name: a speed in meters per second, a voltage, or a
   *     duty cycle
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
      double reference,
      Link link,
      List<Position> positions,
      List<Output> outputs,
      Driven driven)
      implements Joint {
    public Prismatic {
      positions = List.copyOf(positions);
      outputs = List.copyOf(outputs);
      check(Prismatic.class, label, origin, axis, range, reference, positions, outputs);
      Optional<Driven.Motors> motors = driven.motors();
      Optional<Driven.AbsoluteEncoder> encoder = driven.absoluteEncoder();
      Optional<MechanismControl> control = driven.control();
      Checks.on(Prismatic.class, label)
          .check(
              "driven.motors",
              () ->
                  motors.isPresent() && motors.get().pitchRadius().isEmpty()
                      ? "a joint that slides needs its drum: give its motors the pitch radius"
                      : null)
          .check(
              "driven.absoluteEncoder",
              () ->
                  encoder.isPresent()
                      ? "an absolute encoder reads a joint that turns: one that slides has none"
                      : null)
          .check(
              "driven.control",
              () ->
                  control.isPresent() && control.get().gains().gravityAngle().isPresent()
                      ? "a joint that slides is held against gravity the same all along it, so"
                          + " its kG has no angle: give kG alone, withKg(kG)"
                      : null)
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

    /** Equal to another joint the same in everything but how it's driven ({@link #driven}). */
    @Override
    public boolean equals(@Nullable Object other) {
      return other instanceof Prismatic joint
          && label.equals(joint.label)
          && parent.equals(joint.parent)
          && origin.equals(joint.origin)
          && axis.equals(joint.axis)
          && range.equals(joint.range)
          && Double.compare(reference, joint.reference) == 0
          && link.equals(joint.link)
          && positions.equals(joint.positions)
          && outputs.equals(joint.outputs);
    }

    @Override
    public int hashCode() {
      return Objects.hash(label, parent, origin, axis, range, reference, link, positions, outputs);
    }

    /** It between hard stops, resting at one of its positions where the robot starts. */
    public Prismatic range(Range range, Position rest) {
      return with().range(Optional.of(range)).reference(rest.value()).build();
    }

    /** It between hard stops, resting at a distance where the robot starts. */
    public Prismatic range(Range range, Distance rest) {
      return with().range(Optional.of(range)).reference(rest.in(Meters)).build();
    }

    /** It resting at one of its positions where the robot starts, with no hard stops. */
    public Prismatic rests(Position rest) {
      return with().reference(rest.value()).build();
    }

    /** It resting at a distance where the robot starts, with no hard stops. */
    public Prismatic rests(Distance rest) {
      return with().reference(rest.in(Meters)).build();
    }

    /**
     * It past a distance: the angle lies between its reference and where it is. What an interlock's
     * edge usually is, a clearance rather than a place it goes: {@code
     * ELEVATOR.before(Inches.of(32))}.
     *
     * @throws IllegalArgumentException if it's outside its range, or its reference
     */
    public Condition.Past past(Distance position) {
      return ConditionPastBuilder.builder()
          .joint(this)
          .position(position.in(Meters))
          .place(Optional.empty())
          .build();
    }

    /**
     * It before a distance: not yet past it, on its reference's side.
     *
     * @throws IllegalArgumentException if it's outside its range, or its reference
     */
    public Condition.Before before(Distance position) {
      return ConditionBeforeBuilder.builder()
          .joint(this)
          .position(position.in(Meters))
          .place(Optional.empty())
          .build();
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
          .fromPlace(Optional.empty())
          .toPlace(Optional.empty())
          .build();
    }

    /**
     * It running at least a speed, forward: {@code LIFT.atLeast(MetersPerSecond.of(0.5))}.
     *
     * @throws IllegalArgumentException for a speed that isn't forward
     */
    public Condition.AtLeast atLeast(LinearVelocity speed) {
      return ConditionAtLeastBuilder.builder().joint(this).speed(speed.in(MetersPerSecond)).build();
    }

    /** What it moves. */
    public Prismatic carries(Link link) {
      return with().link(link).build();
    }

    /** Its places by name: {@code .positions(ELEVATOR_DOWN, ELEVATOR_CLEAR)}. */
    public Prismatic positions(Position... positions) {
      return with().positions(List.of(positions)).build();
    }

    /** What it runs at, by name. */
    public Prismatic outputs(Output... outputs) {
      return with().outputs(List.of(outputs)).build();
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
      return driven(
          drivenWith()
              .motors(
                  Optional.of(turning(motor, count, gearing, Optional.of(pitchRadius), List.of())))
              .build());
    }

    /** Motors driving it together onto a drum, each on its controller, the leader first. */
    public Prismatic motors(
        Motor motor, Gearing gearing, Distance pitchRadius, MechanismWiring.Device... controllers) {
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

    /**
     * The same joint, its motors through another gearing onto the same drum, their controllers as
     * they are.
     *
     * @throws IllegalArgumentException if no motors drive it
     */
    public Prismatic gearing(Gearing gearing) {
      return driven(drivenWith().motors(Optional.of(regeared(this, gearing))).build());
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
    public Prismatic limitSwitch(
        Sensor.Switch.Limit.End end, SensorWiring.Input input, SensorWiring.Polarity polarity) {
      return limit(
          end, Optional.empty(), Optional.of(new SensorWiring.Connection(input, polarity)));
    }

    /**
     * A limit switch at an end whose position isn't known, on its own motor controller's limit pin.
     */
    public Prismatic limitSwitch(
        Sensor.Switch.Limit.End end, SensorWiring.Direction pin, SensorWiring.Polarity polarity) {
      return limitSwitch(end, SensorWiring.ControllerLimit.of(this, pin), polarity);
    }

    /** A limit switch at an end, tripping at a distance, on an input. */
    public Prismatic limitSwitch(
        Sensor.Switch.Limit.End end,
        Distance at,
        SensorWiring.Input input,
        SensorWiring.Polarity polarity) {
      return limit(
          end,
          Optional.of(at.in(Meters)),
          Optional.of(new SensorWiring.Connection(input, polarity)));
    }

    /** A limit switch at an end, tripping at a distance, on its own motor controller's pin. */
    public Prismatic limitSwitch(
        Sensor.Switch.Limit.End end,
        Distance at,
        SensorWiring.Direction pin,
        SensorWiring.Polarity polarity) {
      return limitSwitch(end, at, SensorWiring.ControllerLimit.of(this, pin), polarity);
    }

    /**
     * It kept out of a stretch of its travel while a condition holds, enforced beneath every
     * command: {@code .keepsOut(Range.meters(0, 0.25)).when(INTAKE.before(Degrees.of(30)))}. Its
     * target is held at the stretch's edge, on the side it's on, until the condition clears.
     */
    public KeepingOut<Prismatic> keepsOut(Range region) {
      return new KeepingOut<>(this, region, this::interlocked);
    }

    /**
     * It moving only while a condition holds, enforced beneath every command: {@code
     * .runsOnlyWhen(CLIMBER.past(Degrees.of(-75)))}. Meanwhile it holds where it is, or stops.
     */
    public Prismatic runsOnlyWhen(Condition condition) {
      return interlocked(Interlock.that(this).runsOnlyWhen(condition));
    }

    private Prismatic interlocked(Interlock interlock) {
      List<Interlock> interlocks = new ArrayList<>(driven.interlocks());
      interlocks.add(interlock);
      return driven(drivenWith().interlocks(interlocks).build());
    }

    /** How the program drives it. */
    public Prismatic control(MechanismControl control) {
      return driven(drivenWith().control(Optional.of(control)).build());
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
      List<Driven.LimitSwitch> switches = new ArrayList<>(driven.limitSwitches());
      switches.add(DrivenLimitSwitchBuilder.builder().end(end).position(at).wiring(wiring).build());
      return driven(drivenWith().limitSwitches(switches).build());
    }
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
        .pitchRadius(pitchRadius)
        .controllers(controllers)
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

  /** A joint's own checks, the same for both kinds. */
  private static void check(
      Class<? extends Record> kind,
      String label,
      Transform3d origin,
      Vector3 axis,
      Optional<Range> range,
      double reference,
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
        .field("reference", reference, finite())
        .check(
            "reference",
            () ->
                range.isEmpty() || range.get().contains(reference)
                    ? null
                    : shown(reference)
                        + " must be inside its range ("
                        + shown(range.get().lower())
                        + " to "
                        + shown(range.get().upper())
                        + ")")
        .check("positions", () -> misplaced(revolute, range, positions))
        .check("outputs", () -> twice(positions, outputs))
        .orThrow();
  }

  /**
   * The named positions of the wrong kind, a length for a joint that turns or an angle for one that
   * slides; else those outside the range; or null when they're all of its kind and in it.
   */
  private static @Nullable String misplaced(
      boolean revolute, Optional<Range> range, List<Position> positions) {
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
    if (!wrong.isEmpty()) {
      return String.join("; ", wrong);
    }
    if (range.isEmpty()) {
      return null;
    }
    List<String> outside = new ArrayList<>();
    for (Position position : positions) {
      if (!range.get().contains(position.value())) {
        outside.add(
            position.name()
                + ", "
                + Places.shownAt(revolute, position.value())
                + ", must be inside its range ("
                + Places.shownAt(revolute, range.get().lower())
                + " to "
                + Places.shownAt(revolute, range.get().upper())
                + ")");
      }
    }
    return outside.isEmpty() ? null : String.join("; ", outside);
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
