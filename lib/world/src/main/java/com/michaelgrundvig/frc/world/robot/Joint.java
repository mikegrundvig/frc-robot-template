package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * A mechanism's joint: one way a link moves on what it hangs from, turning about an axis ({@link
 * Revolute}, in radians) or sliding along one ({@link Prismatic}, in meters). Joints hang from the
 * chassis or from each other, each written after its parent, so they form a tree, and an arm on an
 * elevator is a joint whose parent is the elevator's.
 *
 * <p>Position 0 is the pose its {@link #origin} gives: an arm's zero is wherever CAD's frame put
 * it, and where it is from horizontal is derived ({@code RobotMath.gravityOffset}), never written
 * down as a second number. A joint has no state: where it is now comes from its sensors, as a
 * position the derived functions take.
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
   * A pivot in a line: an arm, an intake's swing, a wrist, turning about an axis between its hard
   * stops, stowed where it starts: {@code Joint.pivot("pivot", Part.CHASSIS, at, Joint.PITCH_DOWN,
   * Range.degrees(0, 135), Degrees.of(1.5), link, Position.degrees("out", 130))}. The builder
   * ({@link Revolute#builder}) says the rest, such as a pivot with no stops.
   *
   * @param at where its axle is, in its parent's frame
   * @param stowed where it starts, inside its range
   */
  static Revolute pivot(
      String label,
      Part parent,
      Translation3d at,
      Vector3 axis,
      Range range,
      Angle stowed,
      Link link,
      Position... positions) {
    return Revolute.builder()
        .label(label)
        .parent(parent)
        .origin(new Transform3d(at, Rotation3d.ZERO))
        .axis(axis)
        .range(Optional.of(range))
        .reference(stowed.in(Radians))
        .link(link)
        .positions(List.of(positions))
        .outputs(List.of())
        .build();
  }

  /**
   * A roller, a belt or a flywheel in a line: a joint that turns freely, from 0, run at its named
   * outputs: {@code Joint.roller("intake", PIVOT, at, link, Output.rpm("intaking", 2000))}.
   *
   * @param at where its axle is, in its parent's frame
   */
  static Revolute roller(
      String label, Part parent, Translation3d at, Link link, Output... outputs) {
    return Revolute.builder()
        .label(label)
        .parent(parent)
        .origin(new Transform3d(at, Rotation3d.ZERO))
        .axis(PITCH_DOWN)
        .range(Optional.empty())
        .reference(0)
        .link(link)
        .positions(List.of())
        .outputs(List.of(outputs))
        .build();
  }

  /**
   * An elevator in a line: a carriage sliding straight up between its hard stops, stowed where it
   * starts: {@code Joint.elevator("elevator", Part.CHASSIS, at, Range.meters(0, 1.2), Meters.of(0),
   * link, Position.of("clear", Inches.of(20)))}.
   *
   * @param at where the slide starts, in its parent's frame
   * @param stowed where it starts, inside its range
   */
  static Prismatic elevator(
      String label,
      Part parent,
      Translation3d at,
      Range range,
      Distance stowed,
      Link link,
      Position... positions) {
    return Prismatic.builder()
        .label(label)
        .parent(parent)
        .origin(new Transform3d(at, Rotation3d.ZERO))
        .axis(UP)
        .range(Optional.of(range))
        .reference(stowed.in(Meters))
        .link(link)
        .positions(List.of(positions))
        .outputs(List.of())
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

  /** How its frame moves from its origin's at a position: turned about, or slid along, its axis. */
  Transform3d motion(double position);

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
   * It at one of its named positions or outputs: what a configuration is made of ({@code
   * Configuration.of("amp", elevator.at("clear"), arm.at("level"))}), and, at a position, an
   * interlock's condition.
   *
   * @throws IllegalArgumentException if it has no position or output by the name
   */
  default Condition.At at(String name) {
    return ConditionAtBuilder.builder().joint(this).name(name).build();
  }

  /**
   * It past one of its named positions: the position lies between its reference and where it is, so
   * a pivot stowed at 1.5° is past "run", at 110°, from 110° on.
   *
   * @throws IllegalArgumentException if it has no position by the name, or it's the reference
   */
  default Condition.Past past(String position) {
    return ConditionPastBuilder.builder().joint(this).position(position).build();
  }

  /**
   * It before one of its named positions: not yet past it, on its reference's side.
   *
   * @throws IllegalArgumentException if it has no position by the name, or it's the reference
   */
  default Condition.Before before(String position) {
    return ConditionBeforeBuilder.builder().joint(this).position(position).build();
  }

  /**
   * It between two of its named positions, both included, whichever way round they're named.
   *
   * @throws IllegalArgumentException if it has no positions by the names, or they're one
   */
  default Condition.Between between(String from, String to) {
    return ConditionBetweenBuilder.builder().joint(this).from(from).to(to).build();
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
      List<Output> outputs)
      implements Joint {
    public Revolute {
      positions = List.copyOf(positions);
      outputs = List.copyOf(outputs);
      check(Revolute.class, label, origin, axis, range, reference, positions, outputs);
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

    /**
     * The common case in a line: motors turning it through a gearing, under its label: {@code
     * arm.motors(Motor.KRAKEN_X60, 2, gearing)}.
     */
    public Drive motors(Motor motor, int count, Gearing gearing) {
      return Drive.builder()
          .label(label)
          .actuator(Actuator.Motors.builder().motor(motor).count(count).build())
          .transmission(Transmission.Geared.builder().joint(this).gearing(gearing).build())
          .build();
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
      List<Output> outputs)
      implements Joint {
    public Prismatic {
      positions = List.copyOf(positions);
      outputs = List.copyOf(outputs);
      check(Prismatic.class, label, origin, axis, range, reference, positions, outputs);
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

    /**
     * The common case in a line: motors driving it through a gearing onto a drum, a sprocket or a
     * pinion, under its label: {@code carriage.motors(Motor.KRAKEN_X60, 2, gearing, radius)}.
     *
     * @param pitchRadius where the belt, chain or rack meets the drum, sprocket or pinion
     */
    public Drive motors(Motor motor, int count, Gearing gearing, Distance pitchRadius) {
      return Drive.builder()
          .label(label)
          .actuator(Actuator.Motors.builder().motor(motor).count(count).build())
          .transmission(
              Transmission.Drum.builder()
                  .joint(this)
                  .gearing(gearing)
                  .pitchRadius(pitchRadius)
                  .build())
          .build();
    }
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
