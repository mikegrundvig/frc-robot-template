package com.michaelgrundvig.frc.world.robot;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.Distance;

/**
 * Where a joint is, by its named positions: at one, past one, before one, or between two. An
 * interlock restricts a joint while another's condition holds ({@link Interlock}), and a
 * configuration is joints at named positions or outputs ({@link Configuration}). Each is written
 * from the joint: {@code pivot.past("run")}, {@code elevator.before("clear")}, {@code
 * pivot.between("run", "out")}, {@code arm.at("level")}.
 *
 * <p>They're static: "past" means the named position lies between the joint's reference and where
 * it is, so a condition means the same whichever way the joint is moving. The definition judges
 * them exactly, at named positions ({@link #holdsAt}); at run time, a joint is at a position within
 * its band.
 */
public sealed interface Condition permits Condition.OnJoint, Condition.OnSensor {
  /**
   * A condition on a joint's position: what interlocks and configurations are made of, and what the
   * definition can judge exactly, at named positions.
   */
  sealed interface OnJoint extends Condition
      permits Condition.At, Condition.Past, Condition.Before, Condition.Between {
    /** The joint whose position it's about. */
    Joint joint();

    /**
     * Whether it holds with its joint at a position, in the joint's units, judged exactly.
     *
     * @throws IllegalStateException for a joint at a named output, which is about how it runs
     */
    boolean holdsAt(double position);
  }

  /**
   * A condition on a sensor's reading: a switch tripped or clear, a rangefinder nearer or farther
   * than a distance. What it means is the team's to name ("coral in claw" is a beam tripped); only
   * the robot, reading the sensor, can judge it.
   */
  sealed interface OnSensor extends Condition
      permits Condition.Tripped, Condition.Clear, Condition.Nearer, Condition.Farther {
    /** The sensor it reads. */
    Sensor sensor();
  }

  /** A switch tripped: pressed, its magnet there, its beam blocked. {@code clawBeam.tripped()}. */
  record Tripped(Sensor.Switch sensor) implements OnSensor {
    @Override
    public String toString() {
      return sensor.label() + " tripped";
    }
  }

  /** A switch clear: not pressed, its beam unbroken. {@code hopperBeam.clear()}. */
  record Clear(Sensor.Switch sensor) implements OnSensor {
    @Override
    public String toString() {
      return sensor.label() + " clear";
    }
  }

  /**
   * Something nearer a rangefinder than a distance: {@code hopperRange.nearerThan(Inches.of(6))}.
   *
   * @param distance inside its range
   */
  record Nearer(Sensor.Rangefinder sensor, Distance distance) implements OnSensor {
    public Nearer {
      Checks.on(Nearer.class, sensor.label())
          .check("distance", () -> beyond(sensor, distance))
          .orThrow();
    }

    @Override
    public String toString() {
      return sensor.label() + " nearer than " + shown(distance);
    }
  }

  /** Nothing nearer a rangefinder than a distance: {@code hopperRange.fartherThan(...)}. */
  record Farther(Sensor.Rangefinder sensor, Distance distance) implements OnSensor {
    public Farther {
      Checks.on(Farther.class, sensor.label())
          .check("distance", () -> beyond(sensor, distance))
          .orThrow();
    }

    @Override
    public String toString() {
      return sensor.label() + " farther than " + shown(distance);
    }
  }

  /** Why a distance isn't one a rangefinder can tell, or null when it is. */
  private static @Nullable String beyond(Sensor.Rangefinder sensor, Distance distance) {
    return distance.in(Meters) > 0 && distance.lte(sensor.range())
        ? null
        : shown(distance)
            + " isn't inside "
            + sensor.label()
            + "'s range, "
            + shown(sensor.range());
  }

  private static String shown(Distance distance) {
    return String.format(Locale.ROOT, "%.3f m", distance.in(Meters));
  }

  /**
   * A joint at one of its named positions or outputs: {@code arm.at("level")}, {@code
   * feeder.at("feeding")}, {@code feeder.at("stopped")}. A configuration is made of these; an
   * interlock's condition is one at a position.
   *
   * @param joint the joint
   * @param name one of its positions or outputs, or {@code "stopped"}, which every joint has
   */
  @ConfigRecord
  record At(Joint joint, String name) implements OnJoint {
    public At {
      Checks.on(At.class, joint.label()).check("name", () -> unknown(joint, name, true)).orThrow();
    }

    /** Starts describing a joint at a named place, one named value at a time. */
    public static ConditionAtBuilder.JointStage builder() {
      return ConditionAtBuilder.builder();
    }

    /** Whether it names an output, rather than a position. */
    public boolean isOutput() {
      return name.equals(Output.STOPPED_NAME)
          || joint.outputs().stream().anyMatch(output -> output.name().equals(name));
    }

    @Override
    public boolean holdsAt(double position) {
      if (isOutput()) {
        throw new IllegalStateException(this + " is an output, not a position");
      }
      return position == joint.position(name).value();
    }

    @Override
    public String toString() {
      return joint.label() + " at " + name;
    }
  }

  /**
   * A joint past one of its named positions: the position between its reference and where it is.
   *
   * @param joint the joint
   * @param position one of its positions, other than its reference
   */
  @ConfigRecord
  record Past(Joint joint, String position) implements OnJoint {
    public Past {
      Checks.on(Past.class, joint.label())
          .check("position", () -> notAside(joint, position))
          .orThrow();
    }

    /** Starts describing a joint past a named position, one named value at a time. */
    public static ConditionPastBuilder.JointStage builder() {
      return ConditionPastBuilder.builder();
    }

    @Override
    public boolean holdsAt(double at) {
      double edge = joint.position(position).value();
      return edge > joint.reference() ? at >= edge : at <= edge;
    }

    @Override
    public String toString() {
      return joint.label() + " past " + position;
    }
  }

  /**
   * A joint before one of its named positions: not yet past it, on its reference's side.
   *
   * @param joint the joint
   * @param position one of its positions, other than its reference
   */
  @ConfigRecord
  record Before(Joint joint, String position) implements OnJoint {
    public Before {
      Checks.on(Before.class, joint.label())
          .check("position", () -> notAside(joint, position))
          .orThrow();
    }

    /** Starts describing a joint before a named position, one named value at a time. */
    public static ConditionBeforeBuilder.JointStage builder() {
      return ConditionBeforeBuilder.builder();
    }

    @Override
    public boolean holdsAt(double at) {
      double edge = joint.position(position).value();
      return edge > joint.reference() ? at < edge : at > edge;
    }

    @Override
    public String toString() {
      return joint.label() + " before " + position;
    }
  }

  /**
   * A joint between two of its named positions, both included, whichever way round they're named.
   *
   * @param joint the joint
   * @param from one of its positions
   * @param to another
   */
  @ConfigRecord
  record Between(Joint joint, String from, String to) implements OnJoint {
    public Between {
      Checks.on(Between.class, joint.label())
          .check("from", () -> unknown(joint, from, false))
          .check("to", () -> unknown(joint, to, false))
          .check(
              "to",
              () -> from.equals(to) ? "a joint between " + from + " and itself is at it" : null)
          .orThrow();
    }

    /** Starts describing a joint between two named positions, one named value at a time. */
    public static ConditionBetweenBuilder.JointStage builder() {
      return ConditionBetweenBuilder.builder();
    }

    @Override
    public boolean holdsAt(double at) {
      double a = joint.position(from).value();
      double b = joint.position(to).value();
      return at >= Math.min(a, b) && at <= Math.max(a, b);
    }

    @Override
    public String toString() {
      return joint.label() + " between " + from + " and " + to;
    }
  }

  /** Why a joint has no position (or output, where one may be) by a name, or null when it has. */
  private static @Nullable String unknown(Joint joint, String name, boolean orOutput) {
    List<String> names = new ArrayList<>();
    joint.positions().forEach(position -> names.add(position.name()));
    if (orOutput) {
      joint.outputs().forEach(output -> names.add(output.name()));
      names.add(Output.STOPPED_NAME);
    }
    return names.contains(name)
        ? null
        : joint.label()
            + " has no "
            + (orOutput ? "position or output" : "position")
            + " named \""
            + name
            + "\": it has "
            + (names.isEmpty() ? "none" : String.join(", ", names));
  }

  /** Why a position names no side of a joint's reference, or null when it does. */
  private static @Nullable String notAside(Joint joint, String position) {
    String unknown = unknown(joint, position, false);
    if (unknown != null) {
      return unknown;
    }
    return joint.position(position).value() == joint.reference()
        ? position
            + " is "
            + joint.label()
            + "'s reference: everywhere is past it, and nowhere before it"
        : null;
  }
}
