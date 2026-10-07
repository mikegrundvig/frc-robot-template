package com.michaelgrundvig.frc.world.robot;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.Distance;

/**
 * Where a joint is: at one of its named positions, or past, before or between positions, given as
 * numbers or as its named positions; how fast it runs; or what a sensor reads. An interlock
 * restricts a joint while another's condition holds ({@link Interlock}), and a configuration is
 * joints at named positions or outputs ({@link Configuration}). Each is written from the joint:
 * {@code PIVOT.past(Degrees.of(110))}, {@code ELEVATOR.before(Inches.of(32))}, {@code
 * ARM.at(ARM_LEVEL)}, {@code SHOOTER.atLeast(RPM.of(3000))}.
 *
 * <p>They're static: "past" means the named position lies between the joint's reference and where
 * it is, so a condition means the same whichever way the joint is moving. The definition judges
 * them exactly, at named positions ({@link #holdsAt}); at run time, a joint is at a position within
 * its band.
 */
public sealed interface Condition permits Condition.OnJoint, Condition.OnSensor, Condition.AtLeast {
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
   * A joint past a position: the position lies between its reference and where it is, so a pivot
   * stowed at 1.5° is past 110° from 110° on, whichever way it's moving.
   *
   * @param joint the joint
   * @param position where, in its units: inside its range, and not its reference
   * @param place the name of its position there, when it's one of them, for what's shown
   */
  @ConfigRecord
  record Past(Joint joint, double position, Optional<String> place) implements OnJoint {
    public Past {
      Checks.on(Past.class, joint.label())
          .check("position", () -> notAside(joint, position, place))
          .orThrow();
    }

    /** Starts describing a joint past a position, one named value at a time. */
    public static ConditionPastBuilder.JointStage builder() {
      return ConditionPastBuilder.builder();
    }

    @Override
    public boolean holdsAt(double at) {
      return position > joint.reference() ? at >= position : at <= position;
    }

    @Override
    public String toString() {
      return joint.label() + " past " + shown(joint, position, place);
    }
  }

  /**
   * A joint before a position: not yet past it, on its reference's side.
   *
   * @param joint the joint
   * @param position where, in its units: inside its range, and not its reference
   * @param place the name of its position there, when it's one of them, for what's shown
   */
  @ConfigRecord
  record Before(Joint joint, double position, Optional<String> place) implements OnJoint {
    public Before {
      Checks.on(Before.class, joint.label())
          .check("position", () -> notAside(joint, position, place))
          .orThrow();
    }

    /** Starts describing a joint before a position, one named value at a time. */
    public static ConditionBeforeBuilder.JointStage builder() {
      return ConditionBeforeBuilder.builder();
    }

    @Override
    public boolean holdsAt(double at) {
      return position > joint.reference() ? at < position : at > position;
    }

    @Override
    public String toString() {
      return joint.label() + " before " + shown(joint, position, place);
    }
  }

  /**
   * A joint between two positions, both included, whichever way round they're given.
   *
   * @param joint the joint
   * @param from one end, in its units, inside its range
   * @param to the other
   * @param fromPlace the name of its position at {@code from}, when it's one of them
   * @param toPlace the name of its position at {@code to}, when it's one of them
   */
  @ConfigRecord
  record Between(
      Joint joint, double from, double to, Optional<String> fromPlace, Optional<String> toPlace)
      implements OnJoint {
    public Between {
      Checks.on(Between.class, joint.label())
          .check("from", () -> notWithin(joint, from, fromPlace))
          .check("to", () -> notWithin(joint, to, toPlace))
          .check(
              "to",
              () ->
                  Math.abs(from - to) <= Range.EDGE
                      ? "a joint between " + shown(joint, from, fromPlace) + " and itself is at it"
                      : null)
          .orThrow();
    }

    /** Starts describing a joint between two positions, one named value at a time. */
    public static ConditionBetweenBuilder.JointStage builder() {
      return ConditionBetweenBuilder.builder();
    }

    @Override
    public boolean holdsAt(double at) {
      return at >= Math.min(from, to) && at <= Math.max(from, to);
    }

    @Override
    public String toString() {
      return joint.label()
          + " between "
          + shown(joint, from, fromPlace)
          + " and "
          + shown(joint, to, toPlace);
    }
  }

  /**
   * A joint running at least a speed, forward: a flywheel up to speed before anything feeds it.
   * Only the robot, reading the joint, can judge it, as a sensor's condition: a joint that isn't
   * answering, or doesn't know where it is, doesn't hold it.
   *
   * @param joint the joint
   * @param speed its speed, in its units a second: radians, or meters
   */
  @ConfigRecord
  record AtLeast(Joint joint, double speed) implements Condition {
    public AtLeast {
      Checks.on(AtLeast.class, joint.label())
          .check(
              "speed",
              () ->
                  Double.isFinite(speed) && speed > 0
                      ? null
                      : "a speed to reach must be forward, and more than none: "
                          + Places.shownSpeed(joint instanceof Joint.Revolute, speed))
          .orThrow();
    }

    /** Starts describing a joint running at least a speed, one named value at a time. */
    public static ConditionAtLeastBuilder.JointStage builder() {
      return ConditionAtLeastBuilder.builder();
    }

    /** Whether it holds with the joint running at a speed, in its units a second. */
    public boolean holdsAt(double velocity) {
      return velocity >= speed;
    }

    @Override
    public String toString() {
      return joint.label()
          + " at least "
          + Places.shownSpeed(joint instanceof Joint.Revolute, speed);
    }
  }

  /** A position as it's shown: its name, when it's one of the joint's, or its value. */
  private static String shown(Joint joint, double position, Optional<String> place) {
    return place.orElseGet(() -> Places.shownAt(joint instanceof Joint.Revolute, position));
  }

  /**
   * Why a position isn't one a joint can be at, or null when it is: inside its range, and, when
   * it's named, one of its positions, there.
   */
  private static @Nullable String notWithin(Joint joint, double position, Optional<String> place) {
    if (!Double.isFinite(position)) {
      return position + " isn't a position";
    }
    if (place.isPresent()) {
      String unknown = unknown(joint, place.get(), false);
      if (unknown != null) {
        return unknown;
      }
      if (joint.position(place.get()).value() != position) {
        return place.get()
            + " is "
            + shown(joint, joint.position(place.get()).value(), Optional.empty())
            + ", not "
            + shown(joint, position, Optional.empty());
      }
    }
    if (joint.range().isPresent() && !joint.range().get().contains(position)) {
      Range range = joint.range().get();
      boolean revolute = joint instanceof Joint.Revolute;
      return shown(joint, position, place)
          + " is outside "
          + joint.label()
          + "'s range, "
          + Places.shownAt(revolute, range.lower())
          + " to "
          + Places.shownAt(revolute, range.upper());
    }
    return null;
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
  private static @Nullable String notAside(Joint joint, double position, Optional<String> place) {
    String within = notWithin(joint, position, place);
    if (within != null) {
      return within;
    }
    // A step apart is the reference: Degrees.of(1.5) and Math.toRadians(1.5) differ in the last
    // bit.
    return Math.abs(position - joint.reference()) <= Range.EDGE
        ? shown(joint, position, place)
            + " is "
            + joint.label()
            + "'s reference: everywhere is past it, and nowhere before it"
        : null;
  }
}
