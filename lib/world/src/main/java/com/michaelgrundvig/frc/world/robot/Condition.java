package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.Locale;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.Distance;

/**
 * Where a joint is: at one of its named positions; at least or at most a position, or between two;
 * how fast it runs; or what a sensor reads. An interlock restricts a joint while another's
 * condition holds ({@link Interlock}), and a configuration is joints at named positions or outputs
 * ({@link Configuration}). Each is written from the joint: {@code PIVOT.atLeast(Degrees.of(110))},
 * {@code ELEVATOR.atMost(Inches.of(32))}, {@code ELEVATOR.atMost(ELEVATOR_CLEAR)}, {@code
 * SHOOTER.upToSpeed(RPM.of(3000))} for an interlock, and {@code ARM.at(ARM_SCORING)} for a
 * configuration.
 *
 * <p>They're static, and none depends on where the joint rests, so a condition means the same
 * whichever way the joint is moving, and when its rest moves. Each is judged exactly ({@link
 * OnJoint#holdsAt}), by the definition at named positions and by the robot at where it measures the
 * joint. So an interlock refuses "at" one of a joint's positions, which a measured position never
 * exactly is.
 *
 * <p>A place a condition names is kept as the code names it, and whether it's the joint's own is
 * for what uses the condition to say ({@link RobotDefinition#whyNotOwn(Condition)}): the definition
 * refuses an interlock or a configuration on another joint's place, and the robot a gate on one, as
 * it refuses a move to one. So {@code PIVOT.atLeast(ELEVATOR_CLEAR)} built while the robot runs is
 * made, and its gate holds false, saying why.
 */
public sealed interface Condition
    permits Condition.OnJoint, Condition.OnSensor, Condition.UpToSpeed {
  /**
   * A condition on a joint's position: what interlocks and configurations are made of, and what the
   * definition can judge exactly, at named positions.
   */
  sealed interface OnJoint extends Condition
      permits Condition.At, Condition.AtLeast, Condition.AtMost, Condition.Between {
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
      Places.refuse(beyond(sensor, distance));
    }

    @Override
    public String toString() {
      return sensor.label() + " nearer than " + shown(distance);
    }
  }

  /** Nothing nearer a rangefinder than a distance: {@code hopperRange.fartherThan(...)}. */
  record Farther(Sensor.Rangefinder sensor, Distance distance) implements OnSensor {
    public Farther {
      Places.refuse(beyond(sensor, distance));
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
            + possessive(sensor.label())
            + " range, "
            + shown(sensor.range());
  }

  private static String shown(Distance distance) {
    return String.format(Locale.ROOT, "%.3f m", distance.in(Meters));
  }

  /**
   * A joint at one of its named positions or outputs: {@code ARM.at(ARM_STOWED)}, {@code
   * FEEDER.at(FEEDER_FEEDING)}, {@code FEEDER.at(Output.STOPPED)}. A configuration is made of
   * these. An interlock refuses one: at a position it holds only exactly there, which a measured
   * position never is, so an interlock's edge is {@link AtLeast}, {@link AtMost} or {@link
   * Between}.
   *
   * @param joint the joint
   * @param position one of its positions, as the code names it; empty for an output
   * @param output one of its outputs, as the code names it, or {@link Output#STOPPED}, which every
   *     joint has; empty for a position
   */
  @ConfigRecord
  record At(Joint joint, Optional<Position> position, Optional<Output> output) implements OnJoint {
    public At {
      if (position.isPresent() == output.isPresent()) {
        throw new IllegalArgumentException(
            joint.label() + " at a place is at one of its positions or one of its outputs");
      }
    }

    /** Starts describing a joint at a named place, one named value at a time. */
    public static ConditionAtBuilder.JointStage builder() {
      return ConditionAtBuilder.builder();
    }

    /** Its place's name: one of the joint's positions or outputs, or {@code "stopped"}. */
    public String name() {
      return position.map(Position::name).orElseGet(() -> output.orElseThrow().name());
    }

    /** Whether it names an output, rather than a position. */
    public boolean isOutput() {
      return output.isPresent();
    }

    @Override
    public boolean holdsAt(double at) {
      if (isOutput()) {
        throw new IllegalStateException(this + " is an output, not a position");
      }
      return at == joint.position(name()).value();
    }

    @Override
    public String toString() {
      return joint.label() + " at " + name();
    }
  }

  /**
   * A joint at a position or beyond it, toward its range's upper end: {@code
   * PIVOT.atLeast(Degrees.of(110))}, or from one of its named positions on, {@code
   * PIVOT.atLeast(PIVOT_OUT)}. What an interlock's edge usually is, a clearance, and the same
   * wherever the joint rests.
   *
   * @param joint the joint
   * @param position where, in its units: inside its range, and short of its lower end, where it
   *     would always hold
   * @param place its position there, as the code names it, when it's one of them
   */
  @ConfigRecord
  record AtLeast(Joint joint, double position, Optional<Position> place) implements OnJoint {
    public AtLeast {
      if (own(joint, place)) {
        Places.refuse(notAnEdge(joint, position, place, true));
      }
    }

    /** Starts describing a joint at least at a position, one named value at a time. */
    public static ConditionAtLeastBuilder.JointStage builder() {
      return ConditionAtLeastBuilder.builder();
    }

    @Override
    public boolean holdsAt(double at) {
      return at >= position;
    }

    @Override
    public String toString() {
      return joint.label() + " at least " + shown(joint, position, place);
    }
  }

  /**
   * A joint at a position or short of it, toward its range's lower end: {@code
   * ELEVATOR.atMost(Inches.of(32))}, or up to one of its named positions, {@code
   * ELEVATOR.atMost(ELEVATOR_CLEAR)}. What an interlock's edge usually is, a clearance, and the
   * same wherever the joint rests.
   *
   * @param joint the joint
   * @param position where, in its units: inside its range, and short of its upper end, where it
   *     would always hold
   * @param place its position there, as the code names it, when it's one of them
   */
  @ConfigRecord
  record AtMost(Joint joint, double position, Optional<Position> place) implements OnJoint {
    public AtMost {
      if (own(joint, place)) {
        Places.refuse(notAnEdge(joint, position, place, false));
      }
    }

    /** Starts describing a joint at most at a position, one named value at a time. */
    public static ConditionAtMostBuilder.JointStage builder() {
      return ConditionAtMostBuilder.builder();
    }

    @Override
    public boolean holdsAt(double at) {
      return at <= position;
    }

    @Override
    public String toString() {
      return joint.label() + " at most " + shown(joint, position, place);
    }
  }

  /**
   * A joint between two positions, both included, whichever way round they're given.
   *
   * @param joint the joint
   * @param from one end, in its units, inside its range
   * @param to the other
   * @param fromPlace its position at {@code from}, as the code names it, when it's one of them
   * @param toPlace its position at {@code to}, the same
   */
  @ConfigRecord
  record Between(
      Joint joint, double from, double to, Optional<Position> fromPlace, Optional<Position> toPlace)
      implements OnJoint {
    public Between {
      if (own(joint, fromPlace) && own(joint, toPlace)) {
        Places.refuse(
            notWithin(joint, from, fromPlace),
            notWithin(joint, to, toPlace),
            Math.abs(from - to) <= Range.EDGE
                ? "a joint between " + shown(joint, from, fromPlace) + " and itself is at it"
                : null);
      }
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
  record UpToSpeed(Joint joint, double speed) implements Condition {
    public UpToSpeed {
      Places.refuse(
          Double.isFinite(speed) && speed > 0
              ? null
              : "a speed to reach must be forward, and more than none: "
                  + Places.shownSpeed(joint instanceof Joint.Revolute, speed));
    }

    /** Starts describing a joint running at least a speed, one named value at a time. */
    public static ConditionUpToSpeedBuilder.JointStage builder() {
      return ConditionUpToSpeedBuilder.builder();
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
  private static String shown(Joint joint, double position, Optional<Position> place) {
    return place
        .map(Position::name)
        .orElseGet(() -> Places.shownAt(joint instanceof Joint.Revolute, position));
  }

  /**
   * Whether a place a condition names, if any, is the joint's own: its other checks are only for
   * one that is. One that isn't is refused by what uses the condition, which says whose it is.
   */
  private static boolean own(Joint joint, Optional<Position> place) {
    return place.isEmpty() || joint.whyNotOwn(place.get()) == null;
  }

  /**
   * Why a position isn't one a joint can be at, or null when it is: inside its range, and, when
   * it's named, where the joint has that position.
   */
  private static @Nullable String notWithin(
      Joint joint, double position, Optional<Position> place) {
    if (!Double.isFinite(position)) {
      return position + " isn't a position";
    }
    if (place.isPresent()) {
      double own = joint.position(place.get().name()).value();
      if (own != position) {
        return place.get().name()
            + " is "
            + shown(joint, own, Optional.empty())
            + ", not "
            + shown(joint, position, Optional.empty());
      }
    }
    if (joint.range().isPresent() && !joint.range().get().contains(position)) {
      Range range = joint.range().get();
      boolean revolute = joint instanceof Joint.Revolute;
      return shown(joint, position, place)
          + " is outside "
          + possessive(joint.label())
          + " range, "
          + Places.shownAt(revolute, range.lower())
          + " to "
          + Places.shownAt(revolute, range.upper());
    }
    return null;
  }

  /**
   * Why a position isn't an edge a joint can be either side of, or null when it is: inside its
   * range, and short of the end where being at least it (or at most) would always hold.
   */
  private static @Nullable String notAnEdge(
      Joint joint, double position, Optional<Position> place, boolean atLeast) {
    String within = notWithin(joint, position, place);
    if (within != null || joint.range().isEmpty()) {
      return within;
    }
    double end = atLeast ? joint.range().get().lower() : joint.range().get().upper();
    return Math.abs(position - end) <= Range.EDGE
        ? shown(joint, position, place)
            + " is "
            + possessive(joint.label())
            + " "
            + (atLeast ? "lowest: it's at least that" : "highest: it's at most that")
            + " everywhere"
        : null;
  }
}
