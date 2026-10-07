package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * An interlock: what one joint may do, by where another is or what a sensor reads. Each is one of
 * two shapes, written from the joint it restricts:
 *
 * <pre>{@code
 * Interlock.that(INTAKE).runsOnlyWhen(PIVOT.past("run"))      // the roller only with the pivot out
 * Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.before("clear"))
 * Interlock.that(LATERATOR).keepsOut(Range.of(0.1, 0.4)).when(CORAL_BEAM.clear())
 * }</pre>
 *
 * <p>Interlocks are facts about the robot, in its definition, enforced beneath every command: no
 * command, binding, auto or button skips one. They speak only of the robot's own joints and
 * sensors; where the field or the game says what's allowed (a trench, a zone, a hub), that's a gate
 * in the robot's code, which knows the field. A sensor that's lost holds no condition: one that
 * keeps a joint out keeps it out, and one it runs only when holds it. The definition checks them
 * when it's built: every joint and sensor one of the robot's, none that follows another, no two
 * joints waiting on each other, no joint held where it can't move or from a named position it can't
 * then reach, and every configuration, and the robot as it starts, obeying every interlock on a
 * joint's position (a sensor's reading isn't known until the robot runs).
 */
public sealed interface Interlock permits Interlock.RunsOnlyWhen, Interlock.KeepsOut {
  /** The joint it restricts. */
  Joint joint();

  /** Where the other joint must be, or what a sensor must read, for it to apply: see each shape. */
  Condition condition();

  /** The other joint its condition is about, where it is or how fast it runs; none for a sensor. */
  default Optional<Joint> other() {
    return switch (condition()) {
      case Condition.OnJoint onJoint -> Optional.of(onJoint.joint());
      case Condition.AtLeast atLeast -> Optional.of(atLeast.joint());
      case Condition.OnSensor onSensor -> Optional.empty();
    };
  }

  /** Starts an interlock about a joint: {@code Interlock.that(INTAKE).runsOnlyWhen(...)}. */
  static Subject that(Joint joint) {
    return new Subject(joint);
  }

  /**
   * A joint that may move only while a condition holds: while it doesn't, a joint held to a
   * position stays where it is, and one run at an output stops, each going on when it holds again.
   * 2611's intake roller runs only with its pivot past "run".
   *
   * @param joint the joint it restricts
   * @param condition where another joint must be, or what a sensor must read, for it to move
   */
  @ConfigRecord
  record RunsOnlyWhen(Joint joint, Condition condition) implements Interlock {
    public RunsOnlyWhen {
      Checks.on(RunsOnlyWhen.class, joint.label())
          .check("condition", () -> notAnother(joint, condition))
          .orThrow();
    }

    /** Starts describing the interlock, one named value at a time. */
    public static InterlockRunsOnlyWhenBuilder.JointStage builder() {
      return InterlockRunsOnlyWhenBuilder.builder();
    }

    @Override
    public String toString() {
      return joint.label() + " runs only when " + condition;
    }
  }

  /**
   * A joint kept out of a region of its travel while a condition holds: its target is held at the
   * region's edge, on the side it's on, until the condition clears. An arm kept out of the bumper's
   * band while the elevator is too low to clear it.
   *
   * @param joint the joint it restricts
   * @param region where the joint may not be meanwhile, in its units: part of its range, and not
   *     all
   * @param condition where another joint is, or what a sensor reads, while the region is closed
   */
  @ConfigRecord
  record KeepsOut(Joint joint, Range region, Condition condition) implements Interlock {
    public KeepsOut {
      Checks.on(KeepsOut.class, joint.label())
          .check("condition", () -> notAnother(joint, condition))
          .check("region", () -> notPartOf(joint, region))
          .orThrow();
    }

    /** Starts describing the interlock, one named value at a time. */
    public static InterlockKeepsOutBuilder.JointStage builder() {
      return InterlockKeepsOutBuilder.builder();
    }

    /** Whether a position is inside the region, short of its edges, where the joint may be. */
    public boolean keepsOut(double position) {
      return position > region.lower() && position < region.upper();
    }

    @Override
    public String toString() {
      boolean revolute = joint instanceof Joint.Revolute;
      return joint.label()
          + " keeps out of "
          + Places.shownAt(revolute, region.lower())
          + " to "
          + Places.shownAt(revolute, region.upper())
          + " while "
          + condition;
    }
  }

  /**
   * A joint an interlock is being written about: say what it may do. Not an interlock until it's
   * said.
   *
   * @param joint the joint
   */
  record Subject(Joint joint) {
    /** It moves only while the condition holds. */
    public RunsOnlyWhen runsOnlyWhen(Condition condition) {
      return RunsOnlyWhen.builder().joint(joint).condition(condition).build();
    }

    /** It's kept out of a region of its travel: say while what ({@link KeepingOut#when}). */
    public KeepingOut keepsOut(Range region) {
      return new KeepingOut(joint, region);
    }
  }

  /**
   * A joint kept out of a region, being written: say while what. Not an interlock until it's said.
   *
   * @param joint the joint
   * @param region where it may not be
   */
  record KeepingOut(Joint joint, Range region) {
    /** While the condition holds. */
    public KeepsOut when(Condition condition) {
      return KeepsOut.builder().joint(joint).region(region).condition(condition).build();
    }
  }

  /**
   * Why a condition can't restrict a joint, or null when it can: one on a joint must be about
   * another joint, where it is or how fast it runs, not one of its named outputs. Any sensor's can.
   */
  private static @Nullable String notAnother(Joint joint, Condition given) {
    if (given instanceof Condition.AtLeast atLeast) {
      return atLeast.joint().equals(joint)
          ? joint.label() + " can't be held by how fast it runs itself: an interlock is on another"
          : null;
    }
    if (!(given instanceof Condition.OnJoint condition)) {
      return null;
    }
    if (condition.joint().equals(joint)) {
      return joint.label()
          + " can't be held by where it is itself: limit it by its range, or an interlock on another"
          + " joint";
    }
    if (condition instanceof Condition.At at && at.isOutput()) {
      return "an interlock speaks of where a joint is, or how fast it runs, and "
          + at.name()
          + " is "
          + at.joint().label()
          + "'s output: say the speed it must reach, "
          + at.joint().label()
          + ".atLeast(...)";
    }
    return null;
  }

  /** Why a region isn't part of a joint's range, and not all of it, or null when it is. */
  private static @Nullable String notPartOf(Joint joint, Range region) {
    if (joint.range().isEmpty()) {
      return null;
    }
    Range range = joint.range().get();
    boolean revolute = joint instanceof Joint.Revolute;
    String regionShown =
        Places.shownAt(revolute, region.lower())
            + " to "
            + Places.shownAt(revolute, region.upper());
    String rangeShown =
        Places.shownAt(revolute, range.lower()) + " to " + Places.shownAt(revolute, range.upper());
    if (region.upper() <= range.lower() || region.lower() >= range.upper()) {
      return regionShown + " is outside " + joint.label() + "'s range, " + rangeShown;
    }
    if (region.lower() <= range.lower() && region.upper() >= range.upper()) {
      return regionShown
          + " is all of "
          + joint.label()
          + "'s range, "
          + rangeShown
          + ": it would have nowhere to be";
    }
    return null;
  }
}
