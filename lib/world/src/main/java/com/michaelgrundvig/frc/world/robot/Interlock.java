package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * An interlock: what one joint may do, by where another is or what a sensor reads. Each is one of
 * two shapes, written from the joint it restricts, in its block, or in the robot's definition for
 * one its block can't name yet ({@code Interlock.that(ELEVATOR).keepsOut(...).when(...)}). A roller
 * that runs only with the pivot out, in the roller's block:
 *
 * {@snippet class=frc.robot.examples.ExampleRobot region=runs-only-when}
 *
 * <p>An arm kept out of the bumper's band while the elevator is too low, in the arm's:
 *
 * {@snippet class=frc.robot.examples.ExampleRobot region=keeps-out}
 *
 * <p>Interlocks are facts about the robot, in its definition, enforced beneath every command: no
 * command, binding, auto or button skips one. They speak only of the robot's own joints and
 * sensors; where the field or the game says what's allowed (a trench, a zone, a hub), that's a gate
 * in the robot's code, which knows the field. A sensor that's lost holds no condition: one that
 * keeps a joint out keeps it out, and one it runs only when holds it. The definition checks them
 * when it's built: every joint and sensor one of the robot's, none that follows another, no joint
 * held where it can't move or from a named position it can't then reach, and every configuration,
 * and the robot as it starts, obeying every interlock on a joint's position (a sensor's reading
 * isn't known until the robot runs), with no joints there holding each other so that none can move
 * to free another.
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
      case Condition.UpToSpeed upToSpeed -> Optional.of(upToSpeed.joint());
      case Condition.OnSensor onSensor -> Optional.empty();
    };
  }

  /** Starts an interlock about a joint: {@code Interlock.that(ROLLER).runsOnlyWhen(...)}. */
  static Subject that(Joint joint) {
    return new Subject(joint);
  }

  /**
   * A joint that may move only while a condition holds: while it doesn't, a joint held to a
   * position stays where it is, and one run at an output stops, each going on when it holds again.
   * 2611's intake roller runs only with its pivot past 110°.
   *
   * @param joint the joint it restricts
   * @param condition where another joint must be, or what a sensor must read, for it to move
   */
  @ConfigRecord
  record RunsOnlyWhen(Joint joint, Condition condition) implements Interlock {
    public RunsOnlyWhen {
      Places.refuse(notAnother(joint, condition));
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
      Places.refuse(notAnother(joint, condition), notPartOf(joint, region));
    }

    /** Starts describing the interlock, one named value at a time. */
    public static InterlockKeepsOutBuilder.JointStage builder() {
      return InterlockKeepsOutBuilder.builder();
    }

    /**
     * Whether a position is inside the region: short of its edges, where the joint may be, and at
     * an edge that reaches an end of the joint's range, past which there's no room to be out. A
     * joint headed there is held at the region's other edge, so it never gets to that end.
     */
    public boolean keepsOut(double position) {
      return (roomBelow() ? position > region.lower() : position >= region.lower())
          && (roomAbove() ? position < region.upper() : position <= region.upper());
    }

    /**
     * Which side of the region a joint at a position is kept to: below it (true) or above. Out of
     * it, the side it's on. Inside, as one there when the robot started or when the region closed
     * is, it may only leave: by the side it was last on, else by the nearer edge, and never by a
     * side with no room past it.
     *
     * @param lastSide the side the joint was last out on: -1 below, 1 above, 0 never or unknown
     */
    public boolean keptBelow(double position, int lastSide) {
      if (position <= region.lower() && roomBelow()) {
        return true;
      }
      if (position >= region.upper() && roomAbove()) {
        return false;
      }
      return !roomAbove()
          || (roomBelow()
              && (lastSide != 0
                  ? lastSide < 0
                  : position - region.lower() <= region.upper() - position));
    }

    /** Whether there's room below the region: it doesn't reach the low end of the joint's range. */
    private boolean roomBelow() {
      Optional<Range> range = joint.range();
      return range.isEmpty() || region.lower() > range.get().lower();
    }

    /** Whether there's room above the region: it doesn't reach the high end of its range. */
    private boolean roomAbove() {
      Optional<Range> range = joint.range();
      return range.isEmpty() || region.upper() < range.get().upper();
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
   * another joint, where it is or how fast it runs, not one of its named outputs, nor exactly at
   * one of its positions, which the joint, measured, never is. Any sensor's can.
   */
  private static @Nullable String notAnother(Joint joint, Condition given) {
    if (given instanceof Condition.UpToSpeed upToSpeed) {
      return upToSpeed.joint().equals(joint)
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
    if (condition instanceof Condition.At at) {
      String other = at.joint().label();
      if (at.isOutput()) {
        return "an interlock speaks of where a joint is, or how fast it runs, and "
            + at.name()
            + " is "
            + possessive(other)
            + " output: say the speed it must reach, "
            + other
            + ".upToSpeed(...)";
      }
      // The robot judges it against where the joint is measured, which is never exactly a place.
      boolean revolute = at.joint() instanceof Joint.Revolute;
      String of = revolute ? "Degrees.of(...)" : "Inches.of(...)";
      return at
          + " holds only with "
          + other
          + " exactly at "
          + at.name()
          + ", "
          + Places.shownAt(revolute, at.joint().position(at.name()).value())
          + ", which a measured position never is, so the interlock would never hold: say the"
          + " edge where the parts clear, "
          + other
          + ".atLeast("
          + of
          + ") or "
          + other
          + ".atMost("
          + of
          + "), or a band around "
          + at.name()
          + ", "
          + other
          + ".between("
          + of
          + ", "
          + of
          + ")";
    }
    return null;
  }

  /**
   * Why a region isn't part of a joint's range, and not all of it, or isn't of its kind, or null
   * when it is.
   */
  private static @Nullable String notPartOf(Joint joint, Range region) {
    boolean revolute = joint instanceof Joint.Revolute;
    if (region.kind() != Position.Kind.of(joint)) {
      return Joint.wrongKind("the region is", revolute);
    }
    if (joint.range().isEmpty()) {
      return null;
    }
    Range range = joint.range().get();
    String regionShown =
        Places.shownAt(revolute, region.lower())
            + " to "
            + Places.shownAt(revolute, region.upper());
    String rangeShown =
        Places.shownAt(revolute, range.lower()) + " to " + Places.shownAt(revolute, range.upper());
    if (region.upper() <= range.lower() || region.lower() >= range.upper()) {
      return regionShown + " is outside " + possessive(joint.label()) + " range, " + rangeShown;
    }
    if (region.lower() <= range.lower() && region.upper() >= range.upper()) {
      return regionShown
          + " is all of "
          + possessive(joint.label())
          + " range, "
          + rangeShown
          + ": it would have nowhere to be";
    }
    return null;
  }
}
