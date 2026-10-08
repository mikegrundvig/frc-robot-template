package frc.robot.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.jspecify.annotations.Nullable;

/**
 * Each of a joint's limit switches checked against where the joint is, once it knows: tripped
 * nowhere near where it trips (a cut wire, if it's normally closed), or clear well past it (a cut
 * wire, if it's normally open), for a quarter second. Said, never acted on: a doubted switch still
 * stops the joint, the safe way to be wrong.
 */
final class SwitchDoubts {
  /** How long a limit switch must disagree with where the joint is before it's doubted. */
  static final long DOUBT_NANOS = 250_000_000L;

  private final Joint joint;
  private final List<Sensor.Switch> switches;

  /** What each switch is doubted for, in their order: empty while it agrees. */
  private final String[] doubts;

  /** Since when each has disagreed, in nanoseconds: -1 while it agrees. */
  private final long[] since;

  /**
   * @param joint the joint, which says where it is in its units
   * @param switches its limit switches
   */
  SwitchDoubts(Joint joint, List<Sensor.Switch> switches) {
    this.joint = joint;
    this.switches = List.copyOf(switches);
    doubts = new String[switches.size()];
    Arrays.fill(doubts, "");
    since = new long[switches.size()];
    Arrays.fill(since, -1);
  }

  /**
   * How far from where a limit switch trips its joint must be, past it or short of it, before the
   * switch's reading is doubted: a tenth of the joint's range, or 15° for one that turns freely. A
   * guess at what a switch bent a little, or flex in the mechanism, never reaches.
   */
  private double margin() {
    return joint
        .range()
        .map(range -> 0.1 * (range.upper() - range.lower()))
        .orElse(Math.toRadians(15));
  }

  /**
   * Checks each switch this loop.
   *
   * @param now this loop's time, in nanoseconds
   * @param knowsWhere whether the joint knows where it is, and its motor controllers answer
   * @param at where it is
   * @param sensors the robot's sensors, as read this loop
   */
  void check(long now, boolean knowsWhere, double at, @Nullable Sensors sensors) {
    String label = joint.label();
    for (int i = 0; i < switches.size(); i++) {
      Sensor.Switch limit = switches.get(i);
      Sensor.Switch.Limit kind = (Sensor.Switch.Limit) limit.kind();
      String doubt = null;
      if (sensors != null && kind.position().isPresent() && knowsWhere && sensors.reads(limit)) {
        double trips = kind.position().get();
        double toward = kind.end() == Sensor.Switch.Limit.End.LOWER ? -1 : 1;
        double past = (at - trips) * toward;
        boolean tripped = sensors.holds(limit.tripped());
        if (tripped && past < -margin()) {
          doubt =
              limit.label()
                  + " reads tripped with "
                  + label
                  + " at "
                  + joint.shown(at)
                  + ", nowhere near where it trips, "
                  + joint.shown(trips)
                  + ": a cut wire, if it's normally closed, or a switch out of place";
        } else if (!tripped && past > margin()) {
          doubt =
              limit.label()
                  + " reads clear with "
                  + label
                  + " at "
                  + joint.shown(at)
                  + ", past where it trips, "
                  + joint.shown(trips)
                  + ": a cut wire, if it's normally open, or a switch out of place";
        }
      }
      if (doubt == null) {
        since[i] = -1;
        doubts[i] = "";
      } else if (since[i] < 0) {
        since[i] = now;
      } else if (now - since[i] >= DOUBT_NANOS) {
        doubts[i] = doubt;
      }
    }
  }

  /** What each switch is doubted for, in their order, as logged: empty for one that agrees. */
  String[] logged() {
    return doubts;
  }

  /** What its switches are doubted for, as of this loop: none while they agree. */
  List<String> doubts() {
    List<String> doubted = new ArrayList<>();
    for (String doubt : doubts) {
      if (!doubt.isEmpty()) {
        doubted.add(doubt);
      }
    }
    return doubted;
  }
}
