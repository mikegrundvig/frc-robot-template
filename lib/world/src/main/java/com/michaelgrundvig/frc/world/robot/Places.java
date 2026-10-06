package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.shown;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.TreeSet;
import org.jspecify.annotations.Nullable;

/**
 * The definition's checks of its named places, across its parts: each joint's named outputs against
 * what its motors can do, and its configurations and interlocks against its joints and each other.
 * Everything here is judged statically, at named positions, with the joints a configuration doesn't
 * name at their references, as the robot is weighed and starts.
 */
final class Places {
  private Places() {}

  /** The share of its free speed past which a named speed is noted: its loop has little left. */
  static final double NOTED_SHARE = 0.9;

  /** How far either side of a condition's edge a position is tried, in the joint's units. */
  private static final double STEP = 1e-6;

  /** A position as people read it: a revolute joint's in degrees, a prismatic one's in meters. */
  static String shownAt(boolean revolute, double value) {
    return revolute ? shown(Math.toDegrees(value)) + "°" : shown(value) + " m";
  }

  /** A speed as people read it: a revolute joint's in rpm, a prismatic one's in m/s. */
  static String shownSpeed(boolean revolute, double value) {
    return revolute ? shown(value * 60 / (2 * Math.PI)) + " rpm" : shown(value) + " m/s";
  }

  /**
   * What's wrong with the joints' names: a joint that follows another naming places of its own,
   * which are its leader's to name; a speed faster than the joint's motors turn it; a voltage past
   * what they're rated at.
   */
  static void jointProblems(RobotMath math, List<String> problems) {
    for (Joint joint : math.joints()) {
      Joint leader = math.leaderOf(joint);
      if (leader != null) {
        if (!joint.positions().isEmpty() || !joint.outputs().isEmpty()) {
          problems.add(
              joint.label()
                  + " follows "
                  + leader.label()
                  + ": its places are "
                  + leader.label()
                  + "'s to name");
        }
        continue;
      }
      if (joint.outputs().isEmpty() || math.drivesOf(joint).isEmpty()) {
        continue;
      }
      double free = math.freeSpeed(joint);
      double nominal = nominalVolts(math, joint);
      for (Output output : joint.outputs()) {
        double size = Math.abs(output.value());
        if (output.kind() == Output.Kind.VELOCITY && size > free * (1 + 1e-9)) {
          problems.add(
              String.format(
                  Locale.ROOT,
                  "%s's %s, %s, is %s times its free speed, %s: its motors can't turn it that fast",
                  joint.label(),
                  output.name(),
                  shownSpeed(joint instanceof Joint.Revolute, output.value()),
                  shown(size / free),
                  shownSpeed(joint instanceof Joint.Revolute, free)));
        } else if (output.kind() == Output.Kind.VOLTAGE && size > nominal * (1 + 1e-9)) {
          problems.add(
              String.format(
                  Locale.ROOT,
                  "%s's %s, %s V, is past its motors' nominal %s V: the battery can't give it",
                  joint.label(),
                  output.name(),
                  shown(output.value()),
                  shown(nominal)));
        }
      }
    }
  }

  /**
   * What's doubtful in the joints' names: a speed so near its free speed its loop can't hold it.
   */
  static List<String> notes(RobotMath math) {
    List<String> notes = new ArrayList<>();
    for (Joint joint : math.joints()) {
      if (joint.outputs().isEmpty()
          || math.leaderOf(joint) != null
          || math.drivesOf(joint).isEmpty()) {
        continue;
      }
      double free = math.freeSpeed(joint);
      for (Output output : joint.outputs()) {
        double share = Math.abs(output.value()) / free;
        if (output.kind() == Output.Kind.VELOCITY && share > NOTED_SHARE && share <= 1 + 1e-9) {
          notes.add(
              String.format(
                  Locale.ROOT,
                  "%s's %s, %s, is %.0f%% of its free speed, %s: a velocity loop has little or"
                      + " nothing left to hold it under load",
                  joint.label(),
                  output.name(),
                  shownSpeed(joint instanceof Joint.Revolute, output.value()),
                  share * 100,
                  shownSpeed(joint instanceof Joint.Revolute, free)));
        }
      }
    }
    return notes;
  }

  /**
   * The joints that name speeds or voltages no motors turn them at, which nothing can check. A duty
   * cycle checks itself.
   */
  static List<Joint> unchecked(RobotMath math) {
    List<Joint> unchecked = new ArrayList<>();
    for (Joint joint : math.joints()) {
      boolean checkable = false;
      for (Output output : joint.outputs()) {
        checkable |= output.kind() != Output.Kind.DUTY_CYCLE;
      }
      if (checkable && math.leaderOf(joint) == null && math.drivesOf(joint).isEmpty()) {
        unchecked.add(joint);
      }
    }
    return unchecked;
  }

  /** The least nominal voltage of the motors that move a joint: what an output's volts may be. */
  private static double nominalVolts(RobotMath math, Joint joint) {
    double nominal = Double.POSITIVE_INFINITY;
    for (Drive drive : math.drivesOf(joint)) {
      nominal = Math.min(nominal, ((Actuator.Motors) drive.actuator()).model().nominalVoltage);
    }
    return nominal;
  }

  /**
   * What's wrong with the configurations: two named alike; a joint not the robot's, or one that
   * follows another; an interlock broken, with the joints a configuration doesn't name at their
   * references. Null when nothing is.
   */
  static @Nullable String configurationProblems(
      RobotMath math, List<Configuration> configurations, List<Interlock> interlocks) {
    List<String> problems = new ArrayList<>();
    unique("configurations", configurations.stream().map(Configuration::name).toList(), problems);
    List<Interlock> known = known(math, interlocks);
    for (Configuration configuration : configurations) {
      boolean whole = true;
      for (Joint joint : configuration.joints()) {
        String why = notOwn(math, joint, "configuration " + configuration.name());
        if (why != null) {
          problems.add(why);
          whole = false;
        }
      }
      if (!whole) {
        continue;
      }
      Map<Joint, Double> given = new LinkedHashMap<>();
      Set<Joint> running = new HashSet<>();
      for (Condition.At target : configuration.targets()) {
        if (target.isOutput()) {
          if (target.joint().output(target.name()).runs()) {
            running.add(target.joint());
          }
        } else {
          given.put(target.joint(), target.joint().position(target.name()).value());
        }
      }
      Map<Joint, Double> at = math.positions(given);
      for (Interlock interlock : known) {
        String broken = broken(interlock, at, running, given.keySet());
        if (broken != null) {
          problems.add("configuration " + configuration.name() + " " + broken);
        }
      }
    }
    return problems.isEmpty() ? null : String.join("; ", problems);
  }

  /**
   * What's wrong with the interlocks: a joint not the robot's, or one that follows another; joints
   * restricting each other, so none can move first; a joint whose conditions to run can't all hold;
   * a named position no positions of the other joints let a joint reach; an interlock broken as the
   * robot starts. Null when nothing is.
   */
  static @Nullable String interlockProblems(RobotMath math, List<Interlock> interlocks) {
    List<String> problems = new ArrayList<>();
    for (Interlock interlock : interlocks) {
      List<Joint> joints = new ArrayList<>(List.of(interlock.joint()));
      switch (interlock.condition()) {
        case Condition.OnJoint onJoint -> joints.add(onJoint.joint());
        case Condition.OnSensor onSensor -> {
          if (!math.sensors().contains(onSensor.sensor())) {
            problems.add(
                "interlock \""
                    + interlock
                    + "\"'s sensor, "
                    + onSensor.sensor().label()
                    + ", isn't one of the robot's sensors: list it");
          }
        }
      }
      for (Joint joint : joints) {
        String why = notOwn(math, joint, "interlock \"" + interlock + "\"");
        if (why != null) {
          problems.add(why);
        }
      }
    }
    if (!problems.isEmpty()) {
      return String.join("; ", problems);
    }
    // A sensor's reading isn't known until the robot runs: only joints' positions are checked.
    List<Interlock> onJoints = onJoints(interlocks);
    rings(onJoints, problems);
    stuck(onJoints, problems);
    unreachable(math, onJoints, problems);
    Map<Joint, Double> start = math.positions(Map.of());
    for (Interlock interlock : onJoints) {
      String broken = broken(interlock, start, Set.of(), Set.of());
      if (broken != null) {
        problems.add("as the robot starts, it " + broken);
      }
    }
    return problems.isEmpty() ? null : String.join("; ", problems);
  }

  /** The interlocks whose joints are all the robot's own: the others are reported as interlocks. */
  private static List<Interlock> known(RobotMath math, List<Interlock> interlocks) {
    return onJoints(interlocks).stream()
        .filter(interlock -> notOwn(math, interlock.joint(), "") == null)
        .filter(interlock -> notOwn(math, on(interlock).joint(), "") == null)
        .toList();
  }

  /** The interlocks on another joint's position: what can be checked before the robot runs. */
  private static List<Interlock> onJoints(List<Interlock> interlocks) {
    return interlocks.stream()
        .filter(interlock -> interlock.condition() instanceof Condition.OnJoint)
        .toList();
  }

  /** An interlock's condition on another joint's position: only for one of {@link #onJoints}. */
  private static Condition.OnJoint on(Interlock interlock) {
    return (Condition.OnJoint) interlock.condition();
  }

  /**
   * How an interlock is broken with the joints at positions and some running, or null when it
   * isn't. A joint the configuration names is at its named place; the rest are at their references.
   *
   * @param named the joints whose positions were given, rather than their references
   */
  private static @Nullable String broken(
      Interlock interlock, Map<Joint, Double> at, Set<Joint> running, Set<Joint> named) {
    Condition.OnJoint condition = on(interlock);
    Joint other = condition.joint();
    double there = at.getOrDefault(other, other.reference());
    boolean holds = condition.holdsAt(there);
    String where =
        other.label()
            + " is at "
            + shownAt(other instanceof Joint.Revolute, there)
            + (named.contains(other) ? "" : ", its reference");
    return switch (interlock) {
      case Interlock.RunsOnlyWhen only ->
          running.contains(only.joint()) && !holds
              ? "runs " + only.joint().label() + ", but " + interlock + ", and " + where
              : null;
      case Interlock.KeepsOut out -> {
        double position = at.getOrDefault(out.joint(), out.joint().reference());
        yield holds && out.keepsOut(position)
            ? "puts "
                + out.joint().label()
                + " at "
                + shownAt(out.joint() instanceof Joint.Revolute, position)
                + ", but "
                + interlock
                + ", and "
                + where
            : null;
      }
    };
  }

  /** Joints that restrict each other in a ring, which none can move first to break. */
  private static void rings(List<Interlock> interlocks, List<String> problems) {
    Map<Joint, List<Interlock>> on = new LinkedHashMap<>();
    for (Interlock interlock : interlocks) {
      on.computeIfAbsent(interlock.joint(), joint -> new ArrayList<>()).add(interlock);
    }
    Set<Set<Joint>> reported = new HashSet<>();
    for (Joint start : on.keySet()) {
      List<Interlock> path = new ArrayList<>();
      ring(start, start, on, path, new HashSet<>(), reported, problems);
    }
  }

  private static void ring(
      Joint start,
      Joint at,
      Map<Joint, List<Interlock>> on,
      List<Interlock> path,
      Set<Joint> seen,
      Set<Set<Joint>> reported,
      List<String> problems) {
    for (Interlock interlock : on.getOrDefault(at, List.of())) {
      Joint next = on(interlock).joint();
      path.add(interlock);
      if (next.equals(start)) {
        Set<Joint> members = new HashSet<>();
        path.forEach(each -> members.add(each.joint()));
        if (reported.add(members)) {
          problems.add(
              String.join(" and ", path.stream().map(each -> each.joint().label()).toList())
                  + " hold each other by their interlocks, so none can move first: a deadlock ("
                  + String.join("; ", path.stream().map(Interlock::toString).toList())
                  + ")");
        }
      } else if (seen.add(next)) {
        ring(start, next, on, path, seen, reported, problems);
      }
      path.remove(path.size() - 1);
    }
  }

  /** A joint that runs only when conditions on another hold that can't all hold at once. */
  private static void stuck(List<Interlock> interlocks, List<String> problems) {
    Map<Joint, Map<Joint, List<Condition.OnJoint>>> needs = new LinkedHashMap<>();
    for (Interlock interlock : interlocks) {
      if (interlock instanceof Interlock.RunsOnlyWhen only) {
        needs
            .computeIfAbsent(only.joint(), joint -> new LinkedHashMap<>())
            .computeIfAbsent(on(only).joint(), joint -> new ArrayList<>())
            .add(on(only));
      }
    }
    needs.forEach(
        (joint, byOther) ->
            byOther.forEach(
                (other, conditions) -> {
                  if (conditions.size() > 1 && !somewhere(other, conditions, true)) {
                    problems.add(
                        joint.label()
                            + " can never run: no position of "
                            + other.label()
                            + " has "
                            + String.join(
                                " and ", conditions.stream().map(Object::toString).toList()));
                  }
                }));
  }

  /**
   * A joint's named position inside regions it keeps out of, whose conditions no positions of the
   * other joints all clear at once.
   */
  private static void unreachable(
      RobotMath math, List<Interlock> interlocks, List<String> problems) {
    for (Joint joint : math.joints()) {
      for (Position position : joint.positions()) {
        Map<Joint, List<Interlock.KeepsOut>> closing = new LinkedHashMap<>();
        for (Interlock interlock : interlocks) {
          if (interlock instanceof Interlock.KeepsOut out
              && out.joint().equals(joint)
              && out.keepsOut(position.value())) {
            closing.computeIfAbsent(on(out).joint(), j -> new ArrayList<>()).add(out);
          }
        }
        closing.forEach(
            (other, outs) -> {
              List<Condition.OnJoint> conditions = outs.stream().map(Places::on).toList();
              if (!somewhere(other, conditions, false)) {
                problems.add(
                    joint.label()
                        + " can never reach "
                        + position.name()
                        + ", "
                        + shownAt(joint instanceof Joint.Revolute, position.value())
                        + ": no position of "
                        + other.label()
                        + " opens it ("
                        + String.join("; ", outs.stream().map(Interlock::toString).toList())
                        + ")");
              }
            });
      }
    }
  }

  /**
   * Whether a joint has a position, in its range, where every condition holds, or where none does.
   * The positions tried are its range's ends (or well beyond the conditions' edges, for a joint
   * that turns freely), each condition's edges and just either side of them, and between them.
   */
  private static boolean somewhere(Joint joint, List<Condition.OnJoint> conditions, boolean all) {
    TreeSet<Double> edges = new TreeSet<>();
    for (Condition.OnJoint condition : conditions) {
      edges.addAll(edges(condition));
    }
    TreeSet<Double> tried = new TreeSet<>();
    if (joint.range().isPresent()) {
      tried.add(joint.range().get().lower());
      tried.add(joint.range().get().upper());
    } else {
      tried.add(edges.first() - 1);
      tried.add(edges.last() + 1);
    }
    Double before = null;
    for (double edge : edges) {
      tried.add(edge);
      tried.add(edge - STEP);
      tried.add(edge + STEP);
      if (before != null) {
        tried.add((before + edge) / 2);
      }
      before = edge;
    }
    for (double position : tried) {
      if (joint.range().isPresent() && !joint.range().get().contains(position)) {
        continue;
      }
      int holding = 0;
      for (Condition.OnJoint condition : conditions) {
        holding += condition.holdsAt(position) ? 1 : 0;
      }
      if (all ? holding == conditions.size() : holding == 0) {
        return true;
      }
    }
    return false;
  }

  /** The positions where a condition begins or ends holding. */
  private static List<Double> edges(Condition.OnJoint condition) {
    Joint joint = condition.joint();
    return switch (condition) {
      case Condition.At at -> List.of(joint.position(at.name()).value());
      case Condition.Past past -> List.of(joint.position(past.position()).value());
      case Condition.Before before -> List.of(joint.position(before.position()).value());
      case Condition.Between between ->
          List.of(joint.position(between.from()).value(), joint.position(between.to()).value());
    };
  }

  /**
   * Why a joint can't be named by a configuration, group or interlock, or null when it can: one of
   * the robot's joints, moving on its own.
   */
  private static @Nullable String notOwn(RobotMath math, Joint joint, String what) {
    if (!math.has(joint)) {
      return what + "'s joint, " + joint.label() + ", isn't one of the robot's joints: list it";
    }
    Joint leader = math.leaderOf(joint);
    return leader == null
        ? null
        : what
            + " names "
            + joint.label()
            + ", which follows "
            + leader.label()
            + ": name "
            + leader.label()
            + " instead";
  }

  private static void unique(String what, List<String> names, List<String> problems) {
    Set<String> seen = new HashSet<>();
    Set<String> twice = new LinkedHashSet<>();
    for (String name : names) {
      if (!seen.add(name)) {
        twice.add(name);
      }
    }
    for (String name : twice) {
      problems.add("two " + what + " are named " + name);
    }
  }
}
