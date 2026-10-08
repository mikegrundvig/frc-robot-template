package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import org.jspecify.annotations.Nullable;

/**
 * The definition's checks of its named places, across its parts: each joint's named outputs against
 * what its motors can do, and its configurations and interlocks against its joints and each other.
 * Everything here is judged statically, at named positions, with the joints a configuration doesn't
 * name where they rest, as the robot is weighed and starts.
 */
final class Places {
  private Places() {}

  /** The share of its free speed past which a named speed is noted: its loop has little left. */
  static final double NOTED_SHARE = 0.9;

  /** How far either side of a condition's edge a position is tried, in the joint's units. */
  private static final double STEP = 1e-6;

  /**
   * A position on a joint, or a distance along it, as every message shows one ({@link
   * Joint#shown}): a revolute joint's in degrees, a prismatic one's in meters, to four significant
   * figures and no finer than a hundredth of a degree or a tenth of a millimeter, so a position
   * measured on the robot reads as where it is, not its noise.
   */
  static String shownAt(boolean revolute, double value) {
    double resolution = revolute ? 0.01 : 0.0001;
    double rounded = Math.round((revolute ? Math.toDegrees(value) : value) / resolution);
    // A whole number of steps, scaled back: 0 rather than -0 for a hair below zero.
    double shown = rounded == 0 ? 0 : rounded * resolution;
    return revolute ? shown(shown) + "°" : shown(shown) + " m";
  }

  /** A speed as people read it: a revolute joint's in rpm, a prismatic one's in m/s. */
  static String shownSpeed(boolean revolute, double value) {
    return revolute ? shown(value * 60 / (2 * Math.PI)) + " rpm" : shown(value) + " m/s";
  }

  /**
   * What an output runs a joint at, as people read it: "1000 rpm", "-4 V", "60% of the battery".
   */
  static String shownOutput(boolean revolute, Output output) {
    return switch (output.kind()) {
      case VELOCITY -> shownSpeed(revolute, output.value());
      case VOLTAGE -> shown(output.value()) + " V";
      case DUTY_CYCLE -> shown(output.value() * 100) + "% of the battery";
    };
  }

  /**
   * Refuses what a condition or an interlock says, as it's written: each reason in plain words,
   * without a record's path a team never wrote.
   *
   * @param whys each reason it can't be, or null for none
   * @throws IllegalArgumentException with every reason given, one a line
   */
  static void refuse(@Nullable String... whys) {
    List<String> refused = new ArrayList<>();
    for (String why : whys) {
      if (why != null) {
        refused.add(why);
      }
    }
    if (!refused.isEmpty()) {
      throw new IllegalArgumentException(String.join("\n", refused));
    }
  }

  /**
   * What's wrong with the joints' names and speeds: a joint that follows another naming places of
   * its own, which are its leader's to name; a named speed, or its profile's cruise, faster than
   * the joint's motors turn it; a voltage past what they're rated at.
   */
  static void jointProblems(RobotMath math, List<Finding> problems) {
    for (Joint joint : math.joints()) {
      Joint leader = math.leaderOf(joint);
      if (leader != null) {
        if (!joint.positions().isEmpty() || !joint.outputs().isEmpty()) {
          problems.add(
              refused(
                  joint,
                  joint.label()
                      + " follows "
                      + leader.label()
                      + ", whose moves carry it: give its places to "
                      + leader.label()
                      + ", and none to "
                      + joint.label()));
        }
        continue;
      }
      if (math.drivesOf(joint).isEmpty()) {
        continue;
      }
      double free = math.freeSpeed(joint);
      joint
          .driven()
          .control()
          .flatMap(MechanismControl::profile)
          .filter(profile -> profile.cruiseVelocity() > free * (1 + 1e-9))
          .ifPresent(profile -> problems.add(refused(joint, tooFast(joint, profile, free))));
      joint
          .driven()
          .control()
          .flatMap(MechanismControl::bands)
          .map(bands -> bandsOfTheOtherLoop(joint, bands))
          .ifPresent(why -> problems.add(refused(joint, why)));
      double nominal = nominalVolts(math, joint);
      for (Output output : joint.outputs()) {
        double size = Math.abs(output.value());
        if (output.kind() == Output.Kind.VELOCITY && size > free * (1 + 1e-9)) {
          problems.add(
              refused(
                  joint,
                  String.format(
                      Locale.ROOT,
                      "%s %s, %s, is %s times its free speed, %s: its motors can't turn it that"
                          + " fast; give it a speed under that",
                      possessive(joint.label()),
                      output.name(),
                      shownSpeed(joint instanceof Joint.Revolute, output.value()),
                      shown(size / free),
                      shownSpeed(joint instanceof Joint.Revolute, free))));
        } else if (output.kind() == Output.Kind.VOLTAGE && size > nominal * (1 + 1e-9)) {
          problems.add(
              refused(
                  joint,
                  String.format(
                      Locale.ROOT,
                      "%s's %s, %s V, is past its motors' nominal %s V: the battery can't give it",
                      joint.label(),
                      output.name(),
                      shown(output.value()),
                      shown(nominal))));
        }
      }
    }
  }

  /**
   * Why a profile's cruise is past its joint's free speed, in the joint's own terms: degrees a
   * second for a joint that turns to places, as its profile is given; rpm for one that only runs, a
   * roller or a flywheel, as its outputs are; m/s for one that slides.
   */
  private static String tooFast(Joint joint, MechanismControl.Profile profile, double free) {
    boolean revolute = joint instanceof Joint.Revolute;
    boolean runs = !joint.holdsPositions();
    return String.format(
        Locale.ROOT,
        "%s's profile cruises at %s, %s times its free speed, %s: its motors can't turn it that"
            + " fast, so its moves would lag their profile; give it a cruise under its free speed",
        joint.label(),
        runs
            ? shownSpeed(revolute, profile.cruiseVelocity())
            : shownProfile(revolute, profile.cruiseVelocity()),
        shown(profile.cruiseVelocity() / free),
        runs ? shownSpeed(revolute, free) : shownProfile(revolute, free));
  }

  /**
   * Why a joint's bands aren't what its loop holds, or null when they are: a roller's are shares of
   * the speed it's asked for, and a joint held at positions has them in angles or lengths.
   */
  private static @Nullable String bandsOfTheOtherLoop(Joint joint, MechanismControl.Bands bands) {
    boolean revolute = joint instanceof Joint.Revolute;
    if (bands.kind().isPresent() && !joint.holdsPositions()) {
      return joint.label()
          + " only runs at outputs, so its bands are shares of the speed it's asked for, not "
          + (bands.kind().get() == Position.Kind.ANGLE ? "angles" : "lengths")
          + ": withBands(Percent.of(3), Percent.of(6)) is 3% to arrive and 6% to stay";
    }
    if (bands.kind().isEmpty() && joint.holdsPositions()) {
      return joint.label()
          + " is held at positions, so its bands are "
          + (revolute ? "angles" : "lengths")
          + ", not shares of a speed: "
          + (revolute
              ? "withBands(Degrees.of(2), Degrees.of(4))"
              : "withBands(Inches.of(0.5), Inches.of(1))")
          + " is "
          + (revolute ? "2° to arrive and 4°" : "half an inch to arrive and an inch")
          + " to stay";
    }
    return null;
  }

  /** A profile's speed as a profile is given: in degrees a second, or meters. */
  private static String shownProfile(boolean revolute, double speed) {
    return revolute ? shown(Math.toDegrees(speed)) + "°/s" : shown(speed) + " m/s";
  }

  /**
   * What's doubtful in the joints' names: a speed so near its free speed its loop can't hold it.
   */
  static List<Finding> doubts(RobotMath math) {
    List<Finding> notes = new ArrayList<>();
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
              Finding.doubt(
                  joint.label(),
                  String.format(
                      Locale.ROOT,
                      "%s's %s, %s, is %.0f%% of its free speed, %s: a velocity loop has little or"
                          + " nothing left to hold it under load",
                      joint.label(),
                      output.name(),
                      shownSpeed(joint instanceof Joint.Revolute, output.value()),
                      share * 100,
                      shownSpeed(joint instanceof Joint.Revolute, free))));
        }
      }
    }
    return notes;
  }

  /**
   * What's doubtful as the robot starts: a joint resting inside a region it's kept out of, an
   * elevator at its bottom stop inside "keep out of the lowest 10 cm while the intake is in". The
   * robot lets it only leave, by the side it's kept to, until the condition clears; a configuration
   * may still not put it there.
   */
  static List<Finding> startDoubts(RobotMath math, List<Interlock> interlocks) {
    List<Finding> doubts = new ArrayList<>();
    Map<Joint, Double> start = math.positions(Map.of());
    for (Interlock interlock : known(math, interlocks)) {
      String broken = broken(interlock, start, Set.of(), Set.of());
      if (broken != null && interlock instanceof Interlock.KeepsOut out) {
        Joint joint = out.joint();
        double here = start.getOrDefault(joint, joint.restPosition());
        double edge = out.keptBelow(here, 0) ? out.region().lower() : out.region().upper();
        doubts.add(
            Finding.doubt(
                joint.label(),
                "as the robot starts, it "
                    + broken
                    + ": "
                    + joint.label()
                    + " may only leave it, by its edge at "
                    + shownAt(joint instanceof Joint.Revolute, edge)
                    + ", until that changes"));
      }
    }
    return doubts;
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
   * follows another; an interlock broken, or joints it names holding each other in a deadlock
   * ({@link #deadlock}), with the joints a configuration doesn't name where they rest. Each is
   * about its configuration.
   */
  static List<Finding> configurationProblems(
      RobotMath math, List<Configuration> configurations, List<Interlock> interlocks) {
    List<Finding> problems = new ArrayList<>();
    unique(configurations, problems);
    List<Interlock> known = known(math, interlocks);
    List<List<Interlock>> rings =
        rings(
            interlocks.stream()
                .filter(interlock -> about(interlock) != null)
                .filter(interlock -> notOwn(math, interlock.joint(), "") == null)
                .filter(
                    interlock -> notOwn(math, Objects.requireNonNull(about(interlock)), "") == null)
                .filter(interlock -> notOwn(interlock.condition()) == null)
                .toList());
    for (Configuration configuration : configurations) {
      boolean whole = true;
      for (Joint joint : configuration.joints()) {
        String why = notOwn(math, joint, "configuration " + configuration.name());
        if (why != null) {
          problems.add(Finding.refused(configuration.name(), why));
          whole = false;
        }
      }
      for (Condition.At target : configuration.targets()) {
        String why = notOwn(target);
        if (why != null) {
          problems.add(
              Finding.refused(
                  configuration.name(), "configuration " + configuration.name() + ": " + why));
          whole = false;
        }
      }
      if (!whole) {
        continue;
      }
      Map<Joint, Double> given = new LinkedHashMap<>();
      Set<Joint> running = new HashSet<>();
      Map<Joint, Double> speeds = new HashMap<>();
      for (Condition.At target : configuration.targets()) {
        if (target.isOutput()) {
          Output output = target.joint().output(target.name());
          if (output.runs()) {
            running.add(target.joint());
            speeds.put(
                target.joint(),
                output.kind() == Output.Kind.VELOCITY ? output.value() : Double.NaN);
          }
        } else {
          given.put(target.joint(), target.joint().position(target.name()).value());
        }
      }
      Map<Joint, Double> at = math.positions(given);
      for (Interlock interlock : known) {
        String broken = broken(interlock, at, running, given.keySet());
        if (broken != null) {
          problems.add(
              Finding.refused(
                  configuration.name(), "configuration " + configuration.name() + " " + broken));
        }
      }
      // A ring whose joints are all where they rest, and still, is judged as the robot starts.
      Set<Set<Joint>> reported = new HashSet<>();
      for (List<Interlock> ring : rings) {
        if (ring.stream()
            .anyMatch(
                interlock ->
                    given.containsKey(interlock.joint()) || running.contains(interlock.joint()))) {
          String deadlock = deadlock(ring, at, speeds);
          if (deadlock != null && reported.add(members(ring))) {
            problems.add(
                Finding.refused(
                    configuration.name(),
                    "configuration " + configuration.name() + " " + deadlock));
          }
        }
      }
    }
    return problems;
  }

  /**
   * What's wrong with the interlocks: a joint not the robot's, or one that follows another; a joint
   * whose conditions to run can't all hold; a named position no positions of the other joints let a
   * joint reach; joints holding each other in a deadlock as the robot starts ({@link #deadlock}).
   * Each is about the joint restricted.
   */
  static List<Finding> interlockProblems(RobotMath math, List<Interlock> interlocks) {
    List<Finding> problems = new ArrayList<>();
    for (Interlock interlock : interlocks) {
      List<Joint> joints = new ArrayList<>(List.of(interlock.joint()));
      switch (interlock.condition()) {
        case Condition.OnJoint onJoint -> joints.add(onJoint.joint());
        case Condition.UpToSpeed upToSpeed -> joints.add(upToSpeed.joint());
        case Condition.OnSensor onSensor -> {
          if (!math.sensors().contains(onSensor.sensor())) {
            problems.add(
                refused(
                    interlock.joint(),
                    "interlock \""
                        + interlock
                        + "\"'s sensor, "
                        + onSensor.sensor().label()
                        + ", isn't one of the robot's sensors: list it"));
          }
        }
      }
      for (Joint joint : joints) {
        String why = notOwn(math, joint, "interlock \"" + interlock + "\"");
        if (why != null) {
          problems.add(refused(interlock.joint(), why));
        }
      }
      String place = notOwn(interlock.condition());
      if (place != null) {
        problems.add(refused(interlock.joint(), "interlock \"" + interlock + "\": " + place));
      }
    }
    if (!problems.isEmpty()) {
      return problems;
    }
    // A sensor's reading isn't known until the robot runs, nor how fast a joint runs, but for one
    // still as the robot starts: beyond joints holding each other there, only joints' positions are
    // checked.
    List<Interlock> onJoints = onJoints(interlocks);
    stuck(onJoints, problems);
    unreachable(math, onJoints, problems);
    // A joint resting inside a region it's kept out of may leave it ({@link #startDoubts}): only
    // joints holding each other there are refused.
    Map<Joint, Double> start = math.positions(Map.of());
    Set<Set<Joint>> reported = new HashSet<>();
    for (List<Interlock> ring :
        rings(interlocks.stream().filter(interlock -> about(interlock) != null).toList())) {
      String deadlock = deadlock(ring, start, Map.of());
      if (deadlock != null && reported.add(members(ring))) {
        problems.add(refused(ring.get(0).joint(), "as the robot starts, it " + deadlock));
      }
    }
    return problems;
  }

  /**
   * The interlocks whose joints and places are all the robot's own: the others are reported as
   * interlocks.
   */
  private static List<Interlock> known(RobotMath math, List<Interlock> interlocks) {
    return onJoints(interlocks).stream()
        .filter(interlock -> notOwn(math, interlock.joint(), "") == null)
        .filter(interlock -> notOwn(math, on(interlock).joint(), "") == null)
        .filter(interlock -> notOwn(interlock.condition()) == null)
        .toList();
  }

  /**
   * Why a place a condition names isn't its joint's own, or null when each is ({@link
   * Joint#whyNotOwn(Position)}): another joint's, or another robot's.
   */
  static @Nullable String notOwn(Condition condition) {
    return switch (condition) {
      case Condition.At at ->
          at.position().isPresent()
              ? at.joint().whyNotOwn(at.position().get())
              : at.joint().whyNotOwn(at.output().orElseThrow());
      case Condition.AtLeast atLeast -> notOwn(atLeast.joint(), atLeast.place());
      case Condition.AtMost atMost -> notOwn(atMost.joint(), atMost.place());
      case Condition.Between between -> {
        String from = notOwn(between.joint(), between.fromPlace());
        yield from != null ? from : notOwn(between.joint(), between.toPlace());
      }
      case Condition.UpToSpeed upToSpeed -> null;
      case Condition.OnSensor onSensor -> null;
    };
  }

  private static @Nullable String notOwn(Joint joint, Optional<Position> place) {
    return place.isPresent() ? joint.whyNotOwn(place.get()) : null;
  }

  /** The interlocks on another joint's position: what can be checked before the robot runs. */
  private static List<Interlock> onJoints(List<Interlock> interlocks) {
    return interlocks.stream()
        .filter(interlock -> interlock.condition() instanceof Condition.OnJoint)
        .toList();
  }

  /**
   * The joint an interlock's condition is about, where it is or how fast, or null for a sensor's.
   */
  private static @Nullable Joint about(Interlock interlock) {
    return interlock.other().orElse(null);
  }

  /** An interlock's condition on another joint's position: only for one of {@link #onJoints}. */
  private static Condition.OnJoint on(Interlock interlock) {
    return (Condition.OnJoint) interlock.condition();
  }

  /**
   * How an interlock is broken with the joints at positions and some running, or null when it
   * isn't. A joint the configuration names is at its named place; the rest are where they rest.
   *
   * @param named the joints whose positions were given, rather than where they rest
   */
  private static @Nullable String broken(
      Interlock interlock, Map<Joint, Double> at, Set<Joint> running, Set<Joint> named) {
    Condition.OnJoint condition = on(interlock);
    Joint other = condition.joint();
    double there = at.getOrDefault(other, other.restPosition());
    boolean holds = condition.holdsAt(there);
    String where =
        other.label()
            + " is at "
            + shownAt(other instanceof Joint.Revolute, there)
            + (named.contains(other) ? "" : ", where it rests");
    return switch (interlock) {
      case Interlock.RunsOnlyWhen only ->
          running.contains(only.joint()) && !holds
              ? "runs " + only.joint().label() + ", but " + interlock + ", and " + where
              : null;
      case Interlock.KeepsOut out -> {
        double position = at.getOrDefault(out.joint(), out.joint().restPosition());
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

  /**
   * Every ring of interlocks: each restricting a joint by where the next is, or how fast it runs,
   * and the last by the first. Each ring once, from the joint of its own that's restricted first in
   * the list; two interlocks between the same joints make two rings.
   */
  private static List<List<Interlock>> rings(List<Interlock> interlocks) {
    Map<Joint, List<Interlock>> on = new LinkedHashMap<>();
    for (Interlock interlock : interlocks) {
      on.computeIfAbsent(interlock.joint(), joint -> new ArrayList<>()).add(interlock);
    }
    List<Joint> order = new ArrayList<>(on.keySet());
    List<List<Interlock>> rings = new ArrayList<>();
    for (Joint start : order) {
      ring(start, start, on, order, new ArrayList<>(), new HashSet<>(), rings);
    }
    return rings;
  }

  private static void ring(
      Joint start,
      Joint at,
      Map<Joint, List<Interlock>> on,
      List<Joint> order,
      List<Interlock> path,
      Set<Joint> onPath,
      List<List<Interlock>> rings) {
    onPath.add(at);
    for (Interlock interlock : on.getOrDefault(at, List.of())) {
      Joint next = Objects.requireNonNull(about(interlock));
      path.add(interlock);
      if (next.equals(start)) {
        rings.add(List.copyOf(path));
      } else if (!onPath.contains(next) && order.indexOf(next) > order.indexOf(start)) {
        ring(start, next, on, order, path, onPath, rings);
      }
      path.remove(path.size() - 1);
    }
    onPath.remove(at);
  }

  /** The joints a ring restricts. */
  private static Set<Joint> members(List<Interlock> ring) {
    Set<Joint> members = new HashSet<>();
    ring.forEach(interlock -> members.add(interlock.joint()));
    return members;
  }

  /**
   * How a ring of interlocks deadlocks its joints at a state, or null when it doesn't. A ring is
   * refused only where it really deadlocks: where every one of its joints is held by the next, and
   * nowhere it may still go frees the joint before it, so none can ever move to free another.
   *
   * <ul>
   *   <li>A joint that runs only when its condition holds is held while it doesn't: it stays where
   *       it is, and still.
   *   <li>A joint kept out of a region is held while its condition holds: it may still go anywhere
   *       on its side of the region, from its range's end to the region's edge (inside it, toward
   *       the side it's nearer, as it would leave).
   *   <li>It frees the joint before it in the ring if that joint's interlock opens anywhere it may
   *       go: by position, judged at the condition's edges and either side of them; by speed,
   *       unless it's held still, since one kept to its side may still run.
   * </ul>
   *
   * <p>So an arm kept out of the bumper's band while its elevator is low, and the elevator kept up
   * while the arm hangs below the band, are no deadlock: kept below the band, the arm can still
   * rise to its edge, out of "below", and let the elevator go. The elevator kept low while the arm
   * is above the band is one: with the arm up and the elevator down, each is held to the side that
   * holds the other.
   *
   * <p>It's judged where the robot is put: as it starts, and at each configuration. Joints can get
   * into a deadlock elsewhere only by moving into it together, each the loop the other's interlock
   * closes, so nothing else is refused; nor are two named positions no configuration puts together,
   * since a pair of interlocks like the last exists to keep the robot out of them.
   *
   * @param at where every joint is
   * @param speeds how fast the joints that run do, in their units a second: NaN for a voltage or a
   *     duty cycle, which isn't known until the robot runs; a joint not here is still
   */
  private static @Nullable String deadlock(
      List<Interlock> ring, Map<Joint, Double> at, Map<Joint, Double> speeds) {
    int size = ring.size();
    List<String> held = new ArrayList<>();
    for (int i = 0; i < size; i++) {
      Interlock interlock = ring.get(i);
      Interlock reading = ring.get((i + size - 1) % size);
      Joint joint = interlock.joint();
      Boolean holds = holds(interlock.condition(), at, speeds);
      if (holds == null || (interlock instanceof Interlock.KeepsOut) != holds) {
        return null;
      }
      boolean revolute = joint instanceof Joint.Revolute;
      double here = at.getOrDefault(joint, joint.restPosition());
      double from = here;
      double to = here;
      String stays;
      if (interlock instanceof Interlock.KeepsOut out) {
        double lowest = joint.range().map(Range::lower).orElse(Double.NEGATIVE_INFINITY);
        double highest = joint.range().map(Range::upper).orElse(Double.POSITIVE_INFINITY);
        Range region = out.region();
        // Inside, it may leave by the side it's kept to, as on the robot.
        if (out.keptBelow(here, 0)) {
          from = lowest;
          to = Math.max(here, region.lower());
        } else {
          from = Math.min(here, region.upper());
          to = highest;
        }
        stays = "stays " + shownSpan(revolute, from, to);
      } else {
        stays =
            reading.condition() instanceof Condition.UpToSpeed
                ? "stays still"
                : "stays at " + shownAt(revolute, here);
      }
      if (frees(reading, interlock, from, to)) {
        return null;
      }
      held.add(interlock + ", so " + joint.label() + " " + stays);
    }
    List<String> labels = ring.stream().map(interlock -> interlock.joint().label()).toList();
    String joints =
        labels.size() == 2
            ? labels.get(0) + " and " + labels.get(1)
            : String.join(", ", labels.subList(0, labels.size() - 1))
                + " and "
                + labels.get(labels.size() - 1);
    return "leaves "
        + joints
        + " holding each other by their interlocks, so "
        + (size == 2 ? "neither can free the other" : "none can free another")
        + ": a deadlock ("
        + String.join("; ", held)
        + ")";
  }

  /**
   * Whether a condition on a joint holds at a state, or null when the state doesn't say: a speed
   * the joint runs at by a voltage, or a sensor's reading.
   */
  private static @Nullable Boolean holds(
      Condition condition, Map<Joint, Double> at, Map<Joint, Double> speeds) {
    return switch (condition) {
      case Condition.OnJoint onJoint ->
          onJoint.holdsAt(at.getOrDefault(onJoint.joint(), onJoint.joint().restPosition()));
      case Condition.UpToSpeed upToSpeed -> {
        double speed = speeds.getOrDefault(upToSpeed.joint(), 0.0);
        yield Double.isNaN(speed) ? null : upToSpeed.holdsAt(speed);
      }
      case Condition.OnSensor onSensor -> null;
    };
  }

  /**
   * Whether a joint held by its interlock frees the joint whose interlock reads it, from somewhere
   * between two positions it may still go to: anywhere a position condition opens that interlock,
   * or, for a speed, unless it's held still.
   */
  private static boolean frees(Interlock reading, Interlock holding, double from, double to) {
    boolean opensWhenHolding = reading instanceof Interlock.RunsOnlyWhen;
    if (!(reading.condition() instanceof Condition.OnJoint condition)) {
      // A speed: one held still can't reach it, and needn't drop below one.
      return !(holding instanceof Interlock.RunsOnlyWhen && opensWhenHolding);
    }
    TreeSet<Double> tried = new TreeSet<>();
    for (double edge : edges(condition)) {
      for (double near : new double[] {edge - STEP, edge, edge + STEP}) {
        if (near >= from && near <= to) {
          tried.add(near);
        }
      }
    }
    if (Double.isFinite(from)) {
      tried.add(from);
    } else {
      tried.add(tried.isEmpty() ? to - 1 : Math.min(tried.first(), to) - 1);
    }
    if (Double.isFinite(to)) {
      tried.add(to);
    } else {
      tried.add(Math.max(tried.last(), from) + 1);
    }
    Double before = null;
    for (double position : List.copyOf(tried)) {
      if (before != null) {
        tried.add((before + position) / 2);
      }
      before = position;
    }
    for (double position : tried) {
      if (condition.holdsAt(position) == opensWhenHolding) {
        return true;
      }
    }
    return false;
  }

  /** A stretch of a joint's travel as people read it: "-30° to 40°", or "0.3 m and up". */
  private static String shownSpan(boolean revolute, double from, double to) {
    if (from == to) {
      return "at " + shownAt(revolute, from);
    }
    if (!Double.isFinite(from)) {
      return "at " + shownAt(revolute, to) + " or below";
    }
    if (!Double.isFinite(to)) {
      return "at " + shownAt(revolute, from) + " or above";
    }
    return "between " + shownAt(revolute, from) + " and " + shownAt(revolute, to);
  }

  /** A joint that runs only when conditions on another hold that can't all hold at once. */
  private static void stuck(List<Interlock> interlocks, List<Finding> problems) {
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
                        refused(
                            joint,
                            joint.label()
                                + " can never run: no position of "
                                + other.label()
                                + " has "
                                + String.join(
                                    " and ", conditions.stream().map(Object::toString).toList())));
                  }
                }));
  }

  /**
   * A joint's named position inside regions it keeps out of, whose conditions no positions of the
   * other joints all clear at once.
   */
  private static void unreachable(
      RobotMath math, List<Interlock> interlocks, List<Finding> problems) {
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
                    refused(
                        joint,
                        joint.label()
                            + " can never reach "
                            + position.name()
                            + ", "
                            + shownAt(joint instanceof Joint.Revolute, position.value())
                            + ": no position of "
                            + other.label()
                            + " opens it ("
                            + String.join("; ", outs.stream().map(Interlock::toString).toList())
                            + ")"));
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
      case Condition.AtLeast atLeast -> List.of(atLeast.position());
      case Condition.AtMost atMost -> List.of(atMost.position());
      case Condition.Between between -> List.of(between.from(), between.to());
    };
  }

  /**
   * Why a joint can't be named by a configuration, group or interlock, or null when it can: one of
   * the robot's joints, moving on its own.
   */
  private static @Nullable String notOwn(RobotMath math, Joint joint, String what) {
    if (!math.has(joint)) {
      return possessive(what)
          + " joint, "
          + joint.label()
          + ", isn't one of the robot's joints: list it";
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

  /** The configurations' names used twice, each once. */
  private static void unique(List<Configuration> configurations, List<Finding> problems) {
    Set<String> seen = new HashSet<>();
    Set<String> twice = new LinkedHashSet<>();
    for (Configuration configuration : configurations) {
      if (!seen.add(configuration.name())) {
        twice.add(configuration.name());
      }
    }
    for (String name : twice) {
      problems.add(Finding.refused(name, "two configurations are named " + name));
    }
  }

  /** What can't be, about a joint. */
  static Finding refused(Joint joint, String message) {
    return Finding.refused(joint.label(), message);
  }
}
