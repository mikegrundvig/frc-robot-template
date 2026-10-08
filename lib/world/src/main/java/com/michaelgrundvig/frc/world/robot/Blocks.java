package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * The definition's checks of each joint's block against itself and the robot it's on: where it
 * rests and its places against its stops, its control against its range, its places and its
 * sensors, and its absolute encoder against its motors' controllers, and a sliding joint's against
 * its drum and its stops. Judged on the whole definition, never on a block half written, so the
 * order of its calls doesn't matter, and a sensor of the robot's on the joint counts as its own
 * does. What the program can't drive yet is the program's to say, not the definition's.
 */
final class Blocks {
  private Blocks() {}

  /** What's wrong with the joints' blocks, each about its joint. */
  static List<Finding> problems(RobotMath math) {
    List<Finding> problems = new ArrayList<>();
    for (Joint joint : math.joints()) {
      pastItsStops(joint).forEach(problem -> add(problems, joint, problem));
      Optional<MechanismControl> control = joint.driven().control();
      if (control.isPresent()) {
        add(problems, joint, otherLoop(joint, control.get()));
        add(problems, joint, outsideStops(joint, control.get()));
        for (Position position : outsideSoftLimits(joint, control.get())) {
          add(
              problems,
              joint,
              String.format(
                  Locale.ROOT,
                  "%s's %s, %s, is outside its soft limits, %s to %s: it's never driven there",
                  joint.label(),
                  position.name(),
                  joint.shown(position.value()),
                  joint.shown(control.get().softLimits().orElseThrow().lower()),
                  joint.shown(control.get().softLimits().orElseThrow().upper())));
        }
        add(problems, joint, nothingToHome(joint, control.get()));
        if (math.motorsAlone(joint)) {
          add(problems, joint, cantHome(math, joint, control.get()));
          add(problems, joint, noSwitch(math, joint, control.get()));
        }
      }
      add(problems, joint, unwiredMotors(joint));
      add(problems, joint, offTheMotorsBus(joint));
      add(problems, joint, slidesUnread(joint));
      add(problems, joint, cantCloseLoop(joint));
      add(problems, joint, noDataPort(joint));
      add(problems, joint, offSparks(joint));
    }
    return problems;
  }

  /**
   * The joints with motors and no control, which the program can't drive: one turned by motors
   * alone, its block's or a drive of the robot's, that follows no other.
   */
  static List<Finding> uncontrolled(RobotMath math) {
    List<Finding> uncontrolled = new ArrayList<>();
    for (Joint joint : math.joints()) {
      if (joint.driven().control().isEmpty()
          && (joint.driven().motors().isPresent() || math.motorsAlone(joint))) {
        uncontrolled.add(
            Finding.refused(
                joint.label(),
                joint.label()
                    + " has motors and no control: give it .control(MechanismControl.position(kP)),"
                    + " or .velocity(kP) for one that only runs"));
      }
    }
    return uncontrolled;
  }

  private static void add(List<Finding> problems, Joint joint, @Nullable String problem) {
    if (problem != null) {
      problems.add(Finding.refused(joint.label(), problem));
    }
  }

  /**
   * Why a joint's control is written for the other loop than the joint runs, or null when it isn't:
   * one held at positions (with a range or named positions) runs a position loop, and one that only
   * runs at outputs, a roller, a velocity loop. Its kP is volts per its loop's error, so one
   * written for the other would run, silently wrong by orders of magnitude.
   */
  private static @Nullable String otherLoop(Joint joint, MechanismControl control) {
    boolean positions = joint.holdsPositions();
    if ((control.loop() == MechanismControl.Loop.POSITION) == positions) {
      return null;
    }
    boolean revolute = joint instanceof Joint.Revolute;
    String per =
        positions
            ? (revolute
                ? "radian of error, or per turn, Volts.per(Rotation).of(...)"
                : "meter of error, or per inch, Volts.per(Inch).of(...)")
            : (revolute
                ? "radian per second of error, or per turn per second,"
                    + " Volts.per(RotationsPerSecond).of(...)"
                : "meter per second of error, or per inch per second,"
                    + " Volts.per(InchesPerSecond).of(...)");
    return joint.label()
        + (positions
            ? " is held at positions (it has a range or named positions), and its control is"
                + " MechanismControl.velocity(...), for a roller run at speeds: write"
                + " MechanismControl.position(kP), its kP in volts per "
            : " only runs at outputs (it has no range and no named positions), and its control is"
                + " MechanismControl.position(...), for a joint held at positions: write"
                + " MechanismControl.velocity(kP), its kP in volts per ")
        + per;
  }

  /**
   * Why the control's soft limits reach past the joint's hard stops, or null when they don't. Plain
   * numbers spanning more than three times its range that fit it as degrees likely are, and it says
   * so: a turn is smaller than a radian's number, so turns never look too large.
   */
  private static @Nullable String outsideStops(Joint joint, MechanismControl control) {
    if (control.softLimits().isEmpty() || joint.range().isEmpty()) {
      return null;
    }
    MechanismControl.SoftLimits soft = control.softLimits().get();
    Range range = joint.range().get();
    if (soft.lower() >= range.lower() && soft.upper() <= range.upper()) {
      return null;
    }
    return String.format(
        Locale.ROOT,
        "%s's soft limits, %s to %s, must be inside its hard stops, %s to %s",
        joint.label(),
        joint.shown(soft.lower()),
        joint.shown(soft.upper()),
        joint.shown(range.lower()),
        joint.shown(range.upper()));
  }

  /**
   * Where a joint rests, and each of its named positions, outside its hard stops, where it can't
   * be. Judged on the finished joint, so a practice robot whose stops and places both move, each
   * past the other's, changes them in either order.
   */
  private static List<String> pastItsStops(Joint joint) {
    if (joint.range().isEmpty()) {
      return List.of();
    }
    Range range = joint.range().get();
    String stops = joint.shown(range.lower()) + " to " + joint.shown(range.upper());
    List<String> past = new ArrayList<>();
    if (!range.contains(joint.restPosition())) {
      past.add(
          joint.rest().isPresent()
              ? joint.label()
                  + " rests at "
                  + joint.shown(joint.restPosition())
                  + ", outside its hard stops, "
                  + stops
                  + ": it can't start there"
              : joint.label()
                  + " has no rest, so it starts at its zero, "
                  + joint.shown(0)
                  + ", outside its hard stops, "
                  + stops
                  + ": it can't start there; give where it rests, .rest(...)");
    }
    for (Position position : joint.positions()) {
      if (!range.contains(position.value())) {
        past.add(
            possessive(joint.label())
                + " "
                + position.name()
                + ", "
                + joint.shown(position.value())
                + ", is outside its hard stops, "
                + stops
                + ": it can't get there");
      }
    }
    return past;
  }

  /**
   * The joint's named positions outside its control's soft limits, but inside its hard stops: where
   * it's never driven.
   */
  private static List<Position> outsideSoftLimits(Joint joint, MechanismControl control) {
    if (control.softLimits().isEmpty()) {
      return List.of();
    }
    MechanismControl.SoftLimits soft = control.softLimits().get();
    return joint.positions().stream()
        .filter(position -> position.value() < soft.lower() || position.value() > soft.upper())
        .filter(
            position -> joint.range().map(stops -> stops.contains(position.value())).orElse(true))
        .toList();
  }

  /**
   * Why a joint's homing recipe can't work, or null when it can: a roller has no position to find,
   * and a hard stop must be one the joint has.
   */
  private static @Nullable String nothingToHome(Joint joint, MechanismControl control) {
    if (control.homing().isEmpty()) {
      return null;
    }
    if (!joint.holdsPositions()) {
      return joint.label() + " only runs at outputs: it has no position to home";
    }
    if (control.homing().get() instanceof MechanismControl.Homing.ToHardStop
        && joint.range().isEmpty()) {
      return joint.label() + " turns freely: it has no hard stop to home to";
    }
    return null;
  }

  /**
   * Why a joint can't find where it is, or null when it can: held at positions, it homes by its
   * absolute encoder, read on a robot through its CANcoder, its SPARK's data port or a Systemcore
   * port, or its control names a recipe.
   */
  private static @Nullable String cantHome(RobotMath math, Joint joint, MechanismControl control) {
    if (!joint.holdsPositions() || control.homing().isPresent()) {
      return null;
    }
    if (math.absoluteEncoder(joint) == null) {
      return joint.label()
          + " has no absolute encoder, so where it is at power on is a guess: give its control a"
          + " homing recipe, Homing.toHardStop, Homing.toSwitch or Homing.assumeAtRest()";
    }
    List<MechanismWiring.Device> controllers =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    boolean encoderWired =
        joint.driven().absoluteEncoder().map(Driven.AbsoluteEncoder::wired).orElse(false);
    if (!controllers.isEmpty() && !encoderWired) {
      return joint.label()
          + " homes by its absolute encoder, and its wiring names no CANcoder, data port or"
          + " Systemcore port to read it on: wire it in its block, "
          + readable(controllers.get(0).controller())
          + ", or give its control a homing recipe";
    }
    return null;
  }

  /**
   * The absolute encoder a joint's leading controller reads, as its block wires it: a SPARK's on
   * its data port, which reads no CANcoder; a TalonFX's or a TalonFXS's, a CANcoder on its bus.
   */
  private static String readable(MechanismWiring.Controller controller) {
    String dataPort = "'s data port, .absoluteEncoder(DATA_PORT.offset(...))";
    String cancoder = "'s bus, .absoluteEncoder(CANCODER.can(...).offset(...))";
    return switch (controller) {
      case SPARK_MAX -> "on its SPARK MAX" + dataPort;
      case SPARK_FLEX -> "on its SPARK Flex" + dataPort;
      case TALON_FX -> "as a CANcoder on its TalonFX" + cancoder;
      case TALON_FXS -> "as a CANcoder on its TalonFXS" + cancoder;
    };
  }

  /**
   * Why a joint can't home to its switch, or null when it can, or doesn't: it has none at that end,
   * or where its switch trips isn't known.
   */
  private static @Nullable String noSwitch(RobotMath math, Joint joint, MechanismControl control) {
    if (!(control.homing().orElse(null) instanceof MechanismControl.Homing.ToSwitch to)) {
      return null;
    }
    boolean lower = to.end() == Sensor.Switch.Limit.End.LOWER;
    Sensor.Switch onSwitch = math.limitSwitch(joint, to.end());
    String end = lower ? "LOWER" : "UPPER";
    String at = joint instanceof Joint.Revolute ? "Degrees.of(...)" : "Inches.of(...)";
    if (onSwitch == null) {
      return joint.label()
          + " homes to its "
          + (lower ? "lower" : "upper")
          + " limit switch, and its block has none: give it one where it trips, .limitSwitch(End."
          + end
          + ", "
          + at
          + ", ...)";
    }
    if (((Sensor.Switch.Limit) onSwitch.kind()).position().isEmpty()) {
      return joint.label()
          + " homes to "
          + onSwitch.label()
          + ", whose position isn't known: measure where it trips and give it, .limitSwitch(End."
          + end
          + ", "
          + at
          + ", ...), or home another way";
    }
    return null;
  }

  /**
   * Why a sliding joint's absolute encoder can't say where it is, or null when it can, or it has
   * none: the encoder turns with its drum, whose pitch radius its block's motors give, and one of
   * its turns must cover the joint's travel, which its hard stops bound.
   */
  private static @Nullable String slidesUnread(Joint joint) {
    if (!(joint instanceof Joint.Prismatic) || joint.driven().absoluteEncoder().isEmpty()) {
      return null;
    }
    if (joint.driven().motors().flatMap(Driven.Motors::pitchRadius).isEmpty()) {
      return possessive(joint.label())
          + " absolute encoder turns with its drum, and its block has no motors to say the drum's"
          + " pitch radius: give them, .motors(motor, gearing, pitchRadius, ...)";
    }
    if (joint.range().isEmpty()) {
      return joint.label()
          + " has an absolute encoder and no hard stops, so nothing says one of the encoder's turns"
          + " covers its travel: give its range, range(Range.meters(...))";
    }
    return null;
  }

  /**
   * Why a joint's loop can't close on its absolute encoder, as its block says it does, or null when
   * it can, or doesn't: the joint must be held at positions, know where it is from that encoder
   * rather than a homing recipe, and have a range, which the definition checks one of the encoder's
   * turns holds, rather than turn freely, where the reading wraps within its travel. A sliding
   * joint without one is refused already ({@link #slidesUnread}).
   */
  private static @Nullable String cantCloseLoop(Joint joint) {
    if (!joint.driven().absoluteEncoder().map(Driven.AbsoluteEncoder::closedLoop).orElse(false)) {
      return null;
    }
    if (!joint.holdsPositions()) {
      return joint.label()
          + " only runs at outputs, so its loop has no position to close on its absolute encoder:"
          + " drop .closesLoop() from its encoder's wiring";
    }
    if (joint.driven().control().flatMap(MechanismControl::homing).isPresent()) {
      return possessive(joint.label())
          + " loop closes on its absolute encoder, which tells it where it is: drop its"
          + " control's homing recipe";
    }
    if (joint.range().isEmpty() && joint instanceof Joint.Revolute) {
      return joint.label()
          + " turns freely, and its absolute encoder's reading wraps each turn: its loop can't"
          + " close on it yet, so let the encoder tell its motor where it is instead";
    }
    return null;
  }

  /**
   * Why a joint's absolute encoder is on a data port its motors' controller hasn't, or null when it
   * isn't: a TalonFX has none.
   */
  private static @Nullable String noDataPort(Joint joint) {
    List<MechanismWiring.Device> controllers =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    boolean onDataPort =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::dataPort).isPresent();
    return onDataPort
            && !controllers.isEmpty()
            && controllers.get(0).controller() == MechanismWiring.Controller.TALON_FX
        ? possessive(joint.label())
            + " wiring names an encoder on its controller's data port, and a TalonFX has none:"
            + " wire it as a CANcoder"
        : null;
  }

  /**
   * Why a joint's SPARKs can't be wired as its block says, or null when they can be: a SPARK is on
   * Systemcore's own CAN, never a CANivore, and reads no CANcoder.
   */
  private static @Nullable String offSparks(Joint joint) {
    List<MechanismWiring.Device> controllers =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    if (controllers.isEmpty()
        || (controllers.get(0).controller() != MechanismWiring.Controller.SPARK_MAX
            && controllers.get(0).controller() != MechanismWiring.Controller.SPARK_FLEX)) {
      return null;
    }
    if (!(controllers.get(0).bus() instanceof CanBus.Port)) {
      return possessive(joint.label())
          + " SPARKs are on a CANivore: a SPARK is on Systemcore's own CAN";
    }
    if (joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder).isPresent()) {
      return possessive(joint.label())
          + " wiring names a CANcoder, which a SPARK doesn't read: plug the encoder into the"
          + " SPARK's data port, or give its control a homing recipe";
    }
    return null;
  }

  /**
   * Why a joint's absolute encoder, read through its motors' controller, can't be: its motors
   * aren't wired. Null when they are, or it's read another way.
   */
  private static @Nullable String unwiredMotors(Joint joint) {
    boolean throughController =
        joint
            .driven()
            .absoluteEncoder()
            .map(Driven.AbsoluteEncoder::readByController)
            .orElse(false);
    boolean motorsWired =
        joint.driven().motors().map(motors -> !motors.controllers().isEmpty()).orElse(false);
    return throughController && !motorsWired
        ? possessive(joint.label())
            + " absolute encoder is wired, and its motors aren't: it's read through their"
            + " controller, so wire them too; a joint your own code drives reads one on a"
            + " Systemcore port instead, MechanismWiring.PortEncoder"
        : null;
  }

  /**
   * Why a joint's CANcoder isn't where its TalonFX can read it, on another bus than its motors; or
   * null when it is, or there's none.
   */
  private static @Nullable String offTheMotorsBus(Joint joint) {
    Optional<MechanismWiring.Cancoder> cancoder =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder);
    List<MechanismWiring.Device> controllers =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    if (cancoder.isEmpty() || controllers.isEmpty()) {
      return null;
    }
    CanBus bus = controllers.get(0).bus();
    return cancoder.get().bus().equals(bus)
        ? null
        : possessive(joint.label())
            + " CANcoder is on "
            + cancoder.get().bus().shown()
            + ", and its motors on "
            + bus.shown()
            + ": their TalonFX reads it over its own bus, so put the CANcoder there";
  }
}
