package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * How the program drives one joint, worked out once from what the joint is and how its block says
 * to drive it, in the joint's own units, radians or meters: its gains, each tuned where its control
 * gives it and seeded from the definition where not ({@link RobotMath#idealGains}); gravity's kind,
 * its kG and where it pulls hardest; its soft limits, current limits and neutral mode; its motors'
 * controllers, and which way each turns; and the one feedforward law ({@link #feedforward}) that
 * every loop driving the joint evaluates: a TalonFX's slot, set from these gains, a SPARK's
 * feedforward, which the robot works out each loop, a loop the robot closes itself, and the
 * simulator's motor controllers. A joint whose block wires no controllers is planned on the one its
 * motor runs on ({@link MechanismWiring.Controller#standIn}), which the simulator runs it as.
 *
 * <p>Vendor-neutral and pure: a motor controller's settings are worked out from it in that vendor's
 * units and terms, and what only one vendor can't take (a ratio past Phoenix's, a kI on a SPARK) is
 * that vendor's to refuse. What no motor controller could do is refused here, by an {@link
 * IllegalArgumentException} saying what and why: motors that don't move the joint, a slide with no
 * drum to count its rotations in ({@link MechanismRotations}), a tuned kG on a joint gravity can't
 * move. What's doubtful is noted beside it ({@link #notes}): a tuned gain far from what the
 * definition derives, a gravity one cosine fits badly. A joint with no kG written is the
 * definition's doubt ({@code RobotDefinition.findings}), not noted here.
 */
public final class ControllerPlan {
  /**
   * How far one cosine may miss gravity's torque, as a share of its peak, before it's noted: beyond
   * a few percent, the joint's feedforward wants {@link RobotMath#gravityTorque} instead.
   */
  public static final double MISFIT = 0.05;

  /**
   * How far a tuned gain may be from the definition's ideal one and still be noted as the same
   * mechanism: from half to twice. Friction and losses put real gains above the ideal, rarely past
   * twice it; past that, a unit or a ratio is likelier than the mechanism.
   */
  static final double SANITY_LOW = 0.5;

  static final double SANITY_HIGH = 2.0;

  /** How close a tuned gain's ratio to the ideal must come to a frame's to be named as one. */
  static final double FRAME_TOLERANCE = 0.1;

  /** Meters in an inch, for naming a gain worked out per inch. */
  private static final double INCH = 0.0254;

  /**
   * Gravity's share of the feedforward: none for a joint gravity can't move (a turret, a slide that
   * runs level), kG steady all along a slide, kG along one cosine for an arm, as Phoenix's {@code
   * Arm_Cosine} and WPILib's {@code ArmFeedforward} take it.
   *
   * @param kind which of the three
   * @param kG the volts that hold the joint: everywhere along a slide, where its weight pulls
   *     hardest on an arm; 0 for none
   * @param offset for an arm, what's added to its position for its angle from where its weight
   *     pulls hardest, in radians: Phoenix's {@code GravityArmPositionOffset}, in rotations; 0
   *     otherwise
   */
  public record Gravity(Kind kind, double kG, double offset) {
    /** Which way gravity's volts follow the joint. */
    public enum Kind {
      /** Gravity can't move the joint. */
      NONE,
      /** The same all along: a slide's. */
      STEADY,
      /** Along one cosine of the joint's angle: an arm's. */
      COSINE
    }

    /** The volts that hold the joint against gravity at a position. */
    public double volts(double position) {
      return switch (kind) {
        case NONE -> 0;
        case STEADY -> kG;
        case COSINE -> kG * Math.cos(position + offset);
      };
    }
  }

  private final Joint joint;
  private final Drive drive;
  private final MechanismControl control;
  private final List<MechanismWiring.Device> controllers;
  private final int motors;
  private final MechanismRotations rotations;
  private final double rotorRatio;
  private final double kV;
  private final double kA;
  private final Gravity gravity;
  private final List<String> notes;

  private ControllerPlan(
      RobotMath math,
      Joint joint,
      Drive drive,
      List<MechanismWiring.Device> controllers,
      MechanismControl control) {
    List<String> noted = new ArrayList<>();
    if (!(drive.actuator() instanceof Actuator.Motors driving)) {
      throw new IllegalArgumentException(
          drive.label() + " isn't motors: a motor controller drives motors");
    }
    if (!controllers.isEmpty() && controllers.size() != driving.count()) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s is %d motors, and its wiring names %d",
              drive.label(),
              driving.count(),
              controllers.size()));
    }
    Sensor.Encoder rotor = drive.rotor();
    rotorRatio = Math.abs(rotor.ratio(joint));
    if (rotorRatio == 0) {
      throw new IllegalArgumentException(drive.label() + " doesn't move " + joint.label());
    }
    for (Transmission.Term term : rotor.terms()) {
      if (!term.joint().equals(joint)) {
        noted.add(
            String.format(
                Locale.ROOT,
                "%s's rotor turns %s times for each %s of %s too, which the controller reads as %s"
                    + " moving: take it out in code, as a swerve module's coupling is",
                drive.label(),
                shown(Math.abs(rotor.ratio(term.joint()))),
                term.joint() instanceof Joint.Revolute ? "turn" : "meter",
                term.joint().label(),
                joint.label()));
      }
    }
    this.joint = joint;
    this.drive = drive;
    this.control = control;
    this.controllers = controllers.isEmpty() ? standIns(driving) : List.copyOf(controllers);
    motors = driving.count();
    rotations = MechanismRotations.of(joint, drive);
    RobotMath.Gains seed = math.idealGains(joint);
    MechanismControl.Gains gains = control.gains();
    kV = gains.kV().orElse(seed.kV());
    if (gains.kV().isPresent()) {
      String far = farFromIdeal("kV", kV, seed.kV(), rotorRatio);
      if (far != null) {
        noted.add(far);
      }
    }
    kA = gains.kA().orElse(seed.kA());
    gravity = gravity(math, gains, seed.kG(), noted);
    notes = List.copyOf(noted);
  }

  /**
   * The plan for a joint as its block says: its control, its motors' controllers.
   *
   * @param joint one the template drives, with a control
   * @throws IllegalArgumentException for a joint with no control, and what no motor controller
   *     could do
   */
  public static ControllerPlan of(RobotMath math, Joint joint) {
    return of(
        math,
        joint,
        joint
            .driven()
            .control()
            .orElseThrow(() -> new IllegalArgumentException(joint.label() + " has no control")));
  }

  /**
   * The plan for a joint driven by another control than its block's: as the pit tunes it. Its drive
   * is the definition's ({@link RobotMath#drivesOf}), its controllers its block's, or its motor's
   * stand-in where they aren't wired ({@link #controllers}).
   *
   * @throws IllegalArgumentException for what no motor controller could do
   */
  public static ControllerPlan of(RobotMath math, Joint joint, MechanismControl control) {
    return of(
        math,
        joint,
        math.drivesOf(joint).get(0),
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of()),
        control);
  }

  /**
   * The plan for a joint from its parts given apart: what a vendor's helper is given, by its tests.
   *
   * @param drive the motors that move it
   * @param controllers each motor's controller, the leader first, as many as its motors; or none,
   *     for its motor's stand-in ({@link #controllers})
   * @throws IllegalArgumentException for what no motor controller could do
   */
  public static ControllerPlan of(
      RobotMath math,
      Joint joint,
      Drive drive,
      List<MechanismWiring.Device> controllers,
      MechanismControl control) {
    return new ControllerPlan(math, joint, drive, controllers, control);
  }

  /** The joint. */
  public Joint joint() {
    return joint;
  }

  /** The motors that move it. */
  public Drive drive() {
    return drive;
  }

  /** How it's driven, as given: its block's, or as the pit tunes it. */
  public MechanismControl control() {
    return control;
  }

  /**
   * Each motor's controller, the leader first: its block's, or, where its block wires none, the
   * controller its motor runs on ({@link MechanismWiring.Controller#standIn}), on Systemcore's CAN
   * at IDs from 0, each turning counterclockwise for positive. A stand-in's ID only tells its
   * motors apart: nothing refused or doubted of the joint reads its ID, its bus or its direction.
   */
  public List<MechanismWiring.Device> controllers() {
    return controllers;
  }

  /** The stand-in controller for each of a joint's motors, its block wiring none. */
  private static List<MechanismWiring.Device> standIns(Actuator.Motors motors) {
    MechanismWiring.Controller controller = MechanismWiring.Controller.standIn(motors.motor());
    List<MechanismWiring.Device> standIns = new ArrayList<>();
    for (int id = 0; id < motors.count(); id++) {
      standIns.add(controller.can(id));
    }
    return List.copyOf(standIns);
  }

  /**
   * Whether a following motor's controller turns the other way from the leader's as the joint moves
   * forward: one opposed to it, rather than aligned.
   */
  public boolean opposes(MechanismWiring.Device follower) {
    return controllers.get(0).turnsClockwise() != follower.turnsClockwise();
  }

  /** How many motors move it together. */
  public int motors() {
    return motors;
  }

  /** Its rotations, as its motor controllers count them. */
  public MechanismRotations rotations() {
    return rotations;
  }

  /**
   * Its motors' rotor turns for each turn (or meter) of the joint, as the definition writes it: a
   * magnitude.
   */
  public double rotorRatio() {
    return rotorRatio;
  }

  /** Volts per radian (or meter) of error. */
  public double kP() {
    return control.gains().kP();
  }

  /** Volts per radian (or meter) of error, per second it lasts. */
  public double kI() {
    return control.gains().kI();
  }

  /** Volts per radian (or meter) per second of error. */
  public double kD() {
    return control.gains().kD();
  }

  /** Volts to overcome friction, in the direction of travel: measured, never seeded. */
  public double kS() {
    return control.gains().kS();
  }

  /** Volts per radian (or meter) per second: tuned, else the definition's seed. */
  public double kV() {
    return kV;
  }

  /** Volts per radian (or meter) per second squared: tuned, else the definition's seed. */
  public double kA() {
    return kA;
  }

  /** Gravity's share of its feedforward: tuned, else the definition's seed from its mass. */
  public Gravity gravity() {
    return gravity;
  }

  /** Where its motor controllers stop driving it, if its control says. */
  public Optional<MechanismControl.SoftLimits> softLimits() {
    return control.softLimits();
  }

  /** Its motors' current limits. */
  public MechanismControl.CurrentLimits currentLimits() {
    return control.currentLimits();
  }

  /** Whether its motors brake when told nothing, rather than coast. */
  public boolean brakes() {
    return control.neutralMode() == MechanismControl.NeutralMode.BRAKE;
  }

  /** Whether it turns freely, so a position is reached the short way round, a turn being a turn. */
  public boolean turnsFreely() {
    return joint instanceof Joint.Revolute && joint.range().isEmpty();
  }

  /** What's doubtful about it, each a sentence for a log or a test. */
  public List<String> notes() {
    return notes;
  }

  /**
   * The feedforward: the volts that move the joint as asked, before any feedback. Friction in the
   * direction of travel, kV at the speed, kA at the acceleration, and gravity at the position.
   *
   * @param position where it is, or where it's asked to be, in its units
   * @param velocity how fast it's asked to move, in its units a second
   * @param acceleration how fast it's asked to speed up, in its units a second squared
   */
  public double feedforward(double position, double velocity, double acceleration) {
    return kS() * Math.signum(velocity)
        + kV * velocity
        + kA * acceleration
        + gravity.volts(position);
  }

  /**
   * Where an absolute encoder its motor controller reads wraps, in the encoder's turns, as the
   * controller reads it: told its direction, so it grows as the joint moves forward, and its
   * offset, so it reads zero with the joint at zero. The definition's discontinuity point ({@link
   * RobotMath#discontinuityPoint}), which reads it as its ratio's sign says, turned round for an
   * encoder geared to turn back: Phoenix's {@code AbsoluteSensorDiscontinuityPoint}, and where a
   * SPARK's reading of its data port wraps.
   *
   * @throws IllegalArgumentException as the definition's does
   */
  public static double wrapPoint(RobotMath math, Sensor.Encoder encoder) {
    double point = math.discontinuityPoint(encoder);
    return encoder.ratio(encoder.joint()) < 0 ? 1 - point : point;
  }

  /**
   * How far inside an absolute encoder's turn its joint's range sits when the encoder wraps at a
   * point, in the encoder's turns, at the nearer end: negative when the range crosses the wrap. The
   * encoder reads as its motor controller reads it, growing with the joint ({@link #wrapPoint}).
   * Infinite for a joint that turns freely, which has no range to keep clear.
   */
  public static double wrapMargin(Sensor.Encoder encoder, double wrapPoint) {
    // The definition reads the encoder as its ratio's sign says; a motor controller, as its wiring
    // sets it, growing with the joint: an encoder geared to turn back wraps at the other end of its
    // turn.
    return RobotMath.wrapMargin(
        encoder, encoder.ratio(encoder.joint()) < 0 ? 1 - wrapPoint : wrapPoint);
  }

  /**
   * What's wrong with an absolute encoder wrapping at a point, or null when nothing is: its joint's
   * range crossing the wrap, or within the definition's margin of it ({@link
   * RobotMath#WRAP_MARGIN}, a fortieth of a turn). A team's own point is checked as the template's
   * is.
   */
  public static @Nullable String wrapProblem(Sensor.Encoder encoder, double wrapPoint) {
    double margin = wrapMargin(encoder, wrapPoint);
    if (margin >= RobotMath.WRAP_MARGIN) {
      return null;
    }
    Joint joint = encoder.joint();
    Range range = joint.range().orElseThrow();
    double atStow =
        Math.min(
            reading(encoder, joint.restPosition()) - (wrapPoint - 1),
            wrapPoint - reading(encoder, joint.restPosition()));
    return String.format(
        Locale.ROOT,
        "%s wraps at %s of its turn, %s %s's range (%s to %s), and %s from where it stows (%s): a"
            + " reading a hair past the wrap reads a turn away",
        encoder.label(),
        shown(wrapPoint),
        margin < 0 ? "inside" : joint.shown(alongJoint(encoder, margin)) + " from",
        joint.label(),
        joint.shown(range.lower()),
        joint.shown(range.upper()),
        joint.shown(alongJoint(encoder, atStow)),
        joint.shown(joint.restPosition()));
  }

  /** The encoder's turns for each of its joint's turns (or meters): a magnitude. */
  private static double sensorRatio(Sensor.Encoder encoder) {
    return Math.abs(encoder.ratio(encoder.joint()));
  }

  /** How far its joint moves, in the joint's units, as an encoder turns this much. */
  private static double alongJoint(Sensor.Encoder encoder, double turns) {
    return turns / sensorRatio(encoder) * perRatioUnit(encoder.joint());
  }

  /** What an encoder reads, in its turns, with its joint at a position. */
  private static double reading(Sensor.Encoder encoder, double position) {
    return sensorRatio(encoder) * position / perRatioUnit(encoder.joint());
  }

  /**
   * The joint's units in each of the units the definition's ratios are per: 2π radians in a turn of
   * a revolute joint, a meter in a meter of a prismatic one.
   */
  private static double perRatioUnit(Joint joint) {
    return joint instanceof Joint.Revolute ? 2 * Math.PI : 1;
  }

  /**
   * Gravity's kind and kG: the tuned kG where given, else the definition's seed from its links'
   * mass, 0 without it. An arm's weight pulls hardest where its control says ({@code
   * gravityAngle}), else where its links' mass puts it, else at its zero.
   */
  private Gravity gravity(
      RobotMath math, MechanismControl.Gains gains, double seed, List<String> noted) {
    Optional<Double> tuned = gains.kG();
    if (!math.gravityActsOn(joint)) {
      if (tuned.isPresent() && tuned.get() != 0) {
        throw new IllegalArgumentException(
            String.format(
                Locale.ROOT,
                "%s has a tuned kG of %s V, but gravity can't move it: it %s. Leave its kG out, or"
                    + " write 0",
                joint.label(),
                shown(tuned.get()),
                joint instanceof Joint.Revolute ? "turns about the vertical" : "slides level"));
      }
      return new Gravity(Gravity.Kind.NONE, 0, 0);
    }
    double kG = tuned.orElse(seed);
    Gravity found;
    if (joint instanceof Joint.Revolute) {
      double offset =
          gains.gravityAngle().isPresent()
              ? -Math.IEEEremainder(gains.gravityAngle().get().in(Radians), 2 * Math.PI)
              : math.gravityOffset(joint);
      found = new Gravity(Gravity.Kind.COSINE, kG, offset);
      RobotMath.GravityFit fit = math.gravityFit(joint);
      if (fit.misfit() > MISFIT) {
        noted.add(
            String.format(
                Locale.ROOT,
                "one cosine misses %s's gravity by %.0f%% of its peak, at %s: a controller's arm"
                    + " cosine can't follow it, so feed RobotMath.gravityTorque at the joint's"
                    + " angle as the request's feedforward",
                joint.label(),
                fit.misfit() * 100,
                joint.shown(fit.worstAt())));
      }
    } else {
      found = new Gravity(Gravity.Kind.STEADY, kG, 0);
    }
    if (tuned.isPresent()) {
      String far = farFromIdeal("kG", kG, seed, 0);
      if (far != null) {
        noted.add(far);
      }
    }
    if (kG != 0) {
      Joint tilting = tilting(math);
      if (tilting != null) {
        noted.add(
            String.format(
                Locale.ROOT,
                "%s rides %s, whose turning tilts its gravity: a controller's gravity term holds it"
                    + " only with %s at rest, so feed RobotMath.gravityTorque at the joints'"
                    + " positions as the request's feedforward",
                joint.label(),
                tilting.label(),
                tilting.label()));
      }
    }
    return found;
  }

  /**
   * The first joint above this one whose turning tilts gravity at it: a revolute joint about an
   * axis that isn't vertical, with the joints where they rest. Null when none does.
   */
  private @Nullable Joint tilting(RobotMath math) {
    Part up = joint.parent();
    while (up instanceof Joint parent) {
      if (parent instanceof Joint.Revolute && math.gravityActsOn(parent)) {
        return parent;
      }
      up = parent.parent();
    }
    return null;
  }

  /** A frame a gain may have been worked out in: how many of its units make the joint's one. */
  private record Frame(String name, double perJointUnit) {}

  /**
   * Why a tuned gain looks wrong beside the definition's ideal, or null when it's within {@link
   * #SANITY_LOW} to {@link #SANITY_HIGH} of it. When the two differ by about the ratio between two
   * frames, the frames are named: a gain worked out per turn of the joint and entered per turn of
   * its rotor is the rotor's ratio times too high.
   *
   * @param rotorRatio the rotor's turns for each turn (or meter) of the joint, or 0 for a gain in
   *     volts
   */
  private @Nullable String farFromIdeal(
      String gain, double tuned, double ideal, double rotorRatio) {
    if (ideal == 0) {
      return null;
    }
    double ratio = tuned / ideal;
    if (ratio >= SANITY_LOW && ratio <= SANITY_HIGH) {
      return null;
    }
    String why =
        String.format(
            Locale.ROOT,
            "%s's tuned %s, %s, is %s times the ideal, %s, from its motors and what it moves",
            joint.label(),
            gain,
            shownGain(gain, tuned),
            shown(ratio),
            shownGain(gain, ideal));
    if (rotorRatio == 0) {
      return why;
    }
    List<Frame> frames = frames(rotorRatio);
    double best = FRAME_TOLERANCE;
    String named = null;
    for (Frame meant : frames) {
      for (Frame entered : frames) {
        if (meant.equals(entered)) {
          continue;
        }
        double off = Math.abs(ratio / (entered.perJointUnit() / meant.perJointUnit()) - 1);
        if (off < best) {
          best = off;
          named =
              String.format(
                  Locale.ROOT,
                  ": the ideal per %s, within %d%%, entered as if per %s, by the look of it",
                  meant.name(),
                  Math.max(1, (int) Math.ceil(off * 100)),
                  entered.name());
        }
      }
    }
    return named == null ? why : why + named;
  }

  private List<Frame> frames(double rotorRatio) {
    if (joint instanceof Joint.Revolute) {
      return List.of(
          new Frame("radian", 1),
          new Frame("degree", 180 / Math.PI),
          new Frame("turn of " + joint.label(), 1 / (2 * Math.PI)),
          new Frame("turn of its rotor", rotorRatio / (2 * Math.PI)));
    }
    List<Frame> frames = new ArrayList<>();
    frames.add(new Frame("meter", 1));
    frames.add(new Frame("inch", 1 / INCH));
    frames.add(new Frame("turn of its rotor", rotorRatio));
    if (rotations.perRotation() != 1) {
      frames.add(new Frame("turn of its drum", 1 / rotations.perRotation()));
    }
    return frames;
  }

  private String shownGain(String gain, double value) {
    String per = joint instanceof Joint.Revolute ? "rad" : "m";
    return switch (gain) {
      case "kV" -> shown(value) + " V/(" + per + "/s)";
      case "kA" -> shown(value) + " V/(" + per + "/s²)";
      default -> shown(value) + " V";
    };
  }
}
