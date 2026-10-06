package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.atMost;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;

/**
 * Where an actuator's force goes: through a gearing to a joint, onto a drum that winds a carriage
 * up, to several joints at once, or along a line between two parts. A {@link Drive} pairs one with
 * its {@link Actuator}.
 *
 * <p>Every ratio has one direction, written once: what the actuator's shaft turns for each turn (or
 * meter) of the joint. Phoenix's ratios, the ideal gains and the simulator's reflected inertia all
 * follow from it.
 */
public sealed interface Transmission {
  /** The share of the actuator's force or torque that gets through: above 0, at most 1. */
  double efficiency();

  /** The joints it moves directly, without following a coupling. */
  List<Joint> joints();

  /**
   * How far the actuator's shaft turns, in radians, for each radian (or meter) a joint it names
   * moves, the others held: 0 for a joint it doesn't name. Nothing for a line between parts, which
   * turns no shaft.
   *
   * @throws IllegalStateException for a {@link Between}, which has no shaft
   */
  double shaftRadiansPer(Joint joint);

  /**
   * A gearing turning a revolute joint: an arm's gearbox, a turret's ring, a flywheel's belt.
   *
   * @param joint what it turns
   * @param gearing the actuator's turns for each of the joint's, and what it loses
   */
  @ConfigRecord
  record Geared(Joint.Revolute joint, Gearing gearing) implements Transmission {
    /** Starts describing a geared transmission, one named value at a time. */
    public static TransmissionGearedBuilder.JointStage builder() {
      return TransmissionGearedBuilder.builder();
    }

    @Override
    public double efficiency() {
      return gearing.efficiency();
    }

    @Override
    public List<Joint> joints() {
      return List.of(joint);
    }

    @Override
    public double shaftRadiansPer(Joint other) {
      return joint.equals(other) ? gearing.ratio() : 0;
    }
  }

  /**
   * A gearing turning a drum, a sprocket or a pinion that moves a prismatic joint: an elevator's
   * winch, a chain or a rack.
   *
   * @param joint what it moves
   * @param gearing the actuator's turns for each of the drum's, and what it loses
   * @param pitchRadius where the rope, belt, chain or rack meets the drum, sprocket or pinion: the
   *     joint moves this far, times 2π, for each of the drum's turns
   */
  @ConfigRecord
  record Drum(Joint.Prismatic joint, Gearing gearing, Distance pitchRadius)
      implements Transmission {
    public Drum {
      Checks.on(Drum.class, joint.label()).field("pitchRadius", pitchRadius, positive()).orThrow();
    }

    /** Starts describing a drum, one named value at a time. */
    public static TransmissionDrumBuilder.JointStage builder() {
      return TransmissionDrumBuilder.builder();
    }

    @Override
    public double efficiency() {
      return gearing.efficiency();
    }

    @Override
    public List<Joint> joints() {
      return List.of(joint);
    }

    @Override
    public double shaftRadiansPer(Joint other) {
      return joint.equals(other) ? gearing.ratio() / pitchRadius.in(Meters) : 0;
    }
  }

  /**
   * One actuator moving several joints at once: its shaft turns the sum of each joint's position
   * times its ratio. A differential wrist's two motors, a wrist chained through the shoulder's
   * axle, a swerve module's drive motor, whose wheel turns as the module steers.
   *
   * @param terms each joint it moves, with its ratio
   * @param efficiency the share of the actuator's torque that gets through, above 0 and at most 1:
   *     1 until the losses are measured
   */
  @ConfigRecord
  record Combined(List<Term> terms, double efficiency) implements Transmission {
    public Combined {
      terms = List.copyOf(terms);
      Checks.on(Combined.class)
          .field("terms", terms, Combined::notTerms)
          .field("terms", terms, Combined::notTurned)
          .field("efficiency", efficiency, positive())
          .field("efficiency", efficiency, atMost(1.0))
          .orThrow();
    }

    /** Starts describing a combined transmission, one named value at a time. */
    public static TransmissionCombinedBuilder.TermsStage builder() {
      return TransmissionCombinedBuilder.builder();
    }

    @Override
    public List<Joint> joints() {
      return terms.stream().map(Term::joint).toList();
    }

    /**
     * Its term's turns, in radians: a revolute joint's turns per turn, a prismatic joint's turns
     * per meter, 2π radians each.
     */
    @Override
    public double shaftRadiansPer(Joint joint) {
      for (Term term : terms) {
        if (term.joint().equals(joint)) {
          return joint instanceof Joint.Revolute
              ? term.ratio()
              : 2 * Math.PI * term.perTurnOrMeter();
        }
      }
      return 0;
    }

    /**
     * Why terms aren't a sum of joints, or null when they are: one joint at least, each once. An
     * encoder's terms are checked the same way.
     */
    static @Nullable String notTerms(List<Term> terms) {
      if (terms.isEmpty()) {
        return "needs a joint";
      }
      Set<String> seen = new HashSet<>();
      List<String> twice = new ArrayList<>();
      for (Term term : terms) {
        if (!seen.add(term.joint().label())) {
          twice.add(term.joint().label());
        }
      }
      return twice.isEmpty() ? null : "name each joint once, not " + String.join(", ", twice);
    }

    /**
     * Why terms can't be a shaft's, or null when they can: a shaft moves a prismatic joint through
     * a drum, a sprocket or a pinion, whose pitch radius its term names, as a {@link Drum} does.
     */
    private static @Nullable String notTurned(List<Term> terms) {
      List<String> bare = new ArrayList<>();
      for (Term term : terms) {
        if (term.joint() instanceof Joint.Prismatic && term.pitchRadius().isEmpty()) {
          bare.add(term.joint().label());
        }
      }
      return bare.isEmpty()
          ? null
          : "a shaft slides "
              + String.join(", ", bare)
              + " through a drum, a sprocket or a pinion: name its pitch radius, and the turns"
              + " for each of its turns";
    }
  }

  /**
   * One joint a {@link Combined} transmission moves, or a {@link Sensor.Encoder} reads.
   *
   * @param joint the joint
   * @param ratio the actuator's turns, or the encoder's, for each turn of a revolute joint, or for
   *     each turn of the drum, sprocket or pinion a prismatic joint's pitch radius names (for one
   *     read straight off the slide, with none, for each meter); negative when it turns back as the
   *     joint moves forward, as a differential's second motor does on its roll
   * @param pitchRadius a prismatic joint's drum, sprocket or pinion, where the belt, chain or rack
   *     meets it, as a {@link Drum}'s: a combined transmission's prismatic term names one, and its
   *     Phoenix unit is that drum's turn; empty for a revolute joint
   */
  @ConfigRecord
  record Term(Joint joint, double ratio, Optional<Distance> pitchRadius) {
    public Term {
      Checks.on(Term.class, joint.label())
          .field(
              "ratio",
              ratio,
              r -> Double.isFinite(r) && r != 0 ? null : r + " must be a number other than 0")
          .field(
              "pitchRadius",
              pitchRadius,
              radius ->
                  radius.isEmpty() || radius.get().in(Meters) > 0
                      ? null
                      : shown(radius.get()) + " must be positive")
          .check(
              "pitchRadius",
              () ->
                  joint instanceof Joint.Revolute && pitchRadius.isPresent()
                      ? joint.label() + " turns: its ratio is per turn, through no drum"
                      : null)
          .orThrow();
    }

    /** Starts describing a term, one named value at a time. */
    public static TransmissionTermBuilder.JointStage builder() {
      return TransmissionTermBuilder.builder();
    }

    /**
     * Its turns for each turn of a revolute joint, or each meter of a prismatic one: its ratio,
     * through its drum where it names one.
     */
    public double perTurnOrMeter() {
      return pitchRadius.map(r -> ratio / (2 * Math.PI * r.in(Meters))).orElse(ratio);
    }
  }

  /**
   * A line of action between two parts: a cylinder's or a gas spring's two ends, each fixed to the
   * chassis or a joint's link. Its coordinate is the length between them, which the joints' angles
   * change, so the torque it makes at a joint changes with them, as a real counterbalance's does.
   *
   * @param a what one end is fixed to
   * @param onA where on it, in its frame
   * @param b what the other end is fixed to
   * @param onB where on it, in its frame
   */
  @ConfigRecord
  record Between(Part a, Translation3d onA, Part b, Translation3d onB) implements Transmission {
    public Between {
      Checks.on(Between.class)
          .field("onA", onA, Between::notAPoint)
          .field("onB", onB, Between::notAPoint)
          .check("b", () -> a.equals(b) ? "the ends must be on two different parts" : null)
          .orThrow();
    }

    /** Starts describing a line between parts, one named value at a time. */
    public static TransmissionBetweenBuilder.AStage builder() {
      return TransmissionBetweenBuilder.builder();
    }

    /** All of it: nothing is lost along a line. */
    @Override
    public double efficiency() {
      return 1;
    }

    @Override
    public List<Joint> joints() {
      List<Joint> joints = new ArrayList<>();
      for (Part part : List.of(a, b)) {
        if (part instanceof Joint joint) {
          joints.add(joint);
        }
      }
      return joints;
    }

    @Override
    public double shaftRadiansPer(Joint joint) {
      throw new IllegalStateException("a line between two parts turns no shaft");
    }

    private static @Nullable String notAPoint(Translation3d point) {
      return Double.isFinite(point.getX() + point.getY() + point.getZ())
          ? null
          : point + " must be a point, numbers all";
    }
  }
}
