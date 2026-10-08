package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Degrees;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * A sensor on the robot, as the physical device it is, and where: an encoder in a joint's gear
 * train, a switch, a rangefinder. Its raw reading is all it is; what it means to the robot ("coral
 * in claw") is a condition over it ({@link Condition.OnSensor}). Which port or CAN ID each is on,
 * which way a switch is wired (normally open or closed), and an absolute encoder's offset are the
 * robot's wiring, measured per install.
 */
public sealed interface Sensor permits Sensor.Encoder, Sensor.Placed {
  /** What logs call it: unique among the robot's sensors. */
  String label();

  /**
   * A sensor mounted on a part, which the 3D view draws and the simulator reads from where it is: a
   * switch or a rangefinder. An encoder is in a gear train, not anywhere in particular.
   */
  sealed interface Placed extends Sensor permits Switch, Rangefinder {
    /** What it's mounted on: the chassis, or a joint's link. */
    Part part();

    /** Where on its part, if known: unknown, it's at the part's origin. */
    Optional<Pose3d> pose();
  }

  /**
   * An encoder on a joint's gear train: on the joint itself, on a shaft between, or on a motor's
   * rotor, which a drive of motors implies ({@link Drive#rotor}) and needn't be written. It reads
   * one joint, or, on a shaft several joints turn, a sum of them, as a combined transmission turns
   * them ({@link Transmission.Combined}): a differential's motor's rotor reads its two sides
   * together.
   *
   * @param label what logs call it
   * @param terms each joint it reads, with its ratio: its turns for each of the joint's turns, or
   *     for each turn of a prismatic joint's drum (each meter, read straight off the slide); 1 on
   *     the joint, a geared encoder's teeth ratio between; negative when it turns back as the joint
   *     moves forward
   * @param absolute whether it knows where it is in its turn at power-up (a CANcoder, a
   *     through-bore's duty cycle), rather than counting from wherever it started. One that is
   *     reads one joint; a motor's rotor never is.
   * @param countsPerTurn how many readings its turn has, from its datasheet: a CANcoder's 4096.
   *     Only a pair of absolute encoders geared nearly alike, decoded by their difference, reads
   *     it, and needs each one's ({@code RobotMath.absoluteSpan}); none for one whose resolution
   *     nothing reads, such as a rotor's
   */
  @ConfigRecord
  record Encoder(
      String label,
      List<Transmission.Term> terms,
      boolean absolute,
      Optional<Integer> countsPerTurn)
      implements Sensor {
    public Encoder {
      terms = List.copyOf(terms);
      List<Transmission.Term> read = terms;
      Checks.on(Encoder.class, label)
          .field("label", label, notBlank())
          .field("terms", terms, Transmission.Combined::notTerms)
          .field("countsPerTurn", countsPerTurn, given(positive()))
          .check(
              "terms",
              () ->
                  absolute && read.size() > 1
                      ? "an absolute encoder reads one joint, not " + read.size()
                      : null)
          .orThrow();
    }

    /** Starts describing an encoder, one named value at a time. */
    public static SensorEncoderBuilder.LabelStage builder() {
      return SensorEncoderBuilder.builder();
    }

    /**
     * An absolute encoder on a revolute joint's axle, turning with it, of a resolution: what the
     * model's own tests write. A team's goes in its joint's block, {@code .absoluteEncoder(...)}.
     *
     * @param countsPerTurn its resolution, from its datasheet: a CANcoder's 4096
     */
    static Encoder absolute(String label, Joint.Revolute joint, int countsPerTurn) {
      return builder()
          .label(label)
          .terms(List.of(Transmission.Term.builder().joint(joint).ratio(1).build()))
          .absolute(true)
          .countsPerTurn(countsPerTurn)
          .build();
    }

    /** The joints it reads. */
    public List<Joint> joints() {
      return terms.stream().map(Transmission.Term::joint).toList();
    }

    /**
     * The one joint it reads.
     *
     * @throws IllegalStateException if it reads several: ask {@link #ratio} about each
     */
    public Joint joint() {
      if (terms.size() > 1) {
        throw new IllegalStateException(
            label + " reads " + terms.size() + " joints: ask its ratio on each");
      }
      return terms.get(0).joint();
    }

    /**
     * Its turns for each of a joint's turns, or each meter of a prismatic one, the others held; 0
     * for a joint it doesn't read.
     */
    public double ratio(Joint joint) {
      for (Transmission.Term term : terms) {
        if (term.joint().equals(joint)) {
          return term.perTurnOrMeter();
        }
      }
      return 0;
    }
  }

  /**
   * A switch: two states, tripped (pressed, its magnet there, its beam blocked) and clear. Never
   * closed and open, which on a normally closed switch mean the other way round: how it's wired is
   * the robot's wiring, and above it every switch reads tripped or clear. Its kind says what trips
   * it, which is what the model knows about it.
   *
   * @param label what logs call it
   * @param part what it's mounted on: the chassis, or a joint's link
   * @param pose where on its part, if known: for the 3D view and the simulator. A team rarely knows
   *     a beam's points, so none is needed: it's drawn at its part's origin, and the missing-data
   *     report says so
   * @param kind what trips it
   * @param wiring where it's connected, and which raw signal means tripped; none for the simulator
   *     alone
   */
  @ConfigRecord
  record Switch(
      String label,
      Part part,
      Optional<Pose3d> pose,
      Kind kind,
      Optional<SensorWiring.Connection> wiring)
      implements Placed {
    public Switch {
      Checks.on(Switch.class, label).field("label", label, notBlank()).orThrow();
    }

    /** Starts describing a switch, one named value at a time. */
    public static SensorSwitchBuilder.LabelStage builder() {
      return SensorSwitchBuilder.builder();
    }

    /**
     * A limit switch, or a hall sensor, at one end of a joint's travel: tripped with the joint at
     * or past its position. Mounted on the joint's parent, where it's pressed. What a joint's block
     * makes of its own ({@code .limitSwitch(End.LOWER, Degrees.of(0), ...)}), named for it.
     *
     * @param end which end of the joint's travel it marks
     * @param position where it trips, in the joint's units, if known: inside the joint's range
     */
    static Switch limit(String label, Joint joint, Limit.End end, double position) {
      return builder()
          .label(label)
          .part(joint.parent())
          .kind(SensorSwitchLimitBuilder.builder().joint(joint).end(end).position(position).build())
          .build();
    }

    /**
     * A limit switch whose position isn't known: it can stop the joint, as a hard limit, but not
     * home it, since nothing says where the joint is when it trips.
     */
    static Switch limit(String label, Joint joint, Limit.End end) {
      return builder()
          .label(label)
          .part(joint.parent())
          .kind(SensorSwitchLimitBuilder.builder().joint(joint).end(end).build())
          .build();
    }

    /** A beam break, through-beam or reflective, on a part: tripped by something in its beam. */
    public static Switch beam(String label, Part part) {
      return builder().label(label).part(part).kind(new Beam()).build();
    }

    /**
     * A switch nothing the model describes trips: a button on the robot, a cage sensor on a hook
     * the model doesn't have.
     */
    public static Switch contact(String label, Part part) {
      return builder().label(label).part(part).kind(new Contact()).build();
    }

    /** The same switch, placed on its part: for the 3D view and the simulator. */
    public Switch at(Pose3d where) {
      return SensorSwitchBuilder.builder(this).pose(where).build();
    }

    /**
     * The same switch, connected: on a CANdi's S1, {@code .on(CANDI.can(30).s1().normallyOpen())},
     * or on a Systemcore port, {@code .on(SensorWiring.port(1).normallyClosed())}. Unconnected, it
     * runs in the simulator alone.
     */
    public Switch on(SensorWiring.Connection connection) {
      return SensorSwitchBuilder.builder(this).wiring(connection).build();
    }

    /** Equal to another switch the same in everything but its wiring. */
    @Override
    public boolean equals(@Nullable Object other) {
      return other instanceof Switch that
          && label.equals(that.label)
          && part.equals(that.part)
          && pose.equals(that.pose)
          && kind.equals(that.kind);
    }

    @Override
    public int hashCode() {
      return Objects.hash(label, part, pose, kind);
    }

    /** Tripped: pressed, its beam blocked. */
    public Condition.Tripped tripped() {
      return new Condition.Tripped(this);
    }

    /** Clear: not pressed, its beam unbroken. */
    public Condition.Clear clear() {
      return new Condition.Clear(this);
    }

    /** What trips a switch. */
    public sealed interface Kind permits Limit, Beam, Contact {}

    /**
     * A joint at one end of its travel trips it: a contact switch its link presses, or a hall
     * sensor its magnet passes.
     *
     * @param joint the joint that trips it
     * @param position where it trips, in the joint's units, if known: inside its range
     * @param end which end of the joint's travel it marks: tripped at or below its position for the
     *     lower, at or above for the upper
     */
    @ConfigRecord
    public record Limit(Joint joint, Optional<Double> position, End end) implements Kind {
      public Limit {
        Checks.on(Limit.class, joint.label())
            .check(
                "position",
                () ->
                    position.isEmpty()
                            || joint.range().isEmpty()
                            || joint.range().get().contains(position.get())
                        ? null
                        : position.get()
                            + " must be inside "
                            + possessive(joint.label())
                            + " range")
            .orThrow();
      }

      /** Which end of a joint's travel. */
      public enum End {
        LOWER,
        UPPER
      }

      /** Whether it's tripped with its joint at a position: never, if its position isn't known. */
      public boolean trippedAt(double at) {
        return position.isPresent()
            && (end == End.LOWER ? at <= position.get() : at >= position.get());
      }
    }

    /** Something in its beam trips it: a beam break, through-beam or reflective. */
    public record Beam() implements Kind {}

    /** Nothing the model describes trips it. */
    public record Contact() implements Kind {}
  }

  /**
   * A rangefinder: the distance to the nearest thing along its axis, within its range and cone. A
   * CANrange, a LaserCAN, an ultrasonic. Used as "is something there", it's still a rangefinder,
   * and the threshold a condition ({@link #nearerThan}).
   *
   * @param label what logs call it
   * @param part what it's mounted on
   * @param pose where on its part, looking along its x, if known, as a switch's
   * @param range the farthest it reads
   * @param fieldOfView how wide its cone is, across
   * @param wiring the CANrange it is, or none for the simulator alone
   */
  @ConfigRecord
  record Rangefinder(
      String label,
      Part part,
      Optional<Pose3d> pose,
      Distance range,
      Angle fieldOfView,
      Optional<SensorWiring.CanRange> wiring)
      implements Placed {
    public Rangefinder {
      Checks.on(Rangefinder.class, label)
          .field("label", label, notBlank())
          .field("range", range, positive())
          .field("fieldOfView", fieldOfView, between(Degrees.of(0), Degrees.of(180)))
          .orThrow();
    }

    /** Starts describing a rangefinder, one named value at a time. */
    public static SensorRangefinderBuilder.LabelStage builder() {
      return SensorRangefinderBuilder.builder();
    }

    /** Something nearer than a distance, inside its range. */
    public Condition.Nearer nearerThan(Distance distance) {
      return new Condition.Nearer(this, distance);
    }

    /** Nothing nearer than a distance. */
    public Condition.Farther fartherThan(Distance distance) {
      return new Condition.Farther(this, distance);
    }

    /**
     * A rangefinder on a part, reading this far within a cone this wide: {@code
     * Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27))}, then {@code
     * .at(pose)} where it's known and {@code .on(CANRANGE.can(31))} once it's wired.
     */
    public static Rangefinder of(String label, Part part, Distance range, Angle fieldOfView) {
      return builder().label(label).part(part).range(range).fieldOfView(fieldOfView).build();
    }

    /** The same rangefinder, placed on its part, looking along its x. */
    public Rangefinder at(Pose3d where) {
      return SensorRangefinderBuilder.builder(this).pose(where).build();
    }

    /** The same rangefinder, on a CANrange. Unconnected, it runs in the simulator alone. */
    public Rangefinder on(SensorWiring.CanRange canRange) {
      return SensorRangefinderBuilder.builder(this).wiring(canRange).build();
    }

    /** Equal to another rangefinder the same in everything but its wiring. */
    @Override
    public boolean equals(@Nullable Object other) {
      return other instanceof Rangefinder that
          && label.equals(that.label)
          && part.equals(that.part)
          && pose.equals(that.pose)
          && range.equals(that.range)
          && fieldOfView.equals(that.fieldOfView);
    }

    @Override
    public int hashCode() {
      return Objects.hash(label, part, pose, range, fieldOfView);
    }
  }
}
