package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.atMost;
import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Value;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.PerUnit;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;

/**
 * How one joint is driven: its gains, its soft limits, its motion profile, its current limits, what
 * it does when told nothing, and how it finds where it is. Its block gives it, beside its motors
 * and their wiring ({@link Driven}): {@code .control(MechanismControl.position(30))}. What the
 * joint is (its gearing, its range, its masses) seeds the feedforward gains not given here. The
 * robot program turns the two into its motor controller's settings, a TalonFX's or a SPARK's.
 *
 * <p>Every value is held in the joint's own units, as the definition's are: radians for a revolute
 * joint, meters for a prismatic one, and seconds. A motor controller's units differ (Phoenix's
 * rotations of its rotor or its sensor); the program converts. Each value is given with its unit,
 * and converted: a profile in degrees per second, soft limits as a {@link Range}, bands in degrees
 * or inches, a roller's as a share of its speed. A value given with its unit remembers whether it's
 * an angle or a length, and the joint refuses one of the other kind. The gains may be plain
 * numbers, in the joint's units, as SysId and Pitwall give them: {@code position(30)} is 30 volts
 * per radian on an arm, per meter on an elevator; one per turn as Phoenix's slot holds it is given
 * with its unit, {@code position(Volts.per(Rotation).of(188))}.
 *
 * @param loop what its loop holds, which its factory says: positions ({@link #position(double)}),
 *     an arm or an elevator, or speeds ({@link #velocity(double)}), a roller or a flywheel. The
 *     definition refuses the other loop than its joint's, since its kP means something else there
 * @param gains the slot's feedback gains, and its feedforward gains where tuned
 * @param softLimits where the motor controller stops driving the joint, inside its hard stops;
 *     empty for none
 * @param profile how fast its moves go, profiled on the robot; empty for a quarter of its free
 *     speed
 * @param currentLimits its motors' current limits
 * @param neutralMode what its motors do when told nothing: hold (brake) or coast
 * @param bands how near a target the joint must come to be there, and how far it may then stray;
 *     empty for the template's: 2° and 4°, 1 cm and 2 cm, or 3% and 6% of a speed
 * @param homing how the joint finds where it is once the robot is on; empty for its absolute
 *     encoder, which the definition must have on it
 * @param deadChannel when one of its motor controllers reads as drawing nothing on its PDH channel
 *     while the joint works: a tripped breaker, unplugged power, or a channel its block names wrong
 * @param encoderGap how far its absolute encoder and its motor's count may disagree before one of
 *     them is taken to have slipped; empty for the template's, worked out from the encoder
 */
@ConfigRecord
public record MechanismControl(
    Loop loop,
    Gains gains,
    Optional<SoftLimits> softLimits,
    Optional<Profile> profile,
    CurrentLimits currentLimits,
    NeutralMode neutralMode,
    Optional<Bands> bands,
    Optional<Homing> homing,
    DeadChannel deadChannel,
    Optional<EncoderGap> encoderGap) {
  /** Starts describing how a mechanism is driven, one named value at a time. */
  public static MechanismControlBuilder.LoopStage builder() {
    return MechanismControlBuilder.builder();
  }

  /**
   * A joint held at positions, an arm or an elevator, in a line: feedback {@code kP}, the
   * feedforward seeded from the definition, no profile (so a move goes at a quarter of its free
   * speed until it has one), the default current limits, and braking. Change any one value after:
   * {@code MechanismControl.position(30).withProfile(DegreesPerSecond.of(450),
   * DegreesPerSecondPerSecond.of(1700))}.
   *
   * @param kP volts per radian (or meter) of error: a kP per turn, as Phoenix's slot holds it, is
   *     2π times this; give that one with its unit
   */
  public static MechanismControl position(double kP) {
    return defaults(Loop.POSITION, kP, Optional.empty());
  }

  /**
   * The same, its kP given with its unit: per turn of the joint as Phoenix's slot holds it, {@code
   * position(Volts.per(Rotation).of(188))}, or per length for a joint that slides, {@code
   * position(Volts.per(Inch).of(30))}. A Phoenix kP per turn of the motor's rotor is its gearing
   * times that per turn of the joint; an elevator's per turn of its drum is that per its drum's
   * circumference.
   *
   * @throws IllegalArgumentException for a kP per anything but an angle or a length
   */
  public static MechanismControl position(Measure<? extends PerUnit<VoltageUnit, ?>> kP) {
    return defaults(
        Loop.POSITION, kP.baseUnitMagnitude(), Optional.of(per("kP", kP, POSITION, null)));
  }

  /**
   * A joint run at speeds, a roller or a flywheel, in a line: feedback {@code kP}, the feedforward
   * seeded from the definition, the default current limits, and braking, so a roller stops a game
   * piece where it is when told nothing. A flywheel coasts instead: {@code
   * MechanismControl.velocity(0.02).coasting()}.
   *
   * @param kP volts per radian (or meter) per second of error: a kP per turn per second, as
   *     Phoenix's slot holds it, is 2π times this; give that one with its unit
   */
  public static MechanismControl velocity(double kP) {
    return defaults(Loop.VELOCITY, kP, Optional.empty());
  }

  /**
   * The same, its kP given with its unit: per turn per second as Phoenix's slot holds it, {@code
   * velocity(Volts.per(RotationsPerSecond).of(0.12))}, or per speed along a joint that slides,
   * {@code Volts.per(MetersPerSecond)}.
   *
   * @throws IllegalArgumentException for a kP per anything but a speed
   */
  public static MechanismControl velocity(Measure<? extends PerUnit<VoltageUnit, ?>> kP) {
    return defaults(Loop.VELOCITY, kP.baseUnitMagnitude(), Optional.of(per("kP", kP, SPEED, null)));
  }

  /** The same, coasting when told nothing: a flywheel spinning down rather than stopping hard. */
  public MechanismControl coasting() {
    return MechanismControlBuilder.builder(this).neutralMode(NeutralMode.COAST).build();
  }

  /**
   * The same, a joint that turns profiled, its moves at most this fast and this quick to get there:
   * {@code withProfile(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(1700))}, or in turns
   * per second as Phoenix's are.
   */
  public MechanismControl withProfile(AngularVelocity cruise, AngularAcceleration acceleration) {
    return profiled(
        cruise.in(RadiansPerSecond),
        acceleration.in(RadiansPerSecondPerSecond),
        Optional.of(Position.Kind.ANGLE));
  }

  /**
   * The same, a joint that slides profiled: {@code withProfile(InchesPerSecond.of(60),
   * InchesPerSecondPerSecond.of(240))}.
   */
  public MechanismControl withProfile(LinearVelocity cruise, LinearAcceleration acceleration) {
    return profiled(
        cruise.in(MetersPerSecond),
        acceleration.in(MetersPerSecondPerSecond),
        Optional.of(Position.Kind.LENGTH));
  }

  /**
   * The same, with another proportional gain, in its loop's units: volts per radian (or meter) of
   * error for a joint held at positions, per radian (or meter) per second for one run at speeds. A
   * practice robot's own, {@code retuned(control -> control.withKp(25))}.
   */
  public MechanismControl withKp(double kP) {
    return with(gainsWith().kP(kP).build());
  }

  /**
   * The same, its proportional gain given with its unit: per angle or length for a joint held at
   * positions, {@code withKp(Volts.per(Rotation).of(150))}, as Phoenix's slot holds it; per speed
   * for one run at speeds, {@code withKp(Volts.per(RotationsPerSecond).of(0.12))}.
   *
   * @throws IllegalArgumentException for a kP per anything else than its loop's, or per the other
   *     kind than its other gains
   */
  public MechanismControl withKp(Measure<? extends PerUnit<VoltageUnit, ?>> kP) {
    int takes = loop == Loop.POSITION ? POSITION : SPEED;
    return with(gainsWith().kP(kP.baseUnitMagnitude()).kind(kind("kP", kP, takes, loop)).build());
  }

  /** The same, with an integral gain: volts per radian (or meter) of error, per second it lasts. */
  public MechanismControl withKi(double kI) {
    return with(gainsWith().kI(kI).build());
  }

  /**
   * The same, with a derivative gain: volts per radian (or meter) per second of error, for a joint
   * held at positions; per radian (or meter) per second squared, for one run at speeds.
   */
  public MechanismControl withKd(double kD) {
    return with(gainsWith().kD(kD).build());
  }

  /**
   * The same, its derivative gain given with its unit: per speed for a joint held at positions,
   * {@code withKd(Volts.per(RotationsPerSecond).of(2))}, as Phoenix's slot holds it; per
   * acceleration for one run at speeds, {@code Volts.per(RotationsPerSecondPerSecond)}.
   *
   * @throws IllegalArgumentException for a kD per anything else than its loop's, or per the other
   *     kind than its other gains
   */
  public MechanismControl withKd(Measure<? extends PerUnit<VoltageUnit, ?>> kD) {
    int takes = loop == Loop.POSITION ? SPEED : ACCELERATION;
    return with(gainsWith().kD(kD.baseUnitMagnitude()).kind(kind("kD", kD, takes, loop)).build());
  }

  /** The same, with SysId's kS: the volts that overcome static friction. */
  public MechanismControl withKs(double kS) {
    return with(gainsWith().kS(kS).build());
  }

  /**
   * The same, with SysId's kV, in place of the definition's seed: volts per radian (or meter) per
   * second. SysId run on Phoenix's signals gives it per turn per second: give that one with its
   * unit.
   */
  public MechanismControl withKv(double kV) {
    return with(gainsWith().kV(kV).build());
  }

  /**
   * The same, with SysId's kV given with its unit: {@code
   * withKv(Volts.per(RotationsPerSecond).of(0.12))}, as SysId gives it on Phoenix's signals.
   *
   * @throws IllegalArgumentException for a kV per anything but a speed, or per the other kind than
   *     its other gains
   */
  public MechanismControl withKv(Measure<? extends PerUnit<VoltageUnit, ?>> kV) {
    return with(gainsWith().kV(kV.baseUnitMagnitude()).kind(kind("kV", kV, SPEED, null)).build());
  }

  /**
   * The same, with SysId's kA, in place of the definition's seed: volts per radian (or meter) per
   * second squared. SysId run on Phoenix's signals gives it per turn per second squared: give that
   * one with its unit.
   */
  public MechanismControl withKa(double kA) {
    return with(gainsWith().kA(kA).build());
  }

  /**
   * The same, with SysId's kA given with its unit: {@code
   * withKa(Volts.per(RotationsPerSecondPerSecond).of(0.01))}.
   *
   * @throws IllegalArgumentException for a kA per anything but an acceleration, or per the other
   *     kind than its other gains
   */
  public MechanismControl withKa(Measure<? extends PerUnit<VoltageUnit, ?>> kA) {
    return with(
        gainsWith().kA(kA.baseUnitMagnitude()).kind(kind("kA", kA, ACCELERATION, null)).build());
  }

  /**
   * The same, holding it against gravity with this many volts, as tuned: an elevator everywhere, an
   * arm where its weight pulls hardest, as its control says, or its links' mass puts it, or, with
   * neither, at its zero. 0 for a joint a counterbalance holds.
   */
  public MechanismControl withKg(double kG) {
    return with(gainsWith().kG(kG).build());
  }

  /**
   * The same, holding an arm against gravity with this many volts, as tuned, its weight pulling
   * hardest at this angle: {@code withKg(0.4, Degrees.of(-90))} for an arm whose zero points
   * straight up, its positive direction tipping it forward.
   */
  public MechanismControl withKg(double kG, Angle gravityAngle) {
    return with(gainsWith().kG(kG).gravityAngle(gravityAngle).build());
  }

  /**
   * The same, the motor controller stopping it inside its hard stops, as its range is written:
   * {@code withSoftLimits(Range.degrees(5, 130))}.
   */
  public MechanismControl withSoftLimits(Range limits) {
    return limited(limits.lower(), limits.upper(), Optional.of(limits.kind()));
  }

  /** The same, its motors' torque capped at this current. */
  public MechanismControl withStatorLimit(Current stator) {
    return MechanismControlBuilder.builder(this)
        .currentLimits(
            MechanismControlCurrentLimitsBuilder.builder(currentLimits).stator(stator).build())
        .build();
  }

  /**
   * The same, what its motors draw from the battery capped at this current, for at most its {@code
   * supplyLowerTime} at a time, then at its {@code supplyLower} (as {@link CurrentLimits} says);
   * below that, at this always.
   */
  public MechanismControl withSupplyLimit(Current supply) {
    return MechanismControlBuilder.builder(this)
        .currentLimits(
            MechanismControlCurrentLimitsBuilder.builder(currentLimits).supply(supply).build())
        .build();
  }

  /**
   * The same, a joint run at speeds, a roller or a flywheel, there once inside {@code enter} of the
   * speed it's asked for, and still while inside {@code stay}, each a share of that speed: {@code
   * withBands(Percent.of(3), Percent.of(6))}.
   */
  public MechanismControl withBands(Dimensionless enter, Dimensionless stay) {
    return banded(enter.in(Value), stay.in(Value), Optional.empty());
  }

  /**
   * The same, a joint that turns held at positions, there once inside {@code enter} of a target,
   * and still while inside {@code stay}: {@code withBands(Degrees.of(1), Degrees.of(2))}.
   */
  public MechanismControl withBands(Angle enter, Angle stay) {
    return banded(enter.in(Radians), stay.in(Radians), Optional.of(Position.Kind.ANGLE));
  }

  /**
   * The same, a joint that slides there within these lengths: {@code withBands(Inches.of(0.5),
   * Inches.of(1))}.
   */
  public MechanismControl withBands(Distance enter, Distance stay) {
    return banded(enter.in(Meters), stay.in(Meters), Optional.of(Position.Kind.LENGTH));
  }

  /** The same, finding where it is by this recipe once the robot is on. */
  public MechanismControl withHoming(Homing homing) {
    return MechanismControlBuilder.builder(this).homing(homing).build();
  }

  /**
   * The same, a motor controller flagged as drawing nothing once, for longer than {@code time}, its
   * joint's leading controller draws more than {@code drawing} and its PDH channel reads under
   * {@code dead}: {@code withDeadChannel(Amps.of(2), Amps.of(0.5), Seconds.of(0.5))} is the
   * template's.
   */
  public MechanismControl withDeadChannel(Current drawing, Current dead, Time time) {
    return MechanismControlBuilder.builder(this)
        .deadChannel(DeadChannel.builder().drawing(drawing).dead(dead).time(time).build())
        .build();
  }

  /**
   * The same, its absolute encoder and its motor's count trusted to disagree by up to this, still,
   * before one of them is taken to have slipped: {@code withEncoderGap(Degrees.of(8))}, an arm
   * whose chain gives a lot. Without it, the template's: 5°, or 2 cm on a joint that slides, or
   * eight of the encoder's counts at the joint where that's more.
   */
  public MechanismControl withEncoderGap(Angle gap) {
    return gapped(gap.in(Radians), Position.Kind.ANGLE);
  }

  /** The same, a joint that slides: {@code withEncoderGap(Inches.of(1))}. */
  public MechanismControl withEncoderGap(Distance gap) {
    return gapped(gap.in(Meters), Position.Kind.LENGTH);
  }

  private MechanismControl gapped(double margin, Position.Kind kind) {
    return MechanismControlBuilder.builder(this)
        .encoderGap(EncoderGap.builder().margin(margin).kind(kind).build())
        .build();
  }

  /**
   * Why its values given with their units don't fit a joint of this kind, one reason for each, or
   * null when they do: an elevator's soft limits in degrees, an arm's kP per inch.
   */
  @Nullable String ofTheOtherKind(Position.Kind joint) {
    boolean turns = joint == Position.Kind.ANGLE;
    List<String> wrong = new ArrayList<>();
    if (gains.kind().isPresent() && gains.kind().get() != joint) {
      wrong.add(
          turns
              ? "its gains are per length, and the joint turns: give them per angle,"
                  + " Volts.per(Rotation)"
              : "its gains are per angle, and the joint slides: give them per length,"
                  + " Volts.per(Meter) or Volts.per(Inch); a gain per turn of its drum is that per"
                  + " the drum's circumference");
    }
    if (softLimits.flatMap(SoftLimits::kind).filter(kind -> kind != joint).isPresent()) {
      wrong.add(
          turns
              ? "its soft limits are lengths, and the joint turns: give them in angles,"
                  + " Range.degrees(...)"
              : "its soft limits are angles, and the joint slides: give them in lengths,"
                  + " Range.meters(...) or Range.of(Inches.of(...), Inches.of(...))");
    }
    if (profile.flatMap(Profile::kind).filter(kind -> kind != joint).isPresent()) {
      wrong.add(
          turns
              ? "its profile is in lengths, and the joint turns: give it in angles,"
                  + " DegreesPerSecond.of(...) and DegreesPerSecondPerSecond.of(...)"
              : "its profile is in angles, and the joint slides: give it in lengths,"
                  + " InchesPerSecond.of(...) and InchesPerSecondPerSecond.of(...)");
    }
    if (bands.flatMap(Bands::kind).filter(kind -> kind != joint).isPresent()) {
      wrong.add(
          turns
              ? "its bands are lengths, and the joint turns: give them in angles, Degrees.of(...)"
              : "its bands are angles, and the joint slides: give them in lengths,"
                  + " Inches.of(...)");
    }
    if (encoderGap.filter(gap -> gap.kind() != joint).isPresent()) {
      wrong.add(
          turns
              ? "its encoder gap is a length, and the joint turns: give it in an angle,"
                  + " withEncoderGap(Degrees.of(...))"
              : "its encoder gap is an angle, and the joint slides: give it in a length,"
                  + " withEncoderGap(Inches.of(...))");
    }
    return wrong.isEmpty() ? null : String.join("; ", wrong);
  }

  private static MechanismControl defaults(Loop loop, double kP, Optional<Position.Kind> per) {
    return builder()
        .loop(loop)
        .gains(MechanismControlGainsBuilder.builder(Gains.seeded()).kP(kP).kind(per).build())
        .currentLimits(CurrentLimits.DEFAULTS)
        .neutralMode(NeutralMode.BRAKE)
        .deadChannel(DeadChannel.DEFAULTS)
        .build();
  }

  private MechanismControlGainsBuilder gainsWith() {
    return MechanismControlGainsBuilder.builder(gains);
  }

  private MechanismControl with(Gains changed) {
    return MechanismControlBuilder.builder(this).gains(changed).build();
  }

  private MechanismControl profiled(
      double cruiseVelocity, double acceleration, Optional<Position.Kind> kind) {
    return MechanismControlBuilder.builder(this)
        .profile(
            Profile.builder()
                .cruiseVelocity(cruiseVelocity)
                .acceleration(acceleration)
                .kind(kind)
                .build())
        .build();
  }

  private MechanismControl limited(double lower, double upper, Optional<Position.Kind> kind) {
    return MechanismControlBuilder.builder(this)
        .softLimits(SoftLimits.builder().lower(lower).upper(upper).kind(kind).build())
        .build();
  }

  private MechanismControl banded(double enter, double stay, Optional<Position.Kind> kind) {
    return MechanismControlBuilder.builder(this)
        .bands(Bands.builder().enter(enter).stay(stay).kind(kind).build())
        .build();
  }

  /** A gain per a position: an angle or a length. */
  private static final int POSITION = 0;

  /** A gain per a speed. */
  private static final int SPEED = 1;

  /** A gain per an acceleration. */
  private static final int ACCELERATION = 2;

  /** What a gain may be per, a position, a speed or an acceleration, and how to write each. */
  private static final List<String> TAKES =
      List.of(
          "an angle, Volts.per(Rotation), or a length, Volts.per(Meter)",
          "a speed, Volts.per(RotationsPerSecond) or Volts.per(MetersPerSecond)",
          "an acceleration, Volts.per(RotationsPerSecondPerSecond) or"
              + " Volts.per(MetersPerSecondPerSecond)");

  /**
   * What a gain given with its unit is per: an angle or a length, and how many times per second
   * ({@link #POSITION}, {@link #SPEED}, {@link #ACCELERATION}).
   */
  private record Per(Position.Kind kind, int perSecond) {}

  /** What a unit a gain is per is, or null for one no gain is per: a time, a current. */
  private static @Nullable Per per(Unit unit) {
    return switch (unit) {
      case AngleUnit _ -> new Per(Position.Kind.ANGLE, POSITION);
      case DistanceUnit _ -> new Per(Position.Kind.LENGTH, POSITION);
      case AngularVelocityUnit _ -> new Per(Position.Kind.ANGLE, SPEED);
      case LinearVelocityUnit _ -> new Per(Position.Kind.LENGTH, SPEED);
      case AngularAccelerationUnit _ -> new Per(Position.Kind.ANGLE, ACCELERATION);
      case LinearAccelerationUnit _ -> new Per(Position.Kind.LENGTH, ACCELERATION);
      default -> null;
    };
  }

  /**
   * What a gain given with its unit is per, an angle or a length: refused unless it's per what the
   * gain takes, a position, a speed or an acceleration ({@code takes}).
   *
   * @param loop the loop that decides what it takes, said in the refusal; null when the gain takes
   *     the same in either
   */
  private static Position.Kind per(
      String gain,
      Measure<? extends PerUnit<VoltageUnit, ?>> value,
      int takes,
      @Nullable Loop loop) {
    Unit unit = value.unit().denominator();
    Per per = per(unit);
    if (per != null && per.perSecond() == takes) {
      return per.kind();
    }
    throw new IllegalArgumentException(
        "Gains."
            + gain
            + ": volts per "
            + unit.name()
            + " can't be one"
            + (loop == null ? "" : " in a control for " + loop.held())
            + ": give it per "
            + TAKES.get(takes));
  }

  /**
   * The gains' kind with one more gain given with its unit: refused if it's per the other kind than
   * the gains already are.
   */
  private Optional<Position.Kind> kind(
      String gain,
      Measure<? extends PerUnit<VoltageUnit, ?>> value,
      int takes,
      @Nullable Loop loop) {
    Position.Kind kind = per(gain, value, takes, loop);
    if (gains.kind().isPresent() && gains.kind().get() != kind) {
      boolean angle = kind == Position.Kind.ANGLE;
      throw new IllegalArgumentException(
          "Gains."
              + gain
              + ": it's per "
              + (angle ? "an angle" : "a length")
              + ", and the other gains are per "
              + (angle ? "a length" : "an angle")
              + ": a control drives one joint, which turns or slides");
    }
    return Optional.of(kind);
  }

  /**
   * What a joint's loop holds: where it is, or how fast it runs. Its control's factory says it,
   * {@code position(kP)} or {@code velocity(kP)}, and its joint must agree: one held at positions
   * (with a range or named positions) runs a position loop, and a roller a velocity loop.
   */
  public enum Loop {
    /** Positions: an arm, an elevator, a turret. Its kP is volts per radian (or meter) of error. */
    POSITION,
    /** Speeds: a roller, a flywheel. Its kP is volts per radian (or meter) per second of error. */
    VELOCITY;

    /** What it holds, as a message says it: "positions" or "speeds". */
    String held() {
      return this == POSITION ? "positions" : "speeds";
    }
  }

  /** What the motors do when told nothing. */
  public enum NeutralMode {
    /** Short the windings: the mechanism holds where it is. */
    BRAKE,
    /** Let it turn freely. */
    COAST
  }

  /**
   * The slot's gains, in volts per the joint's units. The feedback gains are always the team's. The
   * feedforward gains the definition can derive (kV, kA, kG) are seeded from it until tuned: one
   * given here always wins over the seed, and is compared with it.
   *
   * @param kP volts per radian (or meter) of error
   * @param kI volts per radian (or meter) of error, per second it lasts
   * @param kD volts per radian (or meter) per second of error
   * @param kS volts to overcome static friction: measured, never seeded
   * @param kV volts per radian (or meter) per second, where tuned
   * @param kA volts per radian (or meter) per second squared, where tuned
   * @param kG volts to hold the joint against gravity, where tuned: for a revolute joint, at its
   *     worst, positive to hold it up whichever way it's turned, as the seed is; for a prismatic
   *     one, everywhere. It needs no mass: the definition's mass only seeds it, and a link's mass
   *     is optional
   * @param gravityAngle where a revolute joint's weight pulls hardest, the angle at which holding
   *     it takes all of kG forward; where not given, where its links' mass puts it ({@code
   *     RobotMath.gravityOffset}), or its zero, horizontal as WPILib takes an arm's, with no mass.
   *     A prismatic joint has none: it's held the same all along
   * @param kind what its gains given with their units are per: an angle ({@code
   *     Volts.per(Rotation)}) or a length ({@code Volts.per(Meter)}), which the joint must be;
   *     empty when each is a plain number, in the joint's units
   */
  @ConfigRecord
  public record Gains(
      double kP,
      double kI,
      double kD,
      double kS,
      Optional<Double> kV,
      Optional<Double> kA,
      Optional<Double> kG,
      Optional<Angle> gravityAngle,
      Optional<Position.Kind> kind) {
    public Gains {
      Checks.on(Gains.class)
          .field("kP", kP, nonNegative())
          .field("kI", kI, nonNegative())
          .field("kD", kD, nonNegative())
          .field("kS", kS, nonNegative())
          .field("kV", kV, given(nonNegative()))
          .field("kA", kA, given(nonNegative()))
          .field("kG", kG, given(finite()))
          .field("gravityAngle", gravityAngle, given(finite()))
          .orThrow();
    }

    /** Starts describing the gains, one named value at a time. */
    public static MechanismControlGainsBuilder.KPStage builder() {
      return MechanismControlGainsBuilder.builder();
    }

    /** No feedback, no friction, and every feedforward gain the definition's seed. */
    public static Gains seeded() {
      return builder().kP(0).kI(0).kD(0).kS(0).build();
    }
  }

  /**
   * Where the motor controller stops driving the joint, in the joint's units: inside its hard stops
   * ({@code Joint.range}), which the helper checks. They hold only once the motor controller knows
   * where the joint is: from an absolute encoder, or after homing.
   *
   * @param lower the lowest position it drives the joint to
   * @param upper the highest
   * @param kind angles or lengths, as they were given with their units ({@code Range.degrees}),
   *     which the joint must be; empty only when built by name, in the joint's units
   */
  @ConfigRecord
  public record SoftLimits(double lower, double upper, Optional<Position.Kind> kind) {
    public SoftLimits {
      Checks.on(SoftLimits.class)
          .field("lower", lower, finite())
          .field("upper", upper, finite())
          .check(
              "upper",
              () ->
                  upper > lower
                      ? null
                      : shown(upper) + " must be above the lower limit (" + shown(lower) + ")")
          .orThrow();
    }

    /** Starts describing soft limits, one named value at a time. */
    public static MechanismControlSoftLimitsBuilder.LowerStage builder() {
      return MechanismControlSoftLimitsBuilder.builder();
    }
  }

  /**
   * How fast a joint's moves go, in its units: each is profiled on the robot (WPILib's {@code
   * TrapezoidProfile}), from where the joint is to where it's going. The definition refuses a
   * cruise faster than the joint's motors turn it.
   *
   * @param cruiseVelocity the fastest the profile moves it, per second
   * @param acceleration how fast the profile speeds it up and slows it down, per second squared
   * @param kind angles or lengths, as they were given with their units ({@code DegreesPerSecond}),
   *     which the joint must be; empty only when built by name, in the joint's units
   */
  @ConfigRecord
  public record Profile(double cruiseVelocity, double acceleration, Optional<Position.Kind> kind) {
    public Profile {
      Checks.on(Profile.class)
          .field("cruiseVelocity", cruiseVelocity, positive())
          .field("acceleration", acceleration, positive())
          .orThrow();
    }

    /** Starts describing a profile, one named value at a time. */
    public static MechanismControlProfileBuilder.CruiseVelocityStage builder() {
      return MechanismControlProfileBuilder.builder();
    }
  }

  /**
   * Each motor's current limits, all set where they show: Phoenix limits both currents, and steps
   * the supply limit down, even when the code never says so (120 A stator, 70 A supply stepping
   * down to 40 A after 1 s).
   *
   * @param stator the most current through the motor, which caps its torque
   * @param supply the most it draws from the battery, for at most {@code supplyLowerTime} at a time
   * @param supplyLower its supply limit after that, until it draws less again: no more than its
   *     breaker's rating
   * @param supplyLowerTime how long it may draw {@code supply} before the limit steps down
   */
  @ConfigRecord
  public record CurrentLimits(
      Current stator, Current supply, Current supplyLower, Time supplyLowerTime) {
    /** What {@link #position} and {@link #velocity} start from: 40 A, 60 A, then 40 A after 1 s. */
    public static final CurrentLimits DEFAULTS =
        builder()
            .stator(Amps.of(40))
            .supply(Amps.of(60))
            .supplyLower(Amps.of(40))
            .supplyLowerTime(Seconds.of(1))
            .build();

    public CurrentLimits {
      Checks.on(CurrentLimits.class)
          .field("stator", stator, positive())
          .field("supply", supply, positive())
          .field("supplyLower", supplyLower, positive())
          // Phoenix reads a zero time as "never step down": to mean that, set supplyLower to
          // supply.
          .field("supplyLowerTime", supplyLowerTime, positive())
          .field("supplyLowerTime", supplyLowerTime, atMost(Seconds.of(5)))
          .orThrow();
    }

    /** Starts describing the current limits, one named value at a time. */
    public static MechanismControlCurrentLimitsBuilder.StatorStage builder() {
      return MechanismControlCurrentLimitsBuilder.builder();
    }
  }

  /**
   * How near a target a joint must come to be there ({@code enter}), and how far it may then stray
   * and still be ({@code stay}): tight to arrive, looser to stay, so a reading that wavers at the
   * edge doesn't flicker. For a joint held at positions, in its units, radians or meters; for one
   * that only runs at outputs, a roller, as shares of the speed it's asked for.
   *
   * @param enter the most it may be off to come to be there
   * @param stay the most it may be off to stay there: at least {@code enter}
   * @param kind angles or lengths, as they were given with their units ({@code Degrees.of}), which
   *     the joint must be, held at positions; empty for shares of a speed ({@code Percent.of})
   */
  @ConfigRecord
  public record Bands(double enter, double stay, Optional<Position.Kind> kind) {
    public Bands {
      if (!(enter > 0) || !Double.isFinite(enter)) {
        throw new IllegalArgumentException(
            "a band to arrive within is more than none, not " + shownBand(enter, kind));
      }
      if (!(stay >= enter) || !Double.isFinite(stay)) {
        throw new IllegalArgumentException(
            "a band to stay within, "
                + shownBand(stay, kind)
                + ", is at least the band to arrive within, "
                + shownBand(enter, kind)
                + ": give "
                + shownBand(enter, kind)
                + " or more, so it leaves only past where it arrived");
      }
    }

    /** A band as the team gave it: degrees, meters, or a share of a speed. */
    private static String shownBand(double band, Optional<Position.Kind> kind) {
      return kind.isEmpty()
          ? shown(band * 100) + "%"
          : Places.shownAt(kind.get() == Position.Kind.ANGLE, band);
    }

    /** Starts describing bands, one named value at a time. */
    public static MechanismControlBandsBuilder.EnterStage builder() {
      return MechanismControlBandsBuilder.builder();
    }
  }

  /**
   * How far a joint's absolute encoder and its motor's count may disagree, still, before one of
   * them is taken to have slipped: a magnet loose on its shaft, a belt that skipped. Past it for a
   * quarter second, more a loop's travel at the joint's speed (the two aren't read at the same
   * instant), where the joint is isn't known, and it's faulted until they agree again. In the
   * joint's units, radians or meters, as given with its unit.
   *
   * @param margin the most they may disagree, still
   * @param kind an angle or a length, as it was given with its unit, which the joint must be
   */
  @ConfigRecord
  public record EncoderGap(double margin, Position.Kind kind) {
    public EncoderGap {
      if (!(margin > 0) || !Double.isFinite(margin)) {
        throw new IllegalArgumentException(
            "an encoder gap is more than none, not "
                + Places.shownAt(kind == Position.Kind.ANGLE, margin));
      }
    }

    /** Starts describing an encoder gap, one named value at a time. */
    public static MechanismControlEncoderGapBuilder.MarginStage builder() {
      return MechanismControlEncoderGapBuilder.builder();
    }
  }

  /**
   * When one of a joint's motor controllers reads as drawing nothing on its PDH channel while the
   * joint works: its breaker tripped, its power unplugged, or its block naming another channel than
   * the one it's on. Each of a joint's motors drives the same load, so while its leading controller
   * draws from the battery, each draws about as much on its own channel. A controller whose channel
   * reads next to nothing meanwhile, for long enough, is flagged. A joint asked for volts at a
   * steady speed with no load draws next to nothing, alive or not, so working is what its leader
   * draws, not what it's asked: in the simulator, which models no friction, nothing at all. Only a
   * controller whose block says its channel is checked, and only while the PDH answers.
   *
   * @param drawing what the joint's leading controller draws from the battery, by its own reading,
   *     above which each of its motors draws too
   * @param dead what a channel reads under, drawing nothing: no less than the PDH's resolution
   * @param time how long both must hold before it's flagged
   */
  @ConfigRecord
  public record DeadChannel(Current drawing, Current dead, Time time) {
    /** The template's: 2 A drawing, under 0.5 A on the channel, for half a second. */
    public static final DeadChannel DEFAULTS =
        builder().drawing(Amps.of(2)).dead(Amps.of(0.5)).time(Seconds.of(0.5)).build();

    public DeadChannel {
      Checks.on(DeadChannel.class)
          .field("drawing", drawing, positive())
          .field("dead", dead, positive())
          .field("time", time, positive())
          .check(
              "dead",
              () ->
                  dead.lt(drawing)
                      ? null
                      : shown(dead)
                          + " must be under what the joint's leader draws, "
                          + shown(drawing)
                          + ": a motor drawing as much as its leader isn't dead")
          .orThrow();
    }

    /** Starts describing when a channel is dead, one named value at a time. */
    public static MechanismControlDeadChannelBuilder.DrawingStage builder() {
      return MechanismControlDeadChannelBuilder.builder();
    }
  }

  /**
   * How a joint read by its motor alone finds where it is: the motor counts turns from wherever it
   * was at power on, so until then the joint's position is a guess. A joint with an absolute
   * encoder needs none; any other joint held at positions names one, so trusting the start is a
   * choice written down ({@link #assumeAtRest}), never a silent one.
   */
  public sealed interface Homing permits Homing.AssumeAtRest, Homing.ToHardStop, Homing.ToSwitch {
    /** The joint is where its definition rests it whenever the robot powers on: stowed by hand. */
    static Homing assumeAtRest() {
      return new AssumeAtRest();
    }

    /**
     * On the first enable, the joint is pushed gently into one of its hard stops, and its position
     * there is the stop's: 2 V, for as long as its model says crossing its range takes at that
     * push, twice over and half a second more ({@link #secondsAllowed}).
     */
    static Homing toHardStop(Sensor.Switch.Limit.End end) {
      return ToHardStop.builder().end(end).volts(2).build();
    }

    /**
     * The same, for at most this long: for a joint slower than its model says, such as one pushed
     * up against its weight.
     */
    static Homing toHardStop(Sensor.Switch.Limit.End end, Time timeout) {
      return ToHardStop.builder().end(end).volts(2).timeout(timeout).build();
    }

    /**
     * On the first enable, the joint is driven gently toward its limit switch at one end of its
     * travel, and its position, once the switch trips, is the switch's: 2 V, for as long as its
     * model says crossing its range takes at that push, twice over and half a second more ({@link
     * #secondsAllowed}).
     */
    static Homing toSwitch(Sensor.Switch.Limit.End end) {
      return ToSwitch.builder().end(end).volts(2).build();
    }

    /** The same, for at most this long: for a joint slower than its model says. */
    static Homing toSwitch(Sensor.Switch.Limit.End end, Time timeout) {
      return ToSwitch.builder().end(end).volts(2).timeout(timeout).build();
    }

    /**
     * How long a joint takes to cross its travel at a push, from its model, in seconds: its range
     * at the speed the push gives it, its free speed through its gearing scaled by the push over
     * its motors' 12 V (the push over {@link RobotMath#idealGains}'s kV). A joint that turns freely
     * has no range: it reaches anywhere within a turn. A slide with no hard stops goes only between
     * its named places and where it rests.
     *
     * @param volts the push
     * @throws IllegalArgumentException if no motors move it, or it follows another joint
     */
    static double travelSeconds(RobotMath math, Joint joint, double volts) {
      double travel =
          joint
              .range()
              .map(Range::width)
              .orElseGet(() -> joint instanceof Joint.Revolute ? 2 * Math.PI : spread(joint));
      return travel * math.idealGains(joint).kV() / volts;
    }

    /**
     * How long homing may push a joint before it's faulted, in seconds: the timeout its recipe
     * gives, or, left to the joint, twice its travel's time at the push ({@link #travelSeconds})
     * and half a second more, as a move's default deadline is. Twice leaves room for friction, and
     * for gravity, to take half the push; the half second, for a hard stop's quarter second of
     * stillness.
     *
     * @param timeout the recipe's, or empty to work it out from the joint
     * @param volts the push
     */
    static double secondsAllowed(
        Optional<Time> timeout, double volts, RobotMath math, Joint joint) {
      return timeout
          .map(time -> time.in(Seconds))
          .orElseGet(() -> 2 * travelSeconds(math, joint, volts) + 0.5);
    }

    /** How far apart a slide's named places and where it rests lie: its travel, without stops. */
    private static double spread(Joint joint) {
      double lowest = joint.restPosition();
      double highest = joint.restPosition();
      for (Position place : joint.positions()) {
        lowest = Math.min(lowest, place.value());
        highest = Math.max(highest, place.value());
      }
      return highest - lowest;
    }

    /** The joint is where it rests at power on ({@link #assumeAtRest}). */
    record AssumeAtRest() implements Homing {}

    /**
     * Pushed into a hard stop on the first enable, the joint is there once it has stopped moving
     * for a quarter of a second; if it hasn't by the timeout, it's faulted, and its moves refused.
     *
     * @param end which end's hard stop
     * @param volts how hard it's pushed: enough to move it, against gravity if the stop is above
     * @param timeout how long it may take; empty to work it out from the joint ({@link
     *     Homing#secondsAllowed})
     */
    @ConfigRecord
    record ToHardStop(Sensor.Switch.Limit.End end, double volts, Optional<Time> timeout)
        implements Homing {
      public ToHardStop {
        Checks.on(ToHardStop.class)
            .field("volts", volts, positive())
            .field("volts", volts, atMost(Motor.NOMINAL_VOLTS))
            .field("timeout", timeout, given(positive()))
            .orThrow();
      }

      /** How long it may push a joint before the joint is faulted, in seconds. */
      public double secondsOn(RobotMath math, Joint joint) {
        return Homing.secondsAllowed(timeout, volts, math, joint);
      }

      /** Starts describing the homing, one named value at a time. */
      public static MechanismControlHomingToHardStopBuilder.EndStage builder() {
        return MechanismControlHomingToHardStopBuilder.builder();
      }
    }

    /**
     * Driven toward its limit switch at one end on the first enable, the joint is there the loop
     * the switch trips, at the switch's position in the definition; if it hasn't by the timeout,
     * it's faulted, and its moves refused. One resting on its switch backs off it first.
     *
     * @param end which end's limit switch: the definition's limit switch on the joint at that end
     * @param volts how hard it's driven: enough to move it, against gravity if the switch is above
     * @param timeout how long it may take; empty to work it out from the joint ({@link
     *     Homing#secondsAllowed})
     */
    @ConfigRecord
    record ToSwitch(Sensor.Switch.Limit.End end, double volts, Optional<Time> timeout)
        implements Homing {
      public ToSwitch {
        Checks.on(ToSwitch.class)
            .field("volts", volts, positive())
            .field("volts", volts, atMost(Motor.NOMINAL_VOLTS))
            .field("timeout", timeout, given(positive()))
            .orThrow();
      }

      /** How long it may drive a joint before the joint is faulted, in seconds. */
      public double secondsOn(RobotMath math, Joint joint) {
        return Homing.secondsAllowed(timeout, volts, math, joint);
      }

      /** Starts describing the homing, one named value at a time. */
      public static MechanismControlHomingToSwitchBuilder.EndStage builder() {
        return MechanismControlHomingToSwitchBuilder.builder();
      }
    }
  }
}
