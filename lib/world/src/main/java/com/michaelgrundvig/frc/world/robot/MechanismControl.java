package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.atMost;
import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.config.Rule;
import java.util.Optional;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Time;

/**
 * How one mechanism is driven: its gains, its soft limits, its motion profile, its current limits,
 * and what it does when told nothing. What the mechanism is (its gearing, its range, its masses) is
 * the robot's definition, which seeds the feedforward gains not given here; how it's wired is its
 * {@code MechanismWiring}. {@link PhoenixMechanism} turns the three into Phoenix's settings.
 *
 * <p>Every value is in the joint's own units, as the definition's are: radians for a revolute
 * joint, meters for a prismatic one, and seconds. Phoenix's units differ ({@link PhoenixUnits});
 * the helper converts, so no gain is ever written per rotor turn or per degree here.
 *
 * @param gains the slot's feedback gains, and its feedforward gains where tuned
 * @param softLimits where the motor controller stops driving the joint, inside its hard stops;
 *     empty for none
 * @param profile Motion Magic's limits; empty for a mechanism that isn't profiled
 * @param currentLimits its motors' current limits
 * @param neutralMode what its motors do when told nothing: hold (brake) or coast
 * @param bands how near a target the joint must come to be there, and how far it may then stray;
 *     empty for the template's: 2° and 4°, 1 cm and 2 cm, or 3% and 6% of a speed
 * @param homing how the joint finds where it is once the robot is on; empty for its absolute
 *     encoder, which the definition must have on it
 */
@ConfigRecord
public record MechanismControl(
    Gains gains,
    Optional<SoftLimits> softLimits,
    Optional<Profile> profile,
    CurrentLimits currentLimits,
    NeutralMode neutralMode,
    Optional<Bands> bands,
    Optional<Homing> homing) {
  /** Starts describing how a mechanism is driven, one named value at a time. */
  public static MechanismControlBuilder.GainsStage builder() {
    return MechanismControlBuilder.builder();
  }

  /**
   * A joint held at positions, an arm or an elevator, in a line: feedback {@code kP}, the
   * feedforward seeded from the definition, no profile (so a move goes at a quarter of its free
   * speed until it has one), the default current limits, and braking. Change any one value after:
   * {@code MechanismControl.position(30).withProfile(8, 30)}.
   *
   * @param kP volts per radian (or meter) of error
   */
  public static MechanismControl position(double kP) {
    return defaults(kP, NeutralMode.BRAKE);
  }

  /**
   * A joint run at speeds, a roller or a flywheel, in a line: feedback {@code kP}, the feedforward
   * seeded from the definition, the default current limits, and braking, so a roller stops a game
   * piece where it is when told nothing. A flywheel coasts instead: {@code
   * MechanismControl.velocity(0.02).coasting()}.
   *
   * @param kP volts per radian (or meter) per second of error
   */
  public static MechanismControl velocity(double kP) {
    return defaults(kP, NeutralMode.BRAKE);
  }

  /** The same, coasting when told nothing: a flywheel spinning down rather than stopping hard. */
  public MechanismControl coasting() {
    return MechanismControlBuilder.builder(this).neutralMode(NeutralMode.COAST).build();
  }

  /** The same, its moves profiled: at most this fast, and this quick to get there. */
  public MechanismControl withProfile(double cruiseVelocity, double acceleration) {
    return MechanismControlBuilder.builder(this)
        .profile(
            Optional.of(
                Profile.builder()
                    .cruiseVelocity(cruiseVelocity)
                    .acceleration(acceleration)
                    .jerk(0)
                    .build()))
        .build();
  }

  /** The same, with a derivative gain: volts per radian (or meter) per second of error. */
  public MechanismControl withKd(double kD) {
    return MechanismControlBuilder.builder(this)
        .gains(MechanismControlGainsBuilder.builder(gains).kD(kD).build())
        .build();
  }

  /** The same, the motor controller stopping it inside its hard stops. */
  public MechanismControl withSoftLimits(double lower, double upper) {
    return MechanismControlBuilder.builder(this)
        .softLimits(Optional.of(SoftLimits.builder().lower(lower).upper(upper).build()))
        .build();
  }

  /** The same, its motors' torque capped at this current. */
  public MechanismControl withStatorLimit(Current stator) {
    return MechanismControlBuilder.builder(this)
        .currentLimits(
            MechanismControlCurrentLimitsBuilder.builder(currentLimits).stator(stator).build())
        .build();
  }

  /** The same, finding where it is by this recipe once the robot is on. */
  public MechanismControl withHoming(Homing homing) {
    return MechanismControlBuilder.builder(this).homing(Optional.of(homing)).build();
  }

  private static MechanismControl defaults(double kP, NeutralMode neutralMode) {
    return builder()
        .gains(MechanismControlGainsBuilder.builder(Gains.seeded()).kP(kP).build())
        .softLimits(Optional.empty())
        .profile(Optional.empty())
        .currentLimits(CurrentLimits.DEFAULTS)
        .neutralMode(neutralMode)
        .bands(Optional.empty())
        .homing(Optional.empty())
        .build();
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
   *     one, everywhere
   */
  @ConfigRecord
  public record Gains(
      double kP,
      double kI,
      double kD,
      double kS,
      Optional<Double> kV,
      Optional<Double> kA,
      Optional<Double> kG) {
    public Gains {
      Checks.on(Gains.class)
          .field("kP", kP, nonNegative())
          .field("kI", kI, nonNegative())
          .field("kD", kD, nonNegative())
          .field("kS", kS, nonNegative())
          .field("kV", kV, given(nonNegative()))
          .field("kA", kA, given(nonNegative()))
          .field("kG", kG, given(finite()))
          .orThrow();
    }

    /** Starts describing the gains, one named value at a time. */
    public static MechanismControlGainsBuilder.KPStage builder() {
      return MechanismControlGainsBuilder.builder();
    }

    /** No feedback, no friction, and every feedforward gain the definition's seed. */
    public static Gains seeded() {
      return builder()
          .kP(0)
          .kI(0)
          .kD(0)
          .kS(0)
          .kV(Optional.empty())
          .kA(Optional.empty())
          .kG(Optional.empty())
          .build();
    }

    private static Rule<Optional<Double>> given(Rule<Object> rule) {
      return value -> value.isPresent() ? rule.whyNot(value.get()) : null;
    }
  }

  /**
   * Where the motor controller stops driving the joint, in the joint's units: inside its hard stops
   * ({@code Joint.range}), which the helper checks. They hold only once the motor controller knows
   * where the joint is: from an absolute encoder, or after homing.
   *
   * @param lower the lowest position it drives the joint to
   * @param upper the highest
   */
  @ConfigRecord
  public record SoftLimits(double lower, double upper) {
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
   * Motion Magic's limits, in the joint's units: the cruise is held under the joint's free speed,
   * which the helper checks.
   *
   * @param cruiseVelocity the fastest the profile moves it, per second
   * @param acceleration how fast the profile speeds it up and slows it down, per second squared
   * @param jerk how fast the acceleration changes, per second cubed: 0 for none, a trapezoid
   */
  @ConfigRecord
  public record Profile(double cruiseVelocity, double acceleration, double jerk) {
    public Profile {
      Checks.on(Profile.class)
          .field("cruiseVelocity", cruiseVelocity, positive())
          .field("acceleration", acceleration, positive())
          .field("jerk", jerk, nonNegative())
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
   */
  @ConfigRecord
  public record Bands(double enter, double stay) {
    public Bands {
      Checks.on(Bands.class)
          .field("enter", enter, positive())
          .field("stay", stay, finite())
          .check(
              "stay",
              () ->
                  stay >= enter
                      ? null
                      : shown(stay) + " must be at least the band to arrive (" + shown(enter) + ")")
          .orThrow();
    }

    /** Starts describing bands, one named value at a time. */
    public static MechanismControlBandsBuilder.EnterStage builder() {
      return MechanismControlBandsBuilder.builder();
    }
  }

  /**
   * How a joint read by its motor alone finds where it is: the motor counts turns from wherever it
   * was at power on, so until then the joint's position is a guess. A joint with an absolute
   * encoder needs none; any other joint held at positions names one, so trusting the start is a
   * choice written down ({@link #assumeAtReference}), never a silent one.
   */
  public sealed interface Homing
      permits Homing.AssumeAtReference, Homing.ToHardStop, Homing.ToSwitch {
    /** The joint is where its definition rests it whenever the robot powers on: stowed by hand. */
    static Homing assumeAtReference() {
      return new AssumeAtReference();
    }

    /**
     * On the first enable, the joint is pushed gently into one of its hard stops, and its position
     * there is the stop's: 2 V, for at most 3 s.
     */
    static Homing toHardStop(Stop stop) {
      return ToHardStop.builder().stop(stop).volts(2).timeout(Seconds.of(3)).build();
    }

    /**
     * On the first enable, the joint is driven gently toward its limit switch at one end of its
     * travel, and its position, once the switch trips, is the switch's: 2 V, for at most 3 s.
     */
    static Homing toSwitch(Stop end) {
      return ToSwitch.builder().end(end).volts(2).timeout(Seconds.of(3)).build();
    }

    /** Which of a joint's hard stops, or which end of its travel. */
    enum Stop {
      /** The bottom of its range. */
      LOWER,
      /** The top of its range. */
      UPPER
    }

    /** The joint is at its reference at power on ({@link #assumeAtReference}). */
    record AssumeAtReference() implements Homing {}

    /**
     * Pushed into a hard stop on the first enable, the joint is there once it has stopped moving
     * for a quarter of a second; if it hasn't by the timeout, it's faulted, and its moves refused.
     *
     * @param stop which hard stop
     * @param volts how hard it's pushed: enough to move it, against gravity if the stop is above
     * @param timeout how long it may take
     */
    @ConfigRecord
    record ToHardStop(Stop stop, double volts, Time timeout) implements Homing {
      public ToHardStop {
        Checks.on(ToHardStop.class)
            .field("volts", volts, positive())
            .field("volts", volts, atMost(12.0))
            .field("timeout", timeout, positive())
            .orThrow();
      }

      /** Starts describing the homing, one named value at a time. */
      public static MechanismControlHomingToHardStopBuilder.StopStage builder() {
        return MechanismControlHomingToHardStopBuilder.builder();
      }
    }

    /**
     * Driven toward its limit switch at one end on the first enable, the joint is there the loop
     * the switch trips, at the switch's position in the definition; if it hasn't by the timeout,
     * it's faulted, and its moves refused. One resting on its switch is home at once.
     *
     * @param end which end's limit switch: the definition's limit switch on the joint at that end
     * @param volts how hard it's driven: enough to move it, against gravity if the switch is above
     * @param timeout how long it may take
     */
    @ConfigRecord
    record ToSwitch(Stop end, double volts, Time timeout) implements Homing {
      public ToSwitch {
        Checks.on(ToSwitch.class)
            .field("volts", volts, positive())
            .field("volts", volts, atMost(12.0))
            .field("timeout", timeout, positive())
            .orThrow();
      }

      /** Starts describing the homing, one named value at a time. */
      public static MechanismControlHomingToSwitchBuilder.EndStage builder() {
        return MechanismControlHomingToSwitchBuilder.builder();
      }
    }
  }
}
