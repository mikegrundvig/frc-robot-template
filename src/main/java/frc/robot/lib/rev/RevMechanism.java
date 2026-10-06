package frc.robot.lib.rev;

import static org.wpilib.units.Units.Amps;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.MechanismWiringBuilder;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.lib.phoenix.PhoenixUnits;
import frc.robot.lib.wiring.ControllerPins;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * A mechanism's REV SPARK settings: the mechanism the Phoenix helper works out from the robot's
 * definition, the joint's wiring and its control ({@link PhoenixMechanism}), its ratios, gains
 * seeded from the definition, gravity, soft limits, current limits and followers, carried to a
 * SPARK MAX or a SPARK Flex. Every check the Phoenix helper makes holds here too. Pure functions:
 * nothing here touches a device.
 *
 * <p>A SPARK reads its motor's own encoder in rotations and RPM: REVLib 2027 has no conversion
 * factors. With its voltage compensated to {@link #NOMINAL_VOLTS}, its loop's output is a share of
 * that. Its loop runs the slot's kP, per rotor rotation of error in slot 0, for positions, and per
 * rotor RPM in slot 1, for speeds. The rest, the feedforward (kS, kV, kA and gravity) and kD, the
 * robot works out each loop in volts ({@link Feedforward}) and adds as the loop's arbitrary
 * feedforward: volts are the one unit a SPARK takes exactly, whatever the units of its own
 * feedforward and derivative. Measured in REV's simulation (2026-10-05): kP 0.01 on 10 rotations of
 * error gave a share of 0.1, and 1.2 V of feedforward a share of 0.1.
 *
 * <p>A SPARK limits its motor's current (its smart limit, from the control's stator limit), not the
 * battery's, so the supply limits aren't applied. Its kI isn't carried over yet: a joint whose
 * control has one is refused.
 */
public final class RevMechanism {
  private RevMechanism() {}

  /** The voltage a SPARK's loop output is a share of: its voltage compensation. */
  public static final double NOMINAL_VOLTS = 12;

  /** A SPARK's position and speed signals, every loop: 20 ms. */
  private static final int SIGNAL_MS = 20;

  /**
   * One mechanism's SPARK settings.
   *
   * @param leader the leading SPARK's, applied whole at startup
   * @param followers each following SPARK's, in the wiring's order
   * @param units the mechanism's rotations ({@link PhoenixUnits}), which the robot's gains are in
   * @param rotorRatio rotor rotations for each of the mechanism's
   * @param positionKp the loop's kP for positions: a share of {@link #NOMINAL_VOLTS} per rotor
   *     rotation of error
   * @param velocityKp the loop's kP for speeds: a share per rotor RPM of error
   * @param softLimits the soft limits, in rotor rotations, lower then upper, if the control has
   *     them
   * @param feedforward what the robot adds each loop, in volts
   * @param absolute the definition's absolute encoder the SPARK's data port reads, if its wiring
   *     names one: read once at startup, to tell the motor where the joint is
   * @param notes what's doubtful about them, each a sentence for a log or a test
   */
  public record Settings(
      SparkBaseConfig leader,
      List<Follower> followers,
      PhoenixUnits units,
      double rotorRatio,
      double positionKp,
      double velocityKp,
      Optional<double[]> softLimits,
      Feedforward feedforward,
      Optional<Sensor.Encoder> absolute,
      List<String> notes) {
    public Settings {
      followers = List.copyOf(followers);
      notes = List.copyOf(notes);
    }

    /** The settings the pit tunes live, alone: the loop's gains and the current limits. */
    public SparkBaseConfig tuning(MechanismWiring wiring, MechanismControl control) {
      SparkBaseConfig tuning = blank(wiring.controller());
      tuning.smartCurrentLimit(statorAmps(control));
      tuning.closedLoop.p(positionKp, ClosedLoopSlot.kSlot0).p(velocityKp, ClosedLoopSlot.kSlot1);
      return tuning;
    }

    /** The soft limits switched on or off, alone: off while the joint homes past them. */
    public SparkBaseConfig softLimitsEnabled(MechanismWiring wiring, boolean enabled) {
      SparkBaseConfig limits = blank(wiring.controller());
      limits.softLimit.forwardSoftLimitEnabled(enabled).reverseSoftLimitEnabled(enabled);
      return limits;
    }
  }

  /**
   * A following SPARK.
   *
   * @param id its CAN ID
   * @param config its settings: following the leader, aligned or opposed as the wiring's directions
   *     say, with the leader's current limit and idle mode
   */
  public record Follower(int id, SparkBaseConfig config) {}

  /**
   * What the robot adds to a SPARK's loop each loop, in volts: the slot's feedforward and kD, in
   * the mechanism's rotations ({@link PhoenixUnits}), as Phoenix's slot holds them.
   *
   * @param kS volts to overcome friction, in the direction of travel
   * @param kV volts per rotation a second
   * @param kA volts per rotation a second squared: in the program's own feedforward, as on a
   *     TalonFX, and so unused here
   * @param kG volts to hold against gravity: always, for an elevator, or at the arm's angle
   * @param arm whether gravity turns with the joint, as an arm's does
   * @param armOffset what's added to the position for its angle to gravity, in rotations
   * @param kD volts per rotation a second of speed error
   */
  public record Feedforward(
      double kS, double kV, double kA, double kG, boolean arm, double armOffset, double kD) {
    /**
     * Volts to follow a profile's point: friction, speed, gravity, and damping toward its speed.
     *
     * @param position where the profile is, in the mechanism's rotations
     * @param velocity how fast it moves there, in rotations a second
     * @param measured how fast the mechanism moves now, in rotations a second
     */
    public double atPosition(double position, double velocity, double measured) {
      return kS * Math.signum(velocity)
          + kV * velocity
          + gravity(position)
          + kD * (velocity - measured);
    }

    /** Volts to run at a speed: friction, speed and gravity; the loop's kP closes the rest. */
    public double atSpeed(double position, double velocity) {
      return kS * Math.signum(velocity) + kV * velocity + gravity(position);
    }

    private double gravity(double position) {
      return arm ? kG * Math.cos(2 * Math.PI * (position + armOffset)) : kG;
    }
  }

  /**
   * A mechanism on SPARKs, read by its motor's own encoder.
   *
   * @throws IllegalArgumentException for wiring that isn't a SPARK's, names a CANcoder or a
   *     data-port encoder, a control with a kI, and whatever the Phoenix helper refuses
   */
  /**
   * A SPARK's limit inputs, from what's on them ({@link ControllerPins}): a switch's polarity, and
   * whether the SPARK stops on it, only for a limit switch; an input with nothing on it stops
   * nothing. Neither resets its position as it trips: homing is the robot's, where it can be
   * checked.
   *
   * @throws IllegalArgumentException for a CANdi's limit, which a SPARK can't read
   */
  public static void limits(LimitSwitchConfig limits, ControllerPins pins) {
    limits
        .forwardLimitSwitchType(type(pins.forward()))
        .forwardLimitSwitchTriggerBehavior(behavior(pins.forward()))
        .reverseLimitSwitchType(type(pins.reverse()))
        .reverseLimitSwitchTriggerBehavior(behavior(pins.reverse()));
  }

  private static LimitSwitchConfig.Type type(Optional<ControllerPins.Limit> limit) {
    return switch (limit.orElse(null)) {
      case ControllerPins.OnPin pin ->
          pin.polarity() == SensorWiring.Polarity.NORMALLY_CLOSED
              ? LimitSwitchConfig.Type.kNormallyClosed
              : LimitSwitchConfig.Type.kNormallyOpen;
      case ControllerPins.OnCandi candi ->
          throw new IllegalArgumentException("a SPARK can't read a CANdi as its limit");
      case null -> LimitSwitchConfig.Type.kNormallyOpen;
    };
  }

  private static LimitSwitchConfig.Behavior behavior(Optional<ControllerPins.Limit> limit) {
    return limit.map(ControllerPins.Limit::stops).orElse(false)
        ? LimitSwitchConfig.Behavior.kStopMovingMotor
        : LimitSwitchConfig.Behavior.kKeepMovingMotor;
  }

  public static Settings configure(
      RobotMath math, Joint joint, Drive drive, MechanismWiring wiring, MechanismControl control) {
    if (wiring.dataPort().isPresent()) {
      throw new IllegalArgumentException(
          joint.label()
              + "'s wiring names a data-port encoder: give the definition's encoder it is, or"
              + " wire none");
    }
    return build(math, joint, drive, null, wiring, control);
  }

  /**
   * A mechanism on SPARKs with an absolute encoder on the leader's data port: the definition's
   * absolute encoder on the joint, read once at startup, so the joint needs no homing.
   *
   * @throws IllegalArgumentException as the other does, for an encoder that isn't absolute on the
   *     joint alone, or whose joint's range is more than one of its turns tells apart, and for
   *     wiring without a data-port encoder
   */
  public static Settings configure(
      RobotMath math,
      Joint joint,
      Drive drive,
      Sensor.Encoder absolute,
      MechanismWiring wiring,
      MechanismControl control) {
    if (wiring.dataPort().isEmpty()) {
      throw new IllegalArgumentException(
          joint.label()
              + "'s wiring names no data-port encoder for "
              + absolute.label()
              + ": wire it");
    }
    if (!absolute.absolute() || !absolute.joints().equals(List.of(joint))) {
      throw new IllegalArgumentException(
          absolute.label() + " must be an absolute encoder on " + joint.label() + " alone");
    }
    return build(math, joint, drive, absolute, wiring, control);
  }

  private static Settings build(
      RobotMath math,
      Joint joint,
      Drive drive,
      Sensor.@Nullable Encoder absolute,
      MechanismWiring wiring,
      MechanismControl control) {
    MechanismWiring.Controller controller = wiring.controller();
    if (controller != MechanismWiring.Controller.SPARK_MAX
        && controller != MechanismWiring.Controller.SPARK_FLEX) {
      throw new IllegalArgumentException(
          joint.label() + "'s wiring is " + controller.shown() + "'s, not a SPARK's");
    }
    if (wiring.cancoder().isPresent()) {
      throw new IllegalArgumentException(
          joint.label()
              + "'s wiring names a CANcoder, which a SPARK doesn't read: plug the encoder into the"
              + " SPARK's data port, or give its control a homing recipe");
    }
    if (control.gains().kI() != 0) {
      throw new IllegalArgumentException(
          joint.label()
              + "'s control has a kI, which isn't carried to a SPARK yet: its integral's units"
              + " aren't confirmed. Leave kI at 0");
    }
    // The Phoenix helper's own settings for the motor alone: the data port is the SPARK's.
    MechanismWiring motorsAlone =
        MechanismWiringBuilder.builder(wiring).dataPort(Optional.empty()).build();
    PhoenixMechanism.Settings phoenix =
        PhoenixMechanism.configure(math, joint, drive, motorsAlone, control);
    List<String> notes = new ArrayList<>(phoenix.notes());
    if (absolute != null) {
      // One turn of the encoder must tell the joint's range apart; it's read once, at startup.
      math.absoluteWindow(List.of(absolute));
      double point = math.discontinuityPoint(absolute);
      String wraps =
          PhoenixMechanism.wrapProblem(absolute, absolute.ratio(joint) < 0 ? 1 - point : point);
      if (wraps != null) {
        notes.add(wraps);
      }
    }
    TalonFXConfiguration fx = phoenix.leader();
    SlotConfigs slot = SlotConfigs.from(fx.Slot0);
    double ratio = fx.Feedback.SensorToMechanismRatio;
    boolean brake = fx.MotorOutput.NeutralMode == NeutralModeValue.Brake;
    int amps = statorAmps(control);

    SparkBaseConfig leader = blank(controller);
    leader
        .inverted(wiring.leader().clockwise())
        .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(amps)
        .voltageCompensation(NOMINAL_VOLTS);
    double positionKp = slot.kP / (NOMINAL_VOLTS * ratio);
    double velocityKp = slot.kP / (NOMINAL_VOLTS * ratio * 60);
    leader
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(positionKp, ClosedLoopSlot.kSlot0)
        .p(velocityKp, ClosedLoopSlot.kSlot1);
    Optional<double[]> softLimits = Optional.empty();
    if (fx.SoftwareLimitSwitch.ForwardSoftLimitEnable) {
      double lower = fx.SoftwareLimitSwitch.ReverseSoftLimitThreshold * ratio;
      double upper = fx.SoftwareLimitSwitch.ForwardSoftLimitThreshold * ratio;
      leader
          .softLimit
          .reverseSoftLimit(lower)
          .reverseSoftLimitEnabled(true)
          .forwardSoftLimit(upper)
          .forwardSoftLimitEnabled(true);
      softLimits = Optional.of(new double[] {lower, upper});
    }
    if (absolute != null) {
      if (controller == MechanismWiring.Controller.SPARK_MAX) {
        leader.absoluteEncoder.setSparkMaxDataPortConfig();
      }
      leader.signals.absoluteEncoderPositionPeriodMs(SIGNAL_MS);
    }
    leader
        .signals
        .primaryEncoderPositionPeriodMs(SIGNAL_MS)
        .primaryEncoderVelocityPeriodMs(SIGNAL_MS)
        .appliedOutputPeriodMs(SIGNAL_MS)
        .outputCurrentPeriodMs(SIGNAL_MS)
        .busVoltagePeriodMs(SIGNAL_MS);

    List<Follower> followers = new ArrayList<>();
    for (MechanismWiring.Motor motor : wiring.motors().subList(1, wiring.motors().size())) {
      SparkBaseConfig follower = blank(controller);
      follower
          .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
          .smartCurrentLimit(amps)
          .voltageCompensation(NOMINAL_VOLTS)
          .follow(wiring.leader().id(), motor.clockwise() != wiring.leader().clockwise());
      followers.add(new Follower(motor.id(), follower));
    }

    Feedforward feedforward =
        new Feedforward(
            slot.kS,
            slot.kV,
            slot.kA,
            slot.kG,
            slot.GravityType == GravityTypeValue.Arm_Cosine,
            slot.GravityArmPositionOffset,
            slot.kD);
    return new Settings(
        leader,
        followers,
        phoenix.units(),
        ratio,
        positionKp,
        velocityKp,
        softLimits,
        feedforward,
        Optional.ofNullable(absolute),
        notes);
  }

  /** An empty configuration for the controller: only what's set on it is applied. */
  private static SparkBaseConfig blank(MechanismWiring.Controller controller) {
    return controller == MechanismWiring.Controller.SPARK_FLEX
        ? new SparkFlexConfig()
        : new SparkMaxConfig();
  }

  /** The control's stator limit, as a SPARK's smart current limit takes it: whole amps. */
  private static int statorAmps(MechanismControl control) {
    return (int) Math.round(control.currentLimits().stator().in(Amps));
  }
}
