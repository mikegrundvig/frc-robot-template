package frc.robot.lib.rev;

import static org.wpilib.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.MechanismWiringBuilder;
import com.michaelgrundvig.frc.world.robot.Range;
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
 * robot works out each loop in volts ({@link PhoenixMechanism.Feedforward}) and adds as the loop's
 * arbitrary feedforward: volts are the one unit a SPARK takes exactly, whatever the units of its
 * own feedforward and derivative. Measured in REV's simulation (2026-10-05): kP 0.01 on 10
 * rotations of error gave a share of 0.1, and 1.2 V of feedforward a share of 0.1.
 *
 * <p>A SPARK whose loop closes on the absolute encoder on its data port ({@code
 * DataPortEncoder#closesLoop}) runs it on that encoder's turns instead, with no conversion factor
 * either: its setpoints, its soft limits and its kP are per turn of the encoder, read from the low
 * edge of the joint's window, where the SPARK reads zero, so it never wraps inside the joint's
 * range and needs no position wrapping, which would take the short way round, through a hard stop.
 * Its speeds, on the same sensor, are per RPM of the encoder. The feedforward is the robot's, as
 * ever, and the speed it damps toward is the motor's. That REV's soft limits follow the sensor the
 * loop runs on, and that the absolute encoder's speed is in RPM, are as REV documents them, and
 * unconfirmed: REV's simulation runs its loop on the motor's own encoder whatever it's told.
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
   * @param loopRatio turns of the sensor its loop runs on for each of the mechanism's rotations:
   *     the rotor's, or, closed on its absolute encoder, the encoder's
   * @param loopZero what that sensor reads with the mechanism at zero: 0 for its rotor, and for its
   *     absolute encoder, read from the window's edge, minus the edge
   * @param positionKp the loop's kP for positions: a share of {@link #NOMINAL_VOLTS} per turn of
   *     error of the sensor it runs on
   * @param velocityKp the loop's kP for speeds: a share per RPM of error of that sensor
   * @param softLimits the soft limits, in that sensor's turns, lower then upper, if the control has
   *     them
   * @param feedforward what the robot adds each loop, in volts
   * @param absolute the definition's absolute encoder the SPARK's data port reads, if its wiring
   *     names one: read once at startup, to tell the motor where the joint is
   * @param windowEdge where the SPARK's reading of that encoder is zero: the encoder's reading, in
   *     its turns from the joint's zero, at the low edge of the window it tells apart ({@code
   *     RobotMath.absoluteWindow}), so the SPARK's reading, from 0 to 1, wraps outside the joint's
   *     range; 0 without one
   * @param closedLoop whether its loop runs on that encoder, as its wiring says, rather than on its
   *     rotor
   * @param notes what's doubtful about them, each a sentence for a log or a test
   */
  public record Settings(
      SparkBaseConfig leader,
      List<Follower> followers,
      PhoenixUnits units,
      double rotorRatio,
      double loopRatio,
      double loopZero,
      double positionKp,
      double velocityKp,
      Optional<double[]> softLimits,
      PhoenixMechanism.Feedforward feedforward,
      Optional<Sensor.Encoder> absolute,
      double windowEdge,
      boolean closedLoop,
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

  /**
   * A mechanism on SPARKs, read by its motor's own encoder.
   *
   * @throws IllegalArgumentException for wiring that isn't a SPARK's, is on a CANivore, or names a
   *     CANcoder or a data-port encoder, a control with a kI, and whatever the Phoenix helper
   *     refuses
   */
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
   * absolute encoder on the joint, read once at startup, so the joint needs no homing, and the
   * SPARK's loop running on it where the wiring says so.
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
    if (!(wiring.canBus() instanceof CanBus.Port)) {
      throw new IllegalArgumentException(
          joint.label() + "'s SPARKs are on a CANivore: a SPARK is on Systemcore's own CAN");
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
    // The Phoenix helper's own settings for the motor alone: the data port is the SPARK's. The
    // robot works out the SPARK's feedforward, so an arm's gravity offset isn't Phoenix's to limit.
    MechanismWiring motorsAlone =
        MechanismWiringBuilder.builder(wiring).dataPort(Optional.empty()).build();
    PhoenixMechanism.Settings phoenix =
        PhoenixMechanism.configureFeedforwardOnRobot(math, joint, drive, motorsAlone, control);
    List<String> notes = new ArrayList<>(phoenix.notes());
    PhoenixUnits units = phoenix.units();
    double windowEdge = 0;
    double encoderRatio = 0;
    if (absolute != null) {
      // One turn of the encoder must tell the joint's range apart, and wrap outside it, as a
      // CANcoder's discontinuity point puts it.
      Range window = math.absoluteWindow(List.of(absolute));
      double point = math.discontinuityPoint(absolute);
      String wraps =
          PhoenixMechanism.wrapProblem(absolute, absolute.ratio(joint) < 0 ? 1 - point : point);
      if (wraps != null) {
        notes.add(wraps);
      }
      encoderRatio = units.ratioToPhoenix(Math.abs(absolute.ratio(joint)));
      windowEdge = encoderRatio * units.toPhoenix(window.lower());
    }
    boolean closed = absolute != null && wiring.dataPort().orElseThrow().closedLoop();
    TalonFXConfiguration fx = phoenix.leader();
    double ratio = fx.Feedback.SensorToMechanismRatio;
    boolean brake = fx.MotorOutput.NeutralMode == NeutralModeValue.Brake;
    int amps = statorAmps(control);

    SparkBaseConfig leader = blank(controller);
    leader
        .inverted(wiring.leader().clockwise())
        .idleMode(brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
        .smartCurrentLimit(amps)
        .voltageCompensation(NOMINAL_VOLTS);
    // The sensor its loop runs on, and its turns: the rotor's, or the absolute encoder's from the
    // window's edge.
    double loopRatio = closed ? encoderRatio : ratio;
    double loopZero = closed ? -windowEdge : 0;
    double positionKp = phoenix.kP() / (NOMINAL_VOLTS * loopRatio);
    double velocityKp = phoenix.kP() / (NOMINAL_VOLTS * loopRatio * 60);
    leader
        .closedLoop
        .feedbackSensor(closed ? FeedbackSensor.kAbsoluteEncoder : FeedbackSensor.kPrimaryEncoder)
        .p(positionKp, ClosedLoopSlot.kSlot0)
        .p(velocityKp, ClosedLoopSlot.kSlot1);
    if (closed) {
      // Its reading wraps outside the joint's range: the short way round could be through a stop.
      leader.closedLoop.positionWrappingEnabled(false);
    }
    Optional<double[]> softLimits = Optional.empty();
    if (fx.SoftwareLimitSwitch.ForwardSoftLimitEnable) {
      double lower = fx.SoftwareLimitSwitch.ReverseSoftLimitThreshold * loopRatio + loopZero;
      double upper = fx.SoftwareLimitSwitch.ForwardSoftLimitThreshold * loopRatio + loopZero;
      leader
          .softLimit
          .reverseSoftLimit(lower)
          .reverseSoftLimitEnabled(true)
          .forwardSoftLimit(upper)
          .forwardSoftLimitEnabled(true);
      softLimits = Optional.of(new double[] {lower, upper});
    }
    if (absolute != null) {
      MechanismWiring.DataPortEncoder port = wiring.dataPort().orElseThrow();
      if (controller == MechanismWiring.Controller.SPARK_MAX) {
        leader.absoluteEncoder.setSparkMaxDataPortConfig();
      }
      // The SPARK applies the encoder's direction and offset itself, as a CANcoder does: its
      // reading grows with the joint, and reads zero at the window's low edge, so it wraps where
      // the joint never is.
      leader.absoluteEncoder.inverted(port.reversed()).zeroOffset(port.raw(windowEdge));
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

    return new Settings(
        leader,
        followers,
        units,
        ratio,
        loopRatio,
        loopZero,
        positionKp,
        velocityKp,
        softLimits,
        phoenix.feedforward(),
        Optional.ofNullable(absolute),
        windowEdge,
        closed,
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
