package frc.robot.lib.phoenix;

import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import com.michaelgrundvig.frc.world.robot.Transmission;
import frc.robot.lib.wiring.CanPorts;
import frc.robot.lib.wiring.ControllerPins;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation3d;

/**
 * A mechanism's Phoenix 6 settings, from what it is (the robot's definition), how it's wired, and
 * how it's driven: its TalonFX's ratios, gains, gravity, limits, profile, direction and followers,
 * and its CANcoder's, so a mechanism is described once and nothing is retyped. Pure functions: the
 * configurations are new objects for the caller to apply, and no device is touched here.
 *
 * <p>Positions are in {@link PhoenixUnits}: a revolute joint's rotations, a prismatic joint's
 * drum's rotations. Every ratio Phoenix is given is a magnitude, and every direction is the
 * wiring's, as the joint moves forward ({@link MechanismWiring}). The gains go in slot 0.
 *
 * <p>What's wrong is refused, by an {@link IllegalArgumentException} saying what and why: a ratio,
 * a cruise, a soft limit or a gravity offset Phoenix can't hold or the mechanism can't reach.
 * What's doubtful is noted, beside the settings ({@link Settings#notes}): a tuned gain far from
 * what the definition derives, an encoder wrapping close to the joint's range, a gravity one cosine
 * fits badly, a definition that can't seed what Phoenix needs.
 */
public final class PhoenixMechanism {
  private PhoenixMechanism() {}

  // Phoenix's own limits, in 26.70.0-alpha-2's configuration classes.

  /**
   * The largest ratio Phoenix holds: {@code FeedbackConfigs.SensorToMechanismRatio} and {@code
   * RotorToSensorRatio}, and {@code DifferentialSensorsConfigs.SensorToDifferentialRatio}, each
   * from -1000 to 1000.
   */
  static final double MAX_RATIO = 1000;

  /**
   * The farthest {@code Slot0Configs.GravityArmPositionOffset} goes, in rotations, either way: its
   * documented minimum and maximum, -0.25 and 0.25.
   */
  static final double MAX_GRAVITY_OFFSET = 0.25;

  /** kS and kG, in volts: from -128 to 127. */
  static final double MIN_VOLTS = -128;

  static final double MAX_VOLTS = 127;

  /** Motion Magic's cruise, acceleration and jerk: at most 9999 rotations per second, and so on. */
  static final double MAX_PROFILE = 9999;

  /** {@code MotionMagicExpo_kV}, in volts per rotation per second. */
  static final double MIN_EXPO_KV = 0.001;

  static final double MAX_EXPO_KV = 100;

  /** {@code MotionMagicExpo_kA}, in volts per rotation per second squared. */
  static final double MIN_EXPO_KA = 1e-5;

  static final double MAX_EXPO_KA = 100;

  // The helper's own judgment: chosen, each with its reason.

  /**
   * How near Phoenix's ±0.25-rotation limit a gravity offset may come before it's noted: 5°. A
   * center of mass that close to straight above or below the pivot at the joint's zero crosses it
   * with a CAD change (wiring, a heavier roller), and Phoenix can't take the offset then.
   */
  static final double GRAVITY_OFFSET_MARGIN = 5.0 / 360;

  /**
   * How far one cosine may miss gravity's torque, as a share of its peak, before it's noted: beyond
   * a few percent, the joint's feedforward wants {@code RobotMath.gravityTorque} instead.
   */
  static final double MISFIT = 0.05;

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
   * One mechanism's settings.
   *
   * @param leader the leading TalonFX's: everything Phoenix needs to drive the joint
   * @param followers each following TalonFX's, in the wiring's order
   * @param cancoder the CANcoder's, if the mechanism has one
   * @param units the units its positions, speeds and gains are in: what its code converts the
   *     joint's positions with, to and from Phoenix's
   * @param notes what's doubtful about them, each a sentence for a log or a test
   */
  public record Settings(
      TalonFXConfiguration leader,
      List<FollowerSettings> followers,
      Optional<CANcoderConfiguration> cancoder,
      PhoenixUnits units,
      List<String> notes) {
    public Settings {
      followers = List.copyOf(followers);
      notes = List.copyOf(notes);
    }
  }

  /**
   * A following TalonFX: its request and its own configuration, which keeps its current limits,
   * neutral mode and position reading in step with the leader's.
   *
   * @param id its CAN ID
   * @param request what to send it: follow the leader, aligned or opposed as the wiring's
   *     directions say
   * @param configuration its own settings
   */
  public record FollowerSettings(int id, Follower request, TalonFXConfiguration configuration) {}

  /**
   * A mechanism read by its motors' rotors alone.
   *
   * @param math what the robot's definition derives ({@code RobotDefinition.math()})
   * @param joint the joint whose units the motor controller works in
   * @param drive the motors that move it, as many as the wiring names
   * @throws IllegalArgumentException for what Phoenix can't hold or the mechanism can't do, and for
   *     wiring that names a CANcoder: give the definition's encoder it is
   */
  public static Settings configure(
      RobotMath math, Joint joint, Drive drive, MechanismWiring wiring, MechanismControl control) {
    if (wiring.cancoder().isPresent()) {
      throw new IllegalArgumentException(
          joint.label()
              + "'s wiring names a CANcoder: give the definition's encoder it is, or wire none");
    }
    return build(math, joint, drive, null, wiring, control);
  }

  /**
   * A mechanism with a CANcoder: the definition's absolute encoder on the joint, wired as its
   * {@code MechanismWiring.cancoder} says, and read as its feedback says.
   *
   * @throws IllegalArgumentException as the other does, and for an encoder that isn't absolute, or
   *     wiring without a CANcoder
   */
  public static Settings configure(
      RobotMath math,
      Joint joint,
      Drive drive,
      Sensor.Encoder cancoder,
      MechanismWiring wiring,
      MechanismControl control) {
    if (wiring.cancoder().isEmpty()) {
      throw new IllegalArgumentException(
          joint.label() + "'s wiring names no CANcoder for " + cancoder.label() + ": wire it");
    }
    return build(math, joint, drive, cancoder, wiring, control);
  }

  /** The TalonFXS's motor arrangement for each catalog motor it drives, by its JST connector. */
  private static final Map<Motor, MotorArrangementValue> ARRANGEMENTS =
      Map.of(
          Motor.MINION, MotorArrangementValue.Minion_JST,
          Motor.NEO, MotorArrangementValue.NEO_JST,
          Motor.NEO_550, MotorArrangementValue.NEO550_JST,
          Motor.NEO_VORTEX, MotorArrangementValue.VORTEX_JST);

  /**
   * A TalonFXS's settings, from the ones worked out for a TalonFX on the same mechanism: the same
   * slot, limits, profile, direction and neutral mode; its feedback in a TalonFXS's own terms (its
   * motor's hall sensor for a TalonFX's rotor); and the motor plugged into it, which a TalonFXS
   * needs to be told.
   *
   * @throws IllegalArgumentException for a motor a TalonFXS doesn't drive
   */
  public static TalonFXSConfiguration talonFxs(TalonFXConfiguration fx, Motor motor) {
    MotorArrangementValue arrangement = ARRANGEMENTS.get(motor);
    if (arrangement == null) {
      throw new IllegalArgumentException("a TalonFXS doesn't drive a " + motor.name());
    }
    TalonFXSConfiguration fxs = new TalonFXSConfiguration();
    fxs.MotorOutput = fx.MotorOutput;
    fxs.CurrentLimits = fx.CurrentLimits;
    fxs.SoftwareLimitSwitch = fx.SoftwareLimitSwitch;
    fxs.HardwareLimitSwitch = fx.HardwareLimitSwitch;
    fxs.MotionMagic = fx.MotionMagic;
    fxs.ClosedLoopGeneral = fx.ClosedLoopGeneral;
    fxs.Slot0 = fx.Slot0;
    fxs.Commutation.MotorArrangement = arrangement;
    fxs.ExternalFeedback.SensorToMechanismRatio = fx.Feedback.SensorToMechanismRatio;
    fxs.ExternalFeedback.RotorToSensorRatio = fx.Feedback.RotorToSensorRatio;
    fxs.ExternalFeedback.FeedbackRemoteSensorID = fx.Feedback.FeedbackRemoteSensorID;
    fxs.ExternalFeedback.ExternalFeedbackSensorSource =
        switch (fx.Feedback.FeedbackSensorSource) {
          case RemoteCANcoder -> ExternalFeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> ExternalFeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> ExternalFeedbackSensorSourceValue.SyncCANcoder;
          default -> ExternalFeedbackSensorSourceValue.Commutation;
        };
    return fxs;
  }

  /**
   * A CANcoder's settings: its offset and direction from the wiring, and its discontinuity point
   * from the definition ({@code RobotMath.discontinuityPoint}), where its reading wraps as far from
   * the joint's range as a turn allows.
   *
   * @throws IllegalArgumentException for an encoder that isn't absolute, or whose joint's range is
   *     more of its turn than one turn holds: a pair decoded together is read in code ({@code
   *     RobotMath.absoluteWindow})
   */
  public static CANcoderConfiguration cancoder(
      RobotMath math, Sensor.Encoder encoder, MechanismWiring.Cancoder wiring) {
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.MagnetOffset = wiring.offset().in(Rotations);
    config.MagnetSensor.SensorDirection =
        wiring.clockwise()
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = discontinuityPoint(math, encoder);
    return config;
  }

  /**
   * How far inside an absolute encoder's turn its joint's range sits when the encoder wraps at a
   * discontinuity point, in the encoder's turns, at the nearer end: negative when the range crosses
   * the wrap. The encoder reads zero with the joint at zero and grows with it, as its wiring sets
   * it: the definition's {@code RobotMath.wrapMargin}, for Phoenix's point. Infinite for a joint
   * that turns freely, which has no range to keep clear.
   */
  public static double wrapMargin(Sensor.Encoder encoder, double discontinuityPoint) {
    // The definition reads the encoder as its ratio's sign says; Phoenix, as its wiring sets it,
    // growing with the joint: an encoder geared to turn back wraps at the other end of its turn.
    return RobotMath.wrapMargin(
        encoder, encoder.ratio(encoder.joint()) < 0 ? 1 - discontinuityPoint : discontinuityPoint);
  }

  /**
   * What's wrong with an absolute encoder wrapping at a discontinuity point, or null when nothing
   * is: its joint's range crossing the wrap, or within the definition's margin of it ({@code
   * RobotMath.WRAP_MARGIN}, a fortieth of a turn). A team's own point is checked as the helper's
   * is.
   */
  public static @Nullable String wrapProblem(Sensor.Encoder encoder, double discontinuityPoint) {
    double margin = wrapMargin(encoder, discontinuityPoint);
    if (margin >= RobotMath.WRAP_MARGIN) {
      return null;
    }
    Joint joint = encoder.joint();
    Range range = joint.range().orElseThrow();
    double atStow =
        Math.min(
            reading(encoder, joint.reference()) - (discontinuityPoint - 1),
            discontinuityPoint - reading(encoder, joint.reference()));
    return String.format(
        Locale.ROOT,
        "%s wraps at %s of its turn, %s %s's range (%s to %s), and %s from where it stows (%s): a"
            + " reading a hair past the wrap reads a turn away",
        encoder.label(),
        shown(discontinuityPoint),
        margin < 0 ? "inside" : shownJoint(joint, alongJoint(encoder, margin)) + " from",
        joint.label(),
        shownJoint(joint, range.lower()),
        shownJoint(joint, range.upper()),
        shownJoint(joint, alongJoint(encoder, atStow)),
        shownJoint(joint, joint.reference()));
  }

  /** Phoenix's bus for the wiring's. */
  /**
   * A TalonFX's or TalonFXS's limits, from what's on them ({@link ControllerPins}): a switch on its
   * pin, its polarity told; a limit switch on a CANdi on its bus, read over CAN, the CANdi's closed
   * already meaning tripped; and nothing, which stops nothing. It stops only on a limit switch.
   * Neither resets its position as it trips: homing is the robot's, where it can be checked.
   */
  public static void limits(HardwareLimitSwitchConfigs limits, ControllerPins pins) {
    limits.ForwardLimitAutosetPositionEnable = false;
    limits.ReverseLimitAutosetPositionEnable = false;
    limits.ForwardLimitEnable = pins.forward().map(ControllerPins.Limit::stops).orElse(false);
    limits.ReverseLimitEnable = pins.reverse().map(ControllerPins.Limit::stops).orElse(false);
    limits.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    limits.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    limits.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    limits.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    switch (pins.forward().orElse(null)) {
      case ControllerPins.OnPin pin ->
          limits.ForwardLimitType =
              pin.polarity() == SensorWiring.Polarity.NORMALLY_OPEN
                  ? ForwardLimitTypeValue.NormallyOpen
                  : ForwardLimitTypeValue.NormallyClosed;
      case ControllerPins.OnCandi candi -> {
        limits.ForwardLimitSource =
            candi.input() == 1
                ? ForwardLimitSourceValue.RemoteCANdiS1
                : ForwardLimitSourceValue.RemoteCANdiS2;
        limits.ForwardLimitRemoteSensorID = candi.id();
      }
      case null -> {}
    }
    switch (pins.reverse().orElse(null)) {
      case ControllerPins.OnPin pin ->
          limits.ReverseLimitType =
              pin.polarity() == SensorWiring.Polarity.NORMALLY_OPEN
                  ? ReverseLimitTypeValue.NormallyOpen
                  : ReverseLimitTypeValue.NormallyClosed;
      case ControllerPins.OnCandi candi -> {
        limits.ReverseLimitSource =
            candi.input() == 1
                ? ReverseLimitSourceValue.RemoteCANdiS1
                : ReverseLimitSourceValue.RemoteCANdiS2;
        limits.ReverseLimitRemoteSensorID = candi.id();
      }
      case null -> {}
    }
  }

  public static CANBus bus(CanBus bus) {
    return switch (bus) {
      case CanBus.Port port -> new CANBus(CanPorts.of(port.port()));
      case CanBus.Canivore canivore -> new CANBus(canivore.name());
    };
  }

  private static Settings build(
      RobotMath math,
      Joint joint,
      Drive drive,
      Sensor.@Nullable Encoder encoder,
      MechanismWiring wiring,
      MechanismControl control) {
    List<String> notes = new ArrayList<>();
    if (!(drive.actuator() instanceof Actuator.Motors motors)) {
      throw new IllegalArgumentException(drive.label() + " isn't motors: Phoenix drives motors");
    }
    if (wiring.motors().size() != motors.count()) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s is %d motors, and its wiring names %d",
              drive.label(),
              motors.count(),
              wiring.motors().size()));
    }
    Sensor.Encoder rotor = drive.rotor();
    double rotorRatio = Math.abs(rotor.ratio(joint));
    if (rotorRatio == 0) {
      throw new IllegalArgumentException(drive.label() + " doesn't move " + joint.label());
    }
    for (Transmission.Term term : rotor.terms()) {
      if (!term.joint().equals(joint)) {
        notes.add(
            String.format(
                Locale.ROOT,
                "%s's rotor turns %s times for each %s of %s too, which Phoenix reads as %s"
                    + " moving: take it out in code, as a swerve module's coupling is",
                drive.label(),
                shown(Math.abs(rotor.ratio(term.joint()))),
                term.joint() instanceof Joint.Revolute ? "turn" : "meter",
                term.joint().label(),
                joint.label()));
      }
    }

    PhoenixUnits units = PhoenixUnits.of(joint, drive);
    TalonFXConfiguration leader = new TalonFXConfiguration();
    output(leader, wiring.leader(), control);
    Optional<CANcoderConfiguration> cancoder = Optional.empty();
    String ratioName = joint.label() + "'s SensorToMechanismRatio";
    if (encoder == null) {
      leader.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
      leader.Feedback.SensorToMechanismRatio = ratio(ratioName, units.ratioToPhoenix(rotorRatio));
    } else {
      MechanismWiring.Cancoder wired = wiring.cancoder().orElseThrow();
      if (!encoder.absolute() || !encoder.joints().equals(List.of(joint))) {
        throw new IllegalArgumentException(
            encoder.label()
                + " must be an absolute encoder on "
                + joint.label()
                + " alone, as a CANcoder is");
      }
      CANcoderConfiguration config = cancoder(math, encoder, wired);
      String wraps = wrapProblem(encoder, config.MagnetSensor.AbsoluteSensorDiscontinuityPoint);
      if (wraps != null) {
        notes.add(wraps);
      }
      cancoder = Optional.of(config);
      double sensor = sensorRatio(encoder);
      switch (wired.feedback()) {
        case ROTOR -> {
          leader.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
          leader.Feedback.SensorToMechanismRatio =
              ratio(ratioName, units.ratioToPhoenix(rotorRatio));
        }
        case REMOTE, FUSED, SYNC -> {
          leader.Feedback.FeedbackSensorSource = source(wired.feedback());
          leader.Feedback.FeedbackRemoteSensorID = wired.id();
          leader.Feedback.SensorToMechanismRatio = ratio(ratioName, units.ratioToPhoenix(sensor));
          leader.Feedback.RotorToSensorRatio =
              ratio(joint.label() + "'s RotorToSensorRatio", rotorRatio / sensor);
        }
      }
    }
    leader.Slot0 = Slot0Configs.from(slot(math, units, control.gains(), rotorRatio, notes));
    // A joint that turns freely takes the short way round to a position, a turn being a turn.
    leader.ClosedLoopGeneral.ContinuousWrap =
        joint instanceof Joint.Revolute && joint.range().isEmpty();
    control
        .softLimits()
        .ifPresent(
            limits -> {
              joint
                  .range()
                  .ifPresent(
                      range -> {
                        if (limits.lower() < range.lower() || limits.upper() > range.upper()) {
                          throw new IllegalArgumentException(
                              String.format(
                                  Locale.ROOT,
                                  "%s's soft limits, %s to %s, must be inside its hard stops, %s"
                                      + " to %s",
                                  joint.label(),
                                  shownJoint(joint, limits.lower()),
                                  shownJoint(joint, limits.upper()),
                                  shownJoint(joint, range.lower()),
                                  shownJoint(joint, range.upper())));
                        }
                      });
              leader.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
              leader.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                  units.toPhoenix(limits.upper());
              leader.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
              leader.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                  units.toPhoenix(limits.lower());
            });
    profile(math, units, control, leader, notes);

    List<FollowerSettings> followers = new ArrayList<>();
    for (MechanismWiring.Motor motor : wiring.motors().subList(1, wiring.motors().size())) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      output(config, motor, control);
      config.Feedback.SensorToMechanismRatio = ratio(ratioName, units.ratioToPhoenix(rotorRatio));
      followers.add(
          new FollowerSettings(
              motor.id(),
              new Follower(wiring.leader().id(), alignment(wiring.leader(), motor)),
              config));
    }
    return new Settings(leader, followers, cancoder, units, notes);
  }

  /** Phoenix's feedback source for how the wiring reads the CANcoder. */
  private static FeedbackSensorSourceValue source(MechanismWiring.Feedback feedback) {
    return switch (feedback) {
      case ROTOR -> FeedbackSensorSourceValue.RotorSensor;
      case REMOTE -> FeedbackSensorSourceValue.RemoteCANcoder;
      case FUSED -> FeedbackSensorSourceValue.FusedCANcoder;
      case SYNC -> FeedbackSensorSourceValue.SyncCANcoder;
    };
  }

  /** A motor's direction, neutral mode and current limits. */
  private static void output(
      TalonFXConfiguration config, MechanismWiring.Motor motor, MechanismControl control) {
    config.MotorOutput.Inverted =
        motor.clockwise()
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode =
        control.neutralMode() == MechanismControl.NeutralMode.BRAKE
            ? NeutralModeValue.Brake
            : NeutralModeValue.Coast;
    MechanismControl.CurrentLimits limits = control.currentLimits();
    CurrentLimitsConfigs current = config.CurrentLimits;
    current.StatorCurrentLimitEnable = true;
    current.StatorCurrentLimit = limits.stator().in(Amps);
    current.SupplyCurrentLimitEnable = true;
    current.SupplyCurrentLimit = limits.supply().in(Amps);
    current.SupplyCurrentLowerLimit = limits.supplyLower().in(Amps);
    current.SupplyCurrentLowerTime = limits.supplyLowerTime().in(Seconds);
  }

  private static MotorAlignmentValue alignment(
      MechanismWiring.Motor leader, MechanismWiring.Motor follower) {
    return leader.clockwise() == follower.clockwise()
        ? MotorAlignmentValue.Aligned
        : MotorAlignmentValue.Opposed;
  }

  /**
   * A slot's gains in Phoenix's units: the team's where given, the definition's seeds where not,
   * and gravity's type and offset.
   *
   * @param rotorRatio the rotor's turns for each turn (or meter) of the joint, as the definition
   *     writes it, for naming a gain in its frame
   */
  private static SlotConfigs slot(
      RobotMath math,
      PhoenixUnits units,
      MechanismControl.Gains gains,
      double rotorRatio,
      List<String> notes) {
    Joint joint = units.joint();
    RobotMath.Gains seed = math.idealGains(joint);
    SlotConfigs slot = new SlotConfigs();
    slot.kP = units.gainToPhoenix(gains.kP());
    slot.kI = units.gainToPhoenix(gains.kI());
    slot.kD = units.gainToPhoenix(gains.kD());
    slot.kS = volts(joint, "kS", gains.kS());
    double kV = gains.kV().orElse(seed.kV());
    if (gains.kV().isPresent()) {
      String far = farFromIdeal(units, "kV", kV, seed.kV(), rotorRatio);
      if (far != null) {
        notes.add(far);
      }
    }
    slot.kV = units.gainToPhoenix(kV);
    slot.kA = units.gainToPhoenix(gains.kA().orElse(seed.kA()));
    gravity(math, units, gains.kG(), seed.kG(), slot, notes);
    return slot;
  }

  /** Gravity's type, offset and kG in a slot. */
  private static void gravity(
      RobotMath math,
      PhoenixUnits units,
      Optional<Double> tuned,
      double seed,
      SlotConfigs slot,
      List<String> notes) {
    Joint joint = units.joint();
    boolean massless = joint.link().mass().isMassless();
    if (joint instanceof Joint.Revolute) {
      RobotMath.GravityFit fit = math.gravityFit(joint);
      if (fit.peak() == 0) {
        if (tuned.isPresent() && tuned.get() != 0) {
          throw new IllegalArgumentException(
              String.format(
                  Locale.ROOT,
                  "%s has a tuned kG of %s V, but gravity doesn't turn it in the definition (%s):"
                      + " Phoenix's Arm_Cosine needs where its weight sits, which only its mass"
                      + " gives",
                  joint.label(),
                  shown(tuned.get()),
                  massless
                      ? joint.link().label() + " has no mass"
                      : "its weight sits on its axis, or it turns about the vertical"));
        }
        if (massless && weighs(math, joint)) {
          notes.add(
              joint.link().label()
                  + " has no mass: "
                  + joint.label()
                  + " has no gravity seed, kG 0, and no offset for Phoenix's Arm_Cosine");
        }
        return;
      }
      double offset = fit.offset() / (2 * Math.PI);
      slot.GravityType = GravityTypeValue.Arm_Cosine;
      slot.GravityArmPositionOffset = gravityOffset(joint, offset, notes);
      double kG = tuned.orElse(seed);
      slot.kG = volts(joint, "kG", kG);
      if (tuned.isPresent()) {
        String far = farFromIdeal(units, "kG", kG, seed, 0);
        if (far != null) {
          notes.add(far);
        }
      }
      if (fit.misfit() > MISFIT) {
        notes.add(
            String.format(
                Locale.ROOT,
                "one cosine misses %s's gravity by %.0f%% of its peak, at %s: Phoenix's Arm_Cosine"
                    + " can't follow it, so feed RobotMath.gravityTorque at the joint's angle as"
                    + " the request's feedforward",
                joint.label(),
                fit.misfit() * 100,
                shownJoint(joint, fit.worstAt())));
      }
    } else {
      double kG = tuned.orElse(seed);
      slot.GravityType = GravityTypeValue.Elevator_Static;
      slot.kG = volts(joint, "kG", kG);
      if (seed == 0 && massless && tuned.isEmpty() && weighs(math, joint)) {
        notes.add(
            joint.link().label() + " has no mass: " + joint.label() + " has no gravity seed, kG 0");
      }
      if (tuned.isPresent() && seed != 0) {
        String far = farFromIdeal(units, "kG", kG, seed, 0);
        if (far != null) {
          notes.add(far);
        }
      }
      if (kG == 0) {
        return;
      }
    }
    Joint tilting = tilting(math, joint);
    if (tilting != null) {
      notes.add(
          String.format(
              Locale.ROOT,
              "%s rides %s, whose turning tilts its gravity: Phoenix's onboard gravity holds it only"
                  + " with %s at its reference, so feed RobotMath.gravityTorque at the joints'"
                  + " positions as the request's feedforward",
              joint.label(),
              tilting.label(),
              tilting.label()));
    }
  }

  /**
   * Phoenix's {@code GravityArmPositionOffset} for an offset in rotations: refused past ±0.25,
   * noted near it.
   */
  private static double gravityOffset(Joint joint, double offset, List<String> notes) {
    if (Math.abs(offset) > MAX_GRAVITY_OFFSET + 1e-9) {
      double turned = offset > 0 ? 0.5 - offset : -0.5 - offset;
      turned = Math.abs(turned) < 1e-9 ? 0 : turned;
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s's gravity offset is %s rotations, past Phoenix's ±0.25: its positive direction"
                  + " lowers it. Turn its axis around in the definition (and its motors' and"
                  + " encoder's directions in the wiring), so that positive raises it; the offset"
                  + " is then %s",
              joint.label(),
              shown(offset),
              shown(turned)));
    }
    if (Math.abs(offset) > MAX_GRAVITY_OFFSET - GRAVITY_OFFSET_MARGIN) {
      notes.add(
          String.format(
              Locale.ROOT,
              "%s's gravity offset is %s rotations, at Phoenix's limit of ±0.25: its center of mass"
                  + " is %s from straight above or below the pivot at its zero, and a CAD change"
                  + " that moves it past makes the offset one Phoenix can't take. Zero %s nearer"
                  + " horizontal in CAD",
              joint.label(),
              shown(offset),
              shownDegrees((MAX_GRAVITY_OFFSET - Math.abs(offset)) * 2 * Math.PI),
              joint.label()));
    }
    return Math.max(-MAX_GRAVITY_OFFSET, Math.min(MAX_GRAVITY_OFFSET, offset));
  }

  /** Motion Magic's limits, held under the joint's free speed, and Expo's from the slot. */
  private static void profile(
      RobotMath math,
      PhoenixUnits units,
      MechanismControl control,
      TalonFXConfiguration config,
      List<String> notes) {
    Joint joint = units.joint();
    control
        .profile()
        .ifPresent(
            profile -> {
              double free = math.freeSpeed(joint);
              if (profile.cruiseVelocity() > free) {
                throw new IllegalArgumentException(
                    String.format(
                        Locale.ROOT,
                        "%s's Motion Magic cruise, %s %s/s, is %s times its free speed, %s %s/s:"
                            + " the profile would ask more than its motors turn it at 12 V",
                        joint.label(),
                        shown(units.toPhoenix(profile.cruiseVelocity())),
                        units.symbol(),
                        shown(profile.cruiseVelocity() / free),
                        shown(units.toPhoenix(free)),
                        units.symbol()));
              }
              config.MotionMagic.MotionMagicCruiseVelocity =
                  profiled(units, "cruise", profile.cruiseVelocity());
              config.MotionMagic.MotionMagicAcceleration =
                  profiled(units, "acceleration", profile.acceleration());
              config.MotionMagic.MotionMagicJerk = profiled(units, "jerk", profile.jerk());
            });
    // Expo's profile from the slot's own kV and kA, in the same units: its defaults are a bare
    // rotor's, which in a mechanism's units profile it far faster than it goes.
    SlotConfigs slot = SlotConfigs.from(config.Slot0);
    if (slot.kV >= MIN_EXPO_KV
        && slot.kV <= MAX_EXPO_KV
        && slot.kA >= MIN_EXPO_KA
        && slot.kA <= MAX_EXPO_KA) {
      config.MotionMagic.MotionMagicExpo_kV = slot.kV;
      config.MotionMagic.MotionMagicExpo_kA = slot.kA;
    } else {
      notes.add(
          String.format(
              Locale.ROOT,
              "%s's kV and kA, %s and %s, are outside what Motion Magic Expo takes (%s to %s, %s to"
                  + " %s): Expo is left at Phoenix's defaults, a bare rotor's",
              joint.label(),
              shown(slot.kV),
              shown(slot.kA),
              shown(MIN_EXPO_KV),
              shown(MAX_EXPO_KV),
              shown(MIN_EXPO_KA),
              shown(MAX_EXPO_KA)));
    }
  }

  private static double profiled(PhoenixUnits units, String what, double value) {
    Joint joint = units.joint();
    double phoenix = units.toPhoenix(value);
    if (phoenix > MAX_PROFILE) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s's Motion Magic %s, %s, is past the most Phoenix holds, %s",
              joint.label(),
              what,
              shown(phoenix),
              shown(MAX_PROFILE)));
    }
    return phoenix;
  }

  /** A ratio Phoenix holds, or a refusal naming it. */
  private static double ratio(String name, double ratio) {
    if (!(Math.abs(ratio) <= MAX_RATIO) || ratio == 0) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s would be %s, and Phoenix holds from -1000 to 1000, never 0",
              name,
              shown(ratio)));
    }
    return ratio;
  }

  /** Volts Phoenix holds as kS or kG, or a refusal. */
  private static double volts(Joint joint, String gain, double volts) {
    if (volts < MIN_VOLTS || volts > MAX_VOLTS) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s's %s, %s V, is past what Phoenix holds, -128 to 127 V",
              joint.label(),
              gain,
              shown(volts)));
    }
    return volts;
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
  private static @Nullable String farFromIdeal(
      PhoenixUnits units, String gain, double tuned, double ideal, double rotorRatio) {
    Joint joint = units.joint();
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
            shownGain(joint, gain, tuned),
            shown(ratio),
            shownGain(joint, gain, ideal));
    if (rotorRatio == 0) {
      return why;
    }
    List<Frame> frames = frames(units, rotorRatio);
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

  private static List<Frame> frames(PhoenixUnits units, double rotorRatio) {
    Joint joint = units.joint();
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
    if (units.perPhoenixUnit() != 1) {
      frames.add(new Frame("turn of its drum", 1 / units.perPhoenixUnit()));
    }
    return frames;
  }

  /**
   * Whether gravity would move the joint once its link is weighed, so a link without mass leaves a
   * seed missing: an arm with stops about an axis that isn't vertical, or a slide that isn't level.
   * Not a roller, a wheel or a flywheel, which turn freely and balanced, nor a turret.
   */
  private static boolean weighs(RobotMath math, Joint joint) {
    double up = Math.abs(robotAxis(math, joint).getZ());
    return joint instanceof Joint.Revolute ? joint.range().isPresent() && up < 1 - 1e-9 : up > 1e-9;
  }

  /** A joint's axis, a unit vector in the robot's frame, the joints at their references. */
  private static Translation3d robotAxis(RobotMath math, Joint joint) {
    Vector3 axis = joint.axis().unit();
    return new Translation3d(axis.x(), axis.y(), axis.z())
        .rotateBy(math.pose(joint, Map.of()).getRotation());
  }

  /**
   * The first joint above this one whose turning tilts gravity at it: a revolute joint about an
   * axis that isn't vertical, with the joints at their references. Null when none does.
   */
  private static @Nullable Joint tilting(RobotMath math, Joint joint) {
    Part up = joint.parent();
    while (up instanceof Joint parent) {
      if (parent instanceof Joint.Revolute && Math.abs(robotAxis(math, parent).getZ()) < 1 - 1e-9) {
        return parent;
      }
      up = parent.parent();
    }
    return null;
  }

  /**
   * Phoenix's discontinuity point for an absolute encoder, read as its wiring sets it, growing with
   * the joint: the definition's, which reads it as its ratio's sign says, turned round for an
   * encoder geared to turn back.
   */
  private static double discontinuityPoint(RobotMath math, Sensor.Encoder encoder) {
    double point = math.discontinuityPoint(encoder);
    return encoder.ratio(encoder.joint()) < 0 ? 1 - point : point;
  }

  /** The encoder's turns for each of its joint's Phoenix units: a magnitude. */
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

  /** A position or a distance in the joint's units, as people read it: degrees, or meters. */
  private static String shownJoint(Joint joint, double inJointUnits) {
    return joint instanceof Joint.Revolute
        ? shownDegrees(inJointUnits)
        : shown(inJointUnits) + " m";
  }

  private static String shownDegrees(double radians) {
    return String.format(Locale.ROOT, "%.1f°", Math.toDegrees(radians));
  }

  private static String shownGain(Joint joint, String gain, double value) {
    String per = joint instanceof Joint.Revolute ? "rad" : "m";
    return switch (gain) {
      case "kV" -> shown(value) + " V/(" + per + "/s)";
      case "kA" -> shown(value) + " V/(" + per + "/s²)";
      default -> shown(value) + " V";
    };
  }
}
