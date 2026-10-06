package frc.robot.subsystems.mechanisms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.lib.phoenix.PhoenixUnits;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.lib.wiring.ControllerPins;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Angle;

/**
 * A driven joint on CTRE hardware: a TalonFX, or a TalonFXS as its wiring says, leading its drive's
 * motors, the rest following it, and its CANcoder, if its wiring names one. A TalonFXS takes the
 * settings a TalonFX would, in its own terms ({@link PhoenixMechanism#talonFxs}). Every setting
 * comes from the Phoenix helper ({@link PhoenixMechanism}), from the robot's definition, the
 * joint's wiring and its control: ratios, gains seeded from the definition, gravity, soft limits,
 * current limits, directions and followers. Nothing is retyped here.
 *
 * <p>Runs on Phoenix 6 without a Pro license, unless the definition's motors are its FOC ones
 * ({@code Motor.KRAKEN_X60_FOC}): only then do its requests use FOC. Positions go to the motor
 * controller as {@code PositionVoltage} with the profile's velocity, which its own kV, kS and
 * gravity turn into volts; speeds as {@code VelocityVoltage}. A joint read by its rotor is told
 * where it is from its CANcoder's first good reading, when its wiring reads one that way ({@code
 * Feedback.ROTOR}); with no CANcoder, it takes its reference as where it is at power on.
 */
public final class JointIOTalon implements JointIO {
  /** At startup, configuring a device is tried this many times, waiting this long each time. */
  private static final int TRIES = 5;

  private static final double STARTUP_TIMEOUT_SECONDS = 0.25;

  private final Talon leader;
  private final List<Talon> followers = new ArrayList<>();
  private final PhoenixUnits units;
  private final BaseStatusSignal[] signals;
  private final BaseStatusSignal position;
  private final BaseStatusSignal velocity;
  private final BaseStatusSignal volts;
  private final BaseStatusSignal statorCurrent;
  private final BaseStatusSignal supplyCurrent;

  /** What's on the leader's limits: a switch on a pin, whose polarity its reading takes. */
  private final ControllerPins pins;

  /** The leader's limit pins, as they are: closed to ground, or open. */
  private final StatusSignal<ForwardLimitValue> forwardLimit;

  private final StatusSignal<ReverseLimitValue> reverseLimit;

  /** A motor controller counts as silent once it has been for half a second, not at one glitch. */
  private final Debouncer connected = new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final VoltageOut voltageRequest;
  private final DutyCycleOut dutyCycleRequest;
  private final VoltageOut homingRequest;
  private final NeutralOut neutral = new NeutralOut();
  private final String unconfigured;

  /** The Phoenix settings for a control: the joint's, its drive's and its wiring's, with it. */
  private final Function<MechanismControl, PhoenixMechanism.Settings> configure;

  /**
   * The CANcoder the robot reads once, to tell the rotor where the joint is, and the definition's
   * encoder it is: a joint whose motor controller reads its rotor ({@code Feedback.ROTOR}).
   */
  private final @Nullable StatusSignal<Angle> seedFrom;

  private final Sensor.@Nullable Encoder seedModel;
  private final RobotMath math;
  private boolean seeded;

  /** Where it was seeded this loop, in Phoenix's units: NaN otherwise. */
  private double seededAt = Double.NaN;

  /**
   * Configures every device of the joint, waiting for each, at startup.
   *
   * @param math what the robot's definition derives: the joint's drive, ratios and seeds
   * @param setup the joint, its control, and its wiring, which it must have
   * @throws IllegalArgumentException for a setup without wiring, wiring that names a CANcoder the
   *     definition doesn't have, and what the Phoenix helper refuses
   */
  public JointIOTalon(RobotMath math, JointSetup setup) {
    this(math, setup, ControllerPins.NONE);
  }

  /**
   * The same, with what's on its leader's limits: switches on its pins, or a limit switch on a
   * CANdi it reads over CAN.
   *
   * @param pins its limits, from the robot's sensor wiring
   */
  public JointIOTalon(RobotMath math, JointSetup setup, ControllerPins pins) {
    Joint joint = setup.joint();
    MechanismWiring wiring =
        setup
            .wiring()
            .orElseThrow(
                () -> new IllegalArgumentException(joint.label() + " has no wiring: wire it"));
    Drive drive = math.drivesOf(joint).get(0);
    Sensor.Encoder absolute = wiring.cancoder().isPresent() ? absolute(math, joint) : null;
    configure =
        control ->
            absolute == null
                ? PhoenixMechanism.configure(math, joint, drive, wiring, control)
                : PhoenixMechanism.configure(math, joint, drive, absolute, wiring, control);
    PhoenixMechanism.Settings settings = configure.apply(setup.control());
    units = settings.units();
    // What's doubtful in the settings, though allowed: said once, in the log and on the console.
    Logger.recordOutput(
        "Joints/" + joint.label() + "/SettingsNotes", settings.notes().toArray(String[]::new));
    for (String note : settings.notes()) {
      DriverStationErrors.reportWarning(joint.label() + ": " + note, false);
    }
    CANBus bus = PhoenixMechanism.bus(wiring.canBus());
    List<String> failed = new ArrayList<>();
    Motor motor = ((Actuator.Motors) drive.actuator()).motor();
    leader = new Talon(wiring.controller(), wiring.leader().id(), bus, motor);
    TalonFXConfiguration leading = settings.leader();
    PhoenixMechanism.limits(leading.HardwareLimitSwitch, pins);
    if (!applied(() -> leader.apply(leading, STARTUP_TIMEOUT_SECONDS))) {
      failed.add("motor " + wiring.leader().id());
    }
    for (PhoenixMechanism.FollowerSettings each : settings.followers()) {
      Talon follower = new Talon(wiring.controller(), each.id(), bus, motor);
      if (!applied(() -> follower.apply(each.configuration(), STARTUP_TIMEOUT_SECONDS))) {
        failed.add("motor " + each.id());
      }
      follower.device().setControl(each.request());
      followers.add(follower);
    }
    StatusSignal<Angle> cancoderReading = null;
    if (settings.cancoder().isPresent()) {
      CANcoderConfiguration config = settings.cancoder().get();
      int id = wiring.cancoder().orElseThrow().id();
      CANcoder cancoder = new CANcoder(id, bus);
      if (!applied(() -> cancoder.getConfigurator().apply(config, STARTUP_TIMEOUT_SECONDS))) {
        failed.add("CANcoder " + id);
      }
      cancoderReading = cancoder.getAbsolutePosition();
    }
    boolean readsRotor =
        wiring.cancoder().isEmpty()
            || wiring.cancoder().get().feedback() == MechanismWiring.Feedback.ROTOR;
    if (readsRotor) {
      // Where it rests, until a CANcoder read by the robot says where it is (seed).
      leader.device().setPosition(units.toPhoenix(joint.reference()), STARTUP_TIMEOUT_SECONDS);
    }
    this.math = math;
    seedFrom = readsRotor ? cancoderReading : null;
    seedModel = readsRotor ? absolute : null;
    unconfigured = String.join(", ", failed);

    position = leader.device().getPosition();
    velocity = leader.device().getVelocity();
    volts = leader.device().getMotorVoltage();
    statorCurrent = leader.device().getStatorCurrent();
    supplyCurrent = leader.device().getSupplyCurrent();
    this.pins = pins;
    forwardLimit = leader.device().getForwardLimit();
    reverseLimit = leader.device().getReverseLimit();
    signals =
        new BaseStatusSignal[] {
          position, velocity, volts, statorCurrent, supplyCurrent, forwardLimit, reverseLimit
        };
    BaseStatusSignal.setUpdateFrequencyForAll(RobotLoop.FREQUENCY_HZ, signals);
    List<ParentDevice> devices = new ArrayList<>();
    for (Talon follower : followers) {
      devices.add(follower.parent());
    }
    devices.add(leader.parent());
    ParentDevice.optimizeBusUtilizationForAll(devices.toArray(ParentDevice[]::new));

    boolean foc = foc(drive);
    positionRequest = new PositionVoltage(0).withEnableFOC(foc);
    velocityRequest = new VelocityVoltage(0).withEnableFOC(foc);
    voltageRequest = new VoltageOut(0).withEnableFOC(foc);
    dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(foc);
    homingRequest = new VoltageOut(0).withEnableFOC(foc).withIgnoreSoftwareLimits(true);
  }

  @Override
  public void retune(MechanismControl control) {
    PhoenixMechanism.Settings settings = configure.apply(control);
    // Without waiting, as the loop runs: what Phoenix doesn't take shows in the log as it reads.
    leader.apply(settings.leader().Slot0, 0);
    leader.apply(settings.leader().CurrentLimits, 0);
    for (int i = 0; i < followers.size(); i++) {
      followers.get(i).apply(settings.followers().get(i).configuration().CurrentLimits, 0);
    }
  }

  /**
   * Whether a switch on a pin is tripped: closed to ground for a normally open one, open for a
   * normally closed one. Phoenix reports the pin as it is, whatever polarity it's told for its own
   * stop. Nothing on the pin, or a CANdi's limit read over CAN, reads nothing here.
   */
  private static boolean tripped(Optional<ControllerPins.Limit> limit, boolean closed) {
    return limit.orElse(null) instanceof ControllerPins.OnPin pin
        && closed == (pin.polarity() == SensorWiring.Polarity.NORMALLY_OPEN);
  }

  /**
   * Whether the definition's motors are driven with FOC, which needs Phoenix Pro: its FOC motors
   * are named so ({@code Motor.KRAKEN_X60_FOC}, "Kraken X60 FOC").
   */
  static boolean foc(Drive drive) {
    return drive.actuator() instanceof Actuator.Motors motors
        && motors.motor().name().endsWith(" FOC");
  }

  /** The definition's absolute encoder on the joint alone: the one its CANcoder is. */
  private static Sensor.Encoder absolute(RobotMath math, Joint joint) {
    Sensor.Encoder encoder = JointSetup.absoluteEncoder(math, joint);
    if (encoder == null) {
      throw new IllegalArgumentException(
          joint.label()
              + "'s wiring names a CANcoder, and its definition has no absolute encoder on it");
    }
    return encoder;
  }

  /**
   * Tries something a device must confirm, a few times: at startup, where waiting is fine. Whether
   * it took.
   */
  private static boolean applied(Supplier<StatusCode> attempt) {
    for (int i = 0; i < TRIES; i++) {
      if (attempt.get().isOK()) {
        return true;
      }
    }
    return false;
  }

  /**
   * Tells the rotor where the joint is, from its CANcoder's first good reading: once, at startup.
   * The CANcoder's own settings give its offset, direction and wrap.
   */
  private void seed() {
    StatusSignal<Angle> reading = seedFrom;
    Sensor.Encoder model = seedModel;
    if (seeded || reading == null || model == null || !reading.refresh().getStatus().isOK()) {
      return;
    }
    double at = units.toPhoenix(math.fromAbsolute(model, reading.getValueAsDouble()));
    leader.device().setPosition(at, 0);
    seededAt = at;
    seeded = true;
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    seed();
    inputs.connected = connected.calculate(BaseStatusSignal.refreshAll(signals).isOK());
    // Told where it is this loop: its rotor's reading catches up on the next.
    inputs.position =
        units.fromPhoenix(Double.isNaN(seededAt) ? position.getValueAsDouble() : seededAt);
    seededAt = Double.NaN;
    inputs.velocity = units.fromPhoenix(velocity.getValueAsDouble());
    inputs.appliedVolts = volts.getValueAsDouble();
    inputs.statorCurrentAmps = Math.abs(statorCurrent.getValueAsDouble());
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.forwardLimit =
        tripped(pins.forward(), forwardLimit.getValue() == ForwardLimitValue.ClosedToGround);
    inputs.reverseLimit =
        tripped(pins.reverse(), reverseLimit.getValue() == ReverseLimitValue.ClosedToGround);
    inputs.unconfigured = unconfigured;
  }

  @Override
  public void setPosition(double position, double velocity, double feedforwardVolts) {
    leader
        .device()
        .setControl(
            positionRequest
                .withPosition(units.toPhoenix(position))
                .withVelocity(units.toPhoenix(velocity))
                .withFeedForward(feedforwardVolts));
  }

  @Override
  public void setVelocity(double velocity) {
    leader.device().setControl(velocityRequest.withVelocity(units.toPhoenix(velocity)));
  }

  @Override
  public void setVoltage(double volts) {
    leader.device().setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setDutyCycle(double share) {
    leader.device().setControl(dutyCycleRequest.withOutput(share));
  }

  @Override
  public void stop() {
    leader.device().setControl(neutral);
  }

  @Override
  public void home(double volts) {
    leader.device().setControl(homingRequest.withOutput(volts));
  }

  /** Sets the leader's position without waiting for it to confirm: this runs in the loop. */
  @Override
  public void resetPosition(double position) {
    leader.device().setPosition(units.toPhoenix(position), 0);
  }

  /**
   * One motor controller: a TalonFX, or a TalonFXS driving a motor through its connector, which
   * takes the settings worked out for a TalonFX in its own terms.
   */
  private static final class Talon {
    private final @Nullable TalonFX fx;
    private final @Nullable TalonFXS fxs;
    private final Motor motor;

    Talon(MechanismWiring.Controller controller, int id, CANBus bus, Motor motor) {
      this.motor = motor;
      if (controller == MechanismWiring.Controller.TALON_FXS) {
        fx = null;
        fxs = new TalonFXS(id, bus);
      } else {
        fx = new TalonFX(id, bus);
        fxs = null;
      }
    }

    CommonTalon device() {
      return fx != null ? fx : Objects.requireNonNull(fxs);
    }

    ParentDevice parent() {
      return fx != null ? fx : Objects.requireNonNull(fxs);
    }

    StatusCode apply(TalonFXConfiguration config, double timeout) {
      return fx != null
          ? fx.getConfigurator().apply(config, timeout)
          : Objects.requireNonNull(fxs)
              .getConfigurator()
              .apply(PhoenixMechanism.talonFxs(config, motor), timeout);
    }

    StatusCode apply(Slot0Configs slot, double timeout) {
      return fx != null
          ? fx.getConfigurator().apply(slot, timeout)
          : Objects.requireNonNull(fxs).getConfigurator().apply(slot, timeout);
    }

    StatusCode apply(CurrentLimitsConfigs limits, double timeout) {
      return fx != null
          ? fx.getConfigurator().apply(limits, timeout)
          : Objects.requireNonNull(fxs).getConfigurator().apply(limits, timeout);
    }
  }
}
