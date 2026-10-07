package frc.robot.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.util.Signal;
import frc.robot.lib.phoenix.PhoenixUnits;
import frc.robot.lib.rev.RevMechanism;
import frc.robot.lib.wiring.CanPorts;
import frc.robot.lib.wiring.ControllerPins;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.filter.Debouncer;

/**
 * A driven joint on REV hardware: a SPARK MAX or SPARK Flex leading its drive's motors, the rest
 * following it, read by its motor's own encoder. Every setting comes from the REV helper ({@link
 * RevMechanism}), from the robot's definition, the joint's wiring and its control ({@link
 * ControllerSettings#spark}), as a TalonFX's come from the Phoenix helper: nothing is retyped here.
 *
 * <p>Positions go to the SPARK's loop with the profile's point and the feedforward the robot works
 * out in volts ({@code PhoenixMechanism.Feedforward}); speeds the same way. A joint is read by its
 * rotor: one with an absolute encoder on the SPARK's data port is told where it is from that
 * encoder's first good reading; one without takes its reference as where it is at power on, until
 * it homes. One whose loop closes on that encoder is read by it instead, each loop, and its
 * positions go to the SPARK's loop in the encoder's turns ({@link
 * RevMechanism.Settings#loopRatio}). Its settings are kept through a brownout: REV's controllers
 * fall back to what's saved on them.
 */
public final class JointIOSpark implements JointIO {
  /** At startup, configuring a device is tried this many times. */
  private static final int TRIES = 5;

  private final SparkBase leader;
  private final List<SparkBase> followers = new ArrayList<>();
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController loop;
  private final MechanismWiring wiring;
  private final PhoenixUnits units;
  private final double rotorRatio;
  private final String unconfigured;

  /** A controller counts as silent once it has been for half a second, not at one glitch. */
  private final Debouncer connected = new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  /** The REV settings for a control: the joint's, its drive's and its wiring's, with it. */
  private final Function<MechanismControl, RevMechanism.Settings> configure;

  private RevMechanism.Settings settings;

  /** Where the joint is and how fast it moves, last read, in the mechanism's rotations. */
  private double position;

  private double velocity;

  /** Whether its soft limits are switched off, while it homes past them. */
  private boolean softLimitsOff;

  /** What the robot's definition derives: where the joint is from its absolute encoder. */
  private final RobotMath math;

  /** The absolute encoder on the leader's data port, and the definition's, if it has one. */
  private final @Nullable SparkAbsoluteEncoder absolute;

  private final Sensor.@Nullable Encoder absoluteModel;

  /** Whether the motor's encoder has been told where the joint is, from the absolute encoder. */
  private boolean seeded;

  /**
   * Configures every device of the joint, waiting for each, at startup.
   *
   * @param math what the robot's definition derives: the joint's drive, ratios and seeds
   * @param setup the joint, its control, and its wiring, which it must have, on a SPARK
   * @throws IllegalArgumentException for a setup without wiring, and what its settings refuse
   *     ({@link ControllerSettings#spark})
   */
  public JointIOSpark(RobotMath math, JointSetup setup) {
    this(math, setup, ControllerPins.NONE);
  }

  /**
   * The same, with what's on its leading SPARK's limit inputs.
   *
   * @param pins its switches there, from the robot's sensor wiring
   */
  public JointIOSpark(RobotMath math, JointSetup setup, ControllerPins pins) {
    Joint joint = setup.joint();
    wiring =
        setup
            .wiring()
            .orElseThrow(
                () -> new IllegalArgumentException(joint.label() + " has no wiring: wire it"));
    this.math = math;
    configure = control -> ControllerSettings.spark(math, joint, wiring, control);
    settings = configure.apply(setup.control());
    // On Systemcore's own CAN, as the settings require, and the definition's encoder on the data
    // port, which they found.
    CanBus.Port bus = (CanBus.Port) wiring.canBus();
    Sensor.Encoder model =
        wiring.dataPort().isPresent() ? JointSetup.absoluteEncoder(math, joint) : null;
    absoluteModel = model;
    units = settings.units();
    rotorRatio = settings.rotorRatio();
    // What's doubtful in the settings, though allowed: said once, in the log and on the console.
    Logger.recordOutput(
        "Joints/" + joint.label() + "/SettingsNotes", settings.notes().toArray(String[]::new));
    for (String note : settings.notes()) {
      DriverStationErrors.reportWarning(joint.label() + ": " + note, false);
    }
    List<String> failed = new ArrayList<>();
    leader = spark(CanPorts.of(bus.port()), wiring.leader().id());
    SparkBaseConfig leading = settings.leader();
    RevMechanism.limits(leading.limitSwitch, pins);
    if (!applied(
        () ->
            leader.configure(
                leading, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))) {
      failed.add("motor " + wiring.leader().id());
    }
    for (RevMechanism.Follower each : settings.followers()) {
      SparkBase follower = spark(CanPorts.of(bus.port()), each.id());
      if (!applied(
          () ->
              follower.configure(
                  each.config(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters))) {
        failed.add("motor " + each.id());
      }
      followers.add(follower);
    }
    unconfigured = String.join(", ", failed);
    encoder = leader.getEncoder();
    loop = leader.getClosedLoopController();
    absolute = model != null ? leader.getAbsoluteEncoder() : null;
    encoder.setPosition(units.toPhoenix(joint.reference()) * rotorRatio);
  }

  /** The leading SPARK: for the tests, which read what it was told through REV's simulation. */
  SparkBase leader() {
    return leader;
  }

  private SparkBase spark(CANPort port, int id) {
    return wiring.controller() == MechanismWiring.Controller.SPARK_FLEX
        ? new SparkFlex(port, id, SparkLowLevel.MotorType.kBrushless)
        : new SparkMax(port, id, SparkLowLevel.MotorType.kBrushless);
  }

  /** Tries something a device must confirm, a few times: at startup. Whether it took. */
  private static boolean applied(Supplier<REVLibError> attempt) {
    for (int i = 0; i < TRIES; i++) {
      if (attempt.get() == REVLibError.kOk) {
        return true;
      }
    }
    return false;
  }

  @Override
  public void retune(MechanismControl control) {
    settings = configure.apply(control);
    // Without waiting, as the loop runs, and not saved: the pit's tuning is the session's.
    SparkBaseConfig tuning = settings.tuning(wiring, control);
    leader.configureAsync(
        tuning, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    for (SparkBase follower : followers) {
      follower.configureAsync(
          tuning, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    seed();
    Signal<Double> rotor = encoder.getPosition();
    Signal<Double> rpm = encoder.getVelocity();
    double share = leader.getAppliedOutput().get(0.0);
    double current = Math.abs(leader.getOutputCurrent().get(0.0));
    inputs.connected = connected.calculate(rotor.isValid() && rpm.isValid());
    double fromEncoder = fromAbsolute();
    if (settings.closedLoop() && !Double.isNaN(fromEncoder)) {
      // Its loop runs on its absolute encoder, whose word is where the joint is.
      position = fromEncoder;
      seededAt = Double.NaN;
    } else if (!Double.isNaN(seededAt)) {
      // Told where it is this loop: its encoder's reading catches up on the next.
      position = seededAt;
      seededAt = Double.NaN;
    } else {
      position = rotor.get(0.0) / rotorRatio;
    }
    velocity = rpm.get(0.0) / 60 / rotorRatio;
    inputs.absolutePosition =
        Double.isNaN(fromEncoder) ? Double.NaN : units.fromPhoenix(fromEncoder);
    inputs.position = units.fromPhoenix(position);
    inputs.velocity = units.fromPhoenix(velocity);
    inputs.appliedVolts = share * leader.getBusVoltage().get(0.0);
    inputs.statorCurrentAmps = current;
    // A SPARK doesn't measure what it draws from the battery: its share of the motor's current.
    inputs.supplyCurrentAmps = Math.abs(share) * current;
    inputs.forwardLimit = leader.getForwardLimitSwitch().isPressed().get(false);
    inputs.reverseLimit = leader.getReverseLimitSwitch().isPressed().get(false);
    inputs.unconfigured = unconfigured;
  }

  /** Where it was seeded this loop, in the mechanism's rotations: NaN otherwise. */
  private double seededAt = Double.NaN;

  /**
   * Tells the motor's encoder where the joint is, from the absolute encoder's first good reading:
   * once, at startup.
   */
  private void seed() {
    double at = seeded ? Double.NaN : fromAbsolute();
    if (Double.isNaN(at)) {
      return;
    }
    encoder.setPosition(at * rotorRatio);
    seededAt = at;
    seeded = true;
  }

  /**
   * Where the absolute encoder on its data port says the joint is, in the mechanism's rotations:
   * NaN without one, or before its first good reading. The SPARK reads it with its direction and
   * offset applied, from the window's low edge ({@link RevMechanism.Settings#windowEdge}).
   */
  private double fromAbsolute() {
    SparkAbsoluteEncoder reading = absolute;
    Sensor.Encoder model = absoluteModel;
    if (reading == null || model == null) {
      return Double.NaN;
    }
    Signal<Double> fromEdge = reading.getPosition();
    return fromEdge.isValid()
        ? units.toPhoenix(math.fromAbsolute(model, settings.windowEdge() + fromEdge.get()))
        : Double.NaN;
  }

  @Override
  public void setPosition(double position, double velocity, double feedforwardVolts) {
    double at = units.toPhoenix(position);
    double moving = units.toPhoenix(velocity);
    loop.setSetpoint(
        at * settings.loopRatio() + settings.loopZero(),
        SparkLowLevel.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforwardVolts + settings.feedforward().atPosition(at, moving, this.velocity),
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setVelocity(double velocity) {
    double moving = units.toPhoenix(velocity);
    loop.setSetpoint(
        moving * settings.loopRatio() * 60,
        SparkLowLevel.ControlType.kVelocity,
        ClosedLoopSlot.kSlot1,
        settings.feedforward().atSpeed(position, moving),
        SparkClosedLoopController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setDutyCycle(double share) {
    leader.setThrottle(share);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }

  /** Switches its soft limits off first: a SPARK keeps them in its own settings. */
  @Override
  public void home(double volts) {
    if (settings.softLimits().isPresent() && !softLimitsOff) {
      leader.configureAsync(
          settings.softLimitsEnabled(wiring, false),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
      softLimitsOff = true;
    }
    leader.setVoltage(volts);
  }

  /** Sets where the joint is, and its soft limits back on if homing switched them off. */
  @Override
  public void resetPosition(double position) {
    encoder.setPosition(units.toPhoenix(position) * rotorRatio);
    if (softLimitsOff) {
      leader.configureAsync(
          settings.softLimitsEnabled(wiring, true),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);
      softLimitsOff = false;
    }
  }
}
