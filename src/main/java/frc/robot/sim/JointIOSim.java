package frc.robot.sim;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import frc.robot.subsystems.mechanisms.ControllerSettings;
import frc.robot.subsystems.mechanisms.JointIO;
import frc.robot.subsystems.mechanisms.JointSetup;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.system.DCMotor;

/**
 * A simulated joint: its motor controller and its motors, as the robot's definition and the joint's
 * control describe them, moving the joint's link. Built from the model alone ({@link
 * JointSimulation}): the motors' model through the transmission, with its losses; the link and
 * everything it carries, and the motors' rotors, as the inertia ({@code RobotMath.inertia}), the
 * rotors alone for links without mass; gravity ({@link #gravityTorque}); and the joint's hard
 * stops, which stop it dead.
 *
 * <p>Gravity is what the kG tuned in the joint's block holds, through its motors, since that's what
 * a team measures: one cosine about where its weight pulls hardest for an arm, the same all along a
 * slide. Where its block tunes none, gravity is its links' weight ({@code
 * RobotMath.gravityTorque}), with the other joints where they are, and none for links without mass.
 * Gravity is the robot's, from its block, not its motor controller's: a test can run the controller
 * with another kG ({@code JointSetup.of}), and live tuning doesn't move it. An arm's tuned gravity
 * doesn't follow a parent that tilts it, as Phoenix's {@code Arm_Cosine} doesn't.
 *
 * <p>The motor controller runs as a TalonFX runs what the Phoenix helper configures: its loop every
 * millisecond, on the joint's own position and speed; its slot's gains, the control's, its kV, kA
 * and kG seeded from the definition where the control gives none, as on the robot; gravity's
 * feedforward as Phoenix gives it, one cosine of the joint's angle for an arm, steady for a slide;
 * its stator current limit; its soft limits, past which it drives no further; and neutral as its
 * control says, braking or coasting. Like a real one, it knows only where its own count says the
 * joint is: from where the joint was at power on, taken as its reference, until homing tells it
 * otherwise ({@link #powerOnAt}, {@link #resetPosition}). The battery holds 12 V, and the supply
 * current limit isn't modeled, as the drivetrain's isn't ({@code ModuleIOSim}). Friction isn't
 * either: it isn't known until measured. Joints move independently: one joint's motion doesn't push
 * on another's, which waits for the simulator's rigid-body engine.
 *
 * <p>What the robot's own settings for the joint would note ({@link ControllerSettings#notes}), for
 * the controller it's wired to, or a TalonFX with no wiring, it logs once as the robot does ({@code
 * Joints/<label>/SettingsNotes}), so the simulator says what the robot would. What the robot would
 * refuse, its configuration refused before the simulator started.
 */
final class JointIOSim implements JointIO {
  /** A TalonFX runs its closed loop every millisecond (CTRE's Phoenix 6 documentation: 1 kHz). */
  static final double DEVICE_LOOP_SECONDS = 0.001;

  private static final double BATTERY_VOLTS = 12.0;

  private enum Mode {
    NEUTRAL,
    POSITION,
    VELOCITY,
    VOLTAGE,
    DUTY_CYCLE,
    HOMING
  }

  private final Joint joint;
  private final RobotMath math;
  private final CurrentLimitedMotorSim motor;

  /** How many motors turn it: its readings are each one's, as a motor controller reads its own. */
  private final int count;

  private double statorLimitAmps;
  private final boolean brake;
  private double kP;
  private double kI;
  private double kD;
  private double kS;
  private final double kV;
  private final double kG;

  /**
   * For an arm, the angle to add to its position for its angle from where its weight pulls hardest,
   * as its control gives it or its links' mass puts it; NaN for a slide.
   */
  private final double gravityOffset;

  /**
   * Newton meters at the joint for each volt across its motors held still (newtons, for a slide):
   * their stall torque per volt, all of them, through its gearing, less its losses.
   */
  private final double torquePerVolt;

  /**
   * The kG tuned in the joint's block, which sets gravity on it; empty where none is, and its
   * links' mass does.
   */
  private final Optional<Double> weightKg;

  /** As {@link #gravityOffset}, for the block's kG: where the robot's weight pulls hardest. */
  private final double weightOffset;

  private final double softLower;
  private final double softUpper;
  private final double hardLower;
  private final double hardUpper;

  /** Where the motor controller reads the joint, less where it really is: 0 until misplaced. */
  private double offset;

  private Mode mode = Mode.NEUTRAL;
  private double setPosition;
  private double setVelocity;
  private double feedforwardVolts;
  private double setVolts;
  private double integral;

  /** Seconds of the robot's loop not yet run, under a millisecond: the next step's. */
  private double owed;

  /** What the robot's settings for the joint would note, as logged. */
  private final List<String> notes;

  JointIOSim(RobotMath math, JointSetup setup) {
    joint = setup.joint();
    this.math = math;
    MechanismControl control = setup.control();
    Drive drive = math.drivesOf(joint).get(0);
    Actuator.Motors motors = (Actuator.Motors) drive.actuator();
    DCMotor model = motors.model(drive.transmission().efficiency());
    double gearing = Math.abs(drive.transmission().shaftRadiansPer(joint));
    motor = new CurrentLimitedMotorSim(model, math.inertia(joint, Map.of()), gearing);
    torquePerVolt = gearing * model.stallTorque / model.nominalVoltage;
    count = motors.count();
    statorLimitAmps = control.currentLimits().stator().in(Amps) * count;
    brake = control.neutralMode() == MechanismControl.NeutralMode.BRAKE;
    RobotMath.Gains seed = math.idealGains(joint);
    MechanismControl.Gains gains = control.gains();
    kP = gains.kP();
    kI = gains.kI();
    kD = gains.kD();
    kS = gains.kS();
    kV = gains.kV().orElse(seed.kV());
    boolean gravityActs = math.gravityActsOn(joint);
    kG = gravityActs ? gains.kG().orElse(seed.kG()) : 0;
    gravityOffset = gravityOffset(math, gains);
    MechanismControl.Gains block =
        joint
            .driven()
            .control()
            .map(MechanismControl::gains)
            .orElse(MechanismControl.Gains.seeded());
    weightKg = gravityActs ? block.kG() : Optional.empty();
    weightOffset = gravityOffset(math, block);
    softLower =
        control
            .softLimits()
            .map(MechanismControl.SoftLimits::lower)
            .orElse(Double.NEGATIVE_INFINITY);
    softUpper =
        control
            .softLimits()
            .map(MechanismControl.SoftLimits::upper)
            .orElse(Double.POSITIVE_INFINITY);
    hardLower = joint.range().map(Range::lower).orElse(Double.NEGATIVE_INFINITY);
    hardUpper = joint.range().map(Range::upper).orElse(Double.POSITIVE_INFINITY);
    motor.place(joint.reference(), 0);
    // What's doubtful in the robot's settings, though allowed: said once, in the log, as the robot
    // says it. The Driver Station's console is the robot's alone.
    notes = ControllerSettings.notes(math, setup);
    Logger.recordOutput("Joints/" + joint.label() + "/SettingsNotes", notes.toArray(String[]::new));
  }

  /**
   * For an arm, the angle to add to its position for its angle from where its weight pulls hardest:
   * the gains' angle where given, else where its links' mass puts it, else its zero. NaN for a
   * slide.
   */
  private double gravityOffset(RobotMath math, MechanismControl.Gains gains) {
    if (!(joint instanceof Joint.Revolute)) {
      return Double.NaN;
    }
    return gains.gravityAngle().map(angle -> -angle.in(Radians)).orElse(math.gravityOffset(joint));
  }

  /** The joint. */
  Joint joint() {
    return joint;
  }

  /** What the robot's settings for the joint would note, as it logged them: for the tests. */
  List<String> notes() {
    return notes;
  }

  /**
   * Gravity's torque on the joint, positive turning it forward, with the joints where they stand:
   * newton meters, or newtons for a slide. What its block's tuned kG holds, through its motors;
   * where none is tuned, its links' weight.
   */
  double gravityTorque(Map<Joint, Double> positions) {
    if (weightKg.isEmpty()) {
      return -math.gravityTorque(joint, positions);
    }
    double volts =
        Double.isNaN(weightOffset)
            ? weightKg.get()
            : weightKg.get() * Math.cos(position() + weightOffset);
    return -volts * torquePerVolt;
  }

  /** Where the joint really is, in its units. */
  double position() {
    return motor.positionRadians();
  }

  /** How fast it really moves. */
  double velocity() {
    return motor.velocityRadPerSec();
  }

  /** Puts the joint somewhere, still, where its motor controller reads it too: a test's start. */
  void place(double position) {
    motor.place(position, 0);
    offset = 0;
  }

  /**
   * Powers it on somewhere other than its reference, still: its motor controller reads it at its
   * reference anyway, as a joint read by its motor alone does, until it's homed.
   */
  void powerOnAt(double position) {
    motor.place(position, 0);
    offset = joint.reference() - position;
  }

  /**
   * Runs the motor controller and the joint for a time, a millisecond at a time as the motor
   * controller does.
   *
   * @param gravityTorque gravity's torque on the joint through the step, positive turning it
   *     forward: newton meters, or newtons for a slide
   */
  void step(double seconds, double gravityTorque) {
    owed += seconds;
    while (owed >= DEVICE_LOOP_SECONDS - 1e-12) {
      owed -= DEVICE_LOOP_SECONDS;
      deviceLoop(gravityTorque);
    }
  }

  /** One millisecond: the motor controller's loop, then the joint, held at its stops. */
  private void deviceLoop(double gravityTorque) {
    double position = motor.positionRadians() + offset; // where the motor controller reads it
    double velocity = motor.velocityRadPerSec();
    if (mode == Mode.NEUTRAL) {
      if (brake) {
        motor.run(0, statorLimitAmps, gravityTorque, DEVICE_LOOP_SECONDS);
      } else {
        motor.coast(gravityTorque, DEVICE_LOOP_SECONDS);
      }
    } else {
      double volts = Math.clamp(volts(position, velocity), -BATTERY_VOLTS, BATTERY_VOLTS);
      boolean limited = mode != Mode.HOMING; // homing ignores the soft limits
      if (limited
          && ((position >= softUpper && volts > 0) || (position <= softLower && volts < 0))) {
        volts = 0;
      }
      motor.run(volts, statorLimitAmps, gravityTorque, DEVICE_LOOP_SECONDS);
    }
    if (motor.positionRadians() < hardLower) {
      motor.place(hardLower, Math.max(0, motor.velocityRadPerSec()));
    } else if (motor.positionRadians() > hardUpper) {
      motor.place(hardUpper, Math.min(0, motor.velocityRadPerSec()));
    }
  }

  /** What the motor controller asks of the motors this millisecond, as Phoenix works it out. */
  private double volts(double position, double velocity) {
    return switch (mode) {
      case POSITION -> {
        double error = setPosition - position;
        integral += error * DEVICE_LOOP_SECONDS;
        yield kP * error
            + kI * integral
            + kD * (setVelocity - velocity)
            + kV * setVelocity
            + kS * Math.signum(setVelocity)
            + gravity(position)
            + feedforwardVolts;
      }
      case VELOCITY ->
          kP * (setVelocity - velocity)
              + kV * setVelocity
              + kS * Math.signum(setVelocity)
              + gravity(position);
      case VOLTAGE, HOMING -> setVolts;
      case DUTY_CYCLE -> setVolts * BATTERY_VOLTS;
      case NEUTRAL -> 0;
    };
  }

  /** Phoenix's gravity feedforward: kG along one cosine for an arm, steady for a slide. */
  private double gravity(double position) {
    return Double.isNaN(gravityOffset) ? kG : kG * Math.cos(position + gravityOffset);
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    inputs.connected = true;
    inputs.position = motor.positionRadians() + offset;
    inputs.velocity = motor.velocityRadPerSec();
    inputs.appliedVolts = motor.appliedVolts();
    inputs.statorCurrentAmps = Math.abs(motor.currentAmps()) / count;
    inputs.supplyCurrentAmps =
        Math.abs(motor.currentAmps() * motor.appliedVolts()) / BATTERY_VOLTS / count;
    inputs.unconfigured = "";
  }

  @Override
  public void setPosition(double position, double velocity, double feedforwardVolts) {
    if (mode != Mode.POSITION) {
      integral = 0;
    }
    mode = Mode.POSITION;
    setPosition = position;
    setVelocity = velocity;
    this.feedforwardVolts = feedforwardVolts;
  }

  @Override
  public void setVelocity(double velocity) {
    mode = Mode.VELOCITY;
    setVelocity = velocity;
  }

  @Override
  public void setVoltage(double volts) {
    mode = Mode.VOLTAGE;
    setVolts = volts;
  }

  @Override
  public void setDutyCycle(double share) {
    mode = Mode.DUTY_CYCLE;
    setVolts = share;
  }

  @Override
  public void stop() {
    mode = Mode.NEUTRAL;
  }

  @Override
  public void home(double volts) {
    mode = Mode.HOMING;
    setVolts = volts;
  }

  @Override
  public void resetPosition(double position) {
    offset = position - motor.positionRadians();
  }

  @Override
  public void retune(MechanismControl control) {
    statorLimitAmps = control.currentLimits().stator().in(Amps) * count;
    kP = control.gains().kP();
    kI = control.gains().kI();
    kD = control.gains().kD();
    kS = control.gains().kS();
  }
}
