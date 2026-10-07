package frc.robot.sim;

import static org.wpilib.units.Units.Amps;

import com.michaelgrundvig.frc.simulation.JointPlant;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import frc.robot.subsystems.mechanisms.ControllerSettings;
import frc.robot.subsystems.mechanisms.JointIO;
import java.util.Map;

/**
 * A simulated joint: its motor controller, as the joint's control and wiring describe it, driving
 * the joint's plant, its motors moving its link, built from the robot's definition alone ({@link
 * JointPlant}, in the simulation library: the motors through the transmission, the inertia,
 * gravity, the hard stops).
 *
 * <p>Gravity is the plant's: what the kG tuned in the joint's block holds, through its motors, or,
 * where its block tunes none, its links' weight. It's the robot's, from its block in the definition
 * the simulator was built from, not its motor controller's: a test can run the controller with
 * another kG, re-tuning the joint ({@code PIVOT.retuned(control -> control.withKg(0))}) in a robot
 * whose definition keeps its own, and live tuning doesn't move it.
 *
 * <p>The motor controller runs as the controller its block names runs what its vendor's helper
 * configures, from the joint's plan ({@link ControllerPlan}), its loop every millisecond on the
 * joint's own position and speed: a TalonFX's slot, the plan's whole law on the device ({@link
 * TalonFxEmulator}), for a TalonFX, a TalonFXS, or a joint wired to nothing; a SPARK's kP on the
 * device, with the plan's feedforward the robot works out as it asks ({@link SparkEmulator}).
 * Either way: its stator current limit; its soft limits, past which it drives no further; and
 * neutral as its control says, braking or coasting. Like a real one, it knows only where its own
 * count says the joint is: from where the joint was at power on, taken as its reference, until
 * homing tells it otherwise ({@link #powerOnAt}, {@link #resetPosition}). The battery holds 12 V,
 * and the supply current limit isn't modeled, as the drivetrain's isn't ({@code ModuleIOSim}).
 *
 * <p>What the robot would refuse of the joint's settings, its configuration refused before the
 * simulator started; what it would doubt, the robot says at startup, in the simulator as on the
 * robot ({@link ControllerSettings#findings}).
 */
final class JointIOSim implements JointIO {
  /**
   * A motor controller runs its closed loop every millisecond: a TalonFX's (CTRE's Phoenix 6
   * documentation: 1 kHz), and a SPARK's.
   */
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
  private final JointPlant plant;
  private final ControllerEmulator controller;

  /** How its motor controller drives it: its block's control, or as the pit tunes it. */
  private ControllerPlan plan;

  /** Where the motor controller reads the joint, less where it really is: 0 until misplaced. */
  private double offset;

  /** Where, and how fast, the motor controller last told the robot the joint is. */
  private double read;

  private double readVelocity;

  private Mode mode = Mode.NEUTRAL;
  private double setVolts;

  /** Seconds of the robot's loop not yet run, under a millisecond: the next step's. */
  private double owed;

  /**
   * @param math what the robot's definition derives: its joint's block sets the gravity on it
   * @param joint the joint as its motor controller runs it: its block's control and wiring
   */
  JointIOSim(RobotMath math, Joint joint) {
    this.joint = joint;
    this.math = math;
    plan = ControllerPlan.of(math, joint, ControllerSettings.control(joint));
    plant = JointPlant.of(math, joint);
    controller = ControllerEmulator.of(plan);
    read = plant.position();
  }

  /** The joint. */
  Joint joint() {
    return joint;
  }

  /**
   * Gravity's torque on the joint, positive turning it forward, with the joints where they stand:
   * newton meters, or newtons for a slide ({@link JointPlant#gravityTorque}).
   */
  double gravityTorque(Map<Joint, Double> positions) {
    return plant.gravityTorque(positions);
  }

  /**
   * The current each of its motors draws from the battery, in amps: what its controller reads, and
   * what its PDH channel does.
   */
  double supplyAmps() {
    return plant.supplyAmps();
  }

  /** Where the joint really is, in its units. */
  double position() {
    return plant.position();
  }

  /** How fast it really moves. */
  double velocity() {
    return plant.velocity();
  }

  /** Puts the joint somewhere, still, where its motor controller reads it too: a test's start. */
  void place(double position) {
    plant.place(position);
    offset = 0;
  }

  /**
   * Powers it on somewhere other than its reference, still: its motor controller reads it at its
   * reference anyway, as a joint read by its motor alone does, until it's homed.
   */
  void powerOnAt(double position) {
    plant.place(position);
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
    double position = plant.position() + offset; // where the motor controller reads it
    double velocity = plant.velocity();
    double limit = plan.currentLimits().stator().in(Amps);
    if (mode == Mode.NEUTRAL) {
      if (plan.brakes()) {
        plant.run(0, limit, gravityTorque, DEVICE_LOOP_SECONDS);
      } else {
        plant.coast(gravityTorque, DEVICE_LOOP_SECONDS);
      }
      return;
    }
    double volts = Math.clamp(volts(position, velocity), -BATTERY_VOLTS, BATTERY_VOLTS);
    if (mode != Mode.HOMING && pastSoftLimit(position, volts)) {
      volts = 0; // homing ignores the soft limits
    }
    plant.run(volts, limit, gravityTorque, DEVICE_LOOP_SECONDS);
  }

  /** Whether it's at or past a soft limit, driven on past it. */
  private boolean pastSoftLimit(double position, double volts) {
    return plan.softLimits()
        .map(
            limits ->
                (position >= limits.upper() && volts > 0)
                    || (position <= limits.lower() && volts < 0))
        .orElse(false);
  }

  /** What the motor controller asks of the motors this millisecond. */
  private double volts(double position, double velocity) {
    return switch (mode) {
      case POSITION -> controller.holding(position, velocity, DEVICE_LOOP_SECONDS);
      case VELOCITY -> controller.running(position, velocity);
      case VOLTAGE, HOMING -> setVolts;
      case DUTY_CYCLE -> setVolts * BATTERY_VOLTS;
      case NEUTRAL -> 0;
    };
  }

  @Override
  public void updateInputs(JointIOInputs inputs) {
    inputs.connected = true;
    inputs.position = plant.position() + offset;
    inputs.velocity = plant.velocity();
    inputs.appliedVolts = plant.appliedVolts();
    inputs.statorCurrentAmps = plant.statorAmps();
    inputs.supplyCurrentAmps = plant.supplyAmps();
    inputs.unconfigured = "";
    read = inputs.position;
    readVelocity = inputs.velocity;
  }

  @Override
  public void setPosition(double position, double velocity, double feedforwardVolts) {
    controller.hold(
        position, velocity, feedforwardVolts, read, readVelocity, mode != Mode.POSITION);
    mode = Mode.POSITION;
  }

  @Override
  public void setVelocity(double velocity) {
    controller.run(velocity, read);
    mode = Mode.VELOCITY;
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
    offset = position - plant.position();
  }

  /** Takes the pit's tuning: its gains and current limits, as its motor controller would. */
  @Override
  public void retune(MechanismControl control) {
    plan = ControllerPlan.of(math, joint, control);
    controller.retune(plan);
  }
}
