package frc.robot.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.robot.MechanismControl;
import org.littletonrobotics.junction.AutoLog;

/**
 * One driven joint's door to hardware: its motor controllers on a robot ({@link JointIOTalon}), the
 * simulator's model of it ({@code frc.robot.sim.JointIOSim}), or nothing, in replay, where its
 * inputs come from the log. Everything is in the joint's own units, radians or meters, as the
 * robot's definition has them; the hardware's units stay behind the door.
 *
 * <p>Each request holds until the next: the joint's mechanism ({@link JointMechanism}) makes one
 * every loop, so a loop that makes none is a bug, not a pause.
 */
public interface JointIO {
  @AutoLog
  class JointIOInputs {
    /** Whether the motor controllers answered lately: false after half a second of silence. */
    public boolean connected = false;

    /** Where the joint is, in radians for a revolute joint or meters for a prismatic one. */
    public double position = 0.0;

    /** How fast it moves, in radians or meters a second. */
    public double velocity = 0.0;

    /**
     * Where the absolute encoder its leading motor controller reads says the joint is, each loop,
     * in its units: a SPARK's data port's. NaN without one, or before its first good reading; an
     * encoder only the robot reads has inputs of its own ({@link AbsoluteEncoderIO}).
     */
    public double absolutePosition = Double.NaN;

    /** The voltage across the leading motor. */
    public double appliedVolts = 0.0;

    /** The current through the leading motor, either way: what its stator limit holds. */
    public double statorCurrentAmps = 0.0;

    /** The current the leading motor draws from the battery. */
    public double supplyCurrentAmps = 0.0;

    /**
     * Whether its leading motor controller's forward limit input reads tripped, the polarity of the
     * switch on it told to the controller: false with nothing on it.
     */
    public boolean forwardLimit = false;

    /** The same, its reverse limit input. */
    public boolean reverseLimit = false;

    /**
     * The joint's devices that didn't take their configuration at startup, such as "motor 32", or
     * empty: they run on their defaults, without the configured ratios, gains or current limits.
     */
    public String unconfigured = "";
  }

  default void updateInputs(JointIOInputs inputs) {}

  /**
   * Holds the joint at a point of a profile: where it should be, how fast it should be moving
   * there, and the volts the program adds to the motor controller's own feedforward (its kV, kS and
   * gravity), such as the profile's acceleration times kA.
   */
  default void setPosition(double position, double velocity, double feedforwardVolts) {}

  /** Runs the joint at a speed, in its units a second, on the motor controller's velocity loop. */
  default void setVelocity(double velocity) {}

  /** A voltage across the motors. */
  default void setVoltage(double volts) {}

  /** A share of the battery's voltage, from -1 to 1. */
  default void setDutyCycle(double share) {}

  /** Neutral: the motors brake or coast, as the joint's control says. */
  default void stop() {}

  /**
   * A voltage across the motors that ignores the soft limits: homing pushes the joint into a hard
   * stop, past where its soft limits would stop it, since until it's homed they're in the wrong
   * place.
   */
  default void home(double volts) {}

  /** Tells the motor controller where the joint is now: homing found it. */
  default void resetPosition(double position) {}

  /**
   * Takes the pit's tuned gains and current limits, as the motor controller's own loop runs them:
   * its kP, kI, kD and kS, and its current limits. Without waiting: the loop goes on.
   */
  default void retune(MechanismControl control) {}
}
