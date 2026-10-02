package frc.robot.sim;

/**
 * What drives one wheel over a physics sub-step, as its actuators say: its drive motor's voltage
 * and current limit, and its steering's angle ({@link Actuators}). The physics keeps one for each
 * wheel and hands them out to be filled in every sub-step, so nothing is made per step.
 */
public final class WheelDrive {
  private double volts;
  private double voltsPerRadPerSec;
  private double currentLimitAmps = Double.POSITIVE_INFINITY;
  private double steerRadians;

  /**
   * The drive motor over the sub-step.
   *
   * @param volts across the motor as the sub-step starts: what its controller applies, within its
   *     current limit at the wheel's speed then, and within the battery's voltage
   * @param voltsPerRadPerSec how the controller's voltage moves with the wheel's speed, in volts
   *     per radian a second: its velocity loop's gain, negative, through the wheel's radius; zero
   *     for a voltage it simply holds. An engine that integrates implicitly folds it into the
   *     motor's line.
   * @param currentLimitAmps the most current the controller lets through the motor, either way
   */
  public void drive(double volts, double voltsPerRadPerSec, double currentLimitAmps) {
    this.volts = volts;
    this.voltsPerRadPerSec = voltsPerRadPerSec;
    this.currentLimitAmps = currentLimitAmps;
  }

  /**
   * The steering's angle at the sub-step's end, turned in closed form by the actuators.
   *
   * @param radians counterclockwise from the robot's forward axis
   */
  public void steer(double radians) {
    steerRadians = radians;
  }

  /** The voltage across the drive motor as the sub-step starts. */
  public double volts() {
    return volts;
  }

  /** How the controller's voltage moves with the wheel's speed, in volts per radian a second. */
  public double voltsPerRadPerSec() {
    return voltsPerRadPerSec;
  }

  /** The most current the drive's controller lets through its motor. */
  public double currentLimitAmps() {
    return currentLimitAmps;
  }

  /** The steering's angle at the sub-step's end. */
  public double steerRadians() {
    return steerRadians;
  }
}
