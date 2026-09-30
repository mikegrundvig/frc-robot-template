package frc.robot.subsystems.drive;

import java.util.List;

/**
 * How the simulated robot's wheels and sensors differ from what the code believes: the errors that
 * make real odometry drift. The simulator keeps the robot's true pose, the sensors report it with
 * these errors, and the gap between the truth and the code's estimate is what localization has to
 * close.
 *
 * @param wheelRadiusScale each wheel's true radius over the configured one (front left, front
 *     right, back left, back right). 0.98 is tread worn 2% smaller than the code thinks: the robot
 *     travels 2% less than odometry says.
 * @param steerOffsetDegrees how far each module's absolute encoder zero is off: the module points
 *     that much away from where the code thinks it points
 * @param gyroScale the gyro's reading per true turn: 1.003 reads 0.3% too much rotation
 * @param gyroDriftDegreesPerMinute a steady drift, from the gyro's bias
 * @param gyroNoiseDegrees random noise on each reading, as a standard deviation
 * @param seed for the noise, so a run repeats exactly
 */
public record SensorErrors(
    List<Double> wheelRadiusScale,
    List<Double> steerOffsetDegrees,
    double gyroScale,
    double gyroDriftDegreesPerMinute,
    double gyroNoiseDegrees,
    long seed) {
  /**
   * Wheels and sensors exactly as configured: odometry is the truth. For tests of exact numbers.
   */
  public static final SensorErrors NONE =
      new SensorErrors(List.of(1.0, 1.0, 1.0, 1.0), List.of(0.0, 0.0, 0.0, 0.0), 1.0, 0, 0, 1);

  /**
   * A robot partway through a season: tread worn 1-2% and unevenly, absolute encoder zeros off by
   * up to half a degree, and a gyro reading 0.3% high, drifting 0.2° a minute. Plausible estimates,
   * not measurements: replace them with a real robot's once it is calibrated.
   */
  public static final SensorErrors TYPICAL =
      new SensorErrors(
          List.of(0.985, 0.99, 0.98, 0.988), List.of(0.5, -0.3, 0.0, 0.4), 1.003, 0.2, 0.02, 1);

  public SensorErrors {
    if (wheelRadiusScale.size() != 4 || steerOffsetDegrees.size() != 4) {
      throw new IllegalArgumentException("one wheel radius scale and one steer offset per module");
    }
    wheelRadiusScale = List.copyOf(wheelRadiusScale);
    steerOffsetDegrees = List.copyOf(steerOffsetDegrees);
    for (int i = 0; i < 4; i++) {
      if (!(wheelRadiusScale.get(i) > 0) || !Double.isFinite(wheelRadiusScale.get(i))) {
        throw new IllegalArgumentException("a wheel radius scale must be positive");
      }
      if (!Double.isFinite(steerOffsetDegrees.get(i))) {
        throw new IllegalArgumentException("a steer offset must be a number");
      }
    }
    if (!(gyroScale > 0)
        || !Double.isFinite(gyroScale)
        || !Double.isFinite(gyroDriftDegreesPerMinute)
        || !(gyroNoiseDegrees >= 0)
        || !Double.isFinite(gyroNoiseDegrees)) {
      throw new IllegalArgumentException(
          "the gyro's scale must be positive, its drift a number, and its noise zero or more");
    }
  }

  /** The same errors, with every wheel's radius scaled alike: tread worn evenly. */
  public SensorErrors withWheelRadiusScale(double scale) {
    return new SensorErrors(
        List.of(scale, scale, scale, scale),
        steerOffsetDegrees,
        gyroScale,
        gyroDriftDegreesPerMinute,
        gyroNoiseDegrees,
        seed);
  }
}
