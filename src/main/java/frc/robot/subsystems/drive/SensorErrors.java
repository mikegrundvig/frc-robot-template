package frc.robot.subsystems.drive;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Transform3d;

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
 * @param cameraMountErrors by camera name, how each camera is really mounted off its configured
 *     mount, in the robot's frame: a camera knocked 4 in low, or turned 2 degrees
 * @param backupGyro the backup IMU's errors (Systemcore's own), read once a loop
 */
public record SensorErrors(
    List<Double> wheelRadiusScale,
    List<Double> steerOffsetDegrees,
    double gyroScale,
    double gyroDriftDegreesPerMinute,
    double gyroNoiseDegrees,
    long seed,
    Map<String, Transform3d> cameraMountErrors,
    GyroErrors backupGyro) {
  /**
   * Wheels and sensors exactly as configured: odometry is the truth. For tests of exact numbers.
   */
  public static final SensorErrors NONE =
      new SensorErrors(List.of(1.0, 1.0, 1.0, 1.0), List.of(0.0, 0.0, 0.0, 0.0), 1.0, 0, 0, 1);

  /**
   * A robot partway through a season: tread worn 1-2% and unevenly, absolute encoder zeros off by
   * up to half a degree, and a gyro reading 0.3% high, drifting 0.2° a minute; its backup IMU reads
   * 0.5% high and drifts a degree a minute. Plausible estimates, not measurements: replace them
   * with a real robot's once it is calibrated.
   */
  public static final SensorErrors TYPICAL =
      new SensorErrors(
          List.of(0.985, 0.99, 0.98, 0.988),
          List.of(0.5, -0.3, 0.0, 0.4),
          1.003,
          0.2,
          0.02,
          1,
          Map.of(),
          new GyroErrors(1.005, 1.0, 0.05));

  /**
   * How a gyro's readings differ from the truth.
   *
   * @param scale its reading per true turn: 1.003 reads 0.3% too much rotation
   * @param driftDegreesPerMinute a steady drift, from its bias
   * @param noiseDegrees random noise on each reading, as a standard deviation
   */
  public record GyroErrors(double scale, double driftDegreesPerMinute, double noiseDegrees) {
    /** A gyro that reads the truth. */
    public static final GyroErrors EXACT = new GyroErrors(1, 0, 0);

    public GyroErrors {
      if (!(scale > 0)
          || !Double.isFinite(scale)
          || !Double.isFinite(driftDegreesPerMinute)
          || !(noiseDegrees >= 0)
          || !Double.isFinite(noiseDegrees)) {
        throw new IllegalArgumentException(
            "the gyro's scale must be positive, its drift a number, and its noise zero or more");
      }
    }
  }

  /** Errors with every camera mounted exactly as configured, and a backup IMU like the gyro. */
  public SensorErrors(
      List<Double> wheelRadiusScale,
      List<Double> steerOffsetDegrees,
      double gyroScale,
      double gyroDriftDegreesPerMinute,
      double gyroNoiseDegrees,
      long seed) {
    this(
        wheelRadiusScale,
        steerOffsetDegrees,
        gyroScale,
        gyroDriftDegreesPerMinute,
        gyroNoiseDegrees,
        seed,
        Map.of(),
        new GyroErrors(gyroScale, gyroDriftDegreesPerMinute, gyroNoiseDegrees));
  }

  public SensorErrors {
    cameraMountErrors = Map.copyOf(cameraMountErrors);
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
    new GyroErrors(gyroScale, gyroDriftDegreesPerMinute, gyroNoiseDegrees); // checks them
  }

  /** The gyro's errors. */
  public GyroErrors gyro() {
    return new GyroErrors(gyroScale, gyroDriftDegreesPerMinute, gyroNoiseDegrees);
  }

  /** The same errors, with every wheel's radius scaled alike: tread worn evenly. */
  public SensorErrors withWheelRadiusScale(double scale) {
    return new SensorErrors(
        List.of(scale, scale, scale, scale),
        steerOffsetDegrees,
        gyroScale,
        gyroDriftDegreesPerMinute,
        gyroNoiseDegrees,
        seed,
        cameraMountErrors,
        backupGyro);
  }

  /** The same errors, with the backup IMU reading like this. */
  public SensorErrors withBackupGyro(GyroErrors backup) {
    return new SensorErrors(
        wheelRadiusScale,
        steerOffsetDegrees,
        gyroScale,
        gyroDriftDegreesPerMinute,
        gyroNoiseDegrees,
        seed,
        cameraMountErrors,
        backup);
  }

  /** The same errors, with one camera mounted off its configuration. */
  public SensorErrors withCameraMountError(String camera, Transform3d error) {
    Map<String, Transform3d> errors = new HashMap<>(cameraMountErrors);
    errors.put(camera, error);
    return new SensorErrors(
        wheelRadiusScale,
        steerOffsetDegrees,
        gyroScale,
        gyroDriftDegreesPerMinute,
        gyroNoiseDegrees,
        seed,
        errors,
        backupGyro);
  }

  /** How a camera is really mounted off its configuration: none, unless one was set. */
  public Transform3d cameraMountError(String camera) {
    return cameraMountErrors.getOrDefault(camera, Transform3d.ZERO);
  }
}
