package frc.robot.sim;

import frc.robot.lib.config.ConfigRecord;
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
 * <p>Built by name, like the robot's configuration: start from {@link #NONE} or {@link #TYPICAL}
 * and change what the test needs, {@code SensorErrorsBuilder.builder(SensorErrors.NONE)
 * .steerOffsetDegrees(...).build()}, or use one of the {@code with} methods.
 *
 * @param wheelRadiusScale each wheel's true radius over the configured one (front left, front
 *     right, back left, back right). 0.98 is tread worn 2% smaller than the code thinks: the robot
 *     travels 2% less than odometry says.
 * @param steerOffsetDegrees how far each module's absolute encoder zero is off: the module points
 *     that much away from where the code thinks it points
 * @param gyro the gyro's errors
 * @param backupGyro the backup IMU's errors (Systemcore's own), read once a loop
 * @param cameraMountErrors by camera name, how each camera is really mounted off its configured
 *     mount, in the robot's frame: a camera knocked 4 in low, or turned 2 degrees
 * @param seed for the noise, so a run repeats exactly
 */
@ConfigRecord
public record SensorErrors(
    List<Double> wheelRadiusScale,
    List<Double> steerOffsetDegrees,
    GyroErrors gyro,
    GyroErrors backupGyro,
    Map<String, Transform3d> cameraMountErrors,
    long seed) {
  /**
   * Wheels and sensors exactly as configured: odometry is the truth. For tests of exact numbers.
   */
  public static final SensorErrors NONE =
      SensorErrorsBuilder.builder()
          .wheelRadiusScale(List.of(1.0, 1.0, 1.0, 1.0))
          .steerOffsetDegrees(List.of(0.0, 0.0, 0.0, 0.0))
          .gyro(GyroErrors.EXACT)
          .backupGyro(GyroErrors.EXACT)
          .cameraMountErrors(Map.of())
          .seed(1)
          .build();

  /**
   * A robot partway through a season: tread worn 1-2% and unevenly, absolute encoder zeros off by
   * up to half a degree, and a gyro reading 0.3% high, drifting 0.2° a minute; its backup IMU reads
   * 0.5% high and drifts a degree a minute. Plausible estimates, not measurements: replace them
   * with a real robot's once it is calibrated.
   */
  public static final SensorErrors TYPICAL =
      SensorErrorsBuilder.builder()
          .wheelRadiusScale(List.of(0.985, 0.99, 0.98, 0.988))
          .steerOffsetDegrees(List.of(0.5, -0.3, 0.0, 0.4))
          .gyro(
              GyroErrors.builder()
                  .scale(1.003)
                  .driftDegreesPerMinute(0.2)
                  .noiseDegrees(0.02)
                  .build())
          .backupGyro(
              GyroErrors.builder()
                  .scale(1.005)
                  .driftDegreesPerMinute(1.0)
                  .noiseDegrees(0.05)
                  .build())
          .cameraMountErrors(Map.of())
          .seed(1)
          .build();

  /**
   * How a gyro's readings differ from the truth.
   *
   * @param scale its reading per true turn: 1.003 reads 0.3% too much rotation
   * @param driftDegreesPerMinute a steady drift, from its bias
   * @param noiseDegrees random noise on each reading, as a standard deviation
   */
  @ConfigRecord
  public record GyroErrors(double scale, double driftDegreesPerMinute, double noiseDegrees) {
    /** A gyro that reads the truth. */
    public static final GyroErrors EXACT =
        builder().scale(1).driftDegreesPerMinute(0).noiseDegrees(0).build();

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

    /** Starts describing a gyro's errors, one named value at a time. */
    public static SensorErrorsGyroErrorsBuilder.ScaleStage builder() {
      return SensorErrorsGyroErrorsBuilder.builder();
    }
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
  }

  /** The same errors, with every wheel's radius scaled alike: tread worn evenly. */
  public SensorErrors withWheelRadiusScale(double scale) {
    return SensorErrorsBuilder.builder(this)
        .wheelRadiusScale(List.of(scale, scale, scale, scale))
        .build();
  }

  /** The same errors, with both the gyro and the backup IMU reading like this. */
  public SensorErrors withGyros(GyroErrors both) {
    return SensorErrorsBuilder.builder(this).gyro(both).backupGyro(both).build();
  }

  /** The same errors, with the backup IMU reading like this. */
  public SensorErrors withBackupGyro(GyroErrors backup) {
    return SensorErrorsBuilder.builder(this).backupGyro(backup).build();
  }

  /** The same errors, with one camera mounted off its configuration. */
  public SensorErrors withCameraMountError(String camera, Transform3d error) {
    Map<String, Transform3d> errors = new HashMap<>(cameraMountErrors);
    errors.put(camera, error);
    return SensorErrorsBuilder.builder(this).cameraMountErrors(errors).build();
  }

  /** How a camera is really mounted off its configuration: none, unless one was set. */
  public Transform3d cameraMountError(String camera) {
    return cameraMountErrors.getOrDefault(camera, Transform3d.ZERO);
  }
}
