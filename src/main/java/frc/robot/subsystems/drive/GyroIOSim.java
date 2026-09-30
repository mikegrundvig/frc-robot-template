package frc.robot.subsystems.drive;

import java.util.Arrays;
import java.util.Random;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The simulated gyro: the robot's true rotation, reported with the scale error, drift, and noise of
 * {@link SensorErrors}. Like a real gyro, it reads zero wherever the robot pointed when it started,
 * and it follows the robot when someone picks it up and turns it.
 */
final class GyroIOSim implements GyroIO {
  private final DriveSimulation robot;
  private final double scale;
  private final double driftRadiansPerSecond;
  private final double noiseRadians;
  private final Random noise;

  // Readings taken since the last loop, for odometry.
  private long[] sampleTimes = new long[8];
  private double[] sampleYaws = new double[8];
  private int samples;
  private double latestYaw;

  GyroIOSim(DriveSimulation robot, SensorErrors errors) {
    this.robot = robot;
    scale = errors.gyroScale();
    driftRadiansPerSecond = Math.toRadians(errors.gyroDriftDegreesPerMinute()) / 60;
    noiseRadians = Math.toRadians(errors.gyroNoiseDegrees());
    noise = new Random(errors.seed());
  }

  /** Takes a reading for odometry, with the modules' readings. */
  void sample(long timestampNanos) {
    if (samples == sampleTimes.length) {
      sampleTimes = Arrays.copyOf(sampleTimes, samples * 2);
      sampleYaws = Arrays.copyOf(sampleYaws, samples * 2);
    }
    latestYaw =
        robot.yawTurnedRadians() * scale
            + driftRadiansPerSecond * robot.elapsedSeconds()
            + noiseRadians * noise.nextGaussian();
    sampleTimes[samples] = timestampNanos;
    sampleYaws[samples] = latestYaw;
    samples++;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yaw = new Rotation2d(latestYaw);
    inputs.yawVelocityRadPerSec = robot.yawRateRadiansPerSecond() * scale;
    inputs.odometryTimestampsNanos = Arrays.copyOf(sampleTimes, samples);
    inputs.odometryYawsRad = Arrays.copyOf(sampleYaws, samples);
    samples = 0;
  }
}
