package frc.robot.sim;

import com.michaelgrundvig.frc.simulation.SensorErrors;
import frc.robot.subsystems.drive.GyroIO;
import java.util.Arrays;
import java.util.Random;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A simulated gyro: the robot's true rotation, reported with the scale error, drift, and noise of
 * {@link SensorErrors}. Like a real gyro, it reads zero wherever the robot pointed when it started,
 * and it follows the robot when someone picks it up and turns it. The rest of the IMU reads the
 * robot's attitude as it truly is, and its acceleration through its own accelerometer ({@link
 * AccelerometerSim}), which feels a hit's jolt on its own; unplugged, it feels none. It can fail as
 * a real one does, at any moment: disconnected (no readings at all), frozen (still reporting, the
 * same yaw every time), erring differently from now on, or reset to zero, as one that browns out
 * and restarts does.
 */
final class GyroIOSim implements GyroIO {
  private final DriveSimulation robot;
  private final SensorErrors.GyroErrors built;
  private SensorErrors.GyroErrors errors;
  private double scale;
  private double driftRadiansPerSecond;
  private double noiseRadians;
  private final Random noise;
  private final AccelerometerSim accelerometer;

  /**
   * Added to its reading so the reading carries on from where it was when its errors change or it
   * resets: none until one does.
   */
  private double offsetRadians;

  // Readings taken since the last loop, for odometry.
  private long[] sampleTimes = new long[8];
  private double[] sampleYaws = new double[8];
  private int samples;
  private double latestYaw;
  private boolean connected = true;
  private boolean frozen;

  GyroIOSim(
      DriveSimulation robot,
      SensorErrors.GyroErrors errors,
      AccelerometerSim accelerometer,
      long seed) {
    this.robot = robot;
    this.accelerometer = accelerometer;
    built = errors;
    this.errors = errors;
    takeErrors(errors);
    noise = new Random(seed);
  }

  /** Unplugged: no readings until it's plugged back in. */
  void setConnected(boolean connected) {
    this.connected = connected;
  }

  boolean isConnected() {
    return connected;
  }

  /** Stuck: still reporting, the same yaw every time. */
  void setFrozen(boolean frozen) {
    this.frozen = frozen;
  }

  boolean isFrozen() {
    return frozen;
  }

  /**
   * Errs differently from now on: its reading carries on from where it is, then turns at the new
   * scale and drifts at the new rate.
   */
  void setErrors(SensorErrors.GyroErrors errors) {
    double before = trueReading();
    takeErrors(errors);
    this.errors = errors;
    offsetRadians += before - trueReading();
  }

  /** How it errs now. */
  SensorErrors.GyroErrors errors() {
    return errors;
  }

  /** How it erred as the robot was built. */
  SensorErrors.GyroErrors built() {
    return built;
  }

  /** Reads zero from now on wherever the robot points, as a gyro that restarts does. */
  void reset() {
    offsetRadians -= trueReading();
  }

  private void takeErrors(SensorErrors.GyroErrors errors) {
    scale = errors.scale();
    driftRadiansPerSecond = Math.toRadians(errors.driftDegreesPerMinute()) / 60;
    noiseRadians = Math.toRadians(errors.noiseDegrees());
  }

  /** What it reads before noise: the true turn at its scale, with its drift, from its offset. */
  private double trueReading() {
    return offsetRadians
        + robot.yawTurnedRadians() * scale
        + driftRadiansPerSecond * robot.elapsedSeconds();
  }

  /** After each physics step: its accelerometer feels it. */
  void feel(double seconds) {
    accelerometer.step(seconds);
  }

  /** At the end of each loop. */
  void endLoop(double seconds) {
    accelerometer.endLoop(seconds);
  }

  /** A hit's jolt, along the robot's axes: its next reading feels it. */
  void jolt(Translation2d robotAxes) {
    accelerometer.jolt(robotAxes);
  }

  /** Takes a reading for odometry, with the modules' readings. */
  void sample(long timestampNanos) {
    if (!connected) {
      return;
    }
    if (samples == sampleTimes.length) {
      sampleTimes = Arrays.copyOf(sampleTimes, samples * 2);
      sampleYaws = Arrays.copyOf(sampleYaws, samples * 2);
    }
    double reading = trueReading() + noiseRadians * noise.nextGaussian();
    if (!frozen) {
      latestYaw = reading;
    }
    sampleTimes[samples] = timestampNanos;
    sampleYaws[samples] = latestYaw;
    samples++;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = connected;
    inputs.yaw = new Rotation2d(latestYaw);
    inputs.yawVelocityRadPerSec = robot.yawRateRadiansPerSecond() * scale;
    inputs.pitchRad = robot.pitchRadians();
    inputs.rollRad = robot.rollRadians();
    if (connected) {
      Translation2d felt = accelerometer.read();
      inputs.accelerationXMetersPerSecSq = felt.getX();
      inputs.accelerationYMetersPerSecSq = felt.getY();
    } else {
      accelerometer.missed();
    }
    inputs.odometryTimestampsNanos = Arrays.copyOf(sampleTimes, samples);
    inputs.odometryYawsRad = Arrays.copyOf(sampleYaws, samples);
    samples = 0;
  }
}
