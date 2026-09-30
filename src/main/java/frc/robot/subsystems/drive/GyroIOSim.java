package frc.robot.subsystems.drive;

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

  GyroIOSim(DriveSimulation robot, SensorErrors errors) {
    this.robot = robot;
    scale = errors.gyroScale();
    driftRadiansPerSecond = Math.toRadians(errors.gyroDriftDegreesPerMinute()) / 60;
    noiseRadians = Math.toRadians(errors.gyroNoiseDegrees());
    noise = new Random(errors.seed());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yaw =
        new Rotation2d(
            robot.yawTurnedRadians() * scale
                + driftRadiansPerSecond * robot.elapsedSeconds()
                + noiseRadians * noise.nextGaussian());
    inputs.yawVelocityRadPerSec = robot.yawRateRadiansPerSecond() * scale;
  }
}
