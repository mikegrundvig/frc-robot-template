package frc.robot.sim;

import frc.robot.lib.timing.RobotLoop;
import frc.robot.subsystems.drive.GyroIO;
import java.util.Random;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A simulated IMU whose accelerometer reads as a real one mounted on a robot might, for tests of
 * what localization makes of it: off the robot's center (so the robot's turn swings it around), its
 * axes turned from the robot's or flipped (mounted logo down), with a bias and noise. Everything
 * else it reads as the IMU it wraps does.
 *
 * @param place where it sits, from the robot's center
 * @param turnedDegrees how far its axes are turned from the robot's, counterclockwise from above
 * @param upsideDown whether it's mounted logo down, its y axis to the robot's right
 * @param bias its accelerometer's bias along its own axes, in m/s²
 * @param noise its accelerometer's noise on each reading, in m/s²
 */
public record FlawedImu(
    Translation2d place,
    double turnedDegrees,
    boolean upsideDown,
    Translation2d bias,
    double noise) {
  /** Exactly as the simulated IMU reads. */
  public static final FlawedImu NONE =
      new FlawedImu(Translation2d.ZERO, 0, false, Translation2d.ZERO, 0);

  /** The same IMU, sitting somewhere else. */
  public FlawedImu at(Translation2d where) {
    return new FlawedImu(where, turnedDegrees, upsideDown, bias, noise);
  }

  /** The same IMU, its axes turned this far. */
  public FlawedImu turned(double degrees) {
    return new FlawedImu(place, degrees, upsideDown, bias, noise);
  }

  /** The same IMU, mounted logo down. */
  public FlawedImu flipped() {
    return new FlawedImu(place, turnedDegrees, true, bias, noise);
  }

  /** Wraps a simulated IMU, reading as this one does. */
  public GyroIO wrap(GyroIO imu) {
    Random random = new Random(7);
    double[] lastRate = {Double.NaN};
    return new GyroIO() {
      @Override
      public void updateInputs(GyroIOInputs inputs) {
        imu.updateInputs(inputs);
        double ax = inputs.accelerationXMetersPerSecSq;
        double ay = inputs.accelerationYMetersPerSecSq;
        // Off the center, the turn swings it around: -w² r, and alpha x r as the turn changes.
        double rate = inputs.yawVelocityRadPerSec;
        double alpha =
            Double.isNaN(lastRate[0]) ? 0 : (rate - lastRate[0]) / RobotLoop.PERIOD_SECONDS;
        lastRate[0] = rate;
        ax += -rate * rate * place.getX() - alpha * place.getY();
        ay += -rate * rate * place.getY() + alpha * place.getX();
        // Its own axes, turned from the robot's.
        Translation2d own =
            new Translation2d(ax, ay).rotateBy(new Rotation2d(Math.toRadians(-turnedDegrees)));
        double x = own.getX() + bias.getX() + noise * random.nextGaussian();
        double y =
            (upsideDown ? -own.getY() : own.getY()) + bias.getY() + noise * random.nextGaussian();
        inputs.accelerationXMetersPerSecSq = x;
        inputs.accelerationYMetersPerSecSq = y;
      }
    };
  }
}
