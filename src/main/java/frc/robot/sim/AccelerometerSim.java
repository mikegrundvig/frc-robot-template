package frc.robot.sim;

import frc.robot.lib.localization.ImuReadings;
import java.util.Random;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A simulated IMU's accelerometer. It feels the robot's true motion where it really sits (an IMU
 * off the center is swung around it as the robot turns), and gravity's share as the robot tilts,
 * and reads that as the part does ({@link SensorErrors.AccelErrors}): through a low-pass filter,
 * sampled at its rate, with its noise and bias, along axes that may lie a little off where the
 * robot's code takes them, and no further than its range. An ideal one reads the robot's average
 * acceleration over each loop, exactly. A hit's jolt reaches each IMU on its own.
 */
final class AccelerometerSim {
  private final DriveSimulation robot;
  private final SensorErrors.AccelErrors errors;
  private final Translation2d place;
  private final Rotation2d turnedBack;
  private final Random noise;

  /** Seconds between samples; zero for an ideal accelerometer. */
  private final double period;

  /** The point it sits at, its velocity on the field after the last physics step. */
  private Translation2d lastVelocity;

  // The real part: its filter's output and its latest sample, along the robot's axes.
  private double filteredX;
  private double filteredY;
  private double heldX;
  private double heldY;
  private double sinceSample;

  // The ideal one: its velocity at the start of the loop, and the loop's average acceleration.
  private Translation2d loopStartVelocity;
  private Translation2d loopAcceleration = Translation2d.ZERO;

  /** A hit's jolt, along the robot's axes, until this IMU reads it. */
  private Translation2d jolt = Translation2d.ZERO;

  /**
   * @param robot the simulated robot it rides on
   * @param errors how it reads
   * @param configuredPlace where the robot's configuration places it; it really sits off that by
   *     its errors' place
   * @param seed for its noise
   */
  AccelerometerSim(
      DriveSimulation robot,
      SensorErrors.AccelErrors errors,
      Translation2d configuredPlace,
      long seed) {
    this.robot = robot;
    this.errors = errors;
    place = configuredPlace.plus(new Translation2d(errors.forwardMeters(), errors.leftMeters()));
    turnedBack = Rotation2d.fromDegrees(-errors.turnedDegrees());
    noise = new Random(seed);
    period = errors.sampleHz() > 0 ? 1 / errors.sampleHz() : 0;
    lastVelocity = robot.pointVelocity(place);
    loopStartVelocity = lastVelocity;
  }

  /** After each physics step: the part filters what it feels, and samples it at its rate. */
  void step(double seconds) {
    Translation2d velocity = robot.pointVelocity(place);
    if (period > 0) {
      Translation2d felt = alongTheRobot(velocity.minus(lastVelocity).div(seconds));
      double share =
          errors.filterSeconds() > 0 ? 1 - Math.exp(-seconds / errors.filterSeconds()) : 1;
      filteredX += (felt.getX() - filteredX) * share;
      filteredY += (felt.getY() - filteredY) * share;
      sinceSample += seconds;
      if (sinceSample >= period - 1e-9) {
        sinceSample -= period;
        heldX = filteredX + errors.noise() * noise.nextGaussian();
        heldY = filteredY + errors.noise() * noise.nextGaussian();
      }
    }
    lastVelocity = velocity;
  }

  /** At the end of each loop: an ideal accelerometer reads the loop's average acceleration. */
  void endLoop(double seconds) {
    loopAcceleration = alongTheRobot(lastVelocity.minus(loopStartVelocity).div(seconds));
    loopStartVelocity = lastVelocity;
  }

  /** A hit's jolt, along the robot's axes: the next reading feels it. */
  void jolt(Translation2d robotAxes) {
    jolt = jolt.plus(robotAxes);
  }

  /** Forgets a jolt it was never read for: unplugged, it felt nothing. */
  void missed() {
    jolt = Translation2d.ZERO;
  }

  /**
   * What it reads, once a loop, along the axes the robot's code takes for the robot's: forward and
   * left.
   */
  Translation2d read() {
    Translation2d robotAxes = period > 0 ? new Translation2d(heldX, heldY) : loopAcceleration;
    robotAxes =
        robotAxes.plus(jolt).plus(new Translation2d(errors.biasForward(), errors.biasLeft()));
    jolt = Translation2d.ZERO;
    Translation2d read = robotAxes.rotateBy(turnedBack);
    double range = errors.rangeMetersPerSecSq();
    double x = Math.clamp(read.getX(), -range, range);
    double y = Math.clamp(errors.flipped() ? -read.getY() : read.getY(), -range, range);
    return new Translation2d(x, y);
  }

  /**
   * An acceleration on the field, along the robot's axes, with gravity's share as the robot tilts:
   * what the part feels. WPILib's attitude: a positive pitch is nose down, a positive roll the left
   * side up; the floor holding it up, tipped, pushes along both.
   */
  private Translation2d alongTheRobot(Translation2d fieldAcceleration) {
    double pitch = robot.pitchRadians();
    double roll = robot.rollRadians();
    Translation2d gravity =
        new Translation2d(
            -ImuReadings.STANDARD_GRAVITY * Math.sin(pitch),
            ImuReadings.STANDARD_GRAVITY * Math.sin(roll) * Math.cos(pitch));
    return fieldAcceleration.rotateBy(robot.truePose().getRotation().unaryMinus()).plus(gravity);
  }
}
