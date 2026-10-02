package frc.robot.sim;

import com.michaelgrundvig.frc.localization.ImuReadings;
import com.michaelgrundvig.frc.simulation.Seeds;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import java.util.Random;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * A simulated IMU's accelerometer. It feels the robot's true motion where it really sits (an IMU
 * off the center is swung around it as the robot turns), and gravity as the robot tilts, and reads
 * that as the part does ({@link SensorErrors.AccelErrors}): through a low-pass filter, sampled at
 * its rate, with its noise and bias, along its own axes, which may lie a little off where the
 * robot's definition says they do, and no further than its range on each. The robot's IO then turns
 * those axes into the robot's by the definition's orientation, as the real IO does. An ideal one
 * reads the robot's average acceleration over each loop, exactly. A hit's jolt reaches each IMU on
 * its own.
 */
final class AccelerometerSim {
  private final DriveSimulation robot;
  private final SensorErrors.AccelErrors errors;
  private final Translation2d place;

  /** How the robot's definition says the part is turned on the robot. */
  private final Rotation3d orientation;

  /** How it really is: the definition's turn, then its errors' (turned, or the other way up). */
  private final Rotation3d trueOrientation;

  private final Random noise;

  /** The noise on its up axis, a stream apart so the other two draw as they would without it. */
  private final Random upNoise;

  /** Seconds between samples; zero for an ideal accelerometer. */
  private final double period;

  /** The point it sits at, its velocity on the field after the last physics step. */
  private Translation2d lastVelocity;

  // The real part: its filter's output and its latest sample, along the robot's axes.
  private double filteredX;
  private double filteredY;
  private double filteredZ;
  private double heldX;
  private double heldY;
  private double heldZ;
  private double sinceSample;

  // The ideal one: its velocity at the start of the loop, and the loop's average acceleration.
  private Translation2d loopStartVelocity;
  private Translation3d loopAcceleration = Translation3d.ZERO;

  /** A hit's jolt, along the robot's axes, until this IMU reads it. */
  private Translation2d jolt = Translation2d.ZERO;

  /**
   * @param robot the simulated robot it rides on
   * @param errors how it reads
   * @param mount where the robot's definition says it's mounted; it really sits off that by its
   *     errors' place, and is turned off it by their turn
   * @param seed for its noise
   */
  AccelerometerSim(
      DriveSimulation robot, SensorErrors.AccelErrors errors, ImuMount mount, long seed) {
    this.robot = robot;
    this.errors = errors;
    place = mount.place().plus(new Translation2d(errors.forwardMeters(), errors.leftMeters()));
    orientation = mount.orientation();
    trueOrientation =
        orientation.rotateBy(
            new Rotation3d(
                errors.flipped() ? Math.PI : 0, 0, Math.toRadians(errors.turnedDegrees())));
    noise = new Random(seed);
    upNoise = new Random(Seeds.upAxis(seed));
    period = errors.sampleHz() > 0 ? 1 / errors.sampleHz() : 0;
    lastVelocity = robot.pointVelocity(place);
    loopStartVelocity = lastVelocity;
  }

  /** After each physics step: the part filters what it feels, and samples it at its rate. */
  void step(double seconds) {
    Translation2d velocity = robot.pointVelocity(place);
    if (period > 0) {
      Translation3d felt = alongTheRobot(velocity.minus(lastVelocity).div(seconds));
      double share =
          errors.filterSeconds() > 0 ? 1 - Math.exp(-seconds / errors.filterSeconds()) : 1;
      filteredX += (felt.getX() - filteredX) * share;
      filteredY += (felt.getY() - filteredY) * share;
      filteredZ += (felt.getZ() - filteredZ) * share;
      sinceSample += seconds;
      if (sinceSample >= period - 1e-9) {
        sinceSample -= period;
        heldX = filteredX + errors.noise() * noise.nextGaussian();
        heldY = filteredY + errors.noise() * noise.nextGaussian();
        heldZ = filteredZ + errors.noise() * upNoise.nextGaussian();
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
   * What the robot's IO reports, once a loop: the part's reading along its own axes, turned into
   * the robot's forward and left by the definition's orientation.
   */
  Translation2d read() {
    Translation3d robotAxes =
        (period > 0 ? new Translation3d(heldX, heldY, heldZ) : loopAcceleration)
            .plus(new Translation3d(jolt.getX(), jolt.getY(), 0))
            .plus(new Translation3d(errors.biasForward(), errors.biasLeft(), 0));
    jolt = Translation2d.ZERO;
    Translation3d alongItsAxes = robotAxes.rotateBy(trueOrientation.inverse());
    double range = errors.rangeMetersPerSecSq();
    Translation3d read =
        new Translation3d(
                Math.clamp(alongItsAxes.getX(), -range, range),
                Math.clamp(alongItsAxes.getY(), -range, range),
                Math.clamp(alongItsAxes.getZ(), -range, range))
            .rotateBy(orientation);
    return new Translation2d(read.getX(), read.getY());
  }

  /**
   * An acceleration on the field, along the robot's axes, with the floor holding it up against
   * gravity: what the part feels. WPILib's attitude: a positive pitch is nose down, a positive roll
   * the left side up; the floor, tipped, pushes along all three.
   */
  private Translation3d alongTheRobot(Translation2d fieldAcceleration) {
    double pitch = robot.pitchRadians();
    double roll = robot.rollRadians();
    Translation2d driving = fieldAcceleration.rotateBy(robot.truePose().getRotation().unaryMinus());
    return new Translation3d(
        driving.getX() - ImuReadings.STANDARD_GRAVITY * Math.sin(pitch),
        driving.getY() + ImuReadings.STANDARD_GRAVITY * Math.sin(roll) * Math.cos(pitch),
        ImuReadings.STANDARD_GRAVITY * Math.cos(roll) * Math.cos(pitch));
  }
}
