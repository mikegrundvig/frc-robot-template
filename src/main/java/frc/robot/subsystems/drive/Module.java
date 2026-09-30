package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.SwerveConfig;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Distance;

/** One swerve module: converts wheel radians to meters and runs module setpoints. */
public class Module {
  static final String[] NAMES = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};

  /** Below this speed, hold the wheel's current angle instead of steering toward noise. */
  private static final double STEER_HOLD_SPEED_MPS = 0.01;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final String name;
  private final double wheelRadiusMeters;
  private double driveKs;
  private double driveKv;

  /** The wheel speed last sent, in meters per second: zero once stopped. */
  private double commandedMetersPerSecond;

  Module(ModuleIO io, int index, Distance wheelRadius) {
    this.io = io;
    this.name = NAMES[index];
    this.wheelRadiusMeters = wheelRadius.in(Meters);
  }

  /** Takes new gains and limits: the feedforward here, the rest on the motor controllers. */
  void configure(SwerveConfig.Gains gains, SwerveConfig.Limits limits) {
    driveKs = gains.driveKs();
    driveKv = gains.driveKv();
    io.configure(gains, limits);
  }

  /** Reads the hardware (in replay, nothing: the log supplies the inputs next). */
  void readInputs() {
    io.updateInputs(inputs);
  }

  /** Logs the inputs, or in replay, takes them from the log. */
  void logInputs() {
    Logger.processInputs("Drive/Module" + name, inputs);
  }

  /** Drive toward a setpoint. Returns the setpoint actually sent (optimized and cosine-scaled). */
  SwerveModuleVelocity runSetpoint(SwerveModuleVelocity desired) {
    Rotation2d current = inputs.steerAngle;
    SwerveModuleVelocity target =
        Math.abs(desired.velocity) < STEER_HOLD_SPEED_MPS
            ? new SwerveModuleVelocity(0.0, current)
            // Turn at most 90 degrees (reverse the wheel instead), and slow the wheel while it is
            // still pointing the wrong way.
            : desired.optimize(current).cosineScale(current);

    double velocity = target.velocity;
    commandedMetersPerSecond = velocity;
    double feedforward = driveKs * Math.signum(velocity) + driveKv * velocity;
    io.setDriveVelocity(velocity / wheelRadiusMeters, feedforward);
    io.setSteerAngle(target.angle);
    return target;
  }

  void stop() {
    commandedMetersPerSecond = 0;
    io.stop();
  }

  /** Whether the drive motor hit its current limit since the last loop. */
  boolean isTorqueLimited() {
    return inputs.driveTorqueLimited;
  }

  /**
   * How far the wheel's speed falls short of the speed it was last sent, in meters per second:
   * negative when it's faster.
   */
  double speedDeficitMetersPerSecond() {
    return Math.abs(commandedMetersPerSecond)
        - Math.abs(inputs.driveVelocityRadPerSec * wheelRadiusMeters);
  }

  SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.drivePositionRad * wheelRadiusMeters, inputs.steerAngle);
  }

  SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(
        inputs.driveVelocityRadPerSec * wheelRadiusMeters, inputs.steerAngle);
  }

  /** How far the wheel has turned, in radians. */
  double wheelRotationRadians() {
    return inputs.drivePositionRad;
  }

  boolean isConnected() {
    return inputs.driveConnected && inputs.steerConnected;
  }

  /** How many high-rate readings arrived this loop, for odometry. */
  int odometryReadings() {
    return Math.min(
        inputs.odometryTimestampsNanos.length,
        Math.min(inputs.odometryDrivePositionsRad.length, inputs.odometrySteerAnglesRad.length));
  }

  long odometryTimestampNanos(int reading) {
    return inputs.odometryTimestampsNanos[reading];
  }

  /** How far the wheel had rolled, in meters, at a reading. */
  double odometryDistanceMeters(int reading) {
    return inputs.odometryDrivePositionsRad[reading] * wheelRadiusMeters;
  }

  /** Which way the wheel pointed, in radians, at a reading. */
  double odometryAngleRadians(int reading) {
    return inputs.odometrySteerAnglesRad[reading];
  }
}
