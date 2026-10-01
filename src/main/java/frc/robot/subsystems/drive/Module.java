package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.SwerveConfig;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Distance;

/** One swerve module: converts wheel radians to meters and runs module setpoints. */
public class Module {
  static final List<String> NAMES = List.of("FrontLeft", "FrontRight", "BackLeft", "BackRight");

  /** Below this speed, hold the wheel's current angle instead of steering toward noise. */
  private static final double STEER_HOLD_SPEED_MPS = 0.01;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final String name;
  private final double wheelRadiusMeters;
  private double driveKs;
  private double driveKv;

  Module(ModuleIO io, int index, Distance wheelRadius) {
    this.io = io;
    this.name = NAMES.get(index);
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
    double feedforward = driveKs * Math.signum(velocity) + driveKv * velocity;
    io.setDriveVelocity(velocity / wheelRadiusMeters, feedforward);
    io.setSteerAngle(target.angle);
    return target;
  }

  /**
   * Points the wheel at an angle, relative to the robot, with the drive wheel held still: a
   * steering test's motion. Unlike driving, it takes no shortcut (reversing the wheel rather than
   * turning past 90°), so the steering makes the whole move asked of it.
   */
  void holdSteer(Rotation2d angle) {
    io.setDriveVelocity(0, 0);
    io.setSteerAngle(angle);
  }

  void stop() {
    io.stop();
  }

  /** The steering motor's voltage, as of this loop. */
  double steerAppliedVolts() {
    return inputs.steerAppliedVolts;
  }

  /** The most current through the steering motor since the last loop, either way. */
  double steerCurrentAmps() {
    return inputs.steerStatorCurrentAmps;
  }

  /** The module's devices that didn't take their configuration at startup, or empty. */
  String unconfigured() {
    return inputs.unconfigured;
  }

  SwerveModulePosition getPosition() {
    return new SwerveModulePosition(inputs.drivePositionRad * wheelRadiusMeters, inputs.steerAngle);
  }

  SwerveModuleVelocity getVelocity() {
    return new SwerveModuleVelocity(
        inputs.driveVelocityRadPerSec * wheelRadiusMeters, inputs.steerAngle);
  }

  /** Which way the wheel points, as its module measures it. */
  Rotation2d steerAngle() {
    return inputs.steerAngle;
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
