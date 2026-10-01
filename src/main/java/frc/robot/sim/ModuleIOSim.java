// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.sim;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.SwerveConfig;
import frc.robot.subsystems.drive.ModuleIO;
import java.util.Arrays;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.MathUtil;
import org.wpilib.simulation.DCMotorSim;

/**
 * A module simulated the standard way, as in AdvantageKit's swerve templates: each motor turns a
 * fixed load through its gearing, and the motor controllers' loops run once per step. A {@link
 * DriveSimulation} steps it and moves the robot with its true wheel motion; its absolute encoder
 * can read off true ({@link SensorErrors}), and its zero can be knocked further off while it runs.
 * Its encoders' readings can go stale: the wheel turns on, and what the robot's code reads stops
 * changing, as signals that stop arriving do. The drive motor's stator current limit is modeled,
 * since it caps the torque that accelerates the robot and says when a wheel is at the edge of its
 * grip; the battery and the steering motor's limit are not, so this tests the code rather than the
 * robot's power.
 */
final class ModuleIOSim implements ModuleIO {
  // Tuner X's default simulated inertias, as AdvantageKit's template uses.
  private static final double DRIVE_INERTIA_KG_M2 = 0.025;
  private static final double STEER_INERTIA_KG_M2 = 0.004;
  private static final double MAX_VOLTS = 12.0;

  private final DCMotorSim driveSim;
  private final DCMotorSim steerSim;
  private final DCMotor driveMotor;
  private final double driveGearing;
  private final double wheelRadiusMeters;

  /** How far the absolute encoder's zero is off: it reads the true angle minus this. */
  private double steerOffsetRadians;

  /** How far it was off as the robot was built. */
  private final double builtSteerOffsetRadians;

  /** Whether the encoders' readings have stopped changing, and what they're stuck at. */
  private boolean stale;

  private double stalePositionRadians;
  private double staleVelocityRadPerSec;
  private double staleSteerRadians;

  /**
   * Rotation the wheel spun without rolling the robot along: its encoder saw it, the floor didn't.
   */
  private double spinRadians;

  // Readings taken since the last loop, for odometry.
  private long[] sampleTimes = new long[8];
  private double[] samplePositions = new double[8];
  private double[] sampleAngles = new double[8];
  private int samples;

  private double driveKp;
  private double steerKp;
  private double steerKd;

  private boolean driveClosedLoop;
  private double driveSetpointRadPerSec;
  private double driveFeedforwardVolts;
  private double driveVolts;
  private double driveAppliedVolts;
  private double driveStatorLimitAmps = Double.POSITIVE_INFINITY;
  private boolean driveLimited;
  private boolean steering;
  private double steerSetpointRad;
  private double steerVolts;

  /**
   * @param modules the modules as configured; the motor controllers convert with this wheel radius
   * @param steerOffsetRadians how far the absolute encoder's zero is off
   */
  ModuleIOSim(SwerveConfig.Modules modules, double steerOffsetRadians) {
    driveMotor = modules.driveMotor().model();
    driveGearing = modules.driveGearing();
    driveSim = motorSim(driveMotor, DRIVE_INERTIA_KG_M2, driveGearing);
    steerSim = motorSim(modules.steerMotor().model(), STEER_INERTIA_KG_M2, modules.steerGearing());
    wheelRadiusMeters = modules.wheelRadius().in(Meters);
    this.steerOffsetRadians = steerOffsetRadians;
    builtSteerOffsetRadians = steerOffsetRadians;
  }

  /**
   * Its absolute encoder's zero off by this much from now on: the steering's loop, which runs on
   * the encoder, turns the wheel to match.
   */
  void setSteerOffset(double radians) {
    steerOffsetRadians = radians;
  }

  double steerOffsetRadians() {
    return steerOffsetRadians;
  }

  double builtSteerOffsetRadians() {
    return builtSteerOffsetRadians;
  }

  /**
   * The encoders' readings stop changing while the wheel turns on, or catch up again. The motor
   * controllers' own loops keep their live readings: only what the robot's code reads is stale.
   */
  void setStale(boolean stale) {
    if (stale && !this.stale) {
      stalePositionRadians = drivePositionRadians();
      staleVelocityRadPerSec = driveSim.getAngularVelocity();
      staleSteerRadians = steerReadingRadians();
    }
    this.stale = stale;
  }

  boolean isStale() {
    return stale;
  }

  /** A motor turning a load. WPILib 2027 names this model for an arm; without gravity it's one. */
  private static DCMotorSim motorSim(DCMotor motor, double inertiaKgM2, double gearing) {
    return new DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(motor, inertiaKgM2, gearing), motor);
  }

  /** Runs the motor controllers' loops, then the motors, for a step. */
  void step(double seconds) {
    if (driveClosedLoop) {
      double error = driveSetpointRadPerSec - driveSim.getAngularVelocity();
      driveVolts = driveFeedforwardVolts + driveKp * error * wheelRadiusMeters;
    }
    // The steering loop runs on what the absolute encoder reads, right or not.
    steerVolts =
        steering
            ? steerKp * MathUtil.angleModulus(steerSetpointRad - measuredSteerRadians())
                - steerKd * steerSim.getAngularVelocity()
            : 0.0;
    driveVolts = Math.clamp(driveVolts, -MAX_VOLTS, MAX_VOLTS);
    steerVolts = Math.clamp(steerVolts, -MAX_VOLTS, MAX_VOLTS);
    // The motor controller caps the current through the motor, and with it the torque: it applies
    // no more than the motor's back EMF plus the limit's worth of voltage across the windings.
    double backEmf = driveSim.getAngularVelocity() * driveGearing / driveMotor.Kv;
    double headroom = driveStatorLimitAmps * driveMotor.R;
    driveAppliedVolts = Math.clamp(driveVolts, backEmf - headroom, backEmf + headroom);
    driveLimited |= driveAppliedVolts != driveVolts;
    driveSim.setInputVoltage(driveAppliedVolts);
    steerSim.setInputVoltage(steerVolts);
    driveSim.update(seconds);
    steerSim.update(seconds);
  }

  /** Takes a reading for odometry, as a motor controller's high-rate signals would. */
  void sample(long timestampNanos) {
    if (samples == sampleTimes.length) {
      sampleTimes = Arrays.copyOf(sampleTimes, samples * 2);
      samplePositions = Arrays.copyOf(samplePositions, samples * 2);
      sampleAngles = Arrays.copyOf(sampleAngles, samples * 2);
    }
    sampleTimes[samples] = timestampNanos;
    samplePositions[samples] = drivePositionRadians();
    sampleAngles[samples] = steerReadingRadians();
    samples++;
  }

  /** Spins the wheel without the robot moving: a slip its encoder sees and the floor doesn't. */
  void spin(double radians) {
    spinRadians += radians;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.odometryTimestampsNanos = Arrays.copyOf(sampleTimes, samples);
    inputs.odometryDrivePositionsRad = Arrays.copyOf(samplePositions, samples);
    inputs.odometrySteerAnglesRad = Arrays.copyOf(sampleAngles, samples);
    samples = 0;
    inputs.driveConnected = true;
    inputs.drivePositionRad = drivePositionRadians();
    inputs.driveVelocityRadPerSec = stale ? staleVelocityRadPerSec : driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveStatorCurrentAmps = Math.abs(driveSim.getCurrentDraw());
    inputs.driveSupplyCurrentAmps =
        Math.abs(driveAppliedVolts * driveSim.getCurrentDraw()) / MAX_VOLTS;
    inputs.driveTorqueLimited = driveLimited;
    driveLimited = false;
    inputs.steerConnected = true;
    inputs.steerAngle = new Rotation2d(steerReadingRadians());
    inputs.steerVelocityRadPerSec = stale ? 0 : steerSim.getAngularVelocity();
    inputs.steerAppliedVolts = steerVolts;
    inputs.steerStatorCurrentAmps = Math.abs(steerSim.getCurrentDraw());
  }

  /** What the drive encoder reads: every turn of the wheel, slips too, unless it's stale. */
  private double drivePositionRadians() {
    return stale ? stalePositionRadians : driveSim.getAngularPosition() + spinRadians;
  }

  /** What the robot's code reads of the steering's angle: the absolute encoder, unless stale. */
  private double steerReadingRadians() {
    return stale ? staleSteerRadians : measuredSteerRadians();
  }

  /** The wheel's rotation since the start that rolled the robot along, in radians. */
  double wheelRotationRadians() {
    return driveSim.getAngularPosition();
  }

  /** Which way the wheel really points, whatever its encoder reads. */
  Rotation2d trueSteerAngle() {
    return new Rotation2d(steerSim.getAngularPosition());
  }

  private double measuredSteerRadians() {
    return steerSim.getAngularPosition() - steerOffsetRadians;
  }

  /** Takes the gains, and the drive motor's stator current limit. */
  @Override
  public void configure(SwerveConfig.Gains gains, SwerveConfig.Limits limits) {
    driveKp = gains.driveKp();
    steerKp = gains.steerKp();
    steerKd = gains.steerKd();
    driveStatorLimitAmps = limits.driveStatorCurrent().in(Amps);
  }

  @Override
  public void setDriveVelocity(double wheelRadPerSec, double feedforwardVolts) {
    driveClosedLoop = true;
    driveSetpointRadPerSec = wheelRadPerSec;
    driveFeedforwardVolts = feedforwardVolts;
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveClosedLoop = false;
    driveVolts = volts;
  }

  @Override
  public void setSteerAngle(Rotation2d angle) {
    steering = true;
    steerSetpointRad = angle.getRadians();
  }

  @Override
  public void stop() {
    driveClosedLoop = false;
    driveVolts = 0.0;
    steering = false;
  }
}
