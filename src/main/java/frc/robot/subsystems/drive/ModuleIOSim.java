// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Meters;

import frc.robot.lib.config.SwerveConfig;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.MathUtil;
import org.wpilib.simulation.DCMotorSim;

/**
 * A module simulated the standard way, as in AdvantageKit's swerve templates: each motor turns a
 * fixed load through its gearing, and the motor controllers' loops run once per step. A {@link
 * DriveSimulation} steps it and moves the robot with its true wheel motion; its absolute encoder
 * can read off true ({@link SensorErrors}). Current limits and the battery are not modeled, so this
 * tests the code rather than the robot's power.
 */
final class ModuleIOSim implements ModuleIO {
  // Tuner X's default simulated inertias, as AdvantageKit's template uses.
  private static final double DRIVE_INERTIA_KG_M2 = 0.025;
  private static final double STEER_INERTIA_KG_M2 = 0.004;
  private static final double MAX_VOLTS = 12.0;

  private final DCMotorSim driveSim;
  private final DCMotorSim steerSim;
  private final double wheelRadiusMeters;

  /** How far the absolute encoder's zero is off: it reads the true angle minus this. */
  private final double steerOffsetRadians;

  private double driveKp;
  private double steerKp;
  private double steerKd;

  private boolean driveClosedLoop;
  private double driveSetpointRadPerSec;
  private double driveFeedforwardVolts;
  private double driveVolts;
  private boolean steering;
  private double steerSetpointRad;
  private double steerVolts;

  /**
   * @param modules the modules as configured; the motor controllers convert with this wheel radius
   * @param steerOffsetRadians how far the absolute encoder's zero is off
   */
  ModuleIOSim(SwerveConfig.Modules modules, double steerOffsetRadians) {
    driveSim = motorSim(modules.driveMotor().model(), DRIVE_INERTIA_KG_M2, modules.driveGearing());
    steerSim = motorSim(modules.steerMotor().model(), STEER_INERTIA_KG_M2, modules.steerGearing());
    wheelRadiusMeters = modules.wheelRadius().in(Meters);
    this.steerOffsetRadians = steerOffsetRadians;
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
    driveSim.setInputVoltage(driveVolts);
    steerSim.setInputVoltage(steerVolts);
    driveSim.update(seconds);
    steerSim.update(seconds);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPosition();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveVolts;
    inputs.driveStatorCurrentAmps = Math.abs(driveSim.getCurrentDraw());
    inputs.driveSupplyCurrentAmps = Math.abs(driveVolts * driveSim.getCurrentDraw()) / MAX_VOLTS;
    inputs.steerConnected = true;
    inputs.steerAngle = new Rotation2d(measuredSteerRadians());
    inputs.steerVelocityRadPerSec = steerSim.getAngularVelocity();
    inputs.steerAppliedVolts = steerVolts;
    inputs.steerStatorCurrentAmps = Math.abs(steerSim.getCurrentDraw());
  }

  /** The wheel's rotation since the start, in radians: its encoder reads it exactly. */
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

  /** Takes the gains. Current limits are not simulated. */
  @Override
  public void configure(SwerveConfig.Gains gains, SwerveConfig.Limits limits) {
    driveKp = gains.driveKp();
    steerKp = gains.steerKp();
    steerKd = gains.steerKd();
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
