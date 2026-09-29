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
 * fixed load through its gearing, and the motor controllers' loops run here once per robot loop.
 * The wheels never slip, and current limits and the battery are not modeled, so this tests the code
 * rather than the robot's traction or power.
 */
public final class ModuleIOSim implements ModuleIO {
  // Tuner X's default simulated inertias, as AdvantageKit's template uses.
  private static final double DRIVE_INERTIA_KG_M2 = 0.025;
  private static final double STEER_INERTIA_KG_M2 = 0.004;
  private static final double LOOP_PERIOD_S = 0.02;
  private static final double MAX_VOLTS = 12.0;

  private final DCMotorSim driveSim;
  private final DCMotorSim steerSim;
  private final double wheelRadiusMeters;

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

  public ModuleIOSim(SwerveConfig.Modules modules) {
    driveSim = motorSim(modules.driveMotor().model(), DRIVE_INERTIA_KG_M2, modules.driveGearing());
    steerSim = motorSim(modules.steerMotor().model(), STEER_INERTIA_KG_M2, modules.steerGearing());
    wheelRadiusMeters = modules.wheelRadius().in(Meters);
  }

  /** The four modules of a simulated robot: front left, front right, back left, back right. */
  public static ModuleIO[] of(SwerveConfig.Modules modules) {
    return new ModuleIO[] {
      new ModuleIOSim(modules),
      new ModuleIOSim(modules),
      new ModuleIOSim(modules),
      new ModuleIOSim(modules)
    };
  }

  /** A motor turning a load. WPILib 2027 names this model for an arm; without gravity it's one. */
  private static DCMotorSim motorSim(DCMotor motor, double inertiaKgM2, double gearing) {
    return new DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(motor, inertiaKgM2, gearing), motor);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    if (driveClosedLoop) {
      double error = driveSetpointRadPerSec - driveSim.getAngularVelocity();
      driveVolts = driveFeedforwardVolts + driveKp * error * wheelRadiusMeters;
    }
    steerVolts =
        steering
            ? steerKp * MathUtil.angleModulus(steerSetpointRad - steerSim.getAngularPosition())
                - steerKd * steerSim.getAngularVelocity()
            : 0.0;
    driveVolts = Math.clamp(driveVolts, -MAX_VOLTS, MAX_VOLTS);
    steerVolts = Math.clamp(steerVolts, -MAX_VOLTS, MAX_VOLTS);
    driveSim.setInputVoltage(driveVolts);
    steerSim.setInputVoltage(steerVolts);
    driveSim.update(LOOP_PERIOD_S);
    steerSim.update(LOOP_PERIOD_S);

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPosition();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocity();
    inputs.driveAppliedVolts = driveVolts;
    inputs.driveStatorCurrentAmps = Math.abs(driveSim.getCurrentDraw());
    inputs.driveSupplyCurrentAmps = Math.abs(driveVolts * driveSim.getCurrentDraw()) / MAX_VOLTS;
    inputs.steerConnected = true;
    inputs.steerAngle = new Rotation2d(steerSim.getAngularPosition());
    inputs.steerVelocityRadPerSec = steerSim.getAngularVelocity();
    inputs.steerAppliedVolts = steerVolts;
    inputs.steerStatorCurrentAmps = Math.abs(steerSim.getCurrentDraw());
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
