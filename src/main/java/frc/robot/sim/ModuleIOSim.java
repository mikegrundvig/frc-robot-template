// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.sim;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.subsystems.drive.DrivetrainControl;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.PhoenixDrivetrain;
import java.util.Arrays;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.MathUtil;

/**
 * A simulated swerve module's motor controllers and encoders: the actuators that drive one wheel of
 * the {@link Physics}, and the sensors that read it. Each sub-step the physics asks ({@link
 * #actuate}), the drive's controller works out the voltage across its motor from its loop and its
 * stator current limit, at the wheel's true speed, and the steering's runs its loop and turns the
 * module in closed form ({@link CurrentLimitedMotorSim}); the physics turns the wheel and moves the
 * robot. The motors and their gearing are the robot's definition's ({@code
 * DrivetrainGeometry.Modules}). Its absolute encoder can read off true ({@link SensorErrors}), and
 * its zero can be knocked further off while it runs. Its encoders' readings can go stale: the wheel
 * turns on, and what the robot's code reads stops changing, as signals that stop arriving do. Both
 * motors' stator current limits are modeled: the drive's caps the torque that accelerates the robot
 * and says when a wheel is at the edge of its grip, and the steering's caps how hard the module
 * turns and stops. The battery holds 12 V, and the drive's supply current limit, with Phoenix's
 * step-down from it ({@code DrivetrainControl.Limits.driveSupplyLowerCurrent}), isn't modeled: the
 * supply current it reports is what the motor would draw from a steady 12 V with no supply limit.
 * So this tests the code rather than the robot's power; docs/stand-ins.md says when that matters.
 *
 * <p>The steering runs as a TalonFX without a Phoenix Pro license runs it: its loop every
 * millisecond, on the angle the CANcoder last sent over CAN. Those, with its current limit, set how
 * high its gains can go before it overshoots, as on a robot.
 */
final class ModuleIOSim implements ModuleIO {
  private static final double MAX_VOLTS = 12.0;

  /**
   * The battery's voltage, which no motor gets more than: 12 V, steady, as WPILib's simulated
   * battery gives it ({@code DCMotorSim} held its motor to it the same way).
   */
  private static final double BATTERY_VOLTS = 12.0;

  /** A TalonFX runs its closed loop every millisecond (CTRE's Phoenix 6 documentation: 1 kHz). */
  private static final double STEER_LOOP_SECONDS = 0.001;

  /**
   * How often the module's CANcoder sends its angle to the steering motor, in seconds: at the rate
   * the robot's code sets ({@link PhoenixDrivetrain#ENCODER_FREQUENCY_HZ}), every 10 ms at 100 Hz.
   * Without a Phoenix Pro license the steering closes its loop on the CANcoder as a remote sensor,
   * and takes a new angle and speed only when the CANcoder sends them over CAN. In between, its
   * loop runs on what it last heard, up to a period old, and that wait, more than the loop's rate
   * or the current limit, is what makes the steering overshoot at high gains. The fraction of a
   * millisecond a frame takes on the bus is left out. With Pro's FusedCANcoder, the loop would read
   * the motor's own sensor every loop, and this would be {@link #STEER_LOOP_SECONDS}.
   */
  private static final double ENCODER_PERIOD_SECONDS = 1 / PhoenixDrivetrain.ENCODER_FREQUENCY_HZ;

  private static final int ENCODER_PERIOD_LOOPS =
      (int) Math.round(ENCODER_PERIOD_SECONDS / STEER_LOOP_SECONDS);

  // The CANcoder sends on the steering loop's millisecond, and the same number of times every robot
  // loop, so each robot loop finds every CANcoder at the same point in its cycle.
  static {
    double loopsPerRobotLoop = RobotLoop.PERIOD_SECONDS / ENCODER_PERIOD_SECONDS;
    if (Math.abs(ENCODER_PERIOD_LOOPS * STEER_LOOP_SECONDS - ENCODER_PERIOD_SECONDS) > 1e-9
        || Math.abs(loopsPerRobotLoop - Math.round(loopsPerRobotLoop)) > 1e-9) {
      throw new IllegalStateException(
          "the CANcoder's period, "
              + ENCODER_PERIOD_SECONDS
              + " s, must be a whole number of milliseconds and divide the robot's loop");
    }
  }

  private final WheelTruth wheel;
  private final CurrentLimitedMotorSim steerSim;
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
  private double staleSteerRadPerSec;

  /**
   * Rotation the wheel spun without rolling the robot along: its encoder saw it, the floor didn't.
   */
  private double spinRadians;

  /** How fast it keeps spinning beyond what rolls the robot along, in radians per second. */
  private double spinRadPerSec;

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
  private double steerStatorLimitAmps = Double.POSITIVE_INFINITY;

  /** The angle and speed the steering's motor controller last heard from the CANcoder. */
  private double heardSteerRadians;

  private double heardSteerRadPerSec;

  /** The steering's loops until the CANcoder next sends. */
  private int loopsUntilEncoderSends;

  /** The most current through the steering motor since the last loop read it. */
  private double steerPeakAmps;

  /**
   * @param modules the modules as defined: their motors, gearing, and moving parts. The motor
   *     controllers convert with this wheel radius. The steering's inertia sets how fast it turns
   *     at its current limit, and so how high its gains can go (docs/stand-ins.md).
   * @param wheel the wheel it drives, as the physics has it: its encoders read it
   * @param steerOffsetRadians how far the absolute encoder's zero is off
   * @param encoderPhase when the CANcoder first sends, as a share of its period: each module's
   *     CANcoder sends on its own clock
   */
  ModuleIOSim(
      DrivetrainGeometry.Modules modules,
      WheelTruth wheel,
      double steerOffsetRadians,
      double encoderPhase) {
    Actuator.Motors drive = modules.drive();
    Actuator.Motors steer = modules.steer();
    this.wheel = wheel;
    driveMotor = drive.model();
    driveGearing = drive.gearing().ratio();
    wheelRadiusMeters = modules.wheelRadius().in(Meters);
    steerSim =
        new CurrentLimitedMotorSim(
            steer.model(),
            modules.steerInertia().in(KilogramSquareMeters),
            steer.gearing().ratio());
    this.steerOffsetRadians = steerOffsetRadians;
    builtSteerOffsetRadians = steerOffsetRadians;
    heardSteerRadians = measuredSteerRadians();
    loopsUntilEncoderSends = (int) (encoderPhase * ENCODER_PERIOD_LOOPS);
  }

  /**
   * Its absolute encoder's zero off by this much from now on: the steering's loop, which runs on
   * the encoder, turns the wheel to match once the CANcoder next sends.
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
      staleVelocityRadPerSec = wheel.spinRadPerSec();
      staleSteerRadians = heardSteerRadians;
      staleSteerRadPerSec = heardSteerRadPerSec;
    }
    this.stale = stale;
  }

  boolean isStale() {
    return stale;
  }

  /**
   * Runs the motor controllers through a physics sub-step: the drive's loop once, at the wheel's
   * speed as the sub-step starts, and the steering's every millisecond, turning the module. The
   * drive's motor is about sixty times slower than the steering's, and reads its own sensor, so its
   * loop running once a sub-step changes nothing that matters.
   *
   * @param drive what this wheel's drive and steering do over the sub-step, for the physics
   */
  void actuate(double seconds, WheelDrive drive) {
    double speed = wheel.spinRadPerSec();
    if (driveClosedLoop) {
      double error = driveSetpointRadPerSec - speed;
      driveVolts = driveFeedforwardVolts + driveKp * error * wheelRadiusMeters;
    }
    driveVolts = Math.clamp(driveVolts, -MAX_VOLTS, MAX_VOLTS);
    // The motor controller caps the current through the motor, and with it the torque: it applies
    // no more than the motor's back EMF plus the limit's worth of voltage across the windings.
    double backEmf = speed * driveGearing / driveMotor.Kv;
    double headroom = driveStatorLimitAmps * driveMotor.R;
    driveAppliedVolts = Math.clamp(driveVolts, backEmf - headroom, backEmf + headroom);
    driveLimited |= driveAppliedVolts != driveVolts;
    drive.drive(
        driveAppliedVolts * Math.min(1.0, BATTERY_VOLTS / Math.abs(driveAppliedVolts)),
        // A velocity loop's gain, while nothing clamps what it asks for.
        driveClosedLoop && driveAppliedVolts == driveVolts && Math.abs(driveVolts) < MAX_VOLTS
            ? -driveKp * wheelRadiusMeters
            : 0,
        driveStatorLimitAmps);
    spinRadians += spinRadPerSec * seconds;
    int loops = Math.max(1, (int) Math.round(seconds / STEER_LOOP_SECONDS));
    for (int i = 0; i < loops; i++) {
      steer(seconds / loops);
    }
    drive.steer(trueSteerAngle().getRadians());
  }

  /** One pass of the steering's loop, on what the CANcoder last sent, then its motor that long. */
  private void steer(double seconds) {
    if (loopsUntilEncoderSends == 0) {
      heardSteerRadians = measuredSteerRadians();
      heardSteerRadPerSec = steerSim.velocityRadPerSec();
      loopsUntilEncoderSends = ENCODER_PERIOD_LOOPS;
    }
    loopsUntilEncoderSends--;
    // The steering loop runs on what the absolute encoder sent, right or not.
    double volts =
        steering
            ? Math.clamp(
                steerKp * MathUtil.angleModulus(steerSetpointRad - heardSteerRadians)
                    - steerKd * heardSteerRadPerSec,
                -MAX_VOLTS,
                MAX_VOLTS)
            : 0.0;
    steerPeakAmps = Math.max(steerPeakAmps, steerSim.run(volts, steerStatorLimitAmps, seconds));
  }

  /**
   * Takes a reading for odometry, as a motor controller's high-rate signals would. The steering's
   * angle is the one its motor controller last heard from the CANcoder, as on a robot without
   * Phoenix Pro: up to a CANcoder's period old.
   */
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

  /**
   * From now on the wheel spins this much faster than the floor moves it, in meters per second at
   * its configured radius, as one losing its grip does: its encoder counts it, the floor doesn't.
   */
  void spinAt(double metersPerSecond) {
    spinRadPerSec = metersPerSecond / wheelRadiusMeters;
  }

  /** How much faster than the floor moves it the wheel spins, in meters per second. */
  double spinMetersPerSecond() {
    return spinRadPerSec * wheelRadiusMeters;
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.odometryTimestampsNanos = Arrays.copyOf(sampleTimes, samples);
    inputs.odometryDrivePositionsRad = Arrays.copyOf(samplePositions, samples);
    inputs.odometrySteerAnglesRad = Arrays.copyOf(sampleAngles, samples);
    samples = 0;
    inputs.driveConnected = true;
    inputs.drivePositionRad = drivePositionRadians();
    inputs.driveVelocityRadPerSec =
        stale ? staleVelocityRadPerSec : wheel.spinRadPerSec() + spinRadPerSec;
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveStatorCurrentAmps = Math.abs(wheel.driveCurrentAmps());
    inputs.driveSupplyCurrentAmps =
        Math.abs(driveAppliedVolts * wheel.driveCurrentAmps()) / MAX_VOLTS;
    inputs.driveTorqueLimited = driveLimited;
    driveLimited = false;
    inputs.steerConnected = true;
    // The steering's angle and speed as the robot hears them: what the CANcoder last sent.
    inputs.steerAngle = new Rotation2d(steerReadingRadians());
    inputs.steerVelocityRadPerSec = stale ? staleSteerRadPerSec : heardSteerRadPerSec;
    inputs.steerAppliedVolts = steerSim.appliedVolts();
    // The steering's current is the most it drew since the last loop, not what it draws now. It
    // runs at its limit for a few milliseconds as the module starts and stops a turn, then falls to
    // next to nothing, so a reading at the loop's end would almost never see the peak, and the peak
    // is what matters: how close the motor came to its limit. The drive's current changes over many
    // loops (at its limit for as long as the wheel takes to get up to speed), so a reading now
    // shows it, and driveTorqueLimited says whether it reached the limit since the last loop.
    inputs.steerStatorCurrentAmps = steerPeakAmps;
    steerPeakAmps = 0;
  }

  /** What the drive encoder reads: every turn of the wheel, slips too, unless it's stale. */
  private double drivePositionRadians() {
    return stale ? stalePositionRadians : wheel.rotationRadians() + spinRadians;
  }

  /**
   * The steering's angle as the robot's code reads it: what the CANcoder last sent, unless stale.
   */
  private double steerReadingRadians() {
    return stale ? staleSteerRadians : heardSteerRadians;
  }

  /** Which way the wheel really points, whatever its encoder reads. */
  Rotation2d trueSteerAngle() {
    return new Rotation2d(steerSim.positionRadians());
  }

  private double measuredSteerRadians() {
    return steerSim.positionRadians() - steerOffsetRadians;
  }

  /**
   * Takes the gains, and both motors' stator current limits. The drive's supply limit isn't
   * modeled.
   */
  @Override
  public void configure(DrivetrainControl.Gains gains, DrivetrainControl.Limits limits) {
    driveKp = gains.driveKp();
    steerKp = gains.steerKp();
    steerKd = gains.steerKd();
    driveStatorLimitAmps = limits.driveStatorCurrent().in(Amps);
    steerStatorLimitAmps = limits.steerStatorCurrent().in(Amps);
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
