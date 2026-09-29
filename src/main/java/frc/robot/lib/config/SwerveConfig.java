package frc.robot.lib.config;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;

/**
 * A swerve drivetrain: four identical modules, one near each corner of the frame.
 *
 * <p>Robot frame (WPILib's): +x forward, +y left, counterclockwise positive. Module order is always
 * front left, front right, back left, back right.
 *
 * @param modules the module hardware, the same at every corner
 * @param wheelInset how far each wheel's center sits in from the outside of the frame, in both
 *     directions. Corner-mounted modules give it in their footprint drawings.
 * @param limits current and speed limits
 * @param gains control gains
 */
@ConfigRecord
public record SwerveConfig(Modules modules, Distance wheelInset, Limits limits, Gains gains) {
  public SwerveConfig {
    Require.positive("wheelInset", wheelInset);
    if (limits.maxLinearSpeed().gt(modules.freeSpeed())) {
      throw new IllegalArgumentException(
          "maxLinearSpeed "
              + Require.shown(limits.maxLinearSpeed())
              + " is faster than the drivetrain's free speed of "
              + Require.shown(modules.freeSpeed()));
    }
  }

  /** Starts building a swerve drivetrain, one named value at a time. */
  public static SwerveConfigBuilder.ModulesStage builder() {
    return SwerveConfigBuilder.builder();
  }

  /**
   * The module hardware. Wheel radius shrinks as the tread wears; measure it again during the
   * season.
   *
   * @param driveMotor the motor that drives the wheel
   * @param steerMotor the motor that turns the module
   * @param wheelRadius measured on the tread
   * @param driveGearing motor rotations per wheel rotation
   * @param steerGearing motor rotations per full turn of the module
   * @param wheelCoefficientOfFriction the tread's grip on carpet: about 1.0 when worn, 1.3 new
   */
  @ConfigRecord
  public record Modules(
      Motor driveMotor,
      Motor steerMotor,
      Distance wheelRadius,
      double driveGearing,
      double steerGearing,
      double wheelCoefficientOfFriction) {
    public Modules {
      Require.positive("wheelRadius", wheelRadius);
      Require.positive("driveGearing", driveGearing);
      Require.positive("steerGearing", steerGearing);
      Require.positive("wheelCoefficientOfFriction", wheelCoefficientOfFriction);
    }

    /** Starts building the module hardware, one named value at a time. */
    public static SwerveConfigModulesBuilder.DriveMotorStage builder() {
      return SwerveConfigModulesBuilder.builder();
    }

    /** Top wheel speed at the motor's nominal voltage, with no load. The real robot is slower. */
    public LinearVelocity freeSpeed() {
      return MetersPerSecond.of(
          driveMotor.model().freeSpeed / driveGearing * wheelRadius.in(Meters));
    }

    /**
     * The drive kV (volts per meter per second) a frictionless drivetrain would have: the motor's
     * nominal voltage over the free speed. A starting point; measure the real one.
     */
    public double idealDriveKv() {
      return driveMotor.model().nominalVoltage / freeSpeed().in(MetersPerSecond);
    }
  }

  /**
   * Current limits are the main defense against wheel slip (more torque than the carpet can take)
   * and brownouts (more current than the battery can give). Speed limits cap what the driver can
   * ask for.
   *
   * <p>All are {@link Tunable}: dial them in on the robot, then commit what works.
   *
   * @param driveStatorCurrent the most current through each drive motor, which caps its torque
   * @param driveSupplyCurrent the most current each drive motor draws from the battery
   * @param steerStatorCurrent the most current through each steer motor
   * @param maxLinearSpeed the fastest the driver can ask the robot to move
   * @param maxAngularSpeed the fastest the driver can ask the robot to spin
   */
  @ConfigRecord
  public record Limits(
      @Tunable Current driveStatorCurrent,
      @Tunable Current driveSupplyCurrent,
      @Tunable Current steerStatorCurrent,
      @Tunable LinearVelocity maxLinearSpeed,
      @Tunable AngularVelocity maxAngularSpeed) {
    public Limits {
      Require.positive("driveStatorCurrent", driveStatorCurrent);
      Require.positive("driveSupplyCurrent", driveSupplyCurrent);
      Require.positive("steerStatorCurrent", steerStatorCurrent);
      Require.positive("maxLinearSpeed", maxLinearSpeed);
      Require.positive("maxAngularSpeed", maxAngularSpeed);
    }

    /** Starts building the limits, one named value at a time. */
    public static SwerveConfigLimitsBuilder.DriveStatorCurrentStage builder() {
      return SwerveConfigLimitsBuilder.builder();
    }
  }

  /**
   * Drive: velocity control on the motor controller, with feedforward from robot code. Steer:
   * position control on the motor controller.
   *
   * <p>All are {@link Tunable}: dial them in on the robot, then commit what works.
   *
   * @param driveKs volts to overcome static friction
   * @param driveKv volts per meter per second
   * @param driveKp volts per meter per second of error
   * @param steerKp volts per radian of error
   * @param steerKd volts per radian per second
   * @param skewLookahead field-relative commands use the heading the robot will have this far
   *     ahead, because the wheels take time to turn while the robot spins
   */
  @ConfigRecord
  public record Gains(
      @Tunable double driveKs,
      @Tunable double driveKv,
      @Tunable double driveKp,
      @Tunable double steerKp,
      @Tunable double steerKd,
      @Tunable Time skewLookahead) {
    public Gains {
      Require.nonNegative("driveKs", driveKs);
      Require.nonNegative("driveKv", driveKv);
      Require.nonNegative("driveKp", driveKp);
      Require.nonNegative("steerKp", steerKp);
      Require.nonNegative("steerKd", steerKd);
      Require.nonNegative("skewLookahead", skewLookahead);
    }

    /** Starts building the gains, one named value at a time. */
    public static SwerveConfigGainsBuilder.DriveKsStage builder() {
      return SwerveConfigGainsBuilder.builder();
    }
  }

  /**
   * Where each wheel touches the floor, from the robot's center: front left, front right, back
   * left, back right.
   *
   * @throws IllegalArgumentException if the inset leaves no room between the wheels
   */
  public Translation2d[] moduleTranslations(Body body) {
    double x = body.frameLength().in(Meters) / 2 - wheelInset.in(Meters);
    double y = body.frameWidth().in(Meters) / 2 - wheelInset.in(Meters);
    if (x <= 0 || y <= 0) {
      throw new IllegalArgumentException(
          "wheelInset "
              + Require.shown(wheelInset)
              + " puts the wheels at or past the center of the frame");
    }
    return new Translation2d[] {
      new Translation2d(x, y),
      new Translation2d(x, -y),
      new Translation2d(-x, y),
      new Translation2d(-x, -y)
    };
  }
}
