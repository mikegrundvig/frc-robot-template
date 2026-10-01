package frc.robot.lib.config;

import static frc.robot.lib.config.Rules.atMost;
import static frc.robot.lib.config.Rules.nonNegative;
import static frc.robot.lib.config.Rules.positive;
import static frc.robot.lib.config.Rules.shown;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.jspecify.annotations.Nullable;
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
    Checks.on(SwerveConfig.class)
        .field("wheelInset", wheelInset, positive())
        .check(
            "limits.maxLinearSpeed",
            () ->
                limits.maxLinearSpeed().lte(modules.freeSpeed())
                    ? null
                    : shown(limits.maxLinearSpeed())
                        + " is faster than the drivetrain's free speed of "
                        + shown(modules.freeSpeed()))
        .orThrow();
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
      Checks.on(Modules.class)
          .field("wheelRadius", wheelRadius, positive())
          .field("driveGearing", driveGearing, positive())
          .field("steerGearing", steerGearing, positive())
          .field("wheelCoefficientOfFriction", wheelCoefficientOfFriction, positive())
          .orThrow();
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
   * <p>The drive's supply limit steps down. Phoenix 6 lets a drive motor draw up to {@code
   * driveSupplyCurrent}, but once it has been held at that limit for {@code driveSupplyLowerTime},
   * it lowers the limit to {@code driveSupplyLowerCurrent} until the motor draws less than that
   * again. The step-down keeps the motor's breaker from tripping in a long push: a 40 A breaker
   * carries 40 A for as long as you like, but 60 A for as little as 3.9 seconds. Phoenix steps down
   * even when the code never mentions it (to 40 A after 1 s), so the template sets both values
   * here, where they can be seen. A robot that loses its push after a second of hard driving is
   * seeing the step-down.
   *
   * <p>All are tunable on a running robot (the drivetrain says within what): dial them in on the
   * robot, then commit what works.
   *
   * @param driveStatorCurrent the most current through each drive motor, which caps its torque
   * @param driveSupplyCurrent the most current each drive motor draws from the battery, for at most
   *     driveSupplyLowerTime at a time: then the limit steps down to driveSupplyLowerCurrent
   * @param driveSupplyLowerCurrent each drive motor's supply limit after the step-down, until it
   *     draws less than this again: no more than its breaker's rating. With driveSupplyCurrent at
   *     or below it, there's nothing to step down from, and it does nothing.
   * @param driveSupplyLowerTime how long a drive motor may draw driveSupplyCurrent before its limit
   *     steps down to driveSupplyLowerCurrent: longer than a sprint, and well short of the time its
   *     breaker takes to trip
   * @param steerStatorCurrent the most current through each steer motor
   * @param maxLinearSpeed the fastest the driver can ask the robot to move
   * @param maxAngularSpeed the fastest the driver can ask the robot to spin
   */
  @ConfigRecord
  public record Limits(
      Current driveStatorCurrent,
      Current driveSupplyCurrent,
      Current driveSupplyLowerCurrent,
      Time driveSupplyLowerTime,
      Current steerStatorCurrent,
      LinearVelocity maxLinearSpeed,
      AngularVelocity maxAngularSpeed) {
    public Limits {
      Checks.on(Limits.class)
          .field("driveStatorCurrent", driveStatorCurrent, positive())
          .field("driveSupplyCurrent", driveSupplyCurrent, positive())
          .field("driveSupplyLowerCurrent", driveSupplyLowerCurrent, positive())
          // Phoenix reads a zero time as "never step down": to mean that, set the lower current to
          // the supply limit instead, where it shows. 5 s is the longest Phoenix takes.
          .field("driveSupplyLowerTime", driveSupplyLowerTime, positive())
          .field("driveSupplyLowerTime", driveSupplyLowerTime, atMost(Seconds.of(5)))
          .field("steerStatorCurrent", steerStatorCurrent, positive())
          .field("maxLinearSpeed", maxLinearSpeed, positive())
          .field("maxAngularSpeed", maxAngularSpeed, positive())
          .orThrow();
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
   * <p>All are tunable on a running robot (the drivetrain says within what): dial them in on the
   * robot, then commit what works.
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
      double driveKs,
      double driveKv,
      double driveKp,
      double steerKp,
      double steerKd,
      Time skewLookahead) {
    public Gains {
      Checks.on(Gains.class)
          .field("driveKs", driveKs, nonNegative())
          .field("driveKv", driveKv, nonNegative())
          .field("driveKp", driveKp, nonNegative())
          .field("steerKp", steerKp, nonNegative())
          .field("steerKd", steerKd, nonNegative())
          .field("skewLookahead", skewLookahead, nonNegative())
          .orThrow();
    }

    /** Starts building the gains, one named value at a time. */
    public static SwerveConfigGainsBuilder.DriveKsStage builder() {
      return SwerveConfigGainsBuilder.builder();
    }
  }

  /**
   * Why the wheels don't fit this body's frame, or null when they do: the inset must leave room
   * between them.
   */
  public @Nullable String whyWheelsDontFit(Body body) {
    return wheelInset.times(2).lt(body.frameLength()) && wheelInset.times(2).lt(body.frameWidth())
        ? null
        : shown(wheelInset) + " puts the wheels at or past the center of the frame";
  }

  /**
   * Where each wheel touches the floor, from the robot's center: front left, front right, back
   * left, back right.
   *
   * @throws IllegalArgumentException if the inset leaves no room between the wheels
   */
  public Translation2d[] moduleTranslations(Body body) {
    String doesntFit = whyWheelsDontFit(body);
    if (doesntFit != null) {
      throw new IllegalArgumentException("wheelInset " + doesntFit);
    }
    double x = body.frameLength().in(Meters) / 2 - wheelInset.in(Meters);
    double y = body.frameWidth().in(Meters) / 2 - wheelInset.in(Meters);
    return new Translation2d[] {
      new Translation2d(x, y),
      new Translation2d(x, -y),
      new Translation2d(-x, y),
      new Translation2d(-x, -y)
    };
  }
}
