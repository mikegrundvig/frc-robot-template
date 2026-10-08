package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.atMost;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;

/**
 * How the swerve drivetrain is driven: its current and speed limits, and its control gains, all
 * tunable on a running robot (the robot program's {@code Drive.tuning}). What the drivetrain is
 * (its modules, its motors, where its wheels sit) is its geometry, in the robot's definition; this
 * is what the pit dials in on top of it.
 *
 * @param limits current and speed limits
 * @param gains control gains
 */
@ConfigRecord
public record DrivetrainControl(Limits limits, Gains gains) {
  /** Starts building the drivetrain's control, one named value at a time. */
  public static DrivetrainControlBuilder.LimitsStage builder() {
    return DrivetrainControlBuilder.builder();
  }

  /**
   * Why the top speed asked for is past what this drivetrain can reach, or null when it isn't:
   * velocity control needs room under the free speed to correct with. The robot's definition checks
   * it as committed, and the drivetrain's tuning limits end at it as the pit tunes it.
   */
  public @Nullable String whyTooFastFor(DrivetrainGeometry drivetrain) {
    LinearVelocity freeSpeed = drivetrain.freeSpeed();
    return limits.maxLinearSpeed().lte(freeSpeed)
        ? null
        : shown(limits.maxLinearSpeed())
            + " is faster than the drivetrain's free speed of "
            + shown(freeSpeed)
            + ": give maxLinearSpeed at most that";
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
    public static DrivetrainControlLimitsBuilder.DriveStatorCurrentStage builder() {
      return DrivetrainControlLimitsBuilder.builder();
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
   * @param headingKp how hard a drive move turns the robot back onto its profile's heading: radians
   *     a second for each radian it's off
   */
  @ConfigRecord
  public record Gains(
      double driveKs,
      double driveKv,
      double driveKp,
      double steerKp,
      double steerKd,
      Time skewLookahead,
      double headingKp,
      double translationKp) {
    public Gains {
      Checks.on(Gains.class)
          .field("driveKs", driveKs, nonNegative())
          .field("driveKv", driveKv, nonNegative())
          .field("driveKp", driveKp, nonNegative())
          .field("steerKp", steerKp, nonNegative())
          .field("steerKd", steerKd, nonNegative())
          .field("skewLookahead", skewLookahead, nonNegative())
          .field("headingKp", headingKp, nonNegative())
          .field("translationKp", translationKp, nonNegative())
          .orThrow();
    }

    /** Starts building the gains, one named value at a time. */
    public static DrivetrainControlGainsBuilder.DriveKsStage builder() {
      return DrivetrainControlGainsBuilder.builder();
    }
  }

  /**
   * How fast the robot can speed up, and spin up, by its own model: each wheel pushes with the
   * lesser of its motors at the drive's stator current limit, through its gearing, and its tread's
   * grip on its share of the robot's weight; that, over the robot's mass, and its yaw inertia about
   * the wheels, is what the drivetrain can do. Drive moves are profiled under it.
   *
   * @param linear in meters a second squared
   * @param angular in radians a second squared
   */
  public record Accelerations(double linear, double angular) {}

  /** The robot's accelerations by its model, with this drivetrain's current limit. */
  public static Accelerations accelerations(RobotDefinition robot, DrivetrainControl control) {
    List<DrivetrainGeometry.Corner> corners = robot.drivetrain().corners();
    double mass = robot.body().mass().in(Kilograms);
    double amps = control.limits().driveStatorCurrent().in(Amps);
    double force = 0;
    double torque = 0;
    for (DrivetrainGeometry.Corner corner : corners) {
      DrivetrainGeometry.ModuleType module = corner.module();
      Actuator.Motors motors = module.driveMotors();
      double pushed =
          motors.count()
              * motors.motor().model().Kt
              * amps
              * module.driveGearing().ratio()
              * module.driveGearing().efficiency()
              / module.wheelRadius().in(Meters);
      double gripped =
          module.wheelCoefficientOfFrictionOrDefault() * mass * GRAVITY / corners.size();
      double wheel = Math.min(pushed, gripped);
      force += wheel;
      torque += wheel * corner.position().getNorm();
    }
    return new Accelerations(
        force / mass, torque / robot.body().yawMomentOfInertia().in(KilogramSquareMeters));
  }

  /** Standard gravity, in meters a second squared. */
  private static final double GRAVITY = 9.80665;
}
