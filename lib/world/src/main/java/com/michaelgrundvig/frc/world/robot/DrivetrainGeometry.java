package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.MomentOfInertia;

/**
 * A swerve drivetrain, as built: four identical modules, one near each corner of the frame. What it
 * is, not how it's driven: its limits and gains are the drivetrain's own configuration, tuned while
 * the robot runs, and this is fixed until the robot is rebuilt.
 *
 * <p>Robot frame (WPILib's): +x forward, +y left, counterclockwise positive. Module order is always
 * front left, front right, back left, back right.
 *
 * @param modules the module hardware, the same at every corner
 * @param wheelInset how far each wheel's center sits in from the outside of the frame, in both
 *     directions. Corner-mounted modules give it in their footprint drawings.
 */
@ConfigRecord
public record DrivetrainGeometry(Modules modules, Distance wheelInset) {
  public DrivetrainGeometry {
    Checks.on(DrivetrainGeometry.class).field("wheelInset", wheelInset, positive()).orThrow();
  }

  /** Starts describing a swerve drivetrain, one named value at a time. */
  public static DrivetrainGeometryBuilder.ModulesStage builder() {
    return DrivetrainGeometryBuilder.builder();
  }

  /**
   * The module hardware. Wheel radius shrinks as the tread wears; measure it again during the
   * season.
   *
   * @param driveMotor the motor that drives the wheel
   * @param steerMotor the motor that turns the module
   * @param wheelRadius measured on the tread
   * @param driveGearing between the drive motor and the wheel: motor rotations per wheel rotation
   * @param steerGearing between the steering motor and the module: motor rotations per full turn of
   *     the module
   * @param wheelCoefficientOfFriction the tread's grip on carpet: about 1.0 when worn, 1.3 new
   * @param driveInertia the drive's own moving parts (the wheel with its tread, its gears, and the
   *     motor's rotor through them), as inertia at the wheel. The robot's mass the wheel moves
   *     comes on top of it, from the body.
   * @param steerInertia the module turning about its steering axis, with the steering motor's rotor
   *     through its gearing: what the steering motor turns
   */
  @ConfigRecord
  public record Modules(
      Motor driveMotor,
      Motor steerMotor,
      Distance wheelRadius,
      Gearing driveGearing,
      Gearing steerGearing,
      double wheelCoefficientOfFriction,
      MomentOfInertia driveInertia,
      MomentOfInertia steerInertia) {
    public Modules {
      Checks.on(Modules.class)
          .field("wheelRadius", wheelRadius, positive())
          .field("wheelCoefficientOfFriction", wheelCoefficientOfFriction, positive())
          .field("driveInertia", driveInertia, positive())
          .field("steerInertia", steerInertia, positive())
          .orThrow();
    }

    /** Starts describing the module hardware, one named value at a time. */
    public static DrivetrainGeometryModulesBuilder.DriveMotorStage builder() {
      return DrivetrainGeometryModulesBuilder.builder();
    }

    /** What drives the wheel: one drive motor, through the drive's gearing. */
    public Actuator.Motors drive() {
      return Actuator.Motors.builder().motor(driveMotor).count(1).gearing(driveGearing).build();
    }

    /** What turns the module: one steering motor, through the steering's gearing. */
    public Actuator.Motors steer() {
      return Actuator.Motors.builder().motor(steerMotor).count(1).gearing(steerGearing).build();
    }

    /** Top wheel speed at the motor's nominal voltage, with no load. The real robot is slower. */
    public LinearVelocity freeSpeed() {
      return MetersPerSecond.of(
          driveMotor.model().freeSpeed / driveGearing.ratio() * wheelRadius.in(Meters));
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
