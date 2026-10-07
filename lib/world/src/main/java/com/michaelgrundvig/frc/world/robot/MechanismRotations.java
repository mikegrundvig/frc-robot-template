package com.michaelgrundvig.frc.world.robot;

import static org.wpilib.units.Units.Meters;

import org.wpilib.units.measure.Distance;

/**
 * A joint's rotations, as a motor controller counts them, fixed here and nowhere else: what one of
 * a TalonFX's "mechanism rotations" is, and what a SPARK's rotor turns are counted in, through the
 * gearing.
 *
 * <ul>
 *   <li>A revolute joint's rotation is its own turn: a sensor's ratio is its turns for each turn of
 *       the joint, so a position of 0.25 is the joint a quarter turn from its zero.
 *   <li>A prismatic joint's rotation is its drum's: the drum, sprocket or pinion that slides it, a
 *       {@link Transmission.Drum}'s or its term's in a combined transmission, which names one for
 *       every prismatic joint. A sensor's ratio is the gearing alone, as for a revolute joint, and
 *       well inside Phoenix's ±1000 however fine the drum. A rotation is the drum's circumference
 *       of travel, 2π times its pitch radius, and the conversion to and from meters is made here
 *       alone, so a mechanism's control stays in meters.
 * </ul>
 *
 * <p>Velocity and acceleration follow, per second and per second squared, and so do the gains: kP
 * in volts per rotation of error, kV per rotation per second, kA per rotation per second squared.
 * kS and kG are volts in every unit. The robot's definition and a mechanism's control are in
 * radians and meters ({@link MechanismControl}).
 *
 * <p>A joint a drive slides other than through a drum, as a linear servo does, has no rotation, and
 * no motor controller can drive it.
 */
public final class MechanismRotations {
  private final Joint joint;
  private final double perRotation;

  private MechanismRotations(Joint joint, double perRotation) {
    this.joint = joint;
    this.perRotation = perRotation;
  }

  /**
   * A joint's rotations, as a drive moves it.
   *
   * @throws IllegalArgumentException for a prismatic joint the drive doesn't turn through a drum
   */
  public static MechanismRotations of(Joint joint, Drive drive) {
    if (joint instanceof Joint.Revolute) {
      return new MechanismRotations(joint, 2 * Math.PI);
    }
    return new MechanismRotations(joint, 2 * Math.PI * drum(joint, drive).in(Meters));
  }

  /**
   * The pitch radius of the drum, sprocket or pinion a drive slides a prismatic joint with: a
   * {@code Drum}'s, or its term's in a combined transmission, which names one for every prismatic
   * joint it moves.
   */
  private static Distance drum(Joint joint, Drive drive) {
    switch (drive.transmission()) {
      case Transmission.Drum drum when drum.joint().equals(joint) -> {
        return drum.pitchRadius();
      }
      case Transmission.Combined combined -> {
        for (Transmission.Term term : combined.terms()) {
          if (term.joint().equals(joint) && term.pitchRadius().isPresent()) {
            return term.pitchRadius().get();
          }
        }
      }
      default -> {}
    }
    throw new IllegalArgumentException(
        drive.label()
            + " doesn't slide "
            + joint.label()
            + " through a drum, a sprocket or a pinion");
  }

  /** The joint. */
  public Joint joint() {
    return joint;
  }

  /**
   * How many of the joint's own units, radians or meters, are one rotation: 2π for a revolute
   * joint's turn; a drum's circumference, in meters, for a drum's.
   */
  public double perRotation() {
    return perRotation;
  }

  /**
   * A position, a velocity or an acceleration, in the joint's units, in rotations: per second as
   * many times as it's given.
   */
  public double toRotations(double inJointUnits) {
    return inJointUnits / perRotation;
  }

  /** A position, a velocity or an acceleration in rotations, in the joint's units. */
  public double fromRotations(double rotations) {
    return rotations * perRotation;
  }

  /**
   * A gain per the joint's units (volts per radian or meter, per radian or meter per second, per
   * radian or meter per second squared) per rotation: kP, kI, kD, kV, kA. Not kS or kG, which are
   * volts.
   */
  public double gainPerRotation(double perJointUnit) {
    return perJointUnit * perRotation;
  }

  /** A gain per rotation, per the joint's units. */
  public double gainPerJointUnit(double perRotation) {
    return perRotation / this.perRotation;
  }

  /**
   * A ratio as the definition writes it (a sensor's turns for each turn of a revolute joint, or
   * each meter of a prismatic one) per rotation: the sensor's turns for each rotation, a TalonFX's
   * {@code SensorToMechanismRatio}.
   */
  public double ratioPerRotation(double definitionRatio) {
    return joint instanceof Joint.Revolute ? definitionRatio : definitionRatio * perRotation;
  }
}
