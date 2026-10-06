package frc.robot.lib.phoenix;

import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Transmission;
import org.wpilib.units.measure.Distance;

/**
 * Phoenix's units for a joint, as its drive moves it, fixed here and nowhere else: what one of
 * Phoenix's "rotations" is, for a motor controller {@link PhoenixMechanism} configured.
 *
 * <ul>
 *   <li>A revolute joint's Phoenix position is the joint's own rotations: its {@code
 *       SensorToMechanismRatio} is the sensor's turns for each turn of the joint, so a position of
 *       0.25 is the joint a quarter turn from its zero.
 *   <li>A prismatic joint is in its drum's rotations: the drum, sprocket or pinion that slides it,
 *       a {@code Transmission.Drum}'s or its term's in a combined transmission, which names one for
 *       every prismatic joint. Its {@code SensorToMechanismRatio} is the gearing alone, as for a
 *       revolute joint, and well inside Phoenix's ±1000 however fine the drum. A rotation is the
 *       drum's circumference of travel, 2π times its pitch radius, and the conversion to and from
 *       meters is made here alone, so a mechanism's control stays in meters.
 * </ul>
 *
 * <p>Velocity, acceleration and jerk follow, per second, per second squared and per second cubed,
 * and so do the gains: kP in volts per rotation of error, kV per rotation per second, kA per
 * rotation per second squared. kS and kG are volts in every unit. The robot's definition and a
 * mechanism's control are in radians and meters ({@link MechanismControl}).
 */
public final class PhoenixUnits {
  private final Joint joint;
  private final double perPhoenixUnit;
  private final String name;
  private final String symbol;

  private PhoenixUnits(Joint joint, double perPhoenixUnit, String name, String symbol) {
    this.joint = joint;
    this.perPhoenixUnit = perPhoenixUnit;
    this.name = name;
    this.symbol = symbol;
  }

  /**
   * The units a joint's motor controller works in, as a drive moves it.
   *
   * @throws IllegalArgumentException for a prismatic joint the drive doesn't turn through a drum
   */
  public static PhoenixUnits of(Joint joint, Drive drive) {
    if (joint instanceof Joint.Revolute) {
      return new PhoenixUnits(joint, 2 * Math.PI, "rotations of " + joint.label(), "rot");
    }
    Distance pitchRadius = drum(joint, drive);
    return new PhoenixUnits(
        joint,
        2 * Math.PI * pitchRadius.in(Meters),
        "rotations of " + joint.label() + "'s drum",
        "drum rot");
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
   * How many of the joint's own units, radians or meters, are one Phoenix unit: 2π for a revolute
   * joint's rotation; a drum's circumference, in meters, for a drum's.
   */
  public double perPhoenixUnit() {
    return perPhoenixUnit;
  }

  /**
   * A position, a velocity, an acceleration or a jerk, in the joint's units, in Phoenix's: per
   * second as many times as it's given.
   */
  public double toPhoenix(double inJointUnits) {
    return inJointUnits / perPhoenixUnit;
  }

  /** A position, a velocity, an acceleration or a jerk in Phoenix's units, in the joint's. */
  public double fromPhoenix(double inPhoenixUnits) {
    return inPhoenixUnits * perPhoenixUnit;
  }

  /**
   * A gain per the joint's units (volts per radian or meter, per radian or meter per second, per
   * radian or meter per second squared) as Phoenix holds it, per Phoenix unit: kP, kI, kD, kV, kA.
   * Not kS or kG, which are volts.
   */
  public double gainToPhoenix(double perJointUnit) {
    return perJointUnit * perPhoenixUnit;
  }

  /** A gain as Phoenix holds it, per Phoenix unit, per the joint's units. */
  public double gainFromPhoenix(double perUnit) {
    return perUnit / perPhoenixUnit;
  }

  /**
   * A ratio as the definition writes it (a sensor's turns for each turn of a revolute joint, or
   * each meter of a prismatic one) as Phoenix's: the sensor's turns for each Phoenix unit, its
   * {@code SensorToMechanismRatio}.
   */
  public double ratioToPhoenix(double definitionRatio) {
    return joint instanceof Joint.Revolute ? definitionRatio : definitionRatio * perPhoenixUnit;
  }

  /** What one Phoenix unit is, for messages: "rotations of arm", "rotations of lift's drum". */
  public String name() {
    return name;
  }

  /** The unit's symbol, for messages: "rot", "drum rot". */
  public String symbol() {
    return symbol;
  }
}
