package frc.robot.robots2026;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.lib.phoenix.PhoenixMechanism;
import frc.robot.lib.phoenix.PhoenixUnits;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * What the 2026 robots' Phoenix tests are built from: wiring and control with the defaults these
 * tests share, and a team's numbers in another frame turned into the control's. Nothing here is a
 * fact about a robot.
 *
 * <p>A team's Phoenix unit is often not the helper's ({@link PhoenixUnits}: a joint's rotations, a
 * drum's, or meters): its rotor's turns when it sets no ratio, degrees, inches. {@code
 * theirsPerOurs} is how many of the team's units make one of the helper's: the gearing for a
 * rotor's turns, 360 for degrees.
 */
final class PhoenixParts {
  private PhoenixParts() {}

  static MechanismWiring.Motor motor(int id, boolean clockwise) {
    return MechanismWiring.Motor.builder().id(id).clockwise(clockwise).build();
  }

  /** The drive's motors, IDs from 1, each turning counterclockwise to move the joint forward. */
  static MechanismWiring wiring(Drive drive) {
    int count = ((Actuator.Motors) drive.actuator()).count();
    List<MechanismWiring.Motor> motors = new ArrayList<>();
    for (int id = 1; id <= count; id++) {
      motors.add(motor(id, false));
    }
    return wiring(Optional.empty(), motors);
  }

  static MechanismWiring wiring(MechanismWiring.Motor... motors) {
    return wiring(Optional.empty(), List.of(motors));
  }

  static MechanismWiring wiring(
      MechanismWiring.Cancoder cancoder, MechanismWiring.Motor... motors) {
    return wiring(Optional.of(cancoder), List.of(motors));
  }

  private static MechanismWiring wiring(
      Optional<MechanismWiring.Cancoder> cancoder, List<MechanismWiring.Motor> motors) {
    return MechanismWiring.builder()
        .canBus(CanBus.canivore("canivore"))
        .controller(MechanismWiring.Controller.TALON_FX)
        .motors(motors)
        .cancoder(cancoder)
        .dataPort(Optional.empty())
        .build();
  }

  static MechanismWiring.Cancoder cancoder(
      int id, double offsetRotations, boolean clockwise, MechanismWiring.Feedback feedback) {
    return MechanismWiring.Cancoder.builder()
        .id(id)
        .offset(Rotations.of(offsetRotations))
        .clockwise(clockwise)
        .feedback(feedback)
        .build();
  }

  /** The helper's units for a joint, as a drive moves it. */
  static PhoenixUnits units(Joint joint, Drive drive) {
    return PhoenixUnits.of(joint, drive);
  }

  /**
   * A team's gains, as its slot holds them per its own unit, in the control's units; NaN for one it
   * leaves to the seed.
   */
  static MechanismControl.Gains gains(
      PhoenixUnits units, double kP, double kD, double kV, double kG, double theirsPerOurs) {
    return MechanismControl.Gains.builder()
        .kP(gain(units, kP, theirsPerOurs))
        .kI(0)
        .kD(gain(units, kD, theirsPerOurs))
        .kS(0)
        .kV(Double.isNaN(kV) ? Optional.empty() : Optional.of(gain(units, kV, theirsPerOurs)))
        .kA(Optional.empty())
        .kG(Double.isNaN(kG) ? Optional.empty() : Optional.of(kG))
        .gravityAngle(Optional.empty())
        .build();
  }

  /**
   * A gain per a team's unit (per rotor turn, per degree, per inch) as the control takes it, per
   * radian or meter.
   */
  static double gain(PhoenixUnits units, double perTheirUnit, double theirsPerOurs) {
    return units.gainFromPhoenix(perTheirUnit * theirsPerOurs);
  }

  /** A position, a speed or an acceleration in a team's units as the control takes it. */
  static double position(PhoenixUnits units, double theirs, double theirsPerOurs) {
    return units.fromPhoenix(theirs / theirsPerOurs);
  }

  static Optional<MechanismControl.SoftLimits> soft(double lower, double upper) {
    return Optional.of(MechanismControl.SoftLimits.builder().lower(lower).upper(upper).build());
  }

  static Optional<MechanismControl.Profile> profile(double cruise, double acceleration) {
    return Optional.of(
        MechanismControl.Profile.builder()
            .cruiseVelocity(cruise)
            .acceleration(acceleration)
            .jerk(0)
            .build());
  }

  /** The definition's seeds, no feedback, no limits or profile, braking. */
  static MechanismControl control() {
    return control(MechanismControl.Gains.seeded(), Optional.empty(), Optional.empty());
  }

  static MechanismControl control(MechanismControl.Gains gains) {
    return control(gains, Optional.empty(), Optional.empty());
  }

  static MechanismControl control(
      MechanismControl.Gains gains,
      Optional<MechanismControl.SoftLimits> softLimits,
      Optional<MechanismControl.Profile> profile) {
    return MechanismControl.builder()
        .gains(gains)
        .softLimits(softLimits)
        .profile(profile)
        .currentLimits(
            MechanismControl.CurrentLimits.builder()
                .stator(Amps.of(40))
                .supply(Amps.of(40))
                .supplyLower(Amps.of(40))
                .supplyLowerTime(Seconds.of(1))
                .build())
        .neutralMode(MechanismControl.NeutralMode.BRAKE)
        .bands(Optional.empty())
        .homing(Optional.empty())
        .build();
  }

  /** The helper's settings for a drive's motors, rotor alone, seeded. */
  static PhoenixMechanism.Settings seeded(RobotMath math, Joint joint, Drive drive) {
    return PhoenixMechanism.configure(math, joint, drive, wiring(drive), control());
  }

  /** A swerve corner's mechanisms, its CANcoder on its steering's shaft. */
  static RobotMath corner(
      Body body, DrivetrainGeometry.ModuleJoints corner, Sensor.Encoder cancoder) {
    return RobotMath.of(
        body,
        List.of(corner.steer(), corner.wheel()),
        List.of(),
        List.of(corner.drive(), corner.steering()),
        List.of(cancoder));
  }
}
