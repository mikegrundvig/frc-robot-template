package frc.robot.robots2026;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.runtime.lib.phoenix.PhoenixMechanism;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.ControllerPlan;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismRotations;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * What the 2026 robots' Phoenix tests are built from: wiring and control with the defaults these
 * tests share, and a team's numbers in another frame turned into the control's. Nothing here is a
 * fact about a robot.
 *
 * <p>A team's Phoenix unit is often not the helper's ({@link MechanismRotations}: a joint's
 * rotations, a drum's, or meters): its rotor's turns when it sets no ratio, degrees, inches. {@code
 * theirsPerOurs} is how many of the team's units make one of the helper's: the gearing for a
 * rotor's turns, 360 for degrees.
 */
final class PhoenixParts {
  private PhoenixParts() {}

  /** A TalonFX on the robot's CANivore, turning clockwise to move its joint forward, or not. */
  static MechanismWiring.Device motor(int id, boolean clockwise) {
    MechanismWiring.Device talon =
        MechanismWiring.Controller.TALON_FX.can(CanBus.canivore("canivore"), id);
    return clockwise ? talon.clockwise() : talon;
  }

  /** The drive's motors, IDs from 1, each turning counterclockwise to move the joint forward. */
  static List<MechanismWiring.Device> wiring(Drive drive) {
    int count = ((Actuator.Motors) drive.actuator()).count();
    List<MechanismWiring.Device> motors = new ArrayList<>();
    for (int id = 1; id <= count; id++) {
      motors.add(motor(id, false));
    }
    return motors;
  }

  static List<MechanismWiring.Device> wiring(MechanismWiring.Device... motors) {
    return List.of(motors);
  }

  /** The helper's units for a joint, as a drive moves it: its rotations. */
  static MechanismRotations units(Joint joint, Drive drive) {
    return MechanismRotations.of(joint, drive);
  }

  /**
   * A team's gains, as its slot holds them per its own unit, in the control's units; NaN for one it
   * leaves to the seed.
   */
  static MechanismControl.Gains gains(
      MechanismRotations units, double kP, double kD, double kV, double kG, double theirsPerOurs) {
    return MechanismControl.Gains.builder()
        .kP(gain(units, kP, theirsPerOurs))
        .kI(0)
        .kD(gain(units, kD, theirsPerOurs))
        .kS(0)
        .kV(Double.isNaN(kV) ? Optional.empty() : Optional.of(gain(units, kV, theirsPerOurs)))
        .kG(Double.isNaN(kG) ? Optional.empty() : Optional.of(kG))
        .build();
  }

  /**
   * A gain per a team's unit (per rotor turn, per degree, per inch) as the control takes it, per
   * radian or meter.
   */
  static double gain(MechanismRotations units, double perTheirUnit, double theirsPerOurs) {
    return units.gainPerJointUnit(perTheirUnit * theirsPerOurs);
  }

  /** A position, a speed or an acceleration in a team's units as the control takes it. */
  static double position(MechanismRotations units, double theirs, double theirsPerOurs) {
    return units.fromRotations(theirs / theirsPerOurs);
  }

  static Optional<MechanismControl.SoftLimits> soft(double lower, double upper) {
    return Optional.of(MechanismControl.SoftLimits.builder().lower(lower).upper(upper).build());
  }

  static Optional<MechanismControl.Profile> profile(double cruise, double acceleration) {
    return Optional.of(
        MechanismControl.Profile.builder()
            .cruiseVelocity(cruise)
            .acceleration(acceleration)
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
        .loop(MechanismControl.Loop.POSITION)
        .gains(gains)
        .currentLimits(
            MechanismControl.CurrentLimits.builder()
                .stator(Amps.of(40))
                .supply(Amps.of(40))
                .supplyLower(Amps.of(40))
                .supplyLowerTime(Seconds.of(1))
                .build())
        .neutralMode(MechanismControl.NeutralMode.BRAKE)
        .deadChannel(MechanismControl.DeadChannel.DEFAULTS)
        .softLimits(softLimits)
        .profile(profile)
        .build();
  }

  /** The helper's settings for a drive's motors, rotor alone, seeded. */
  static PhoenixMechanism.Settings seeded(RobotMath math, Joint joint, Drive drive) {
    return configure(math, joint, drive, wiring(drive), control());
  }

  /** The helper's settings for a mechanism's parts, through the plan the program works out. */
  static PhoenixMechanism.Settings configure(
      RobotMath math,
      Joint joint,
      Drive drive,
      List<MechanismWiring.Device> controllers,
      MechanismControl control) {
    return PhoenixMechanism.configure(ControllerPlan.of(math, joint, drive, controllers, control));
  }

  /** The same, with a CANcoder. */
  static PhoenixMechanism.Settings configure(
      RobotMath math,
      Joint joint,
      Drive drive,
      Sensor.Encoder encoder,
      MechanismWiring.Cancoder cancoder,
      List<MechanismWiring.Device> controllers,
      MechanismControl control) {
    return PhoenixMechanism.configure(
        math, ControllerPlan.of(math, joint, drive, controllers, control), encoder, cancoder);
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
