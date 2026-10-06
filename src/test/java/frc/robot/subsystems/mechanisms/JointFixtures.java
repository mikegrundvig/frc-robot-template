package frc.robot.subsystems.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import frc.robot.Robots;
import frc.robot.lib.phoenix.MechanismControl;
import frc.robot.lib.phoenix.MechanismControlBuilder;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * A robot with the joints the runtime's tests drive, on the template's: an intake's pivot with a
 * roller on its end, an elevator, and a turret that turns freely. Numbers are chosen to read, not
 * taken from a robot.
 */
public final class JointFixtures {
  private JointFixtures() {}

  /** Turning about this, a joint whose zero points forward raises it as its position grows. */
  static final Vector3 PITCH_UP = new Vector3(0, -1, 0);

  /**
   * The intake's pivot: 2 kg, 0.3 m out along its zero, which points forward, level; stowed at
   * 1.5°, its roller run from 110°, out at 130°, its stops at 0° and 135°.
   */
  public static final Joint.Revolute PIVOT =
      Joint.Revolute.builder()
          .label("pivot")
          .parent(Part.CHASSIS)
          .origin(new Transform3d(new Translation3d(0.2, 0, 0.3), Rotation3d.ZERO))
          .axis(PITCH_UP)
          .range(Optional.of(Range.degrees(0, 135)))
          .reference(Math.toRadians(1.5))
          .link(link("intake arm", 2, new Translation3d(0.3, 0, 0)))
          .positions(
              List.of(
                  Position.degrees("in", 1.5),
                  Position.degrees("run", 110),
                  Position.degrees("out", 130)))
          .outputs(List.of(Output.volts("push", 3)))
          .build();

  /** Its roller, through 3:1: intaking at 1500 rpm, outtaking at 6 V back, ejecting at half. */
  public static final Joint.Revolute ROLLER =
      Joint.Revolute.builder()
          .label("roller")
          .parent(PIVOT)
          .origin(new Transform3d(new Translation3d(0.35, 0, 0), Rotation3d.ZERO))
          .axis(PITCH_UP)
          .range(Optional.empty())
          .reference(0)
          .link(link("roller", 0.5, new Translation3d(0, 0, 0)))
          .positions(List.of())
          .outputs(
              List.of(
                  Output.rpm("intaking", 1500),
                  Output.volts("outtaking", -6),
                  Output.dutyCycle("ejecting", -0.5)))
          .build();

  /** An elevator, 0 to 1.2 m, carrying 5 kg: down, clear at 0.4 m, and at the top. */
  public static final Joint.Prismatic ELEVATOR =
      Joint.Prismatic.builder()
          .label("elevator")
          .parent(Part.CHASSIS)
          .origin(new Transform3d(new Translation3d(-0.2, 0, 0.1), Rotation3d.ZERO))
          .axis(Vector3.UP)
          .range(Optional.of(Range.meters(0, 1.2)))
          .reference(0)
          .link(link("carriage", 5, new Translation3d(0, 0, 0.1)))
          .positions(
              List.of(
                  Position.of("down", Inches.of(0)),
                  Position.of("clear", Inches.of(0.4 / 0.0254)),
                  Position.of("top", Inches.of(1.1 / 0.0254))))
          .outputs(List.of())
          .build();

  /** A turret that turns freely about the vertical, facing forward at 0°, back at 180°. */
  public static final Joint.Revolute TURRET =
      Joint.Revolute.builder()
          .label("turret")
          .parent(Part.CHASSIS)
          .origin(new Transform3d(new Translation3d(0, 0, 0.5), Rotation3d.ZERO))
          .axis(Vector3.UP)
          .range(Optional.empty())
          .reference(0)
          .link(link("turret", 3, new Translation3d(0, 0, 0)))
          .positions(List.of(Position.degrees("front", 0), Position.degrees("back", 180)))
          .outputs(List.of())
          .build();

  public static final Drive PIVOT_MOTOR = PIVOT.motors(Motor.KRAKEN_X60, 1, gearing(25));
  public static final Drive ROLLER_MOTOR = ROLLER.motors(Motor.KRAKEN_X60, 1, gearing(3));
  public static final Drive ELEVATOR_MOTORS =
      ELEVATOR.motors(Motor.KRAKEN_X60, 2, gearing(10), Inches.of(1));
  public static final Drive TURRET_MOTOR = TURRET.motors(Motor.KRAKEN_X60, 1, gearing(50));

  /** The intake out and rolling, and the elevator clear: three joints at once. */
  public static final Configuration INTAKING =
      Configuration.of("intaking", PIVOT.at("out"), ROLLER.at("intaking"), ELEVATOR.at("clear"));

  /** The template's robot with these joints. */
  public static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(Robots.SIM.definition())
          .name("joints")
          .joints(List.of(PIVOT, ROLLER, ELEVATOR, TURRET))
          .drives(List.of(PIVOT_MOTOR, ROLLER_MOTOR, ELEVATOR_MOTORS, TURRET_MOTOR))
          .configurations(List.of(INTAKING))
          .build();

  /**
   * Each joint as the tests drive it: gains that track, a profile, no soft limits, braking. None
   * has an absolute encoder, so each held at positions is stowed by hand, at its reference.
   */
  public static final List<JointSetup> SETUPS =
      List.of(
          setup(PIVOT, assumed(control(30, 1, profile(8, 30)))),
          setup(ROLLER, control(0.02, 0, Optional.empty())),
          setup(ELEVATOR, assumed(control(60, 2, profile(1.2, 5)))),
          setup(TURRET, assumed(control(20, 0.5, profile(3, 10)))));

  /** A control whose joint is where its definition rests it at power on. */
  public static MechanismControl assumed(MechanismControl control) {
    return homed(control, MechanismControl.Homing.assumeAtReference());
  }

  /** A control with a homing recipe. */
  public static MechanismControl homed(MechanismControl control, MechanismControl.Homing homing) {
    return MechanismControlBuilder.builder(control).homing(Optional.of(homing)).build();
  }

  /** A joint's setup, unwired: it runs in the simulator. */
  public static JointSetup setup(Joint joint, MechanismControl control) {
    return JointSetup.of(joint, control);
  }

  /** Feedback gains, the feedforward seeded from the definition, a profile, 40 A, braking. */
  public static MechanismControl control(
      double kP, double kD, Optional<MechanismControl.Profile> profile) {
    return control(kP, kD, Optional.empty(), profile, 40);
  }

  public static MechanismControl control(
      double kP,
      double kD,
      Optional<MechanismControl.SoftLimits> softLimits,
      Optional<MechanismControl.Profile> profile,
      double statorAmps) {
    return MechanismControl.builder()
        .gains(
            MechanismControl.Gains.builder()
                .kP(kP)
                .kI(0)
                .kD(kD)
                .kS(0)
                .kV(Optional.empty())
                .kA(Optional.empty())
                .kG(Optional.empty())
                .build())
        .softLimits(softLimits)
        .profile(profile)
        .currentLimits(
            MechanismControl.CurrentLimits.builder()
                .stator(Amps.of(statorAmps))
                .supply(Amps.of(60))
                .supplyLower(Amps.of(40))
                .supplyLowerTime(Seconds.of(1))
                .build())
        .neutralMode(MechanismControl.NeutralMode.BRAKE)
        .bands(Optional.empty())
        .homing(Optional.empty())
        .build();
  }

  public static Optional<MechanismControl.Profile> profile(double cruise, double acceleration) {
    return Optional.of(
        MechanismControl.Profile.builder()
            .cruiseVelocity(cruise)
            .acceleration(acceleration)
            .jerk(0)
            .build());
  }

  public static Optional<MechanismControl.SoftLimits> soft(double lower, double upper) {
    return Optional.of(MechanismControl.SoftLimits.builder().lower(lower).upper(upper).build());
  }

  public static Gearing gearing(double ratio) {
    return Gearing.builder().ratio(ratio).efficiency(1).build();
  }

  /**
   * A link of a mass centered at a point in its joint's frame, a 10 cm cube's inertia, no shape.
   */
  static Link link(String label, double kilograms, Translation3d cg) {
    return Link.builder()
        .label(label)
        .mass(
            MassProperties.builder()
                .mass(Kilograms.of(kilograms))
                .cg(cg)
                .inertia(Inertia.box(Kilograms.of(kilograms), new Translation3d(0.1, 0.1, 0.1)))
                .build())
        .shapes(List.of())
        .build();
  }
}
