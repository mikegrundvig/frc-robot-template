package frc.robot.examples;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANRANGE;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CameraConfigBuilder;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Driven.AbsoluteEncoder;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Pdh;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.RobotConfig;
import frc.robot.RobotConfigBuilder;
import frc.robot.Robots;
import java.util.List;
import org.wpilib.math.geometry.Translation3d;

/**
 * The robot docs/mechanisms.md describes, as Robots.java would hold it: an intake (a pivot that
 * swings out, with a roller on its end), a hood, an elevator with an arm on it, and a shooter fed
 * by a feeder and an indexer. The guide's blocks, and Robots.java's example, are this code
 * (DocExamplesTest), and ExampleRobotSimTest builds it as the robot does, so a robot the guide
 * describes is one the robot would start.
 */
final class ExampleRobot {
  private ExampleRobot() {}

  // @start region=intake
  static final Position PIVOT_IN = Position.degrees("in", 1.5);
  static final Position PIVOT_OUT = Position.degrees("out", 130);

  static final Joint.Revolute PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
          .range(Range.degrees(0, 135))
          .rest(PIVOT_IN) // its hard stops, and where it rests
          .carries(
              Link.box(
                  "intake arm", new Translation3d(0, 0, 0.2), new Translation3d(0.05, 0.55, 0.36)))
          .positions(PIVOT_IN, PIVOT_OUT)
          .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(32).pdh(4, Amps.of(40)))
          .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))
          .control(
              MechanismControl.position(30)
                  .withProfile(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(1700)));

  static final Output INTAKING = Output.rpm("intaking", 2000);
  static final Output OUTTAKING = Output.volts("outtaking", -4);

  static final Joint.Revolute ROLLER =
      Joint.roller("roller", PIVOT, new Translation3d(0, 0, 0.35)) // it rides on the pivot's end
          .outputs(INTAKING, OUTTAKING)
          .motor(Motor.KRAKEN_X60, Gearing.of(3), TALON_FX.can(31).pdh(5, Amps.of(40)))
          // @start region=runs-only-when
          // The roller runs only once the pivot is out of the frame.
          .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))
          // @end region=runs-only-when
          .control(MechanismControl.velocity(0.02));

  static final Configuration INTAKE =
      Configuration.of("intake", PIVOT.at(PIVOT_OUT), ROLLER.at(INTAKING));

  // @end region=intake

  /** The claw's beam break, on the pivot's arm, and a rangefinder looking into the hopper. */
  static final Sensor.Switch CLAW_BEAM =
      Sensor.Switch.beam("claw beam", PIVOT).on(CANDI.can(30).s1().normallyOpen());

  static final Sensor.Rangefinder HOPPER =
      Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27))
          .on(CANRANGE.can(31));

  static final Position HOOD_DOWN = Position.degrees("down", 0);

  /** A hood zeroed level, raised as its angle grows: its kG holds it, along one cosine. */
  static final Joint.Revolute HOOD =
      Joint.pivot("hood", Part.CHASSIS, new Translation3d(-0.2, 0, 0.55), Joint.PITCH_UP)
          .range(Range.degrees(0, 40))
          .rest(HOOD_DOWN)
          .positions(HOOD_DOWN)
          .motor(Motor.KRAKEN_X44, Gearing.of(50), TALON_FX.can(40).pdh(6, Amps.of(30)))
          .absoluteEncoder(CANCODER.can(41).offset(Rotations.of(-0.3)))
          .control(MechanismControl.position(30).withKg(0.35));

  static final Position ELEVATOR_DOWN = Position.of("down", Inches.of(0));
  static final Position ELEVATOR_CLEAR = Position.of("clear", Inches.of(32));

  /**
   * An elevator whose CANcoder is geared to its drum, so it knows where it is at startup: its 1.2 m
   * of travel turn its 0.75 in drum ten times, and the CANcoder less than once.
   */
  static final Joint.Prismatic ELEVATOR =
      Joint.elevator("elevator", Part.CHASSIS, new Translation3d(0, 0, 0.1))
          .range(Range.meters(0, 1.2))
          .rest(ELEVATOR_DOWN)
          .positions(ELEVATOR_DOWN, ELEVATOR_CLEAR)
          .motors(
              Motor.KRAKEN_X60,
              Gearing.of(5),
              Inches.of(0.75),
              TALON_FX.can(20).pdh(0, Amps.of(40)),
              TALON_FX.can(21).clockwise().pdh(1, Amps.of(40)))
          // Its CANcoder turns once for twelve turns of the drum: less than once over its travel.
          .absoluteEncoder(
              AbsoluteEncoder.of().ratio(1.0 / 12).on(CANCODER.can(24).offset(Rotations.of(0.31))))
          .control(MechanismControl.position(30).withKg(0.35));

  static final Position ARM_STOWED = Position.degrees("stowed", -45);
  static final Position ARM_SCORING = Position.degrees("scoring", 120);

  /**
   * An arm on the elevator, standing straight up at its zero and swinging back as its angle grows.
   */
  static final Joint.Revolute ARM =
      Joint.pivot("arm", ELEVATOR, new Translation3d(0.1, 0, 0.3), Joint.PITCH_UP)
          .range(Range.degrees(-60, 150))
          .rest(ARM_STOWED)
          .positions(ARM_STOWED, ARM_SCORING)
          .motor(Motor.KRAKEN_X60, Gearing.of(60), TALON_FX.can(22).pdh(2, Amps.of(40)))
          .absoluteEncoder(CANCODER.can(23).offset(Rotations.of(0.12)))
          // @start region=keeps-out
          // The arm keeps out of the bumper's band while the elevator is too low to clear it.
          .keepsOut(Range.degrees(-10, 30))
          .when(ELEVATOR.atMost(Inches.of(32)))
          // @end
          .control(MechanismControl.position(30).withKg(0.35, Degrees.of(-90)));

  static final Output FEEDER_FEEDING = Output.rpm("feeding", 1000);
  static final Output INDEXER_FEEDING = Output.rpm("feeding", 1500);

  static final Joint.Revolute FEEDER =
      Joint.roller("feeder", Part.CHASSIS, new Translation3d(-0.1, 0, 0.35))
          .outputs(FEEDER_FEEDING)
          .motor(Motor.KRAKEN_X60, Gearing.of(4), TALON_FX.can(50).pdh(7, Amps.of(40)))
          .control(MechanismControl.velocity(0.02));

  static final Joint.Revolute INDEXER =
      Joint.roller("indexer", Part.CHASSIS, new Translation3d(0.05, 0, 0.15))
          .outputs(INDEXER_FEEDING)
          .motor(Motor.KRAKEN_X60, Gearing.of(3), TALON_FX.can(51).pdh(8, Amps.of(40)))
          .control(MechanismControl.velocity(0.02));

  /** A flywheel on two motors, the second turned the other way: it coasts when told nothing. */
  static final Joint.Revolute SHOOTER =
      Joint.roller("shooter", Part.CHASSIS, new Translation3d(-0.2, 0, 0.45))
          .motors(
              Motor.KRAKEN_X60,
              Gearing.of(1),
              TALON_FX.can(52).pdh(9, Amps.of(40)),
              TALON_FX.can(53).clockwise().pdh(10, Amps.of(40)))
          .control(MechanismControl.velocity(0.02).coasting());

  // @start region=configurations
  static final Configuration FEEDING =
      Configuration.of("feeding", FEEDER.at(FEEDER_FEEDING), INDEXER.at(INDEXER_FEEDING));

  static final Configuration STOWED =
      Configuration.of(
          "stowed", PIVOT.at(PIVOT_IN), ELEVATOR.at(ELEVATOR_DOWN), ARM.at(ARM_STOWED));

  // @end

  /** The template's robot's back-left camera, as Robots.java mounts it, 21 in up. */
  static final CameraConfig BACK_LEFT = Robots.SIM.definition().cameras().get(2);

  /** The competition robot: the template's, with these mechanisms, each motor fed by the PDH. */
  static final RobotConfig COMPETITION =
      RobotConfigBuilder.builder(Robots.SIM)
          .definition(
              RobotDefinitionBuilder.builder(Robots.SIM.definition())
                  .name("example")
                  .pdh(Pdh.can(1))
                  .joints(List.of(PIVOT, ROLLER, HOOD, ELEVATOR, ARM, FEEDER, INDEXER, SHOOTER))
                  .sensors(List.of(CLAW_BEAM, HOPPER))
                  .configurations(List.of(FEEDING, STOWED, INTAKE))
                  .build())
          .build();

  /**
   * A practice robot of the same design: its pivot's motor on another CAN ID and its pivot out at
   * 128°, its arm's CANcoder offset for its own magnet, its arm lighter and stowed at -30°, its
   * claw's beam on another CANdi, and its back-left camera half an inch lower. Each keeps the rest
   * of what it changes, so a regear of the competition robot's pivot reaches it, and the same code
   * sends its pivot to its own out.
   */
  // @start region=practice
  static final RobotConfig PRACTICE =
      COMPETITION
          .named("practice")
          .with(
              PIVOT
                  .motorsOn(TALON_FX.can(33).pdh(4, Amps.of(40))) // another CAN ID
                  .replaced(PIVOT_OUT, Degrees.of(128)), // it swings out to 128°
              ARM.encoderOffset(Rotations.of(0.47)) // its magnet's own offset
                  .retuned(control -> control.withKg(0.3)) // a lighter arm
                  .rest(Degrees.of(-30))) // it stows at -30°
          .with(CLAW_BEAM.on(CANDI.can(36).s1().normallyOpen())) // a beam on another CANdi
          .with(CameraConfigBuilder.builder(BACK_LEFT).up(Inches.of(20.5)).build()); // lower

  /** Every robot of the design, as Robots.all() lists them. */
  static List<RobotConfig> all() {
    return List.of(COMPETITION, PRACTICE);
  }
  // @end
}
