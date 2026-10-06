package frc.robot.robots2026;

import static frc.robot.robots2026.Parts.IN;
import static frc.robot.robots2026.Parts.body;
import static frc.robot.robots2026.Parts.camera;
import static frc.robot.robots2026.Parts.drive;
import static frc.robot.robots2026.Parts.inches;
import static frc.robot.robots2026.Parts.point;
import static frc.robot.robots2026.Parts.sensor;
import static frc.robot.robots2026.Parts.yaw;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.Body.Weight;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometryModuleTypeBuilder;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.Interlock;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Transmission;
import frc.robot.RobotConfig;
import frc.robot.lib.phoenix.MechanismControl;
import frc.robot.subsystems.drive.DrivetrainControl;
import frc.robot.subsystems.mechanisms.JointSetup;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Translation3d;

/**
 * 2611's 2026 competition robot, as its code on main configures it: {@code
 * ~/src/2611-Vectors/2026-Beta-Bot} at 6a5da76. Paths are under {@code src/main/java/frc/robot/}
 * unless they say otherwise. That code carries the AdvantageKit license ({@code
 * AdvantageKit-License.md}) and WPILib's ({@code WPILib-License.md}): its numbers are copied here
 * with citations, never its code.
 *
 * <p>"Guess:" marks a number the code doesn't hold, with its basis; guesses validate nothing. No
 * link's mass is guessed: the code gives none, so each is none, and the missing-data report lists
 * them. "Model:" marks what the model couldn't say, and the approximation written instead.
 *
 * <p>The hood is written but not built on this commit (RobotContainer.java:88 is commented out):
 * it's here as the code describes it, its geometry guessed.
 */
final class Team2611 {
  private Team2611() {}

  /**
   * Guess: a 26 in square frame, PathPlanner's 0.81 m robot width less 3 in bumpers
   * (src/main/deploy/pathplanner/settings.json:2), around the modules' 20.75 in track (below).
   * Guess: 3 in bumpers, from the same width; the bumper zone for their heights, 30 in tall, and
   * the center of mass centered 0.2 m up.
   *
   * <p>137.95 lb (Constants.java:187; PathPlanner's 62.57 kg, settings.json:15), taken as weighed
   * with the battery (Guess: the code doesn't say; without it, the robot would be over R408). Its
   * yaw inertia, 5.5 kg m² (Constants.java:188), which Drive.java:65-69 hands PathPlanner; the
   * PathPlanner app's own file holds 6.883 (settings.json:16), which the code never reads. The
   * model's box gives its pitch and roll.
   */
  static final Body BODY =
      body(
          26,
          26,
          30,
          3,
          3,
          2.5,
          5.75,
          Weight.weighed(Pounds.of(137.95)),
          point(0, 0, 0.2),
          yaw(5.5));

  /**
   * SDS modules, R2 (tuner-swerve-project.json: SwerveModuleConfiguration): 6.027:1 drive and
   * 26.09:1 steer, coupling 3.857 (generated/TunerConstants.java:80-83), wheels 1.93 in
   * (TunerConstants.java:84; Tuner's own file says 2.0). Kraken X60 drive with FOC (Tuner's 96.7
   * rps free speed, and Drive.java:74's PathPlanner motor), Kraken X44 steer (Tuner's file).
   * Tuner's simulation inertias, 0.01 kg m² each (TunerConstants.java:92-93), and the wheel's grip
   * as PathPlanner's 2.255 (Constants.java:189).
   */
  static final DrivetrainGeometry.ModuleType MODULES =
      DrivetrainGeometry.ModuleType.builder()
          .driveMotor(Motor.KRAKEN_X60_FOC)
          .steerMotor(Motor.KRAKEN_X44)
          .wheelRadius(Inches.of(1.93))
          .driveGearing(Gearing.of(6.026785714285714))
          .steerGearing(Gearing.of(26.09090909090909))
          .wheelCoefficientOfFriction(2.255)
          .driveInertia(KilogramSquareMeters.of(0.01))
          .steerInertia(KilogramSquareMeters.of(0.01))
          .couplingRatio(3.857142857142857)
          .build();

  /**
   * The modules at ±10.375 in (generated/TunerConstants.java:135-169; Tuner's swerve project,
   * tuner-swerve-project.json: VerticalTrackSizeInches 20.75).
   */
  static final DrivetrainGeometry DRIVETRAIN =
      DrivetrainGeometry.uniform(MODULES, Inches.of(10.375), Inches.of(10.375));

  /** Guess: its PhotonVision cameras' sensors, the template's typical AprilTag camera. */
  private static final CameraConfig.Sensor CAMERA =
      sensor(1280, 800, 80, CameraConfig.Across.DIAGONAL);

  /**
   * "Camera6", the left rear (Constants.java:119-126): WPILib's Rotation3d(-2°, -10°, 180.985°),
   * pitch down positive, so tilted up 10°.
   */
  static final CameraConfig LEFT_REAR =
      camera("Camera6", inches(-10.5, 12, 21), -2, 10, 180.985, CAMERA, Optional.empty());

  /** "FrontLeft" (Constants.java:129-136). */
  static final CameraConfig LEFT_FRONT =
      camera("FrontLeft", inches(-7.75, 12.25, 21), 0, 10, 23.162, CAMERA, Optional.empty());

  /**
   * "Camera4", the right rear (Constants.java:139-147): (-11.6875, -12, 16.837) in, turned (0°,
   * -12.5°, 180.985°), then {@code .plus(groundToRobotTopPlate)}, a 3.868 in lift
   * (Constants.java:112-116). Transform3d.plus applies that lift in the camera's own turned frame,
   * not the robot's, so the lens lands 0.84 in further forward and 0.09 in lower than a lift
   * straight up would put it. What their code uses is written here, worked by hand: the team's
   * quirk, which the test checks against WPILib's composition.
   */
  static final CameraConfig RIGHT_REAR =
      camera(
          "Camera4",
          inches(-10.8504, -11.9856, 20.6133),
          0,
          12.5,
          180.985,
          CAMERA,
          Optional.empty());

  /** "Camera3", the right front (Constants.java:150-157). */
  static final CameraConfig RIGHT_FRONT =
      camera("Camera3", inches(-8.625, -12.25, 20.875), 0, 8.2, -23.162, CAMERA, Optional.empty());

  /**
   * The intake's pivot (subsystems/Pivot.java): a Kraken X60 (Pivot.java:21) through 25:1
   * (Constants.java:85 writes it 1/25, output over motor, as Pivot.java:83 divides by it), read by
   * a duty-cycle absolute encoder (Pivot.java:23-24, DIO 1 at Constants.java:82; its offset, 296°,
   * is wiring). In at 1.5°, out at 130°, run from 110° (Constants.java:89-91).
   *
   * <p>Its named positions are its code's three angles: "in" to stow, "out" to intake, and "run",
   * from where its roller may run (PIVOT_RUN_ANGLE, Pivot.java:72-73's intakeCanRun).
   *
   * <p>Guess: the hard stops at 0° and 135° (the code holds only its setpoints), the pivot 0.25 m
   * forward and 0.32 m up, the arm straight up at 0° and swung forward and down as the angle grows
   * (Pivot.java:30-32 invert both the encoder and the motor, so the sign is a guess too), and the
   * arm drawn as a 0.36 m box out to the roller.
   */
  static final Joint.Revolute PIVOT =
      Joint.pivot(
          "pivot",
          Part.CHASSIS,
          point(0.25, 0, 0.32),
          Joint.PITCH_DOWN,
          Range.degrees(0, 135),
          Degrees.of(1.5),
          Link.box("intake arm", point(0, 0, 0.2), point(0.05, 0.55, 0.36)),
          Position.degrees("in", 1.5),
          Position.degrees("run", 110),
          Position.degrees("out", 130));

  /**
   * The intake's roller, on the arm's end: a Kraken X60 (subsystems/Intake.java:21) through 3:1
   * (Constants.java:86, 1/3, as Intake.java:43 divides by it), velocity-controlled in output rpm
   * (Intake.java:43). Its speeds are the operator's, "intaking" at 2000 rpm and "outtaking" at 500
   * back (Intake.java:51-53), the 2000 its auto's too (RobotContainer.java:143). Guess: its size.
   */
  static final Joint.Revolute INTAKE =
      Joint.roller(
          "intake",
          PIVOT,
          point(0, 0, 0.35),
          Link.box("intake roller", point(0, 0, 0), point(0.05, 0.5, 0.05)),
          Output.rpm("intaking", 2000),
          Output.rpm("outtaking", -500));

  /**
   * The shooter's wheels: four Kraken X60s (subsystems/Shooter.java:23-27), three following the
   * first (Shooter.java:34-36), 3 in wheels (Constants.java:62). Guess: 1:1, since Shooter.java:51
   * commands the leader's rotor as the shooter's speed with no ratio anywhere; and where it sits,
   * behind the center at the launch height, 18.33 in (Constants.java:65).
   */
  static final Joint.Revolute SHOOTER =
      Joint.roller(
          "shooter",
          Part.CHASSIS,
          point(-0.2, 0, 18.33 * IN),
          Link.box("shooter wheels", point(0, 0, 0), point(0.076, 0.5, 0.076)));

  /**
   * "Full send", the feeder into the shooter: a Kraken X60 (subsystems/FullSend.java:21) through
   * 4:1 (Constants.java:75 writes 1/4; FullSend.java:42 divides by it), in output rpm. Its speeds
   * are the driver's, "feeding" at 1000 rpm and "unjamming" at 1000 back (FullSend.java:49-51).
   * Their autos feed at 5000 (RobotContainer.java:146, AutoTarget.java:62, RobotTarget.java:36),
   * which its motor can't turn it at, so it isn't here: the definition refuses it
   * (Team2611Test.theAutosFeederSpeedIsRefused). Guess: where.
   */
  static final Joint.Revolute FULL_SEND =
      Joint.roller(
          "full send",
          Part.CHASSIS,
          point(-0.1, 0, 0.35),
          Link.box("full send rollers", point(0, 0, 0), point(0.05, 0.4, 0.05)),
          Output.rpm("feeding", 1000),
          Output.rpm("unjamming", -1000));

  /**
   * The transition's lower rollers, the only ones built (subsystems/Transition.java:21; the upper
   * pair is commented out, :23-24): a Kraken X60 through 2:1 (Constants.java:100, 1/2;
   * Transition.java:72), in output rpm. Its speeds are the driver's, "feeding" at 2000 rpm and
   * "unjamming" at 2000 back (Transition.java:79-81); its autos run it at 1000 and 2000
   * (RobotContainer.java:145, AutoTarget.java:63). Guess: where.
   */
  static final Joint.Revolute TRANSITION =
      Joint.roller(
          "transition",
          Part.CHASSIS,
          point(0, 0, 0.15),
          Link.box("transition rollers", point(0, 0, 0), point(0.05, 0.4, 0.05)),
          Output.rpm("feeding", 2000),
          Output.rpm("unjamming", -2000));

  /**
   * The hood (subsystems/Hood.java), turned by two linear servos, one each side, told the same
   * share of their travel (Hood.java:22-32). Guess: everything about where it sits: its pivot 0.1 m
   * above the shooter's wheels, raised as its angle grows, from 0° to 40°; its shape unknown. Its
   * servos are placed below so that the code's lowest command, 0.65, is the hood at 0°, and their
   * full stroke 40°.
   */
  static final Joint.Revolute HOOD =
      Joint.pivot(
          "hood",
          Part.CHASSIS,
          point(-0.2, 0, 18.33 * IN + 0.1),
          Joint.PITCH_UP,
          Range.degrees(0, 40),
          Degrees.of(0),
          Link.unseen("hood"));

  /**
   * The hood's servos, WCP-0408s (VectorKit/hardware/WCP_0408.java:11), Actuonix L16-100-35-6-R
   * linear servos: 168 mm eye to eye closed, a 100 mm stroke, 50 N at most and 32 mm/s free (WCP's
   * Miniature Linear Servo Actuators user guide: L16 35:1 table). WPILib's Servo bounds of 1000 to
   * 2000 µs (WCP_0408.java:15) are the guide's own, 1 ms fully retracted and 2 ms fully extended,
   * so {@code Servo.set}'s 0 to 1 is the model's command. On PWM 0 and 1 (Constants.java:56-57),
   * held to 0.65 to 1.0 (Constants.java:59-60), "0.65 is our current minimum" (Hood.java:24).
   */
  static final Actuator.Servo WCP_0408 =
      Actuator.Servo.builder().from(0.168).to(0.268).force(50).speed(0.032).build();

  /**
   * Guess: each servo's base on the chassis 0.233 m below its end on the hood, which is 54 mm out
   * along the hood's zero and 0.1 m to its side.
   */
  static final Drive HOOD_LEFT = hoodServo("hood left", 0.1);

  static final Drive HOOD_RIGHT = hoodServo("hood right", -0.1);

  static final Drive PIVOT_MOTOR = PIVOT.motors(Motor.KRAKEN_X60, 1, Gearing.of(25));

  // Velocity control without FOC (VectorKit/hardware/KrakenX60.java:48): the plain X60's numbers.
  static final Drive INTAKE_MOTOR = INTAKE.motors(Motor.KRAKEN_X60, 1, Gearing.of(3));
  static final Drive SHOOTER_MOTORS = SHOOTER.motors(Motor.KRAKEN_X60, 4, Gearing.of(1));
  static final Drive FULL_SEND_MOTOR = FULL_SEND.motors(Motor.KRAKEN_X60, 1, Gearing.of(4));
  static final Drive TRANSITION_MOTOR = TRANSITION.motors(Motor.KRAKEN_X60, 1, Gearing.of(2));

  /**
   * The pivot's absolute encoder, a duty-cycle encoder on DIO 1 (VectorKit/hardware/
   * AbsoluteEncoder.java, a WPILib DutyCycleEncoder). Guess: on the pivot's axle, 1:1, since
   * AbsoluteEncoder.get (VectorKit/hardware/AbsoluteEncoder.java:55-61) reads its turn straight as
   * the pivot's; and 1024 counts a turn, a REV through-bore's absolute output, the code naming no
   * part. Each motor's own rotor needs no writing: its drive implies it ({@code
   * PIVOT_MOTOR.rotor()}).
   */
  static final Sensor.Encoder PIVOT_ABSOLUTE =
      Sensor.Encoder.absolute("pivot absolute", PIVOT, 1024);

  /**
   * The driver's feed, both rollers at once, on the right trigger (RobotContainer.java:220-223):
   * the feed that gates on nothing, so the shooter may not be up to speed.
   */
  static final Configuration FEEDING =
      Configuration.of("feeding", FULL_SEND.at("feeding"), TRANSITION.at("feeding"));

  /** The same backward, on the right bumper (RobotContainer.java:225-228). */
  static final Configuration UNJAMMING =
      Configuration.of("unjamming", FULL_SEND.at("unjamming"), TRANSITION.at("unjamming"));

  /**
   * The intake's roller runs only with the pivot past "run": their auto's guard
   * (RobotContainer.java:143, Pivot.java:72-73), which the operator's buttons don't apply
   * (RobotContainer.java:216-218). As an interlock, it holds beneath every command.
   */
  static final Interlock INTAKE_ONLY_WHEN_OUT =
      Interlock.that(INTAKE).runsOnlyWhen(PIVOT.past("run"));

  /**
   * 2611's 2026 design: the joints its code commands and the configurations that name them, which
   * its behavior reads ({@code parts.pivot()}), never a label. One robot fills it here; a practice
   * robot would fill the same record with its own.
   *
   * @param pivot the intake's pivot
   * @param intake the intake's roller
   * @param shooter the shooter's wheels
   * @param fullSend the feeder into the shooter
   * @param transition the transition's rollers
   * @param hood the shooter's hood, on its servos
   * @param feeding both feed rollers forward
   * @param unjamming both feed rollers backward
   */
  @ConfigRecord
  record Parts(
      Joint.Revolute pivot,
      Joint.Revolute intake,
      Joint.Revolute shooter,
      Joint.Revolute fullSend,
      Joint.Revolute transition,
      Joint.Revolute hood,
      Configuration feeding,
      Configuration unjamming) {}

  /** The robot's parts. */
  static final Parts PARTS =
      Team2611PartsBuilder.builder()
          .pivot(PIVOT)
          .intake(INTAKE)
          .shooter(SHOOTER)
          .fullSend(FULL_SEND)
          .transition(TRANSITION)
          .hood(HOOD)
          .feeding(FEEDING)
          .unjamming(UNJAMMING)
          .build();

  /**
   * The robot. Its one IMU is a Pigeon 2 (generated/TunerConstants.java:89), given no mount pose
   * (TunerConstants.java:67, null configs). Guess: at the center, level. No backup IMU: a 2026
   * roboRIO robot has no Systemcore's own.
   */
  static final RobotDefinition ROBOT =
      RobotDefinition.builder()
          .name("2611 2026")
          .body(BODY)
          .drivetrain(DRIVETRAIN)
          .cameras(List.of(LEFT_REAR, LEFT_FRONT, RIGHT_REAR, RIGHT_FRONT))
          .imus(Imus.builder().gyro(ImuMount.UNCHECKED).backup(Optional.empty()).build())
          .joints(List.of(PIVOT, INTAKE, SHOOTER, FULL_SEND, TRANSITION, HOOD))
          .couplings(List.of())
          .drives(
              List.of(
                  PIVOT_MOTOR,
                  INTAKE_MOTOR,
                  SHOOTER_MOTORS,
                  FULL_SEND_MOTOR,
                  TRANSITION_MOTOR,
                  HOOD_LEFT,
                  HOOD_RIGHT))
          .sensors(List.of(PIVOT_ABSOLUTE))
          .configurations(List.of(FEEDING, UNJAMMING))
          .interlocks(List.of(INTAKE_ONLY_WHEN_OUT))
          .build();

  /**
   * How the template drives the joints its motors move, chosen to track in the simulator: the
   * feedforward is the definition's seed, the pivot's profile 8 rad/s at 30 rad/s², and the pivot
   * homes by its absolute encoder. Their code's own gains aren't the point here. The hood's servos
   * aren't driven by the template yet, so it has none.
   */
  static final List<JointSetup> JOINTS =
      List.of(
          JointSetup.of(PIVOT, MechanismControl.position(30).withKd(1).withProfile(8, 30)),
          JointSetup.of(INTAKE, MechanismControl.velocity(0.02)),
          JointSetup.of(SHOOTER, MechanismControl.velocity(0.02)),
          JointSetup.of(FULL_SEND, MechanismControl.velocity(0.02)),
          JointSetup.of(TRANSITION, MechanismControl.velocity(0.02)));

  /**
   * The robot as the template's program runs it in the simulator, without its cameras (odometry and
   * the gyro alone, which the behavior here doesn't depend on, and quicker to run). Its tread grips
   * as the template's does, 1.2: their 2.255 is PathPlanner's tuning value, more grip than the
   * simulator's collisions allow. The drive's limits and gains are the template's kind, chosen for
   * the simulator, not their Tuner X values.
   */
  static final RobotConfig CONFIG =
      RobotConfig.builder()
          .definition(
              RobotDefinitionBuilder.builder(ROBOT)
                  .cameras(List.of())
                  .drivetrain(
                      DrivetrainGeometry.uniform(
                          DrivetrainGeometryModuleTypeBuilder.builder(MODULES)
                              .wheelCoefficientOfFriction(1.2)
                              .build(),
                          Inches.of(10.375),
                          Inches.of(10.375)))
                  .build())
          .drivetrain(
              DrivetrainControl.builder()
                  .limits(
                      DrivetrainControl.Limits.builder()
                          .driveStatorCurrent(Amps.of(60))
                          .driveSupplyCurrent(Amps.of(60))
                          .driveSupplyLowerCurrent(Amps.of(40))
                          .driveSupplyLowerTime(Seconds.of(1))
                          .steerStatorCurrent(Amps.of(40))
                          .maxLinearSpeed(MetersPerSecond.of(4.0))
                          .maxAngularSpeed(RotationsPerSecond.of(1))
                          .build())
                  .gains(
                      DrivetrainControl.Gains.builder()
                          .driveKs(0.0)
                          .driveKv(MODULES.idealDriveKv())
                          .driveKp(2.0)
                          .steerKp(16.0)
                          .steerKd(0.08)
                          .skewLookahead(Seconds.of(0.03))
                          .build())
                  .build())
          .drivetrainWiring(Optional.empty())
          .vision(VisionConfig.DEFAULTS)
          .estimator(EstimatorConfig.DEFAULTS)
          .joints(JOINTS)
          .sensors(List.of())
          .build();

  private static Drive hoodServo(String label, double side) {
    Translation3d onHood = point(0.054, side, 0);
    return drive(
        label,
        WCP_0408,
        Transmission.Between.builder()
            .a(Part.CHASSIS)
            .onA(HOOD.origin().getTranslation().plus(point(0.054, side, -0.233)))
            .b(HOOD)
            .onB(onHood)
            .build());
  }
}
