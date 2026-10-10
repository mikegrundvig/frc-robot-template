package frc.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.Body.Weight;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.List;
import org.wpilib.math.geometry.Translation3d;

/**
 * A team's 2026 competition robot, the Beta robot, as its original code configures it. Paths are
 * under {@code src/main/java/frc/robot/} of that code unless they say otherwise; those starting
 * {@code hardware/} are its helper library's. That code carries the AdvantageKit license ({@code
 * AdvantageKit-License.md}) and WPILib's ({@code WPILib-License.md}): its numbers are copied here
 * with citations, never its code.
 *
 * <p>"Guess:" marks a number the code doesn't hold, with its basis; guesses validate nothing. No
 * link's mass is guessed: the code gives none, so each is none, and the missing-data report lists
 * them. "Model:" marks what the model couldn't say, and the approximation written instead.
 *
 * <p>The hood is written but not built in this code (RobotContainer.java:88 is commented out): it's
 * here as the code describes it, its geometry guessed.
 */
public final class BetaRobot {
  private BetaRobot() {}

  /** Meters in an inch: the team's code measures in inches as often as in meters. */
  private static final double IN = 0.0254;

  private static Translation3d point(double x, double y, double z) {
    return new Translation3d(x, y, z);
  }

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
  public static final Body BODY =
      Body.builder()
          .frameLength(Inches.of(26))
          .frameWidth(Inches.of(26))
          .height(Inches.of(30))
          .bumperEnds(Inches.of(3))
          .bumperSides(Inches.of(3))
          .bumperBottom(Inches.of(2.5))
          .bumperTop(Inches.of(5.75))
          .weight(Weight.weighed(Pounds.of(137.95)))
          .inertia(Body.Turning.yaw(KilogramSquareMeters.of(5.5)))
          .cg(point(0, 0, 0.2))
          .build();

  /**
   * SDS modules, R2 (tuner-swerve-project.json: SwerveModuleConfiguration): 6.027:1 drive and
   * 26.09:1 steer, coupling 3.857 (generated/TunerConstants.java:80-83), wheels 1.93 in
   * (TunerConstants.java:84; Tuner's own file says 2.0). Kraken X60 drive with FOC (Tuner's 96.7
   * rps free speed, and Drive.java:74's PathPlanner motor), Kraken X44 steer (Tuner's file).
   * Tuner's simulation inertias, 0.01 kg m² each (TunerConstants.java:92-93). The tread's grip
   * isn't measured, so it's left out and the robot says so, planning on the default, 1.2: their
   * 2.255 (Constants.java:189) is PathPlanner's tuning value, not the tread's, which grips 1.0 to
   * 1.3 on carpet. Given, the robot would start and say it doubts it, since at a grip that high the
   * slip check can't tell a slide from a collision ({@code LocalizationService.doubts}).
   */
  public static final DrivetrainGeometry.ModuleType MODULES =
      DrivetrainGeometry.ModuleType.builder()
          .driveMotor(Motor.KRAKEN_X60_FOC)
          .steerMotor(Motor.KRAKEN_X44)
          .wheelRadius(Inches.of(1.93))
          .driveGearing(Gearing.of(6.026785714285714))
          .steerGearing(Gearing.of(26.09090909090909))
          .couplingRatio(3.857142857142857)
          .driveInertia(KilogramSquareMeters.of(0.01))
          .steerInertia(KilogramSquareMeters.of(0.01))
          .build();

  /** Their swerve's CANivore (generated/TunerConstants.java:71). */
  public static final CanBus CANIVORE = CanBus.canivore("canivore");

  /**
   * The front left module's wiring, as Tuner X generated it
   * (generated/TunerConstants.java:128-164): its motors' and its CANcoder's IDs and the CANcoder's
   * offset, nothing turned around.
   */
  private static final SwerveWiring.ModuleWiring FRONT_LEFT =
      SwerveWiring.ModuleWiring.builder()
          .driveId(23)
          .steerId(24)
          .encoderOffset(Rotations.of(0.3779296875))
          .driveInverted(false)
          .steerInverted(false)
          .encoderInverted(false)
          .encoderId(12)
          .build();

  /** The front right's, its drive motor turned around, as the right side's are (:86-87). */
  private static final SwerveWiring.ModuleWiring FRONT_RIGHT =
      SwerveWiring.ModuleWiring.builder()
          .driveId(21)
          .steerId(22)
          .encoderOffset(Rotations.of(0.058349609375))
          .driveInverted(true)
          .steerInverted(false)
          .encoderInverted(false)
          .encoderId(11)
          .build();

  /** The back left's. */
  private static final SwerveWiring.ModuleWiring BACK_LEFT =
      SwerveWiring.ModuleWiring.builder()
          .driveId(25)
          .steerId(26)
          .encoderOffset(Rotations.of(0.000244))
          .driveInverted(false)
          .steerInverted(false)
          .encoderInverted(false)
          .encoderId(13)
          .build();

  /** The back right's, its drive motor turned around. */
  private static final SwerveWiring.ModuleWiring BACK_RIGHT =
      SwerveWiring.ModuleWiring.builder()
          .driveId(27)
          .steerId(28)
          .encoderOffset(Rotations.of(-0.200439453125))
          .driveInverted(true)
          .steerInverted(false)
          .encoderInverted(false)
          .encoderId(14)
          .build();

  /**
   * The modules at ±10.375 in (generated/TunerConstants.java:135-169; Tuner's swerve project,
   * tuner-swerve-project.json: VerticalTrackSizeInches 20.75), wired as Tuner X generated them:
   * each corner's drive, steer and CANcoder IDs and the CANcoder's offset
   * (TunerConstants.java:128-164), the right side's drive motors turned around (:86-87), the Pigeon
   * 2 at ID 2 (:89).
   */
  public static final DrivetrainGeometry DRIVETRAIN =
      DrivetrainGeometry.uniform(MODULES, Inches.of(10.375), Inches.of(10.375))
          .wired(
              SwerveWiring.builder()
                  .canBus(CANIVORE)
                  .driveController(MechanismWiring.Controller.TALON_FX)
                  .steerController(MechanismWiring.Controller.TALON_FX)
                  .gyroId(2)
                  .gyroYawScale(1.0)
                  .modules(List.of(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT))
                  .build());

  /** Guess: its PhotonVision cameras' sensors, the template's typical AprilTag camera. */
  private static final CameraConfig.Sensor CAMERA =
      CameraConfig.Sensor.builder()
          .widthPixels(1280)
          .heightPixels(800)
          .fieldOfView(Degrees.of(80))
          .fieldOfViewAcross(CameraConfig.Across.DIAGONAL)
          .frameRate(Hertz.of(30))
          .latency(Milliseconds.of(30))
          .sightRange(Meters.of(7))
          .build();

  /**
   * "Camera6", the left rear (Constants.java:119-126): WPILib's Rotation3d(-2°, -10°, 180.985°),
   * pitch down positive, so tilted up 10°.
   */
  public static final CameraConfig LEFT_REAR =
      CameraConfig.builder()
          .name("Camera6")
          .forward(Inches.of(-10.5))
          .left(Inches.of(12))
          .up(Inches.of(21))
          .roll(Degrees.of(-2))
          .tiltUp(Degrees.of(10))
          .yaw(Degrees.of(180.985))
          .sensor(CAMERA)
          .noiseScale(1.0)
          .build();

  /** "FrontLeft" (Constants.java:129-136). */
  public static final CameraConfig LEFT_FRONT =
      CameraConfig.builder()
          .name("FrontLeft")
          .forward(Inches.of(-7.75))
          .left(Inches.of(12.25))
          .up(Inches.of(21))
          .roll(Degrees.of(0))
          .tiltUp(Degrees.of(10))
          .yaw(Degrees.of(23.162))
          .sensor(CAMERA)
          .noiseScale(1.0)
          .build();

  /**
   * "Camera4", the right rear (Constants.java:139-147): (-11.6875, -12, 16.837) in, turned (0°,
   * -12.5°, 180.985°), then {@code .plus(groundToRobotTopPlate)}, a 3.868 in lift
   * (Constants.java:112-116). Transform3d.plus applies that lift in the camera's own turned frame,
   * not the robot's, so the lens lands 0.84 in further forward and 0.09 in lower than a lift
   * straight up would put it. What their code uses is written here, worked by hand: the team's
   * quirk, which the test checks against WPILib's composition.
   */
  public static final CameraConfig RIGHT_REAR =
      CameraConfig.builder()
          .name("Camera4")
          .forward(Inches.of(-10.8504))
          .left(Inches.of(-11.9856))
          .up(Inches.of(20.6133))
          .roll(Degrees.of(0))
          .tiltUp(Degrees.of(12.5))
          .yaw(Degrees.of(180.985))
          .sensor(CAMERA)
          .noiseScale(1.0)
          .build();

  /** "Camera3", the right front (Constants.java:150-157). */
  public static final CameraConfig RIGHT_FRONT =
      CameraConfig.builder()
          .name("Camera3")
          .forward(Inches.of(-8.625))
          .left(Inches.of(-12.25))
          .up(Inches.of(20.875))
          .roll(Degrees.of(0))
          .tiltUp(Degrees.of(8.2))
          .yaw(Degrees.of(-23.162))
          .sensor(CAMERA)
          .noiseScale(1.0)
          .build();

  /** The pivot's places: in to stow, out to intake, and run, from where its roller may run. */
  public static final Position PIVOT_IN = Position.degrees("in", 1.5);

  public static final Position PIVOT_OUT = Position.degrees("out", 130);

  /**
   * The intake's pivot (subsystems/Pivot.java): a Kraken X60 at CAN ID 32 (Pivot.java:21,
   * Constants.java:79), turning clockwise for positive (Pivot.java:32), through 25:1
   * (Constants.java:85 writes it 1/25, output over motor, as Pivot.java:83 divides by it), read by
   * a duty-cycle absolute encoder on DIO 1 (Pivot.java:23-24, Constants.java:82), inverted
   * (Pivot.java:30, WPILib's DutyCycleEncoder.setInverted), its offset 296° taken off its inverted
   * reading (Constants.java:83; hardware/AbsoluteEncoder.java:55-61): an offset of -296°, inverted.
   * In at 1.5°, out at 130°, run from 110° (Constants.java:89-91).
   *
   * <p>Its named positions are where its code sends it: "in" to stow, "out" to intake. Its code's
   * third angle, PIVOT_RUN_ANGLE at 110° (Constants.java:91), is no place it goes: it's where the
   * roller may run from (Pivot.java:72-73's intakeCanRun), the intake's interlock.
   *
   * <p>Guess: the hard stops at 0° and 135° (the code holds only its setpoints), the pivot 0.25 m
   * forward and 0.32 m up, the arm straight up at 0° and swung forward and down as the angle grows
   * (Pivot.java:30-32 invert both the encoder and the motor, so the sign is a guess too), and the
   * arm drawn as a 0.36 m box out to the roller. Guess: its encoder on the pivot's axle, 1:1, since
   * AbsoluteEncoder.get (hardware/AbsoluteEncoder.java:55-61) reads its turn straight as the
   * pivot's, and 1024 counts a turn, a REV through-bore's absolute output, the code naming no part.
   *
   * <p>How the template drives each joint is chosen to track in the simulator, its feedforward the
   * definition's seed: their code's own gains aren't the point here. The hood's servos aren't
   * driven by the template yet, so it has no control.
   */
  public static final Joint.Revolute PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, point(0.25, 0, 0.32), Joint.PITCH_DOWN)
          .range(Range.degrees(0, 135))
          .rest(PIVOT_IN)
          .carries(Link.box("intake arm", point(0, 0, 0.2), point(0.05, 0.55, 0.36)))
          .positions(PIVOT_IN, PIVOT_OUT)
          .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(32).clockwise())
          .absoluteEncoder(
              MechanismWiring.PortEncoder.dutyCycle(1).offset(Degrees.of(-296)).inverted())
          .control(
              MechanismControl.position(30)
                  .withKd(1)
                  .withProfile(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(1700)));

  /** The intake roller's speeds, the operator's. */
  public static final Output INTAKING = Output.rpm("intaking", 2000);

  public static final Output OUTTAKING = Output.rpm("outtaking", -500);

  /**
   * The intake's roller, on the arm's end: a Kraken X60 at CAN ID 31 (subsystems/Intake.java:21,
   * Constants.java:80; counterclockwise for positive, Intake.java:27) through 3:1
   * (Constants.java:86, 1/3, as Intake.java:43 divides by it), velocity-controlled in output rpm
   * (Intake.java:43). Its speeds are the operator's, "intaking" at 2000 rpm and "outtaking" at 500
   * back (Intake.java:51-53), the 2000 its auto's too (RobotContainer.java:143). Guess: its size.
   * Its rollers run velocity control without FOC (hardware/KrakenX60.java:48): the plain X60's
   * numbers, here and on the shooter, feeder and transition.
   *
   * <p>It runs only with the pivot at 110° or beyond: their auto's guard (RobotContainer.java:143,
   * Pivot.java:72-73, PIVOT_RUN_ANGLE at Constants.java:91), which the operator's buttons don't
   * apply (RobotContainer.java:216-218). As an interlock, it holds beneath every command.
   */
  public static final Joint.Revolute INTAKE =
      Joint.roller("intake", PIVOT, point(0, 0, 0.35))
          .carries(Link.box("intake roller", point(0, 0, 0), point(0.05, 0.5, 0.05)))
          .outputs(INTAKING, OUTTAKING)
          .motor(Motor.KRAKEN_X60, Gearing.of(3), TALON_FX.can(31))
          .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))
          .control(
              MechanismControl.velocity(0.02)
                  .withStatorLimit(Amps.of(60))
                  .withSupplyLimit(Amps.of(60)));

  /**
   * The shooter's wheels: four Kraken X60s at CAN IDs 41 to 44 (subsystems/Shooter.java:23-27,
   * Constants.java:51-54), three following the first (Shooter.java:34-36), 42 aligned with it and
   * the right pair opposed, so turning clockwise where it turns counterclockwise (:37), 3 in wheels
   * (Constants.java:62). Guess: 1:1, since Shooter.java:51 commands the leader's rotor as the
   * shooter's speed with no ratio anywhere; and where it sits, behind the center at the launch
   * height, 18.33 in (Constants.java:65).
   */
  public static final Joint.Revolute SHOOTER =
      Joint.roller("shooter", Part.CHASSIS, point(-0.2, 0, 18.33 * IN))
          .carries(Link.box("shooter wheels", point(0, 0, 0), point(0.076, 0.5, 0.076)))
          .motors(
              Motor.KRAKEN_X60,
              Gearing.of(1),
              TALON_FX.can(41),
              TALON_FX.can(42),
              TALON_FX.can(43).clockwise(),
              TALON_FX.can(44).clockwise())
          .control(
              MechanismControl.velocity(0.02)
                  .withStatorLimit(Amps.of(40))
                  .withSupplyLimit(Amps.of(70)));

  /** The feeder's speeds, the driver's. */
  public static final Output FULL_SEND_FEEDING = Output.rpm("feeding", 1000);

  public static final Output FULL_SEND_UNJAMMING = Output.rpm("unjamming", -1000);

  /**
   * "Full send", the feeder into the shooter: a Kraken X60 at CAN ID 60
   * (subsystems/FullSend.java:21, Constants.java:74; clockwise for positive, FullSend.java:27)
   * through 4:1 (Constants.java:75 writes 1/4; FullSend.java:42 divides by it), in output rpm. Its
   * speeds are the driver's, "feeding" at 1000 rpm and "unjamming" at 1000 back
   * (FullSend.java:49-51). Their autos feed at 5000 (RobotContainer.java:146, AutoTarget.java:62,
   * RobotTarget.java:36), which its motor can't turn it at, so it isn't here: the definition
   * refuses it (BetaRobotTest.theAutosFeederSpeedIsRefused). Guess: where.
   */
  public static final Joint.Revolute FULL_SEND =
      Joint.roller("full send", Part.CHASSIS, point(-0.1, 0, 0.35))
          .carries(Link.box("full send rollers", point(0, 0, 0), point(0.05, 0.4, 0.05)))
          .outputs(FULL_SEND_FEEDING, FULL_SEND_UNJAMMING)
          .motor(Motor.KRAKEN_X60, Gearing.of(4), TALON_FX.can(60).clockwise())
          .control(
              MechanismControl.velocity(0.02)
                  .withStatorLimit(Amps.of(50))
                  .withSupplyLimit(Amps.of(70)));

  /** The transition's speeds, the driver's. */
  public static final Output TRANSITION_FEEDING = Output.rpm("feeding", 2000);

  public static final Output TRANSITION_UNJAMMING = Output.rpm("unjamming", -2000);

  /**
   * The transition's lower rollers, the only ones built (subsystems/Transition.java:21; the upper
   * pair is commented out, :23-24): a Kraken X60 at CAN ID 52 (Constants.java:97; clockwise for
   * positive, Transition.java:34) through 2:1 (Constants.java:100, 1/2; Transition.java:72), in
   * output rpm. Its speeds are the driver's, "feeding" at 2000 rpm and "unjamming" at 2000 back
   * (Transition.java:79-81); its autos run it at 1000 and 2000 (RobotContainer.java:145,
   * AutoTarget.java:63). Guess: where.
   */
  public static final Joint.Revolute TRANSITION =
      Joint.roller("transition", Part.CHASSIS, point(0, 0, 0.15))
          .carries(Link.box("transition rollers", point(0, 0, 0), point(0.05, 0.4, 0.05)))
          .outputs(TRANSITION_FEEDING, TRANSITION_UNJAMMING)
          .motor(Motor.KRAKEN_X60, Gearing.of(2), TALON_FX.can(52).clockwise())
          .control(
              MechanismControl.velocity(0.02)
                  .withStatorLimit(Amps.of(60))
                  .withSupplyLimit(Amps.of(70)));

  /**
   * The hood's servos, WCP-0408s (hardware/WCP_0408.java:11), Actuonix L16-100-35-6-R linear
   * servos: 168 mm eye to eye closed, a 100 mm stroke, 50 N at most and 32 mm/s free (WCP's
   * Miniature Linear Servo Actuators user guide: L16 35:1 table). WPILib's Servo bounds of 1000 to
   * 2000 µs (WCP_0408.java:15) are the guide's own, 1 ms fully retracted and 2 ms fully extended,
   * so {@code Servo.set}'s 0 to 1 is the model's command. On PWM 0 and 1 (Constants.java:56-57),
   * held to 0.65 to 1.0 (Constants.java:59-60), "0.65 is our current minimum" (Hood.java:24).
   */
  public static final Actuator.Servo WCP_0408 =
      Actuator.Servo.builder().from(0.168).to(0.268).force(50).speed(0.032).build();

  /**
   * The hood (subsystems/Hood.java), turned by two linear servos, one each side, told the same
   * share of their travel (Hood.java:22-32). Guess: everything about where it sits: its pivot 0.1 m
   * above the shooter's wheels, raised as its angle grows, from 0° to 40°; its shape unknown. Its
   * servos are placed below so that the code's lowest command, 0.65, is the hood at 0°, and their
   * full stroke 40°. Guess: each servo's base on the chassis 0.233 m below its end on the hood,
   * which is 54 mm out along the hood's zero and 0.1 m to its side.
   */
  public static final Joint.Revolute HOOD =
      Joint.pivot("hood", Part.CHASSIS, point(-0.2, 0, 18.33 * IN + 0.1), Joint.PITCH_UP)
          .range(Range.degrees(0, 40))
          .rest(Degrees.of(0))
          .servo("hood left", WCP_0408, point(0.054, 0.1, -0.233), point(0.054, 0.1, 0))
          .servo("hood right", WCP_0408, point(0.054, -0.1, -0.233), point(0.054, -0.1, 0));

  /**
   * The driver's feed, both rollers at once, on the right trigger (RobotContainer.java:220-223):
   * the feed that gates on nothing, so the shooter may not be up to speed.
   */
  public static final Configuration FEEDING =
      Configuration.of(
          "feeding", FULL_SEND.at(FULL_SEND_FEEDING), TRANSITION.at(TRANSITION_FEEDING));

  /** The same backward, on the right bumper (RobotContainer.java:225-228). */
  public static final Configuration UNJAMMING =
      Configuration.of(
          "unjamming", FULL_SEND.at(FULL_SEND_UNJAMMING), TRANSITION.at(TRANSITION_UNJAMMING));

  /**
   * The robot. Its one IMU is a Pigeon 2 (generated/TunerConstants.java:89), given no mount pose
   * (TunerConstants.java:67, null configs). Guess: at the center, level. No backup IMU: a 2026
   * roboRIO robot has no Systemcore's own.
   */
  public static final RobotDefinition ROBOT =
      RobotDefinition.builder()
          .name("beta")
          .body(BODY)
          .drivetrain(DRIVETRAIN)
          .imus(Imus.builder().gyro(ImuMount.UNCHECKED).build())
          .cameras(List.of(LEFT_REAR, LEFT_FRONT, RIGHT_REAR, RIGHT_FRONT))
          .joints(List.of(PIVOT, INTAKE, SHOOTER, FULL_SEND, TRANSITION, HOOD))
          .configurations(List.of(FEEDING, UNJAMMING))
          // No PDH declared: the code reads a REV PDH at 1
          // (hardware/LoggedPDH.java:31-36)
          // and sums its channels by mechanism (LoggedPDH.java:111-134), but never says which
          // motor is on which channel, nor any breaker, so no motor's feed is given.
          .build();

  /**
   * The robot as the template's program runs it, its drivetrain driven as the template drives one,
   * wired as the team's Tuner X generated it. The drive's limits and gains are the template's kind,
   * chosen for the simulator, not their Tuner X values. The program doesn't start on it: {@link
   * Robots#current} decides that, and still names the template's robot.
   */
  public static final RobotConfig CONFIG =
      RobotConfig.builder()
          .definition(
              ROBOT.with(
                  DRIVETRAIN.control(
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
                                  .headingKp(5.0)
                                  .translationKp(3.0)
                                  .build())
                          .build())))
          .vision(VisionConfig.DEFAULTS)
          .estimator(EstimatorConfig.DEFAULTS)
          .build();
}
