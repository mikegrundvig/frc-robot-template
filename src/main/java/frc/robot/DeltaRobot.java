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
import org.wpilib.units.measure.Angle;

/**
 * A team's 2026 Delta robot, as its original code configures it: the Beta robot's design ({@link
 * BetaRobot}) with other hardware. Paths are under {@code src/main/java/frc/robot/} of that code
 * unless they say otherwise; those starting {@code hardware/} are its helper library's. That code
 * carries the AdvantageKit license ({@code AdvantageKit-License.md}) and WPILib's ({@code
 * WPILib-License.md}): its numbers are copied here with citations, never its code.
 *
 * <p>Against Beta, Delta's code has: other swerve modules (a 5.27:1 drive on 2 in wheels, other
 * CANcoder offsets, a Pigeon 2 at ID 1, and a CAN bus its Tuner X left named "Default Name"); a
 * pivot on another encoder offset, not inverted, that it drives open loop, out at 5 V and in at -3
 * V; an intake roller on two Krakens through 2:1; a shooter led by another motor; a camera named
 * "Camera5" where Beta's is "FrontLeft"; and no hood.
 *
 * <p>"Guess:" marks a number the code doesn't hold, with its basis; guesses validate nothing. A
 * number Delta's code doesn't hold but Beta's does is Beta's, marked a guess as Beta marks it. No
 * link's mass is guessed: the code gives none, so each is none, and the missing-data report lists
 * them. "Model:" marks what the model couldn't say, and the approximation written instead.
 */
public final class DeltaRobot {
  private DeltaRobot() {}

  /** Meters in an inch: the team's code measures in inches as often as in meters. */
  private static final double IN = 0.0254;

  private static Translation3d point(double x, double y, double z) {
    return new Translation3d(x, y, z);
  }

  /**
   * Guess: a 26 in square frame, PathPlanner's 0.81 m robot width less 3 in bumpers
   * (src/main/deploy/pathplanner/settings.json:2, a copy of Beta's, never edited for Delta), around
   * the modules' 20.75 in track (below). Guess: 3 in bumpers, from the same width; the bumper zone
   * for their heights, 30 in tall, and the center of mass centered 0.2 m up.
   *
   * <p>137.95 lb (Constants.java:193; PathPlanner's 62.57 kg, settings.json:15), taken as weighed
   * with the battery (Guess: the code doesn't say; without it, the robot would be over R408). Its
   * yaw inertia, 5.5 kg m² (Constants.java:194). Delta's code reads neither: Drive.java:64-66 hands
   * PathPlanner 74.088 kg, 6.883 and a friction of 1.2, the numbers the team's earlier robots carry
   * unchanged, so they are placeholders, not Delta's, and Constants.java's are Beta's, copied. The
   * model's box gives the pitch and roll.
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
   * Modules of a 5.2734375:1 drive and 26.09:1 steer, coupling 3.375, wheels 2 in
   * (generated/TunerConstants.java:86-90). Their kSpeedAt12Volts, 5.85 m/s (:82), is what a Kraken
   * X60 with FOC turns the wheel at, free. Kraken X60 drive with FOC (Drive.java:75-76's
   * PathPlanner motor). Guess: Kraken X44 steer, as Beta's Tuner project has it: Delta's repository
   * has no Tuner project file, and its simulation's steer motor, an X60 (ModuleIOSim.java:38), is
   * the template's. Tuner's simulation inertias, 0.01 kg m² each (TunerConstants.java:98-99). The
   * tread's grip isn't measured, so it's left out and the robot says so, planning on the default,
   * 1.2: Drive.java:66's 1.2 is the placeholder above, and Constants.java:195's 2.255 is Beta's
   * PathPlanner tuning value, unread, not the tread's.
   */
  public static final DrivetrainGeometry.ModuleType MODULES =
      DrivetrainGeometry.ModuleType.builder()
          .driveMotor(Motor.KRAKEN_X60_FOC)
          .steerMotor(Motor.KRAKEN_X44)
          .wheelRadius(Inches.of(2))
          .driveGearing(Gearing.of(5.2734375))
          .steerGearing(Gearing.of(26.09090909090909))
          .couplingRatio(3.375)
          .driveInertia(KilogramSquareMeters.of(0.01))
          .steerInertia(KilogramSquareMeters.of(0.01))
          .build();

  /**
   * Their swerve's CAN bus (generated/TunerConstants.java:78), named as Tuner X's generator left
   * it, "Default Name"; Beta's is "canivore".
   */
  public static final CanBus CANIVORE = CanBus.canivore("Default Name");

  /**
   * The front left module's wiring, as Tuner X generated it
   * (generated/TunerConstants.java:137-142): its motors' and its CANcoder's IDs and the CANcoder's
   * offset, nothing turned around.
   */
  private static final SwerveWiring.ModuleWiring FRONT_LEFT =
      SwerveWiring.ModuleWiring.builder()
          .driveId(23)
          .steerId(24)
          .encoderOffset(Rotations.of(0.10693359375))
          .driveInverted(false)
          .steerInverted(false)
          .encoderInverted(false)
          .encoderId(12)
          .build();

  /** The front right's, its drive motor turned around, as the right side's are (:93). */
  private static final SwerveWiring.ModuleWiring FRONT_RIGHT =
      SwerveWiring.ModuleWiring.builder()
          .driveId(21)
          .steerId(22)
          .encoderOffset(Rotations.of(-0.031494140625))
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
          .encoderOffset(Rotations.of(-0.12939453125))
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
          .encoderOffset(Rotations.of(-0.38427734375))
          .driveInverted(true)
          .steerInverted(false)
          .encoderInverted(false)
          .encoderId(14)
          .build();

  /**
   * The modules at ±10.375 in (generated/TunerConstants.java:144-178), wired as Tuner X generated
   * them: each corner's drive, steer and CANcoder IDs and the CANcoder's offset
   * (TunerConstants.java:137-175), the right side's drive motors turned around (:93), the Pigeon 2
   * at ID 1 (:95).
   */
  public static final DrivetrainGeometry DRIVETRAIN =
      DrivetrainGeometry.uniform(MODULES, Inches.of(10.375), Inches.of(10.375))
          .wired(
              SwerveWiring.builder()
                  .canBus(CANIVORE)
                  .driveController(MechanismWiring.Controller.TALON_FX)
                  .steerController(MechanismWiring.Controller.TALON_FX)
                  .gyroId(1)
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
   * "Camera6", the left rear (Constants.java:119-127): WPILib's Rotation3d(-2°, -10°, 180.985°),
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

  /** "Camera5", the left front (Constants.java:130-138); Beta's is "FrontLeft". */
  public static final CameraConfig LEFT_FRONT =
      CameraConfig.builder()
          .name("Camera5")
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
   * "Camera4", the right rear (Constants.java:141-150): (-11.6875, -12, 16.837) in, turned (0°,
   * -12.5°, 180.985°), then {@code .plus(groundToRobotTopPlate)}, a 3.938 in lift, 1.938 in and the
   * wheel's radius, 2 in (Constants.java:111-116; Beta's wheels are 1.93 in, its lift 3.868 in).
   * Transform3d.plus applies that lift in the camera's own turned frame, not the robot's, so the
   * lens lands 0.85 in further forward and 0.09 in lower than a lift straight up would put it. What
   * their code uses is written here, worked by hand: the team's quirk, which the test checks
   * against WPILib's composition.
   */
  public static final CameraConfig RIGHT_REAR =
      CameraConfig.builder()
          .name("Camera4")
          .forward(Inches.of(-10.8353))
          .left(Inches.of(-11.9853))
          .up(Inches.of(20.6817))
          .roll(Degrees.of(0))
          .tiltUp(Degrees.of(12.5))
          .yaw(Degrees.of(180.985))
          .sensor(CAMERA)
          .noiseScale(1.0)
          .build();

  /** "Camera3", the right front (Constants.java:153-161). */
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

  /**
   * The pivot's three angles as Constants.java:88-90 names them: where it reads in (the in-drive
   * ends at or under it), where it reads out (the out-drive ends at or past it), and where the
   * roller may run from, which is the same 130°.
   */
  public static final Angle PIVOT_IN_ANGLE = Degrees.of(3);

  public static final Angle PIVOT_OUT_ANGLE = Degrees.of(130);

  public static final Angle PIVOT_RUN_ANGLE = Degrees.of(130);

  /** The pivot's places: in to stow, and out to intake. */
  public static final Position PIVOT_IN = Position.of("in", PIVOT_IN_ANGLE);

  public static final Position PIVOT_OUT = Position.of("out", PIVOT_OUT_ANGLE);

  /**
   * The pivot's open-loop drive, as the operator's buttons run it (Pivot.java:51-79): out at +5 V
   * until it reads out, in at -3 V until it reads in. Its closed loop is never used: its
   * controller's gains are all 0 (Pivot.java:21-22) and the commands that use it are bound to
   * nothing.
   */
  public static final Output PIVOT_SWINGING_OUT = Output.volts("swinging out", 5);

  public static final Output PIVOT_SWINGING_IN = Output.volts("swinging in", -3);

  /**
   * The intake's pivot (subsystems/Pivot.java): a Kraken X60 at CAN ID 32 (Pivot.java:16,
   * Constants.java:77), turning clockwise for positive (Pivot.java:27), through 25:1
   * (Constants.java:84 writes it 1/25, output over motor, as Pivot.java:96 divides by it), read by
   * a duty-cycle absolute encoder on DIO 1 (Pivot.java:19, Constants.java:81), not inverted
   * (Pivot.java:25), its offset -100° taken off its reading (Constants.java:82;
   * hardware/AbsoluteEncoder.java:55-61): an offset of +100°. In at 3°, out at 130°, run from 130°
   * (Constants.java:88-90; Beta: 1.5°, 130°, 110°).
   *
   * <p>Its named positions are where its code sends it: "in" to stow, "out" to intake. Its code's
   * third angle, PIVOT_RUN_ANGLE at 130° (Constants.java:90), is no place it goes: it's where the
   * roller may run from (Pivot.java:85-87's intakeCanRun), the intake's interlock. It's also where
   * the pivot is out: the same angle as {@code PIVOT_OUT}.
   *
   * <p>Guess: the hard stops at 0° and 135° (the code holds only its setpoints), the pivot 0.25 m
   * forward and 0.32 m up, the arm straight up at 0° and swung forward and down as the angle grows
   * (Pivot.java:27 turns the motor clockwise for out, so the sign is a guess), and the arm drawn as
   * a 0.36 m box out to the roller. Guess: its encoder on the pivot's axle, 1:1, since
   * AbsoluteEncoder.get (hardware/AbsoluteEncoder.java:55-61) reads its turn straight as the
   * pivot's, and 1024 counts a turn, a REV through-bore's absolute output, the code naming no part.
   *
   * <p>How the template drives each joint is chosen to track in the simulator, its feedforward the
   * definition's seed: their code's own gains aren't the point here. The pivot's voltages are
   * outputs, run until a gate holds; the code then sets 0 V and the brake holds the arm (Pivot.java
   * :26), where the model's idle holds its goal, the pose it came to a stop at. Model: that hold.
   */
  public static final Joint.Revolute PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, point(0.25, 0, 0.32), Joint.PITCH_DOWN)
          .range(Range.degrees(0, 135))
          .rest(PIVOT_IN)
          .carries(Link.box("intake arm", point(0, 0, 0.2), point(0.05, 0.55, 0.36)))
          .positions(PIVOT_IN, PIVOT_OUT)
          .outputs(PIVOT_SWINGING_OUT, PIVOT_SWINGING_IN)
          .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(32).clockwise())
          .absoluteEncoder(MechanismWiring.PortEncoder.dutyCycle(1).offset(Degrees.of(100)))
          .control(
              MechanismControl.position(30)
                  .withKd(1)
                  .withProfile(DegreesPerSecond.of(450), DegreesPerSecondPerSecond.of(1700)));

  /** The intake roller's speeds, the operator's. */
  public static final Output INTAKING = Output.rpm("intaking", 3000);

  public static final Output OUTTAKING = Output.rpm("outtaking", -500);

  /**
   * The intake's roller, on the arm's end: two Kraken X60s, at CAN IDs 31 and 33
   * (subsystems/Intake.java:17-18, Constants.java:78-79), 31 leading, counterclockwise for positive
   * (Intake.java:25), 33 following it opposed (Intake.java:23) and so turning clockwise, through
   * 2:1 (Constants.java:85, 1/2, as Intake.java:42 divides by it), velocity-controlled in output
   * rpm (Intake.java:42). Its speeds are the operator's, "intaking" at 3000 rpm and "outtaking" at
   * 500 back (Intake.java:51-52), the 2000 its auto's (RobotContainer.java:166). Guess: its size.
   * Its rollers run velocity control without FOC (hardware/KrakenX60.java:49): the plain X60's
   * numbers, here and on the shooter, feeder and transition.
   *
   * <p>It runs only with the pivot at 130° or beyond: their auto's guard (RobotContainer.java:164
   * -166, Pivot.java:85-87, PIVOT_RUN_ANGLE at Constants.java:90), which the operator's buttons
   * don't apply (RobotContainer.java:250-252). As an interlock, it holds beneath every command.
   */
  public static final Joint.Revolute INTAKE =
      Joint.roller("intake", PIVOT, point(0, 0, 0.35))
          .carries(Link.box("intake roller", point(0, 0, 0), point(0.05, 0.5, 0.05)))
          .outputs(INTAKING, OUTTAKING)
          .motors(Motor.KRAKEN_X60, Gearing.of(2), TALON_FX.can(31), TALON_FX.can(33).clockwise())
          .runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN_ANGLE))
          .control(
              MechanismControl.velocity(0.02)
                  .withStatorLimit(Amps.of(40))
                  .withSupplyLimit(Amps.of(70)));

  /**
   * The shooter's wheels: four Kraken X60s at CAN IDs 41 to 44 (subsystems/Shooter.java:18-21,
   * Constants.java:49-52), 43 leading, clockwise for positive (Shooter.java:32), the other three
   * following it (Shooter.java:28-30): 44 aligned, 41 and 42 opposed, so turning counterclockwise
   * where it turns clockwise. 3 in wheels (Constants.java:60). Guess: 1:1, since Shooter.java:45
   * commands the leader's rotor as the shooter's speed with no ratio anywhere; and where it sits,
   * behind the center at the launch height, 18.33 in (Constants.java:63).
   */
  public static final Joint.Revolute SHOOTER =
      Joint.roller("shooter", Part.CHASSIS, point(-0.2, 0, 18.33 * IN))
          .carries(Link.box("shooter wheels", point(0, 0, 0), point(0.076, 0.5, 0.076)))
          .motors(
              Motor.KRAKEN_X60,
              Gearing.of(1),
              TALON_FX.can(43).clockwise(),
              TALON_FX.can(44).clockwise(),
              TALON_FX.can(41),
              TALON_FX.can(42))
          .control(
              MechanismControl.velocity(0.02)
                  .withStatorLimit(Amps.of(40))
                  .withSupplyLimit(Amps.of(70)));

  /** The feeder's speeds, the driver's. */
  public static final Output FULL_SEND_FEEDING = Output.rpm("feeding", 1000);

  public static final Output FULL_SEND_UNJAMMING = Output.rpm("unjamming", -1000);

  /**
   * "Full send", the feeder into the shooter: a Kraken X60 at CAN ID 60
   * (subsystems/FullSend.java:16, Constants.java:72; clockwise for positive, FullSend.java:21)
   * through 4:1 (Constants.java:73 writes 1/4; FullSend.java:37 divides by it), in output rpm. Its
   * speeds are the driver's, "feeding" at 1000 rpm and "unjamming" at 1000 back (FullSend.java:46).
   * Their targeting feeds at 5000 (RobotContainer.java:171, RobotTarget.java:38,
   * AutoTarget.java:70), which its motor can't turn it at, so it isn't here: the definition refuses
   * it (DeltaRobotTest.theTargetingFeederSpeed IsRefused). Guess: where.
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
   * The transition's lower rollers, the only ones built (subsystems/Transition.java:17; the upper
   * pair is commented out, :19-22, and "provides absolutely nothing", Constants.java:94-95): a
   * Kraken X60 at CAN ID 52 (Constants.java:96; clockwise for positive, Transition.java:34) through
   * 2:1 (Constants.java:99, 1/2; Transition.java:74), in output rpm. Its speeds are the driver's,
   * "feeding" at 2000 rpm and "unjamming" at 2000 back (Transition.java:83); its targeting runs it
   * at 1000 and 2000 (RobotContainer.java:170, AutoTarget.java:71, RobotTarget.java:39). Guess:
   * where.
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
   * The driver's feed, both rollers at once, on the right trigger (RobotContainer.java:254-259):
   * the feed that gates on nothing, so the shooter may not be up to speed.
   */
  public static final Configuration FEEDING =
      Configuration.of(
          "feeding", FULL_SEND.at(FULL_SEND_FEEDING), TRANSITION.at(TRANSITION_FEEDING));

  /** The same backward, on the right bumper (RobotContainer.java:261-266). */
  public static final Configuration UNJAMMING =
      Configuration.of(
          "unjamming", FULL_SEND.at(FULL_SEND_UNJAMMING), TRANSITION.at(TRANSITION_UNJAMMING));

  /**
   * The robot. Its one IMU is a Pigeon 2 (generated/TunerConstants.java:95), given no mount pose
   * (TunerConstants.java:74, null configs). Guess: at the center, level. No backup IMU: a 2026
   * roboRIO robot has no Systemcore's own.
   */
  public static final RobotDefinition ROBOT =
      RobotDefinition.builder()
          .name("delta")
          .body(BODY)
          .drivetrain(DRIVETRAIN)
          .imus(Imus.builder().gyro(ImuMount.UNCHECKED).build())
          .cameras(List.of(LEFT_REAR, LEFT_FRONT, RIGHT_REAR, RIGHT_FRONT))
          .joints(List.of(PIVOT, INTAKE, SHOOTER, FULL_SEND, TRANSITION))
          .configurations(List.of(FEEDING, UNJAMMING))
          // No PDH declared: the code reads a REV PDH at 1
          // (hardware/LoggedPDH.java:31-36)
          // and sums its channels by mechanism (LoggedPDH.java:111-134), but never says which
          // motor is on which channel, nor any breaker, so no motor's feed is given.
          .build();

  /**
   * The robot as the template's program runs it, its drivetrain driven as the template drives one,
   * wired as the team's Tuner X generated it. The drive's limits and gains are the template's kind,
   * chosen for the simulator, not their Tuner X values; the stator limits are theirs: the drive's
   * is the slip current, 60 A (generated/TunerConstants.java:58, subsystems/drive/
   * ModuleIOTalonFX.java:107-110), the steering's 40 A (TunerConstants.java:70). The program
   * doesn't start on it: {@link Robots#current} decides that, and still names the template's robot.
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
