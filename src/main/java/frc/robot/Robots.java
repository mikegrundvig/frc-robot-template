package frc.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import frc.robot.game.Rebuilt;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The team's robots, one {@link RobotConfig} each. A test builds every one, so an invalid value
 * fails the build rather than the robot.
 *
 * <p>Every value is set by name through a builder, which asks for each in turn: none can be left
 * out or land in the wrong place.
 *
 * <p>The template's robot is a typical competition build, at the 2026 rule maximums where the rules
 * set one (the next season's rules replace them at kickoff). Replace each value with your robot's
 * as you design and measure it: docs/values.md says how to measure each, and where today's number
 * came from. Note a measured value where it's set, as the calibration procedures do: {@code
 * .weight(Body.Weight.weighed(Pounds.of(131.2))) // measured YYYY-MM-DD: two scales, bumpers and
 * battery on}.
 *
 * <p>A robot's mechanisms are written here too, each joint in one block: where it is, its named
 * places and outputs, its motors and their wiring, its own sensors, and how the program drives it.
 * The configurations the robot takes, and the interlocks between joints, which hold beneath every
 * command, name the joints and their places. The template's robot has none; an intake would read:
 *
 * <pre>{@code
 * static final Position PIVOT_IN = Position.degrees("in", 1.5);
 * static final Position PIVOT_RUN = Position.degrees("run", 110);
 * static final Position PIVOT_OUT = Position.degrees("out", 130);
 *
 * static final Joint.Revolute PIVOT =
 *     Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
 *         .range(Range.degrees(0, 135), PIVOT_IN)
 *         .positions(PIVOT_IN, PIVOT_RUN, PIVOT_OUT)
 *         .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(32))
 *         .control(MechanismControl.position(30).withProfile(8, 30));
 *
 * static final Output INTAKING = Output.rpm("intaking", 2000);
 * static final Joint.Revolute ROLLER =
 *     Joint.roller("roller", PIVOT, new Translation3d(0, 0, 0.35))
 *         .outputs(INTAKING)
 *         .motor(Motor.KRAKEN_X60, Gearing.of(3), TALON_FX.can(31))
 *         .control(MechanismControl.velocity(0.02));
 *
 * static final Configuration INTAKE =
 *     Configuration.of("intake", PIVOT.at(PIVOT_OUT), ROLLER.at(INTAKING));
 * static final Interlock ROLLER_ONLY_WHEN_OUT =
 *     Interlock.that(ROLLER).runsOnlyWhen(PIVOT.past(PIVOT_RUN));
 * }</pre>
 *
 * <p>The definition refuses what can't be: a position outside its joint's range, a speed past what
 * its motors turn it at or a voltage past what they're rated for, a configuration that breaks an
 * interlock, interlocks that hold two joints by each other. {@code RobotDefinition.notes()} lists
 * what's doubtful, a speed so near its motors' free speed that its loop can't hold it, and the
 * program says it at startup: in the log ({@code Robot/DefinitionDoubts}) and, on a robot, on the
 * Driver Station's console.
 *
 * <p>Behavior code names the constants themselves: {@code joints.of(PIVOT)}, {@code
 * pivot.goTo(PIVOT_OUT)}. A practice robot of the same design is the same definition with its
 * differences, {@code COMPETITION.with(PIVOT.motor(Motor.KRAKEN_X60, Gearing.of(25),
 * TALON_FX.can(33)))}: a joint wired or tuned otherwise is still the same joint, so the same code
 * runs on either. 2611's 2026 robot shows it (src/test/java/frc/robot/robots2026/Team2611.java).
 * docs/mechanisms.md says how to describe a mechanism and use its commands.
 */
public final class Robots {
  private Robots() {}

  /**
   * Not weighed: the most the season allows, 135 lb of robot and bumpers (R408) plus a 14.5 lb
   * battery (R601). Weigh the robot and write what the scale says ({@code
   * Body.Weight.weighed(Pounds.of(131.2))}).
   */
  private static final Body.Weight WEIGHT = Rebuilt.LIMITS.unweighed();

  /**
   * Frame 27.5 x 27.5 in, the largest square within 2026's 110 in frame perimeter (R104), with
   * bumpers 4 in beyond it (R403) filling the bumper zone, 2.5 to 5.75 in up (R405), and 30 in
   * tall, 2026's starting height limit (R104). The 6.7 kg m^2 yaw inertia is an estimate, not a
   * measurement; pitch and roll are the model's box, provisional until CAD or a tilt test gives
   * them. The center of gravity is centered and low: the best case.
   */
  private static final Body BODY =
      Body.builder()
          .frameLength(Inches.of(27.5))
          .frameWidth(Inches.of(27.5))
          .height(Inches.of(30))
          .bumperEnds(Inches.of(4))
          .bumperSides(Inches.of(4))
          .bumperBottom(Inches.of(2.5))
          .bumperTop(Inches.of(5.75))
          .weight(WEIGHT)
          .cg(new Translation3d(0, 0, 0.15))
          .inertia(Body.Turning.yaw(KilogramSquareMeters.of(6.7)))
          .build();

  /**
   * SDS MK4i modules with L2 gearing (6.75:1 drive, 150/7:1 steer) and 4 in wheels, driven by
   * Kraken X60s without FOC: it needs a Phoenix Pro license, and under this drivetrain's 30 A limit
   * it adds nothing. Free speed: 4.73 m/s. The gearing's losses aren't modeled yet (efficiency 1).
   * The drive's moving parts, as inertia at the wheel, are an estimate (a 4 in wheel, about 0.0004
   * kg m², and a Kraken X60's rotor through 6.75:1, about 0.0006); the steering's is Phoenix Tuner
   * X's generated simulation value. The coupling, 50/14 drive motor turns per turn of the module,
   * is Tuner X's for the MK4i, as 2611's 2024 code had it.
   */
  private static final DrivetrainGeometry.ModuleType MODULES =
      DrivetrainGeometry.ModuleType.builder()
          .driveMotor(Motor.KRAKEN_X60)
          .steerMotor(Motor.KRAKEN_X60)
          .wheelRadius(Inches.of(2))
          .driveGearing(Gearing.builder().ratio(6.75).efficiency(1.0).build())
          .steerGearing(Gearing.builder().ratio(150.0 / 7.0).efficiency(1.0).build())
          .wheelCoefficientOfFriction(1.2)
          .driveInertia(KilogramSquareMeters.of(0.001))
          .steerInertia(KilogramSquareMeters.of(0.004))
          .couplingRatio(50.0 / 14.0)
          .build();

  /**
   * Drive stator limit 30 A. Under hard acceleration, weight shifts off the front wheels, and by a
   * simple weight-transfer estimate they start to slip near 47 A with this tread (friction 1.2;
   * about 50 A at 1.3, new) and this low center of gravity, or near 31 A with worn tread (friction
   * 1.0) and a center of gravity at 0.3 m. More current than that only spins the wheels and drains
   * the battery. Steering needs little torque: 40 A, the value Phoenix Tuner X's swerve generator
   * writes, as the 2026 code kept it.
   *
   * <p>Supply limit 60 A per drive motor, stepping down to 40 A once a motor has been held at 60 A
   * for a second, until it draws less than 40 A again. That's Phoenix's own step-down, which it
   * applies unless told otherwise, written out here so nobody is surprised by it: it keeps a 40 A
   * breaker from tripping in a long push (one can trip in 3.9 s at 60 A), and no sprint holds 60 A
   * for anywhere near a second. With the 30 A stator limit, each drive motor draws at most its 30 A
   * from the battery, so neither comes into play until the stator limit is raised past them.
   *
   * <p>Top speed 4.4 m/s leaves the velocity control headroom under the free speed; spinning is
   * capped at a driver-friendly 1 rotation per second. kV is the ideal value for these motors and
   * gearing; kS and the drive's kP are starting values from simulation, and the steering's kP and
   * kD are the 2026 code's (100 V per rotation and 0.5, converted). Measure and tune them all on
   * the real robot.
   */
  private static final DrivetrainControl DRIVING =
      DrivetrainControl.builder()
          .limits(
              DrivetrainControl.Limits.builder()
                  .driveStatorCurrent(Amps.of(30))
                  .driveSupplyCurrent(Amps.of(60))
                  .driveSupplyLowerCurrent(Amps.of(40))
                  .driveSupplyLowerTime(Seconds.of(1))
                  .steerStatorCurrent(Amps.of(40))
                  .maxLinearSpeed(MetersPerSecond.of(4.4))
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
          .build();

  /**
   * The modules sit on the corners, where MK4i wheels are 2.625 in in from the frame's edges,
   * driven as above. No wiring: on real hardware the drivetrain stays still and says so.
   */
  private static final DrivetrainGeometry DRIVETRAIN =
      DrivetrainGeometry.inset(MODULES, BODY, Inches.of(2.625)).control(DRIVING);

  /**
   * A 1280 x 800 global-shutter camera with an 80 degree diagonal field of view, typical for
   * AprilTags. The frame rate and latency are guesses: measure yours (the Vision log's {@code
   * LatencySeconds} for each camera). The latency matters on the robot, not only in the simulator,
   * which runs its cameras with it: an image arriving more than {@code
   * VisionConfig.latencyMarginSeconds} later than it is left out, so a guess too low drops images.
   * The 7 m sight range is a guess too, the simulator's all along: measure where yours stops
   * finding a tag (docs/characterization.md, camera noise).
   */
  private static final CameraConfig.Sensor APRILTAG_CAMERA =
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
   * Four cameras as a 2026 competition robot mounted them: two facing forward and out at 23
   * degrees, two facing backward, all near the frame's sides about 21 in up and tilted up 8 to 12.5
   * degrees. Positions as that team's 2026 code gives them, from the robot's origin on the floor.
   */
  private static final List<CameraConfig> CAMERAS =
      List.of(
          CameraConfig.builder()
              .name("front-left")
              .forward(Inches.of(-7.75))
              .left(Inches.of(12.25))
              .up(Inches.of(21))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(10))
              .yaw(Degrees.of(23.162))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .joint(Optional.empty())
              .build(),
          CameraConfig.builder()
              .name("front-right")
              .forward(Inches.of(-8.625))
              .left(Inches.of(-12.25))
              .up(Inches.of(20.875))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(8.2))
              .yaw(Degrees.of(-23.162))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .joint(Optional.empty())
              .build(),
          CameraConfig.builder()
              .name("back-left")
              .forward(Inches.of(-10.5))
              .left(Inches.of(12))
              .up(Inches.of(21))
              .roll(Degrees.of(-2))
              .tiltUp(Degrees.of(10))
              .yaw(Degrees.of(180.985))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .joint(Optional.empty())
              .build(),
          CameraConfig.builder()
              .name("back-right")
              .forward(Inches.of(-11.6875))
              .left(Inches.of(-12))
              .up(Inches.of(20.705))
              .roll(Degrees.of(0))
              .tiltUp(Degrees.of(12.5))
              .yaw(Degrees.of(180.985))
              .sensor(APRILTAG_CAMERA)
              .noiseScale(1.0)
              .joint(Optional.empty())
              .build());

  /**
   * The simulator's IMUs: at the center, 3 in up on the belly pan, level with x forward, and
   * checked (they read along their axes by construction).
   */
  private static final ImuMount SIM_IMU =
      ImuMount.builder()
          .forward(Meters.of(0))
          .left(Meters.of(0))
          .up(Inches.of(3))
          .orientation(Rotation3d.ZERO)
          .accelerometerChecked(true)
          .build();

  /**
   * The template's robot. It has no wiring: on real hardware its drivetrain stays still and says
   * so. Give a real robot its own entry, with its wiring from Phoenix Tuner X's swerve generator.
   */
  public static final RobotConfig SIM =
      RobotConfig.builder()
          .definition(
              RobotDefinition.builder()
                  .name("sim")
                  .body(BODY)
                  .drivetrain(DRIVETRAIN)
                  .cameras(CAMERAS)
                  // The simulator's IMUs sit at the center on the belly pan, level, x forward, and
                  // read along their axes by construction. A real robot starts from
                  // Imus.UNCHECKED, its IMUs measured where they're mounted, until each passes the
                  // accelerometer check (docs/calibration.md).
                  .imus(Imus.builder().gyro(SIM_IMU).backup(Optional.of(SIM_IMU)).build())
                  .joints(List.of())
                  .couplings(List.of())
                  .drives(List.of())
                  .sensors(List.of())
                  .configurations(List.of())
                  .interlocks(List.of())
                  .build())
          .vision(VisionConfig.DEFAULTS)
          .estimator(EstimatorConfig.DEFAULTS)
          .build();

  /** Every robot the team has. Add each real robot as it is built and measured. */
  public static List<RobotConfig> all() {
    return List.of(SIM);
  }

  /**
   * The robot this program is running on: the one place that decides, so a new robot is one line
   * here. With several robots, choose by something the robot itself knows (a device's serial
   * number, a file on its storage) rather than by editing this before each deploy.
   */
  public static RobotConfig current() {
    return SIM;
  }
}
