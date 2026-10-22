package frc.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.localization.EstimatorConfig;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.runtime.RobotConfig;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CanBus;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.ImuMount;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.SwerveWiring;
import java.util.List;
import org.wpilib.units.measure.Angle;

/**
 * The swerve test chassis: a bare square frame on four SDS MK5n swerve modules, with nothing else
 * on it. Each module has two Kraken motors, run by the Talon FX built into each one: one drives the
 * wheel and one steers it. A CANcoder on top of each module reads which way its wheel points. A
 * Pigeon 2 tells the robot which way it faces. All of them are on one CANivore.
 *
 * <p>Run it in the simulator: {@code ./gradlew run}. Drive the real chassis from a laptop, with its
 * CANivore plugged into the laptop: {@code ./gradlew run -PhwSim} (runtime/docs/bench.md says how,
 * and what to check first).
 *
 * <p>Read it from the top. First come the values to check on the real chassis, all in one block.
 * Then what the chassis is: its frame and its modules. Last, how the program drives it, with
 * current limits kept low for the bench.
 */
public final class ChassisRobot {
  private ChassisRobot() {}

  // ==== CHECK THESE ON THE CHASSIS ===============================================================
  //
  // Every value in this block is copied from the 2026 robot's code, not read off this chassis, so
  // each one is marked "Unconfirmed". Check each in Phoenix Tuner X: its device list shows every
  // device's CAN ID and the CANivore it's on. If a value differs, change it here. Once a value is
  // checked, change its "Unconfirmed" to "Checked" and the date.

  /** Unconfirmed: the CANivore's name, as Tuner X shows it. */
  static final String CANIVORE_NAME = "canivore";

  /** Unconfirmed: the Pigeon 2's CAN ID. */
  static final int PIGEON_ID = 1;

  // Each module's two motors and its CANcoder. Unconfirmed: all twelve CAN IDs.
  static final int FRONT_LEFT_DRIVE_ID = 23;
  static final int FRONT_LEFT_STEER_ID = 24;
  static final int FRONT_LEFT_CANCODER_ID = 12;

  static final int FRONT_RIGHT_DRIVE_ID = 21;
  static final int FRONT_RIGHT_STEER_ID = 22;
  static final int FRONT_RIGHT_CANCODER_ID = 11;

  static final int BACK_LEFT_DRIVE_ID = 25;
  static final int BACK_LEFT_STEER_ID = 26;
  static final int BACK_LEFT_CANCODER_ID = 13;

  static final int BACK_RIGHT_DRIVE_ID = 27;
  static final int BACK_RIGHT_STEER_ID = 28;
  static final int BACK_RIGHT_CANCODER_ID = 14;

  // Unconfirmed: each module's zero, the CANcoder offset that makes it read 0 with its wheel
  // pointing straight ahead. These are the 2026 robot's, so they will be wrong here: measure them
  // again. In Tuner X, set the CANcoder's magnet offset to 0, point the wheel straight ahead (all
  // four wheels turned the same way round), read its absolute position, and write minus that here.
  static final Angle FRONT_LEFT_ZERO = Rotations.of(0.10693359375);
  static final Angle FRONT_RIGHT_ZERO = Rotations.of(-0.031494140625);
  static final Angle BACK_LEFT_ZERO = Rotations.of(-0.12939453125);
  static final Angle BACK_RIGHT_ZERO = Rotations.of(-0.38427734375);

  // Unconfirmed: which drive motors run backward. With the chassis on blocks, drive forward slowly:
  // every wheel should roll forward. A wheel that rolls backward is on a side to flip here.
  static final boolean LEFT_DRIVES_INVERTED = false;
  static final boolean RIGHT_DRIVES_INVERTED = true;

  // Unconfirmed: the drive gear ratio, motor turns for one turn of the wheel. An MK5n's ratio is
  // set by the pinion gear on its drive motor. This is the 2026 robot's, 5.27 to 1; the other 2026
  // robot's code has 6.03 to 1, a smaller pinion. Look at the pinion, or turn a wheel once by hand
  // and count the drive motor's turns in Tuner X (its rotor position).
  static final double DRIVE_GEAR_RATIO = 5.2734375;

  // Unconfirmed: the steering motor. SDS sells the MK5n for a Kraken X44 on its steering, as the
  // other 2026 robot's Tuner X project has it. An X44 is narrower than the X60 that drives the
  // wheel (44 mm across, against 60 mm), and its label says which it is.
  static final Motor STEER_MOTOR = Motor.KRAKEN_X44;

  // ==== END OF THE VALUES TO CHECK ===============================================================

  /**
   * The frame: about 26 inches square, from where the 2026 robot's code puts its modules, 10.375
   * inches from the middle each way, with a module's wheel 2.625 inches in from the frame's edge.
   * Measure it and change it if it's different.
   *
   * <p>It has no bumpers on the bench, but the robot's model needs them, so these are the 3 inch
   * bumpers it would wear. Only the simulator uses them, to know how big the robot is.
   *
   * <p>It hasn't been weighed yet, so the weight is left out and the program takes it as heavy as
   * the season allows, and says so at startup. Weigh it with its battery on, and add what the scale
   * says: {@code .weight(Pounds.of(42.5))}. How hard it is to spin isn't measured either, so the
   * model works it out as a solid box's of that weight and size.
   */
  public static final Body BODY =
      Body.builder()
          .frameLength(Inches.of(26))
          .frameWidth(Inches.of(26))
          .overallHeight(Inches.of(8))
          .bumperThickness(Inches.of(3))
          .bumperBottom(Inches.of(2.5))
          .bumperTop(Inches.of(5.75))
          .build();

  /**
   * One SDS MK5n module, all four the same. A Kraken X60 drives the wheel, without FOC: FOC needs a
   * Phoenix Pro license on the motor. The steering turns 287/11 (about 26.09) times for one turn of
   * the module, as SDS says. The wheels are 4 inches across, so 2 inches from the middle to the
   * tread.
   *
   * <p>Turning a module also turns its wheel a little, through the gears inside it: 3.375 turns of
   * the drive motor for each turn of the module, the 2026 robot's number for its 5.27 to 1 modules.
   * The program takes that out of the wheel's distance. Change it with the drive gear ratio: it's
   * the drive gear ratio divided by 1.5625 (25/16), so 3.857 for a 6.03 to 1 module.
   *
   * <p>How hard the wheel and the steering are to spin up, 0.01 kg m² each, is Tuner X's guess,
   * which only the simulator uses.
   */
  public static final DrivetrainGeometry.ModuleType MODULE =
      DrivetrainGeometry.ModuleType.builder()
          .driveMotor(Motor.KRAKEN_X60)
          .steerMotor(STEER_MOTOR)
          .wheelRadius(Inches.of(2))
          .driveGearing(Gearing.of(DRIVE_GEAR_RATIO))
          .steerGearing(Gearing.of(287.0 / 11.0))
          .couplingRatio(DRIVE_GEAR_RATIO / (25.0 / 16.0))
          .driveInertia(KilogramSquareMeters.of(0.01))
          .steerInertia(KilogramSquareMeters.of(0.01))
          .build();

  /** The CANivore everything is on. */
  public static final CanBus CANIVORE = CanBus.canivore(CANIVORE_NAME);

  /** The front left module's motors and CANcoder. Its steering motor isn't turned around. */
  private static final SwerveWiring.ModuleWiring FRONT_LEFT =
      SwerveWiring.ModuleWiring.builder()
          .driveId(FRONT_LEFT_DRIVE_ID)
          .steerId(FRONT_LEFT_STEER_ID)
          .encoderOffset(FRONT_LEFT_ZERO)
          .driveInverted(LEFT_DRIVES_INVERTED)
          .steerInverted(false)
          .encoderInverted(false)
          .encoderId(FRONT_LEFT_CANCODER_ID)
          .build();

  /** The front right module's. */
  private static final SwerveWiring.ModuleWiring FRONT_RIGHT =
      SwerveWiring.ModuleWiring.builder()
          .driveId(FRONT_RIGHT_DRIVE_ID)
          .steerId(FRONT_RIGHT_STEER_ID)
          .encoderOffset(FRONT_RIGHT_ZERO)
          .driveInverted(RIGHT_DRIVES_INVERTED)
          .steerInverted(false)
          .encoderInverted(false)
          .encoderId(FRONT_RIGHT_CANCODER_ID)
          .build();

  /** The back left module's. */
  private static final SwerveWiring.ModuleWiring BACK_LEFT =
      SwerveWiring.ModuleWiring.builder()
          .driveId(BACK_LEFT_DRIVE_ID)
          .steerId(BACK_LEFT_STEER_ID)
          .encoderOffset(BACK_LEFT_ZERO)
          .driveInverted(LEFT_DRIVES_INVERTED)
          .steerInverted(false)
          .encoderInverted(false)
          .encoderId(BACK_LEFT_CANCODER_ID)
          .build();

  /** The back right module's. */
  private static final SwerveWiring.ModuleWiring BACK_RIGHT =
      SwerveWiring.ModuleWiring.builder()
          .driveId(BACK_RIGHT_DRIVE_ID)
          .steerId(BACK_RIGHT_STEER_ID)
          .encoderOffset(BACK_RIGHT_ZERO)
          .driveInverted(RIGHT_DRIVES_INVERTED)
          .steerInverted(false)
          .encoderInverted(false)
          .encoderId(BACK_RIGHT_CANCODER_ID)
          .build();

  /**
   * The four modules, 10.375 inches forward or back of the middle and 10.375 inches left or right,
   * and how they're wired: Talon FXs on the CANivore, the Pigeon 2 among them. The Pigeon reads
   * turns at their true size (a yaw scale of 1) until it's checked by spinning the chassis ten
   * times (runtime/docs/calibration.md).
   */
  public static final DrivetrainGeometry DRIVETRAIN =
      DrivetrainGeometry.uniform(MODULE, Inches.of(10.375), Inches.of(10.375))
          .wired(
              SwerveWiring.builder()
                  .canBus(CANIVORE)
                  .driveController(MechanismWiring.Controller.TALON_FX)
                  .steerController(MechanismWiring.Controller.TALON_FX)
                  .gyroId(PIGEON_ID)
                  .gyroYawScale(1.0)
                  .modules(List.of(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT))
                  .build());

  /**
   * The chassis as a whole: its frame, its drivetrain, and its Pigeon 2, taken as sitting in the
   * middle, level, until its place is measured. No cameras, joints or PDH: the chassis has none. No
   * backup IMU either: that's Systemcore's own, and the chassis has no Systemcore yet.
   */
  public static final RobotDefinition ROBOT =
      RobotDefinition.builder()
          .name("chassis")
          .body(BODY)
          .drivetrain(DRIVETRAIN)
          .imus(Imus.builder().gyro(ImuMount.UNCHECKED).build())
          .build();

  /**
   * Current limits for the bench: low on purpose, while the chassis is on blocks. A wheel spinning
   * in the air needs only a few amps, so these leave plenty, and a jammed wheel or a wrong setting
   * can't draw much. Raise them on purpose before driving on the floor (the guide's robot uses 30 A
   * on the drive's stator and 40 A on the steering's: {@code ExampleRobot}).
   *
   * <p>The top speeds are low too: 1.5 meters a second, and half a turn a second.
   */
  public static final DrivetrainControl.Limits BENCH_LIMITS =
      DrivetrainControl.Limits.builder()
          .driveStatorCurrent(Amps.of(20))
          .driveSupplyCurrent(Amps.of(20))
          .driveSupplyLowerCurrent(Amps.of(10))
          .driveSupplyLowerTime(Seconds.of(1))
          .steerStatorCurrent(Amps.of(20))
          .maxLinearSpeed(MetersPerSecond.of(1.5))
          .maxAngularSpeed(RotationsPerSecond.of(0.5))
          .build();

  /**
   * How hard the motors work to follow what they're asked. The drive's kV is what the X60 and the
   * gearing should need, worked out from their numbers. Its kS and kP, and the steering's kP and
   * kD, are the 2026 robot's Tuner X gains, changed into this program's units (Tuner X's are per
   * turn; these are per meter and per radian). Tune them on the chassis, live from Pitwall's Tune
   * view, once tuning is switched on there.
   */
  public static final DrivetrainControl.Gains GAINS =
      DrivetrainControl.Gains.builder()
          .driveKs(0.14)
          .driveKv(MODULE.idealDriveKv())
          .driveKp(0.3)
          .steerKp(16.0)
          .steerKd(0.08)
          .skewLookahead(Seconds.of(0.03))
          .headingKp(5.0)
          .translationKp(3.0)
          .build();

  /**
   * The chassis as the program runs it, the robot {@link Robots#current} runs unless another is
   * asked for by name. Its cameras' and pose estimator's settings are the template's own: it has no
   * cameras.
   */
  public static final RobotConfig CONFIG =
      RobotConfig.builder()
          .definition(
              ROBOT.with(
                  DRIVETRAIN.control(
                      DrivetrainControl.builder().limits(BENCH_LIMITS).gains(GAINS).build())))
          .vision(VisionConfig.DEFAULTS)
          .estimator(EstimatorConfig.DEFAULTS)
          .build();
}
