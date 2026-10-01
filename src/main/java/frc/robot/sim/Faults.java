package frc.robot.sim;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.pitwall.Limits;
import com.michaelgrundvig.frc.pitwall.simulator.Component;
import com.michaelgrundvig.frc.pitwall.simulator.Components;
import com.michaelgrundvig.frc.pitwall.simulator.Fault;
import com.michaelgrundvig.frc.pitwall.simulator.Values;
import frc.robot.lib.field.Bump;
import frc.robot.subsystems.coprocessors.CoprocessorIOSim;
import frc.robot.subsystems.vision.VisionSim;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * What can go wrong with the simulated world's parts while it runs, declared to Pitwall's simulator
 * controls: each camera, the gyro, the backup IMU, each module, the robot itself, the field's tags,
 * and each coprocessor, with their faults. The page lists them, applies them, and undoes them; the
 * scenarios ({@link Scenarios}) are made of them. The robot's code is never told: it has to find
 * out from its sensors.
 *
 * <p>A fault that lasts (a camera unplugged) says whether it's on by asking the world, and undoing
 * it puts the part back as the world was built: a season-worn robot's sensors ({@link
 * SensorErrors}), not perfect ones. An event (a hit) happens once; nothing undoes it, and each says
 * how the robot recovers.
 *
 * <p>Names are the page's, and the scenarios' ({@link #GYRO}, {@link #UNPLUGGED}). What the world
 * can't make fail yet is listed in docs/simulator.md.
 */
public final class Faults {
  // The parts.
  public static final String GYRO = "Gyro";
  public static final String BACKUP_IMU = "Backup IMU";
  public static final String ROBOT = "Robot";
  public static final String FIELD = "Field";

  /** The modules' names, front left first, in the drivetrain's order. */
  static final List<String> MODULES =
      List.of("Front left module", "Front right module", "Back left module", "Back right module");

  // What can go wrong with them.
  public static final String UNPLUGGED = "Unplugged";
  public static final String FROZEN = "Frozen";
  public static final String MOVED = "Moved on its mount";
  public static final String TURNED = "Turned on its mount";
  public static final String LATE = "Late";
  public static final String ERRORS = "Errors";
  public static final String RESET = "Reset";
  public static final String STALE = "Stale encoder";
  public static final String ZEROED_OFF = "Zeroed off";
  public static final String WORN = "Worn tread";
  public static final String SLIPS = "Slips";
  public static final String SHOVED = "Shoved";
  public static final String HIT = "Hit";
  public static final String TILTED = "Tilted";
  public static final String SLIPPING = "Slipping";
  public static final String PLACED = "Placed";
  public static final String TO_A_BUMP = "Carried to a bump";
  public static final String TAG_MOVED = "Tag moved";
  public static final String UNREACHABLE = "Unreachable";
  public static final String WRONG_VERSION = "Wrong version";
  public static final String HOT = "Hot";
  public static final String CAMERA_UNPLUGGED = "Camera unplugged";
  public static final String PHOTONVISION_RESTARTS = "PhotonVision restarts";
  public static final String DUPLICATED = "Duplicated address";
  public static final String SETTINGS_CHANGED = "Settings changed";

  // Their parameters, as the scenarios give them.
  public static final String FORWARD = "Forward";
  public static final String LEFT = "Left";
  public static final String UP = "Up";
  public static final String YAW = "Yaw";
  public static final String PITCH = "Pitch";
  public static final String ROLL = "Roll";
  public static final String SCALE = "Scale";
  public static final String DRIFT = "Drift";
  public static final String NOISE = "Noise";
  public static final String OFFSET = "Offset";
  public static final String TURNS = "Turns";
  public static final String GRIP = "Grip";
  public static final String EXTRA = "Extra";
  public static final String TURN = "Turn";
  public static final String X = "X";
  public static final String Y = "Y";
  public static final String HEADING = "Heading";
  public static final String TAG = "Tag";
  public static final String BELOW_TRIP = "Below its trip point";
  public static final String CAMERA = "Camera";

  /** How far short of a bump's ramp the robot is set down, from its center: clear of the ramp. */
  static final double BUMP_CLEARANCE_METERS = 1.0;

  /** The farthest the page may knock a camera, or push the robot, in one go. */
  private static final double MOUNT_INCHES = 12;

  private static final double MOUNT_DEGREES = 30;
  private static final double SHOVE_METERS = 3;

  private static final String RECOVERS =
      "Nothing undoes it: place the robot again, or let its cameras find it";

  private Faults() {}

  /** A camera's name on the page: {@code "Camera front-left"}. */
  public static String camera(String name) {
    return "Camera " + name;
  }

  /** A coprocessor's name on the page: {@code "Coprocessor vision-front"}. */
  public static String coprocessor(String name) {
    return "Coprocessor " + name;
  }

  /**
   * A module's name on the page: {@code "Front left module"}.
   *
   * @param module front left, front right, back left, back right: 0 to 3
   */
  public static String module(int module) {
    return MODULES.get(module);
  }

  /** Declares every part of the world that can fail. */
  static void declare(SimulatedWorld world, Components components) {
    for (int i = 0; i < world.cameraCount(); i++) {
      components.add(camera(world.camera(i)));
    }
    DriveSimulation drivetrain = world.drivetrain();
    components.add(imu(GYRO, "gyro", drivetrain.gyroSim()));
    components.add(imu(BACKUP_IMU, "backup IMU", drivetrain.backupGyroSim()));
    for (int i = 0; i < MODULES.size(); i++) {
      components.add(module(drivetrain, i));
    }
    components.add(robot(world));
    if (!world.vision().tagIds().isEmpty()) {
      components.add(field(world.vision()));
    }
    for (CoprocessorIOSim coprocessor : world.coprocessorSims()) {
      components.add(coprocessor(coprocessor));
    }
  }

  /** A coprocessor, as its health agent tells the robot of it. */
  private static Component coprocessor(CoprocessorIOSim coprocessor) {
    List<Fault> faults = new ArrayList<>();
    faults.add(
        Fault.lasting(UNREACHABLE)
            .does(
                "Its health agent stops answering, as a coprocessor unplugged, off, or off the"
                    + " network: the robot's code finds it missing.")
            .isOn(coprocessor::isUnreachable)
            .apply(values -> coprocessor.unreachable(true))
            .undo("Answers again.", () -> coprocessor.unreachable(false))
            .build());
    faults.add(
        Fault.lasting(WRONG_VERSION)
            .does(
                "Its image carries PhotonVision "
                    + CoprocessorIOSim.OTHER_VERSION
                    + ", as a board left on last season's image would, and it publishes that"
                    + " version.")
            .isOn(coprocessor::hasWrongVersion)
            .apply(values -> coprocessor.wrongVersion(true))
            .undo("Back to this build's image.", () -> coprocessor.wrongVersion(false))
            .build());
    faults.add(
        Fault.lasting(HOT)
            .does(
                "Its big cores run this far below their trip point; at it or past it, the kernel"
                    + " caps their clock.")
            .parameter(BELOW_TRIP, "°C", Limits.between(-10, 30), 2)
            .isOn(coprocessor::isHot)
            .apply(values -> coprocessor.heat(values.get(BELOW_TRIP)))
            .undo("Back to its usual temperature.", () -> coprocessor.heat(Double.NaN))
            .build());
    if (coprocessor.cameraCount() > 0) {
      faults.add(
          Fault.lasting(CAMERA_UNPLUGGED)
              .does(
                  "One of its cameras, numbered from 1 in the table's order, is unplugged: its USB"
                      + " port is empty.")
              .parameter(CAMERA, "", Limits.between(1, coprocessor.cameraCount()), 1)
              .isOn(coprocessor::isCameraUnplugged)
              .apply(values -> coprocessor.unplugCamera(cameraIndex(values.get(CAMERA))))
              .undo("Plugs it back in.", () -> coprocessor.unplugCamera(-1))
              .build());
    }
    faults.add(
        Fault.event(PHOTONVISION_RESTARTS)
            .does("PhotonVision exits, and systemd restarts it: it's back after a few seconds.")
            .apply(values -> coprocessor.restartPhotonVision())
            .undoneBy(
                "Nothing undoes it: the count of restarts since boot stays, until it reboots.")
            .build());
    faults.add(
        Fault.lasting(DUPLICATED)
            .does(
                "A second board with its image answers at its address too, the two taking turns,"
                    + " as a spare drive in another board would.")
            .isOn(coprocessor::hasTwoBoards)
            .apply(values -> coprocessor.twoBoards(true))
            .undo("Only its own board answers.", () -> coprocessor.twoBoards(false))
            .build());
    faults.add(
        Fault.lasting(SETTINGS_CHANGED)
            .does(
                "Its PhotonVision settings change from those it was stamped with, as calibrating or"
                    + " tuning in PhotonVision's own page does: they differ from Git's, if Git has"
                    + " any.")
            .isOn(coprocessor::hasSettingsChanged)
            .apply(values -> coprocessor.settingsChanged(true))
            .undo("Back to its stamped settings.", () -> coprocessor.settingsChanged(false))
            .build());
    return new Component(coprocessor(coprocessor.name()), "coprocessor", faults);
  }

  /** A camera's number, from 1, as an index from 0. */
  private static int cameraIndex(double number) {
    if (number != Math.rint(number)) {
      throw new IllegalArgumentException("a camera's number is a whole number, not " + number);
    }
    return (int) number - 1;
  }

  private static Component camera(CameraIOSim camera) {
    return Component.of(
        camera(camera.name()),
        "camera",
        Fault.lasting(UNPLUGGED)
            .does("No results at all: the robot's code sees it disconnected.")
            .isOn(camera::isUnplugged)
            .apply(values -> camera.unplug(true))
            .undo("Plugs it back in.", () -> camera.unplug(false))
            .build(),
        Fault.lasting(FROZEN)
            .does(
                "Keeps sending its last image's tags as new ones, at its frame rate: to the robot's"
                    + " code, the robot seems not to move.")
            .isOn(camera::isFrozen)
            .apply(values -> camera.freeze(true))
            .undo("Sends new images again.", () -> camera.freeze(false))
            .build(),
        Fault.lasting(MOVED)
            .does(
                "Knocked this far from where it's mounted, in the robot's frame (forward, left,"
                    + " up); the robot's code still believes its configured mount.")
            .parameter(FORWARD, "in", Limits.between(-MOUNT_INCHES, MOUNT_INCHES), 0)
            .parameter(LEFT, "in", Limits.between(-MOUNT_INCHES, MOUNT_INCHES), 0)
            .parameter(UP, "in", Limits.between(-MOUNT_INCHES, MOUNT_INCHES), -4)
            .isOn(camera::isMoved)
            .apply(
                values ->
                    camera.move(
                        new Translation3d(
                            Inches.of(values.get(FORWARD)),
                            Inches.of(values.get(LEFT)),
                            Inches.of(values.get(UP)))))
            .undo("Puts it back where it was mounted.", () -> camera.move(Translation3d.ZERO))
            .build(),
        Fault.lasting(TURNED)
            .does(
                "Turned this far from how it's mounted, in the robot's frame; the robot's code"
                    + " still believes its configured mount.")
            .parameter(ROLL, "°", Limits.between(-MOUNT_DEGREES, MOUNT_DEGREES), 0)
            .parameter(PITCH, "°", Limits.between(-MOUNT_DEGREES, MOUNT_DEGREES), 0)
            .parameter(YAW, "°", Limits.between(-MOUNT_DEGREES, MOUNT_DEGREES), 2)
            .isOn(camera::isTurned)
            .apply(
                values ->
                    camera.turn(
                        new Rotation3d(
                            Degrees.of(values.get(ROLL)),
                            Degrees.of(values.get(PITCH)),
                            Degrees.of(values.get(YAW)))))
            .undo("Turns it back as it was mounted.", () -> camera.turn(Rotation3d.ZERO))
            .build(),
        Fault.lasting(LATE)
            .does(
                "Its images arrive this much later than its configuration says, still carrying"
                    + " when they were taken; past its frame time, it sends fewer of them.")
            .parameter(EXTRA, "ms", Limits.between(0, 1000), 100)
            .isOn(camera::isLate)
            .apply(values -> camera.delay(values.get(EXTRA)))
            .undo("Back to its usual latency.", () -> camera.delay(0))
            .build());
  }

  /** The gyro, or the backup IMU: they fail alike. */
  private static Component imu(String name, String kind, GyroIOSim imu) {
    SensorErrors.GyroErrors built = imu.built();
    return Component.of(
        name,
        kind,
        Fault.lasting(UNPLUGGED)
            .does("No readings at all: the robot's code sees it disconnected.")
            .isOn(() -> !imu.isConnected())
            .apply(values -> imu.setConnected(false))
            .undo("Plugs it back in.", () -> imu.setConnected(true))
            .build(),
        Fault.lasting(FROZEN)
            .does("Keeps reporting, the same yaw every time, however the robot turns.")
            .isOn(imu::isFrozen)
            .apply(values -> imu.setFrozen(true))
            .undo("Reports the robot's turns again.", () -> imu.setFrozen(false))
            .build(),
        Fault.lasting(ERRORS)
            .does(
                "Errs this way from now on: its reading per true turn (1.15 reads 15% too much),"
                    + " a steady drift, and noise on each reading. It carries on from what it"
                    + " reads now.")
            .parameter(SCALE, "", around(0.5, 1.5, built.scale()), built.scale())
            .parameter(
                DRIFT,
                "°/min",
                around(-10, 10, built.driftDegreesPerMinute()),
                built.driftDegreesPerMinute())
            .parameter(NOISE, "°", around(0, 2, built.noiseDegrees()), built.noiseDegrees())
            .isOn(() -> !imu.errors().equals(built))
            .apply(
                values ->
                    imu.setErrors(
                        SensorErrors.GyroErrors.builder()
                            .scale(values.get(SCALE))
                            .driftDegreesPerMinute(values.get(DRIFT))
                            .noiseDegrees(values.get(NOISE))
                            .build()))
            .undo("Errs as it did as the robot was built.", () -> imu.setErrors(built))
            .build(),
        Fault.event(RESET)
            .does(
                "Its yaw resets to zero wherever the robot points, as a gyro that browns out and"
                    + " restarts does.")
            .apply(values -> imu.reset())
            .undoneBy("Nothing undoes it: the robot's code has to notice the jump.")
            .build());
  }

  private static Component module(DriveSimulation drivetrain, int index) {
    ModuleIOSim module = drivetrain.moduleSim(index);
    double builtOffset = Math.toDegrees(module.builtSteerOffsetRadians());
    double builtTread = drivetrain.builtTreadScale(index);
    return Component.of(
        module(index),
        "module",
        Fault.lasting(STALE)
            .does(
                "Its encoders' readings stop changing while the wheel turns on, as signals that"
                    + " stop arriving do: odometry thinks the wheel stands still, pointing where it"
                    + " last did.")
            .isOn(module::isStale)
            .apply(values -> module.setStale(true))
            .undo("Its readings catch up with the wheel.", () -> module.setStale(false))
            .build(),
        Fault.lasting(ZEROED_OFF)
            .does(
                "Its absolute encoder's zero this far off: the module points that far from where"
                    + " the robot's code thinks.")
            .parameter(OFFSET, "°", around(-45, 45, builtOffset), builtOffset)
            .isOn(
                () ->
                    Math.abs(module.steerOffsetRadians() - module.builtSteerOffsetRadians()) > 1e-9)
            .apply(values -> module.setSteerOffset(Math.toRadians(values.get(OFFSET))))
            .undo(
                "Back to its zero as the robot was built.",
                () -> module.setSteerOffset(module.builtSteerOffsetRadians()))
            .build(),
        Fault.lasting(WORN)
            .does(
                "Its wheel's true radius over the configured one: 0.97 is tread worn 3%, and the"
                    + " robot goes 3% less far than this wheel's encoder counts.")
            .parameter(SCALE, "", around(0.8, 1.1, builtTread), builtTread)
            .isOn(() -> drivetrain.treadScale(index) != builtTread)
            .apply(values -> drivetrain.wearTread(index, values.get(SCALE)))
            .undo(
                "Back to its tread as the robot was built.",
                () -> drivetrain.wearTread(index, builtTread))
            .build(),
        Fault.event(SLIPS)
            .does(
                "Spins this many turns without moving the robot: its encoder counts travel the"
                    + " floor never saw.")
            .parameter(TURNS, "turns", Limits.between(-10, 10), 0.5)
            .apply(values -> drivetrain.spin(index, 2 * Math.PI * values.get(TURNS)))
            .undoneBy(
                "Nothing undoes it: odometry has to notice the wheels disagreeing, and the cameras"
                    + " correct what it missed.")
            .build());
  }

  private static Component robot(SimulatedWorld world) {
    DriveSimulation drivetrain = world.drivetrain();
    double length = world.field().length();
    double width = world.field().width();
    List<Fault> faults = new ArrayList<>();
    faults.add(
        Fault.event(SHOVED)
            .does(
                "Pushed this far in its own frame without its wheels turning, as another robot"
                    + " pushes it: its gyro feels a turn, and nothing feels the move.")
            .parameter(FORWARD, "m", Limits.between(-SHOVE_METERS, SHOVE_METERS), 0)
            .parameter(LEFT, "m", Limits.between(-SHOVE_METERS, SHOVE_METERS), 0.6)
            .parameter(TURN, "°", Limits.between(-180, 180), 0)
            .apply(
                values ->
                    drivetrain.shove(
                        new Transform2d(
                            values.get(FORWARD),
                            values.get(LEFT),
                            Rotation2d.fromDegrees(values.get(TURN)))))
            .undoneBy(RECOVERS)
            .build());
    faults.add(
        Fault.event(HIT)
            .does(
                "Knocked this far in its own frame, as a collision knocks it, with a 4 g jolt its"
                    + " IMU feels.")
            .parameter(FORWARD, "m", Limits.between(-SHOVE_METERS, SHOVE_METERS), 0)
            .parameter(LEFT, "m", Limits.between(-SHOVE_METERS, SHOVE_METERS), 0.6)
            .apply(
                values ->
                    drivetrain.hit(
                        new Transform2d(values.get(FORWARD), values.get(LEFT), Rotation2d.ZERO)))
            .undoneBy(RECOVERS)
            .build());
    faults.add(
        Fault.lasting(TILTED)
            .does(
                "Tilted by hand, whatever the ground under it, as a wheel up on a game piece"
                    + " tilts it: nose up is a negative pitch, left side up a positive roll. Its IMU"
                    + " reads the tilt.")
            .parameter(PITCH, "°", Limits.between(-MOUNT_DEGREES, MOUNT_DEGREES), -5)
            .parameter(ROLL, "°", Limits.between(-MOUNT_DEGREES, MOUNT_DEGREES), 5)
            .isOn(drivetrain::isTiltedByHand)
            .apply(
                values ->
                    drivetrain.tilt(
                        Math.toRadians(values.get(PITCH)), Math.toRadians(values.get(ROLL))))
            .undo(
                "Sets it back on the ground, which tilts it as the terrain does.",
                () -> drivetrain.setTerrain(drivetrain.terrain()))
            .build());
    faults.add(
        Fault.lasting(SLIPPING)
            .does(
                "Only this share of its wheels' rolling moves it, as with wheels in the air or on"
                    + " a slick floor; the encoders count all of it, alike, so the wheels never"
                    + " disagree.")
            .parameter(GRIP, "", Limits.between(0, 1), 0.75)
            .isOn(() -> drivetrain.grip() < 1)
            .apply(values -> drivetrain.grip(values.get(GRIP)))
            .undo("Full grip again.", () -> drivetrain.grip(1))
            .build());
    faults.add(
        Fault.event(PLACED)
            .does(
                "Carried here by the drive team, facing this way (from the blue end): its wheels"
                    + " don't turn, and its gyro feels any turn.")
            .parameter(X, "m", Limits.between(0, length), length / 2)
            .parameter(Y, "m", Limits.between(0, width), width / 2)
            .parameter(HEADING, "°", Limits.between(-180, 180), 0)
            .apply(values -> world.place(placed(values)))
            .undoneBy("Place it again.")
            .build());
    if (hasBumps(world)) {
      faults.add(
          Fault.event(TO_A_BUMP)
              .does(
                  "Carried to 1 m short of the nearest bump's ramp, facing across it, to drive"
                      + " over. Its gyro feels the turn but its wheels see no move, so only its"
                      + " cameras can say where it is.")
              .apply(values -> beforeTheNearestBump(world))
              .undoneBy("Place it again.")
              .build());
    }
    return new Component(ROBOT, "robot", faults);
  }

  /** The field's tags: one can be knocked off where the field was built. */
  private static Component field(VisionSim vision) {
    List<Integer> ids = vision.tagIds();
    int lowest = ids.stream().mapToInt(Integer::intValue).min().orElseThrow();
    int highest = ids.stream().mapToInt(Integer::intValue).max().orElseThrow();
    return Component.of(
        FIELD,
        "field",
        Fault.lasting(TAG_MOVED)
            .does(
                "One tag moved this far from where the field was built, in the field's frame (x"
                    + " away from the blue wall, y to its left, up), as a tag knocked at an event"
                    + " is; moving another puts this one back. The robot's code, and the cameras'"
                    + " multi-tag solves, still believe the field's layout.")
            .parameter(TAG, "", Limits.between(lowest, highest), ids.get(0))
            .parameter(X, "in", Limits.between(-MOUNT_INCHES, MOUNT_INCHES), 4)
            .parameter(Y, "in", Limits.between(-MOUNT_INCHES, MOUNT_INCHES), 0)
            .parameter(UP, "in", Limits.between(-MOUNT_INCHES, MOUNT_INCHES), 0)
            .isOn(vision::isTagMoved)
            .apply(
                values ->
                    vision.moveTag(
                        tagId(values.get(TAG)),
                        new Translation3d(
                            Inches.of(values.get(X)),
                            Inches.of(values.get(Y)),
                            Inches.of(values.get(UP)))))
            .undo("Puts it back where the field was built.", vision::putTagsBack)
            .build());
  }

  /** A tag's ID, given as a number. */
  private static int tagId(double id) {
    if (id != Math.rint(id)) {
      throw new IllegalArgumentException("a tag's ID is a whole number, not " + id);
    }
    return (int) id;
  }

  /** Whether the field the world was built on has bumps, to carry the robot to. */
  static boolean hasBumps(SimulatedWorld world) {
    return !world.drivetrain().terrain().bumps().isEmpty();
  }

  private static Pose2d placed(Values values) {
    return new Pose2d(values.get(X), values.get(Y), Rotation2d.fromDegrees(values.get(HEADING)));
  }

  /** A range that holds what the robot was built with, however far out that is. */
  private static Limits around(double min, double max, double built) {
    return Limits.between(Math.min(min, built), Math.max(max, built));
  }

  /**
   * Puts the robot down short of the bump nearest it, on the side it's on, facing across the crest:
   * the drive team carrying it there. Its wheels don't turn, so odometry doesn't see the move; its
   * gyro feels any turn.
   *
   * @throws IllegalStateException on a field without bumps
   */
  static void beforeTheNearestBump(SimulatedWorld world) {
    Translation2d robot = world.truePose().getTranslation();
    Bump bump =
        world.drivetrain().terrain().bumps().stream()
            .min(Comparator.comparingDouble(each -> middle(each).getDistance(robot)))
            .orElseThrow(
                () ->
                    new IllegalStateException(
                        "carrying the robot to a bump needs a field with bumps, and this one has"
                            + " none"));
    boolean onTheAcrossSide = bump.acrossOffset(robot) >= 0;
    double out = bump.depthMeters() / 2 + BUMP_CLEARANCE_METERS;
    Rotation2d across = bump.across();
    Translation2d spot = middle(bump).plus(new Translation2d(onTheAcrossSide ? out : -out, across));
    world.place(new Pose2d(spot, onTheAcrossSide ? across.plus(Rotation2d.PI) : across));
  }

  /** The middle of a bump's crest. */
  private static Translation2d middle(Bump bump) {
    return bump.crestStart().interpolate(bump.crestEnd(), 0.5);
  }
}
