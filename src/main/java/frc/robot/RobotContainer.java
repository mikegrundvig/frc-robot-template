package frc.robot;

import frc.robot.lib.config.Body;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.field.Place;
import frc.robot.lib.field.Placement;
import frc.robot.lib.field.Zone;
import frc.robot.lib.field.ZoneTrigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveSimulation;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.SensorErrors;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;

/**
 * The composition root: builds every mechanism with the right hardware for the current mode, and
 * binds controls to commands. This is the one file that shows how the whole robot fits together, so
 * keep it readable.
 */
public final class RobotContainer implements AutoCloseable {
  private static final int DRIVER_PORT = 0;

  /** The robot loop's period: the simulator's physics steps this far each loop. */
  private static final double LOOP_PERIOD_SECONDS = 0.02;

  /** How far from a start position, and how far turned from it, still counts as on it. */
  static final double START_TOLERANCE_METERS = 0.10;

  static final double START_TOLERANCE_DEGREES = 5;

  /** Zone triggers look this far ahead along the robot's path, and turn off this far outside. */
  static final double ZONE_LOOKAHEAD_SECONDS = 0.4;

  static final double ZONE_EXIT_MARGIN_METERS = 0.1;

  private final FieldModel field;
  private final Map<Zone, ZoneTrigger> zones = new LinkedHashMap<>();

  /** The simulator's field view: the robot, its start, and the field's places. Null in a robot. */
  private final @Nullable Field2d fieldView;

  /** The simulator's physics, which knows where the robot really is. Null unless simulating. */
  private final @Nullable DriveSimulation simulation;

  private final Drive drive;
  private final CommandGamepad driver;
  private final Alert noHardware =
      new Alert(
          "Drive",
          "The drivetrain has no motor-controller code yet: it runs only in simulation. Add a"
              + " ModuleIO for your hardware (subsystems/drive).",
          Alert.Level.HIGH);
  private final Alert allianceUnknown =
      new Alert(
          "Field",
          "allianceUnknown",
          "Alliance unknown: driving assumes the blue drivers' side of the field",
          Alert.Level.MEDIUM);
  private final Alert offField =
      new Alert("Field", "offField", "Odometry puts the robot off the field", Alert.Level.MEDIUM);

  /** This loop's alliance and station (1, 2, 3), from the Driver Station or the field system. */
  private Optional<Alliance> alliance = Optional.empty();

  private OptionalInt location = OptionalInt.empty();

  /** The start chosen for the match, and where it is for the alliance, as of this loop. */
  private Optional<Place> start = Optional.empty();

  private Optional<Pose2d> startPose = Optional.empty();
  private boolean enabledSinceBoot;
  private Placement placement = new Placement(Placement.Status.ALLIANCE_UNKNOWN, null, 0, 0);

  /**
   * In the simulator, the robot's wheels and sensors differ from its configuration as a real
   * robot's do ({@link SensorErrors#TYPICAL}), so its estimated pose drifts from the truth.
   *
   * @param body the robot's frame and mass
   * @param drivetrain the drivetrain's configuration, with any values tuned while running
   * @param field the field the robot plays on
   */
  public RobotContainer(
      Constants.Mode mode,
      Scheduler scheduler,
      Body body,
      Supplier<SwerveConfig> drivetrain,
      FieldModel field) {
    this(mode, scheduler, body, drivetrain, field, SensorErrors.TYPICAL);
  }

  /**
   * @param simulatedErrors in the simulator, how the robot's wheels and sensors differ from its
   *     configuration
   */
  RobotContainer(
      Constants.Mode mode,
      Scheduler scheduler,
      Body body,
      Supplier<SwerveConfig> drivetrain,
      FieldModel field,
      SensorErrors simulatedErrors) {
    this.field = field;
    for (Zone zone : field.zones()) {
      zones.put(zone, new ZoneTrigger(zone, ZONE_LOOKAHEAD_SECONDS, ZONE_EXIT_MARGIN_METERS));
    }
    driver = new CommandGamepad(scheduler, DRIVER_PORT);
    ModuleIO[] modules;
    GyroIO gyro;
    if (mode == Constants.Mode.SIM) {
      // The simulator knows where the robot really is, and its sensors report that with errors,
      // so the robot's own estimate drifts from the truth as a real robot's does.
      simulation =
          new DriveSimulation(
              drivetrain.get().modules(),
              drivetrain.get().moduleTranslations(body),
              simulatedErrors);
      modules = simulation.modules();
      gyro = simulation.gyro();
    } else {
      simulation = null;
      // In replay the inputs come from the log. On a robot there is no hardware code yet, so the
      // drivetrain stays still rather than guessing at motors, and says so.
      modules =
          new ModuleIO[] {
            new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}
          };
      gyro = new GyroIO() {};
      noHardware.set(mode == Constants.Mode.REAL);
    }
    drive = new Drive("Drive", gyro, modules, body, drivetrain, scheduler);
    fieldView = mode == Constants.Mode.SIM ? new Field2d() : null;
    // Until the alliance is known, so is the start: the middle of the field stands in for it.
    Pose2d middle = new Pose2d(field.length() / 2, field.width() / 2, Rotation2d.ZERO);
    if (simulation != null) {
      simulation.place(middle);
    }
    drive.resetPose(middle);

    // Driver: left stick translates (field-relative, from the driver's end of the field), right
    // stick turns.
    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> leftStick().getX(), () -> leftStick().getY(), () -> -driver.getRightX()));
  }

  /**
   * The driver's left stick as a direction on the field. Gamepad Y is negative when pushed forward,
   * so negate it for forward = away from the driver, then turn it the way the driver faces: a red
   * driver's forward is the field's -x.
   */
  private Translation2d leftStick() {
    return new Translation2d(-driver.getLeftY(), -driver.getLeftX())
        .rotateBy(FieldModel.driverForward(alliance.orElse(Alliance.BLUE)));
  }

  /** Mechanism periodic methods, called once per loop before the command scheduler runs. */
  public void periodic() {
    DriveSimulation sim = simulation;
    if (sim != null) {
      // The world moves, then the robot reads its sensors, as on a robot between loops.
      sim.step(LOOP_PERIOD_SECONDS);
    }
    boolean enabled = RobotState.isEnabled();
    alliance = MatchState.getAlliance();
    location = MatchState.getLocation();
    Optional<Place> chosen = chosenStart();
    Optional<Pose2d> chosenPose = alliance.flatMap(ours -> chosen.map(s -> s.on(field, ours)));
    // The robot stands where the drive team put it: on the chosen start. It is put there whenever
    // the choice changes before the match or while disabled (so changing the alliance station in
    // the simulator moves it), never while it drives. With cameras, the tags will say instead.
    Optional<Pose2d> placed =
        !chosenPose.equals(startPose) && (!enabled || !enabledSinceBoot)
            ? chosenPose
            : Optional.empty();
    if (sim != null) {
      placed.ifPresent(sim::place); // in the simulator, the drive team really moves it
    }
    start = chosen;
    startPose = chosenPose;
    enabledSinceBoot |= enabled;

    drive.periodic(enabled);
    // Told where it is after reading its gyro, so a turn made in placing it isn't counted twice.
    placed.ifPresent(drive::resetPose);
    Pose2d pose = drive.getPose();
    placement =
        Placement.check(
            field,
            alliance,
            pose,
            start.orElse(null),
            START_TOLERANCE_METERS,
            START_TOLERANCE_DEGREES);
    allianceUnknown.set(alliance.isEmpty());
    offField.set(!field.contains(pose.getTranslation()));
    updateZones(pose);
    show(pose);
  }

  /**
   * The start chosen for the match. Until autos choose theirs, the Driver Station's location picks
   * it: stations 1, 2, and 3 choose the field's first, second, and third start.
   */
  private Optional<Place> chosenStart() {
    List<Place> starts = field.starts();
    if (location.isEmpty() || location.getAsInt() < 1 || location.getAsInt() > starts.size()) {
      return Optional.empty();
    }
    return Optional.of(starts.get(location.getAsInt() - 1));
  }

  private void updateZones(Pose2d pose) {
    ChassisVelocities robotRelative = drive.getRobotRelativeVelocities();
    Translation2d velocity =
        new Translation2d(robotRelative.vx, robotRelative.vy).rotateBy(pose.getRotation());
    for (ZoneTrigger trigger : zones.values()) {
      boolean active =
          alliance.isPresent()
              && trigger.update(field, alliance.get(), pose.getTranslation(), velocity);
      Logger.recordOutput("Field/Zones/" + trigger.zone().name(), active);
    }
  }

  /** Logs the robot's place on the field, and shows it in the simulator's field view. */
  private void show(Pose2d pose) {
    Logger.recordOutput("Field/Alliance", allianceStation());
    Logger.recordOutput("Field/Start", startPose.stream().toArray(Pose2d[]::new));
    Logger.recordOutput("Field/Placement", placement.describe());
    for (Place place : field.places()) {
      Logger.recordOutput(
          "Field/Places/" + place.name(),
          alliance.stream().map(ours -> place.on(field, ours)).toArray(Pose2d[]::new));
    }
    DriveSimulation sim = simulation;
    if (sim != null) {
      // How far the robot's estimate is from the truth: what localization has to close.
      Pose2d truth = sim.truePose();
      Logger.recordOutput("Sim/TruePose", truth);
      Logger.recordOutput(
          "Sim/PoseErrorMeters", truth.getTranslation().getDistance(pose.getTranslation()));
      Logger.recordOutput(
          "Sim/HeadingErrorDegrees", pose.getRotation().minus(truth.getRotation()).getDegrees());
    }
    if (fieldView != null) {
      fieldView.setRobotPose(pose);
      if (sim != null) {
        fieldView.getObject("Truth").setPose(sim.truePose());
      }
      fieldView.getObject("Start").setPoses(startPose.stream().toList());
      for (Place place : field.places()) {
        fieldView
            .getObject(place.name())
            .setPoses(alliance.stream().map(ours -> place.on(field, ours)).toList());
      }
      Telemetry.log("Field", fieldView);
    }
  }

  /** The alliance and station, such as "Red 2", or "Unknown". */
  String allianceStation() {
    if (alliance.isEmpty()) {
      return "Unknown";
    }
    String name = alliance.get() == Alliance.RED ? "Red" : "Blue";
    return location.isPresent() ? name + " " + location.getAsInt() : name;
  }

  /**
   * Whether a zone's trigger is on: the robot is in the zone, or will be within {@link
   * #ZONE_LOOKAHEAD_SECONDS}. Bind mechanisms to it, such as stowing before a low bar.
   */
  boolean inZone(Zone zone) {
    ZoneTrigger trigger = zones.get(zone);
    return alliance.isPresent() && trigger != null && trigger.isActive();
  }

  /** The pre-match placement check, as of the last loop. */
  Placement placement() {
    return placement;
  }

  /** A pit check for the Pitwall page: drive forward slowly for two seconds. */
  Command driveCheck() {
    return drive.driveForward(0.5, 2.0);
  }

  Drive drive() {
    return drive;
  }

  /** The simulator's physics, or null outside the simulator. */
  @Nullable DriveSimulation simulation() {
    return simulation;
  }

  /** Whether the alert saying the drivetrain has no hardware code is active. */
  boolean isNoHardwareAlertActive() {
    return noHardware.get();
  }

  /** Releases what holds global resources (alerts), so a test can build another robot. */
  @Override
  public void close() {
    drive.close();
    noHardware.close();
    allianceUnknown.close();
    offField.close();
    if (fieldView != null) {
      fieldView.close();
    }
  }
}
