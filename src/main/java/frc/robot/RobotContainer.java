package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.localization.FrameVerdict;
import com.michaelgrundvig.frc.localization.LocalizationEstimate;
import com.michaelgrundvig.frc.localization.LocalizationService;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.SwerveKinematicsModel;
import com.michaelgrundvig.frc.localization.VisionConfig;
import com.michaelgrundvig.frc.pitwall.Actions;
import com.michaelgrundvig.frc.pitwall.ConfigRules;
import com.michaelgrundvig.frc.pitwall.LiveConfig;
import com.michaelgrundvig.frc.pitwall.Pitwall;
import com.michaelgrundvig.frc.pitwall.experiment.Experiment;
import com.michaelgrundvig.frc.pitwall.experiment.Profile;
import com.michaelgrundvig.frc.pitwall.experiment.Profiles;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Placement;
import com.michaelgrundvig.frc.world.field.Zone;
import com.michaelgrundvig.frc.world.field.ZoneTrigger;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.DrivetrainControl;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.SensorWiring;
import frc.robot.commands.Calibration;
import frc.robot.dashboard.Dashboard;
import frc.robot.dashboard.RobotStatus;
import frc.robot.lib.timing.LoopMonitor;
import frc.robot.lib.timing.RobotLoop;
import frc.robot.sim.DriveSimulation;
import frc.robot.sim.Scenarios;
import frc.robot.sim.SimulatedWorld;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveHardware;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOSystemcore;
import frc.robot.subsystems.drive.PhoenixDrivetrain;
import frc.robot.subsystems.localization.Localization;
import frc.robot.subsystems.mechanisms.AbsoluteEncoderIO;
import frc.robot.subsystems.mechanisms.AbsoluteEncoderIOPort;
import frc.robot.subsystems.mechanisms.JointIO;
import frc.robot.subsystems.mechanisms.JointIOSpark;
import frc.robot.subsystems.mechanisms.JointIOTalon;
import frc.robot.subsystems.mechanisms.JointMechanism;
import frc.robot.subsystems.mechanisms.JointSetup;
import frc.robot.subsystems.mechanisms.Joints;
import frc.robot.subsystems.mechanisms.SensorIO;
import frc.robot.subsystems.mechanisms.SensorIOCanRange;
import frc.robot.subsystems.mechanisms.SensorIOCandi;
import frc.robot.subsystems.mechanisms.SensorIOPort;
import frc.robot.subsystems.mechanisms.Sensors;
import frc.robot.subsystems.vision.CameraIO;
import frc.robot.subsystems.vision.CameraIOPhotonVision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.venues.Venues;
import frc.robot.views.world.WorldView;
import java.time.Duration;
import java.time.LocalDate;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandGamepad;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.util.Alert;

/**
 * The composition root: builds every mechanism with the right hardware for the current mode, and
 * binds controls to commands. This is the one file that shows how the whole robot fits together, so
 * keep it readable.
 */
public final class RobotContainer implements AutoCloseable {
  private static final int DRIVER_PORT = 0;

  /** How far from a start position, and how far turned from it, still counts as on it. */
  static final double START_TOLERANCE_METERS = 0.10;

  static final double START_TOLERANCE_DEGREES = 5;

  /** How well a drive team places the robot on its start: within about 5 cm and 2 degrees. */
  static final PoseCovariance PLACED = PoseCovariance.fromStdDevs(0.05, 0.05, Math.toRadians(2));

  /** How straight a driver lines the robot up by eye for a heading reset: about 5 degrees. */
  static final double HEADING_BY_EYE_RADIANS = Math.toRadians(5);

  /** Zone triggers look this far ahead along the robot's path, and turn off this far outside. */
  static final double ZONE_LOOKAHEAD_SECONDS = 0.4;

  static final double ZONE_EXIT_MARGIN_METERS = 0.1;

  private final Scheduler scheduler;

  /**
   * What the pit can tune on a robot: each mechanism's configuration, with the limits it may be
   * tuned within. The one list of them: the container publishes these, and the tests check every
   * one (its committed values within its limits, a meaning for each value, and a row in the values
   * register, docs/values.md). A mechanism with tuning adds it here.
   *
   * @param drivetrain the drivetrain's gains and limits ({@link Drive#tuning})
   * @param vision how far the cameras are trusted ({@link Vision#tuning})
   * @param joints each driven joint's gains and current limits, by its label ({@link
   *     JointMechanism#tuning})
   */
  record Tunings(
      LiveConfig<DrivetrainControl> drivetrain,
      LiveConfig<VisionConfig> vision,
      Map<String, LiveConfig<MechanismControl>> joints) {
    /** Every one. */
    List<LiveConfig<?>> all() {
      List<LiveConfig<?>> all = new ArrayList<>(List.of(drivetrain, vision));
      all.addAll(joints.values());
      return List.copyOf(all);
    }
  }

  /** Each mechanism's tuning, starting from a robot's committed configuration. */
  static Tunings tunings(RobotConfig robot) {
    Map<String, LiveConfig<MechanismControl>> joints = new LinkedHashMap<>();
    for (JointSetup setup : robot.joints()) {
      String label = setup.joint().label();
      joints.put(label, JointMechanism.tuning(label, setup.control()));
    }
    return new Tunings(
        Drive.tuning(robot.drivetrain()), Vision.tuning(robot.vision()), Map.copyOf(joints));
  }

  /** The drivetrain's and vision's configurations as the pit tunes them. */
  private final LiveConfig<DrivetrainControl> drivetrainTuning;

  private final LiveConfig<VisionConfig> visionTuning;

  /** The field the robot believes it plays on: the official layout, or the venue selected. */
  private FieldModel field;

  private final Venues venues;
  private final Map<Zone, ZoneTrigger> zones = new LinkedHashMap<>();

  /** The simulator's field view: the robot, its start, and the field's places. Null in a robot. */
  private final @Nullable Field2d fieldView;

  /** The simulator's world, which knows where the robot really is. Null unless simulating. */
  private final @Nullable SimulatedWorld world;

  private final Localization localization;

  /** What the estimator did with the last loop's camera measurements, for the World view. */
  private List<FrameVerdict> frames = List.of();

  /** What the robot is, as configured, for the World view. */
  private final RobotDefinition robotDefinition;

  private final Drive drive;

  /** Every driven joint's mechanism, and the joints without wiring on a robot. */
  private final Joints joints;

  /** Whether a camera rides on a joint, so localization needs the joints' positions. */
  private final boolean camerasOnJoints;

  private final Alert jointsUnwired;

  /** Sensors with no wiring on a robot: nothing waiting on them holds. */
  private final Alert sensorsUnwired;

  /** Sensors that stopped answering, named: their conditions hold neither way. */
  private final Alert sensorsLost;

  /** Limit switches that disagree with where their joints are: each said, with what to check. */
  private final Alert sensorsDoubted;

  /** What the limit switches were doubted for as of the last loop, as the alert says. */
  private List<String> doubted = List.of();

  /** The sensors lost as of the last loop, as the alert names them. */
  private List<String> lost = List.of();

  /** The sensors with no wiring on a robot: said once, by their own alert, not as lost. */
  private final List<String> unwiredSensors = new ArrayList<>();

  private final Vision vision;
  private final Calibration calibration;
  private final CommandGamepad driver;
  private final Alert noHardware =
      new Alert(
          "Drive",
          "This robot's configuration has no drivetrain wiring (CAN IDs and encoder offsets), so"
              + " the drivetrain stays still. Add it to the robot's entry in Robots.",
          Alert.Level.HIGH);

  /** The drivetrain's CTRE hardware, on a robot whose configuration has its wiring. */
  private final @Nullable PhoenixDrivetrain hardware;

  private final Alert allianceUnknown =
      new Alert(
          "Field",
          "allianceUnknown",
          "Alliance unknown: driving assumes the blue drivers' side of the field",
          Alert.Level.MEDIUM);
  private final Alert offField =
      new Alert("Field", "offField", "Odometry puts the robot off the field", Alert.Level.MEDIUM);

  /** Every loop's time, section by section: the robot marks its sections, this marks its own. */
  private final LoopHealth loopHealth;

  private final LoopMonitor loop;

  /** This loop's alliance and station (1, 2, 3), from the Driver Station or the field system. */
  private Optional<Alliance> alliance = Optional.empty();

  private OptionalInt location = OptionalInt.empty();

  /** The start chosen for the match, and where it is for the alliance, as of this loop. */
  private Optional<Place> start = Optional.empty();

  private Optional<Pose2d> startPose = Optional.empty();
  private boolean enabledSinceBoot;
  private boolean wasAutonomous;
  private Placement placement = new Placement(Placement.Status.ALLIANCE_UNKNOWN, null, 0, 0);

  /**
   * Built by {@link Robot#assemble}, for the robot and for tests alike.
   *
   * @param scheduler runs every command: the controls', and the Pitwall page's actions
   * @param robot the robot: its frame, wiring, and cameras
   * @param field the field the robot plays on, with its official layout
   * @param venues the venue the pit selected, which takes the field's place while disabled
   * @param simulatedErrors in the simulator, how the robot's wheels and sensors differ from its
   *     configuration ({@link SensorErrors#TYPICAL} makes its estimated pose drift from the truth
   *     as a real robot's does)
   * @param loopHealth the loop's timing, which the robot starts and finishes and this marks the
   *     sections of; closed with this
   */
  RobotContainer(
      Constants.Mode mode,
      Scheduler scheduler,
      RobotConfig robot,
      FieldModel field,
      Venues venues,
      SensorErrors simulatedErrors,
      LoopHealth loopHealth) {
    // What the pit can tune, published where dashboards and the Pitwall page find it; the
    // mechanisms read it every loop.
    Tunings tunings = tunings(robot);
    drivetrainTuning = tunings.drivetrain();
    TunableRegistry.publish("Drive/config", drivetrainTuning);
    visionTuning = tunings.vision();
    TunableRegistry.publish("Vision/config", visionTuning);
    tunings
        .joints()
        .forEach((label, tuning) -> TunableRegistry.publish("Joints/" + label + "/config", tuning));
    // The rules across mechanisms: checked on every change the pit makes, and here, as committed,
    // so a configuration that breaks one stops the program with every rule it breaks.
    ConfigRules rules = new ConfigRules();
    RobotDefinition definition = robot.definition();
    RobotRules.register(rules, definition, drivetrainTuning, visionTuning);
    rules.checkCommitted();
    reportDoubts(definition, mode);
    Supplier<DrivetrainControl> drivetrain = drivetrainTuning::get;
    this.scheduler = scheduler;
    this.loopHealth = loopHealth;
    loop = loopHealth.monitor();
    List<CameraConfig> cameras = definition.cameras();
    robotDefinition = definition;
    this.field = field;
    this.venues = venues;
    for (Zone zone : field.zones()) {
      zones.put(zone, new ZoneTrigger(zone, ZONE_LOOKAHEAD_SECONDS, ZONE_EXIT_MARGIN_METERS));
    }
    driver = new CommandGamepad(scheduler, DRIVER_PORT);

    // The drivetrain's hardware: the simulator's, the robot's own, or none (a replay reads the log,
    // and a robot without wiring stays still rather than guess at motors, and says so).
    DriveHardware driveHardware;
    if (mode == Constants.Mode.SIM) {
      world =
          new SimulatedWorld(
              field,
              definition.drivetrain(),
              definition.body(),
              cameras,
              definition.imus(),
              simulatedErrors);
      hardware = null;
      driveHardware = world.drivetrainHardware();
    } else if (mode == Constants.Mode.REAL && robot.drivetrainWiring().isPresent()) {
      world = null;
      PhoenixDrivetrain phoenix =
          new PhoenixDrivetrain(
              robot.drivetrainWiring().get(),
              definition.drivetrain(),
              definition.imus().gyro().orientation());
      hardware = phoenix;
      driveHardware =
          DriveHardware.builder()
              .modules(List.of(phoenix.modules()))
              .gyro(phoenix.gyro())
              .backupImu(
                  definition
                      .imus()
                      .backup()
                      .<GyroIO>map(backup -> new GyroIOSystemcore(backup.orientation()))
                      .orElseGet(() -> new GyroIO() {}))
              .odometryLock(phoenix.odometryLock())
              .build();
    } else {
      world = null;
      hardware = null;
      driveHardware = DriveHardware.none();
      noHardware.set(mode == Constants.Mode.REAL);
    }

    // Until the robot is placed it could be anywhere: localization starts it in the middle of the
    // field, as unsure as the field is big, so the first camera measurement wins. The simulated
    // robot stands there too.
    Pose2d middle = new Pose2d(field.length() / 2, field.width() / 2, Rotation2d.ZERO);
    localization =
        new Localization(
            new LocalizationService(
                field,
                definition,
                new SwerveKinematicsModel(definition.moduleTranslations()),
                robot.estimator(),
                RobotLoop.PERIOD_SECONDS));
    drive =
        new Drive("Drive", driveHardware, definition, drivetrain, scheduler, localization::pose);
    fieldView = mode == Constants.Mode.SIM ? new Field2d() : null;
    if (world != null) {
      world.place(middle);
    }

    // Every driven joint, a v3 mechanism the template builds from the definition and the joint's
    // setup: on its motor controllers, the simulator's, or none (a replay reads the log, and a
    // joint without wiring stays still rather than guess at motors, and says so).
    RobotMath math = definition.math();
    SimulatedWorld simulated = world;
    List<String> unwired = new ArrayList<>();
    camerasOnJoints = definition.cameras().stream().anyMatch(camera -> camera.joint().isPresent());
    joints =
        new Joints(
            definition,
            robot.joints(),
            setup -> Objects.requireNonNull(tunings.joints().get(setup.joint().label()))::get,
            setup -> {
              if (simulated != null) {
                return simulated.joint(math, setup);
              }
              if (mode == Constants.Mode.REAL && setup.wiring().isPresent()) {
                switch (setup.wiring().get().controller()) {
                  case TALON_FX, TALON_FXS -> {
                    return new JointIOTalon(
                        math, setup, Sensors.pins(definition, robot.sensors(), setup));
                  }
                  case SPARK_MAX, SPARK_FLEX -> {
                    return new JointIOSpark(
                        math, setup, Sensors.pins(definition, robot.sensors(), setup));
                  }
                }
              } else if (mode == Constants.Mode.REAL) {
                unwired.add(setup.joint().label());
              }
              return new JointIO() {};
            },
            setup -> {
              if (simulated != null) {
                return simulated.absoluteEncoder(setup);
              }
              var port =
                  setup.joint().driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port);
              return mode == Constants.Mode.REAL && port.isPresent()
                  ? new AbsoluteEncoderIOPort(port.get())
                  : new AbsoluteEncoderIO() {};
            },
            sensor -> {
              if (simulated != null) {
                return simulated.sensor(sensor);
              }
              if (mode == Constants.Mode.REAL) {
                SensorIO io = sensorIo(robot, sensor);
                if (io != null) {
                  return io;
                }
                unwiredSensors.add(sensor.label());
              }
              return new SensorIO() {};
            },
            scheduler);
    jointsUnwired =
        new Alert(
            "Joints",
            "unwired",
            "No wiring for "
                + String.join(", ", unwired)
                + ", so they stay still. Add it to the robot's entry in Robots.",
            Alert.Level.HIGH);
    jointsUnwired.set(!unwired.isEmpty());
    this.sensorsUnwired =
        new Alert(
            "Sensors",
            "unwired",
            "No wiring for "
                + String.join(", ", unwiredSensors)
                + ", so nothing waiting on them holds. Add it to the robot's entry in Robots.",
            Alert.Level.HIGH);
    this.sensorsUnwired.set(!unwiredSensors.isEmpty());
    sensorsLost = new Alert("Sensors", "lost", "", Alert.Level.HIGH);
    sensorsDoubted = new Alert("Sensors", "doubted", "", Alert.Level.HIGH);

    // Cameras: PhotonVision on the robot, its simulation in the simulator, the log in a replay.
    CameraIO[] cameraIOs = new CameraIO[cameras.size()];
    if (world != null) {
      cameraIOs = world.cameras();
    } else {
      for (int i = 0; i < cameras.size(); i++) {
        cameraIOs[i] =
            mode == Constants.Mode.REAL
                ? new CameraIOPhotonVision(cameras.get(i).name())
                : new CameraIO() {};
      }
    }
    vision =
        new Vision(
            cameras, cameraIOs, localization.service(), field, visionTuning::get, () -> alliance);
    calibration =
        new Calibration(
            drive,
            vision,
            localization,
            cameras,
            robot.drivetrainWiring(),
            definition.drivetrain(),
            LocalDate::now);

    // Driver: left stick translates (field-relative, from the driver's end of the field), right
    // stick turns. Back, with the robot facing straight downfield, resets its heading: the fallback
    // when the heading has gone wrong. Enabled or not.
    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> leftStick().getX(), () -> leftStick().getY(), () -> -driver.getRightX()));
    driver
        .back()
        .onTrue(Command.noRequirements(coroutine -> faceDownfield()).named("Reset heading"));
  }

  /**
   * The driver says the robot faces straight downfield, away from the driver's own wall: its
   * heading becomes the alliance's forward, as sure as lining up by eye allows. Its position stays.
   * If the driver was wrong, the cameras' headings will agree against it and the estimate turns
   * back to them.
   */
  void faceDownfield() {
    localization.resetHeading(
        FieldModel.driverForward(alliance.orElse(Alliance.BLUE)), HEADING_BY_EYE_RADIANS);
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

  /**
   * One loop of every mechanism, before the command scheduler runs: the drivetrain reads its
   * sensors, localization takes them, the cameras theirs, and the field's checks follow.
   */
  public void periodic() {
    boolean enabled = RobotState.isEnabled();
    // The venue first: a selection the pit made takes effect at the start of a loop, while
    // disabled.
    venues.update(!enabled, RobotState.isFMSAttached()).ifPresent(this::venue);
    alliance = MatchState.getAlliance().map(RobotContainer::alliance);
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
    SimulatedWorld sim = world;
    if (sim != null) {
      // In the simulator the drive team really moves it. Then the world moves on and the robot
      // reads its sensors, as on a robot between loops: its gyro feels the turn this loop.
      placed.ifPresent(sim::place);
      sim.step(RobotController.getTime());
      loop.mark("Simulation", System.nanoTime());
    }
    boolean autonomous = RobotState.isAutonomousEnabled();
    if (autonomous && !wasAutonomous) {
      vision.autonomousStarted(RobotController.getTime());
    }
    wasAutonomous = autonomous;
    start = chosen;
    startPose = chosenPose;
    enabledSinceBoot |= enabled;

    drive.periodic(enabled);
    loop.mark("Drive", System.nanoTime());
    long read = RobotController.getTime();
    joints.read(read, enabled);
    if (camerasOnJoints) {
      // Where each camera on a joint is, for the images it takes.
      localization.service().joints(read, joints.positions());
    }
    List<String> nowLost = new ArrayList<>(joints.sensors().lost());
    nowLost.removeAll(unwiredSensors);
    if (!nowLost.equals(lost)) {
      lost = nowLost;
      sensorsLost.setText(
          String.join(", ", lost)
              + (lost.size() == 1 ? " is" : " are")
              + " lost: nothing waiting on "
              + (lost.size() == 1 ? "it" : "them")
              + " holds. Check the wiring and the CAN bus.");
      sensorsLost.set(!lost.isEmpty());
    }
    List<String> nowDoubted = joints.switchDoubts();
    if (!nowDoubted.equals(doubted)) {
      doubted = nowDoubted;
      sensorsDoubted.setText(String.join("; ", doubted));
      sensorsDoubted.set(!doubted.isEmpty());
    }
    loop.mark("Joints", System.nanoTime());
    localization.periodic(drive.readings(), enabled);
    // Told where it is after odometry, so a turn made in placing it isn't counted twice.
    String placedOn = "placed on " + chosen.map(Place::name).orElse("its start");
    placed.ifPresent(pose -> localization.place(pose, PLACED, placedOn));
    vision.periodic(RobotController.getTime());
    loop.mark("Vision", System.nanoTime());
    frames = localization.update().frames();
    vision.record(frames);
    loop.mark("Localization", System.nanoTime());
    Pose2d pose = localization.pose();
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
    loop.mark("Field", System.nanoTime());
    Dashboard.publish(status());
    loop.mark("Dashboard", System.nanoTime());
  }

  /**
   * What happens after the command scheduler, each loop: each joint does what its commands asked of
   * it this loop, or its idle.
   */
  public void afterCommands() {
    joints.write();
  }

  /**
   * A sensor's hardware on a robot, from its wiring, or null when it has none. A switch on a motor
   * controller's limit input is read by its joint's controller, with the joint.
   */
  private static @Nullable SensorIO sensorIo(RobotConfig robot, Sensor.Placed sensor) {
    for (SensorWiring wiring : robot.sensors()) {
      if (!wiring.sensor().equals(sensor.label())) {
        continue;
      }
      if (wiring instanceof SensorWiring.OfSwitch onSwitch) {
        return switch (onSwitch.input()) {
          case SensorWiring.Port port -> new SensorIOPort(port.port(), onSwitch.polarity());
          case SensorWiring.Candi candi -> new SensorIOCandi(onSwitch, candi);
          case SensorWiring.CanRange range -> null;
          case SensorWiring.ControllerLimit limit ->
              SensorIO.onPin(limit.joint(), limit.direction());
        };
      }
      if (wiring.input() instanceof SensorWiring.CanRange range
          && sensor instanceof Sensor.Rangefinder rangefinder) {
        return new SensorIOCanRange(rangefinder, range);
      }
    }
    return null;
  }

  /** One line for the pit: how long loops take, and whether they keep up. */
  String loopStatus() {
    return loopHealth.describe();
  }

  /**
   * What's doubtful or unknown in the robot's definition: allowed, but said once at startup, in the
   * log ({@code Robot/DefinitionDoubts}) and, on a robot, on the Driver Station's console, so
   * nobody has to run a test to read it.
   */
  private static void reportDoubts(RobotDefinition definition, Constants.Mode mode) {
    List<String> doubts = new ArrayList<>(definition.notes());
    for (RobotDefinition.Missing missing : definition.missing()) {
      doubts.add("not known: " + missing);
    }
    for (Configuration configuration : definition.configurations()) {
      String cantRun = Joints.cantRun(definition, configuration);
      if (cantRun != null) {
        doubts.add(cantRun);
      }
    }
    Logger.recordOutput("Robot/DefinitionDoubts", doubts.toArray(String[]::new));
    if (mode == Constants.Mode.REAL) {
      for (String doubt : doubts) {
        DriverStationErrors.reportWarning(definition.name() + ": " + doubt, false);
      }
    }
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
    SimulatedWorld sim = world;
    if (sim != null) {
      sim.log(pose);
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

  /**
   * Plays on a venue from now on: localization solves images against its tags, the cameras' mount
   * checks judge against them, and places derived from tags move with them.
   */
  private void venue(FieldModel venue) {
    field = venue;
    localization.service().venue(venue);
    vision.venue(venue);
  }

  /**
   * The Driver Station's alliance, as the field model names it: the model has its own, so that
   * nothing reading it needs WPILib's Driver Station.
   */
  static Alliance alliance(org.wpilib.driverstation.Alliance driverStation) {
    return switch (driverStation) {
      case BLUE -> Alliance.BLUE;
      case RED -> Alliance.RED;
    };
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

  /** The robot's state, as the drive team's dashboard shows it, as of this loop. */
  RobotStatus status() {
    return RobotStatus.builder()
        .readyToScore(localization.readyToScore())
        .scoring(localization.scoring())
        .station(allianceStation())
        .onStart(placement.status() == Placement.Status.READY)
        .placement(placement.describe())
        .localization(localization.describe())
        .cameras(vision.summary())
        .loop(loopStatus())
        .build();
  }

  /** One line for the pit: what the pose rests on, and how sure it is. */
  String localizationStatus() {
    return localization.describe();
  }

  /** Whether the estimate is good enough to score from: bind scoring and aiming to it. */
  boolean readyToScore() {
    return localization.readyToScore();
  }

  /** One line for the pit and the drive team: ready to score, or why not. */
  String scoring() {
    return localization.scoring();
  }

  /** One line for the pit: the cameras, and how their measurements fare. */
  String cameras() {
    return vision.summary();
  }

  Vision vision() {
    return vision;
  }

  Localization localization() {
    return localization;
  }

  /** The localization estimate, as of the last loop. */
  LocalizationEstimate estimate() {
    return localization.estimate();
  }

  /** The pre-match placement check, as of the last loop. */
  Placement placement() {
    return placement;
  }

  /** The calibration procedures, for the Pitwall page. */
  Calibration calibration() {
    return calibration;
  }

  /** A pit check for the Pitwall page: drive forward slowly for two seconds. */
  Command driveCheck() {
    return drive.driveForward(0.5, 2.0);
  }

  /** The most often a status line that follows the sensors changes on the Pitwall page. */
  static final Duration STATUS_INTERVAL = Duration.ofSeconds(1);

  /** The drive check's name on the Pitwall page. */
  static final String DRIVE_CHECK = "Drive forward slowly (2 s)";

  /** The steering's experiment on the Pitwall page, and its motion. */
  static final String STEER_STEP = "Steer to 90° and back";

  static final Profile STEER_STEP_PROFILE =
      Profiles.stepAndBack().from(Degrees.of(0)).to(Degrees.of(90)).hold(Seconds.of(0.8));

  /**
   * Puts this robot's pit tools on its Pitwall page: the drive check and the calibrations as
   * actions, the steering's experiment for tuning it, and one status line each for the loop, the
   * alliance, the placement, localization, scoring, the last calibration, and the cameras. In the
   * simulator, the simulated world's parts that can fail, and the scenarios, for the page to apply.
   */
  void addTo(Pitwall pitwall) {
    SimulatedWorld sim = world;
    if (sim != null) {
      pitwall
          .simulator()
          .ifPresent(
              simulator -> {
                sim.addTo(simulator.components());
                Scenarios.addTo(simulator.components(), sim);
              });
    }
    pitwall.actions().add(DRIVE_CHECK, Actions.command(() -> scheduler, this::driveCheck));
    calibration.addTo(pitwall.actions(), scheduler);
    pitwall
        .experiments()
        .add(new Experiment(STEER_STEP, drive.steeringRig(drivetrainTuning), STEER_STEP_PROFILE));
    // The World view: the field, the robot where localization believes it is, its cameras' sight
    // lines with their verdicts, and in the simulator, the truth.
    pitwall
        .scenes()
        .add(
            WorldView.scene(
                () -> field,
                robotDefinition,
                worldReadings(),
                sim != null ? () -> sim.drivetrain().truePose3d() : null,
                camera -> Optional.of(Calibration.cameraMountAction(camera))));
    // Each change goes to every open page: the lines that follow sensors change at most once a
    // second, and are written to change only when something worth reading does.
    pitwall
        .status()
        .add("Loop", this::loopStatus, STATUS_INTERVAL)
        .add("Alliance", this::allianceStation)
        .add("Venue", venues::active)
        .add("Placement", () -> placement.describe(), STATUS_INTERVAL)
        .add("Localization", this::localizationStatus, STATUS_INTERVAL)
        .add("Scoring", this::scoring)
        .add("Calibration", calibration::lastResult)
        .add("Cameras", this::cameras, STATUS_INTERVAL);
  }

  Drive drive() {
    return drive;
  }

  /** Every driven joint's mechanism, for behavior code to look up by its joint. */
  Joints joints() {
    return joints;
  }

  /** What the World view reads each loop while a page shows it: localization's and the wheels'. */
  private WorldView.Readings worldReadings() {
    return new WorldView.Readings() {
      @Override
      public LocalizationEstimate estimate() {
        return localization.estimate();
      }

      @Override
      public double pitchRadians() {
        return localization.pitchRadians();
      }

      @Override
      public double rollRadians() {
        return localization.rollRadians();
      }

      @Override
      public SwerveModuleVelocity[] modules() {
        return drive.moduleVelocities();
      }

      @Override
      public List<FrameVerdict> frames() {
        return frames;
      }
    };
  }

  /** The drivetrain's configuration as the pit tunes it. */
  LiveConfig<DrivetrainControl> drivetrainTuning() {
    return drivetrainTuning;
  }

  /** The simulator's world, or null outside the simulator. */
  @Nullable SimulatedWorld world() {
    return world;
  }

  /** The simulator's drivetrain physics, or null outside the simulator. */
  @Nullable DriveSimulation simulation() {
    SimulatedWorld sim = world;
    return sim != null ? sim.drivetrain() : null;
  }

  /** Whether the alert saying the drivetrain has no hardware code is active. */
  boolean isNoHardwareAlertActive() {
    return noHardware.get();
  }

  /** Releases what holds global resources (alerts), so a test can build another robot. */
  @Override
  public void close() {
    TunableRegistry.remove(drivetrainTuning);
    TunableRegistry.remove(visionTuning);
    drive.close();
    localization.close();
    noHardware.close();
    jointsUnwired.close();
    sensorsUnwired.close();
    sensorsLost.close();
    sensorsDoubted.close();
    allianceUnknown.close();
    offField.close();
    venues.close();
    loopHealth.close();
    vision.close();
    if (world != null) {
      world.close();
    }
    if (hardware != null) {
      hardware.close();
    }
    if (fieldView != null) {
      fieldView.close();
    }
  }
}
