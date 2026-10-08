package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.config.Finding;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;

/**
 * What one robot is: its body, its drivetrain, where its cameras and IMUs are mounted, and its
 * mechanisms: their joints, how the joints are coupled, what drives them, and their sensors. Every
 * reader of the robot reads this one description: localization (the wheels' positions, the mounts),
 * the simulator (all of it), the mechanisms' own code (their ratios, ranges and ideal gains, {@link
 * #math}), and the views. How each joint is driven (gains, limits) and wired (CAN IDs, an encoder's
 * offset) is in its block too ({@link Driven}), as the drivetrain's is in its geometry, but isn't
 * part of what it is: a joint re-wired or re-tuned is the same joint, so a practice robot is the
 * competition robot with its differences ({@link #with}). The PDH that feeds them is the robot's
 * ({@link #pdh}).
 *
 * <p>It also names the robot's places: each joint's named positions and outputs, the configurations
 * the robot takes ({@link Configuration}), and the interlocks between joints ({@link Interlock}).
 * They're facts about the robot, read by everything that reads it; what to do with them is the
 * robot program's. A place is a name on its joint, its value this robot's: the configurations and
 * interlocks, written of the joints' constants, are taken as about this robot's own joints, each
 * named place where its joint puts it, so a practice robot's joint, its places re-placed, is
 * checked and driven at its own.
 *
 * <p>Each part checks its own values when built; the checks here are those across parts, each
 * saying what it finds ({@link Finding}): a camera mounted inside the body, a drive's joint among
 * the robot's joints, the chassis left once the links are taken off the body still a body, an
 * absolute encoder telling its joint's whole range apart, a named output within what its joint's
 * motors can do; each joint's block against itself (where it rests and its places inside its hard
 * stops, judged here on the finished joint so a practice robot changes them in any order, its
 * control's soft limits inside its hard stops and around its places, a homing recipe it can carry
 * out, a loop on its absolute encoder only where the encoder tells its whole range); the
 * drivetrain's top speed within its free speed; one sensor on each input, and a switch on a motor
 * controller's limit input only where there is one; every configuration obeying every interlock, no
 * interlocks holding joints by each other, no two devices of a kind on one CAN bus at one ID, and
 * every device fed from one of the PDH's channels, a channel to each, through a breaker no bigger
 * than it takes. What the program needs beyond what the robot is, a control for each joint it
 * drives, it asks of {@link #findings}. Every reference between parts is the part itself, never a
 * name, so a joint built after its parent can't be its own ancestor; labels are for logs and
 * writers, and unique.
 *
 * @param name which robot this is, as its logs and its checks' messages name it
 * @param body the frame, bumpers, and mass
 * @param drivetrain the swerve drivetrain, as built
 * @param cameras the AprilTag cameras
 * @param imus where each IMU is mounted, and whether its accelerometer has been checked to read
 *     along the robot's axes
 * @param pdh its power distribution hub, whose channels feed its motor controllers, each fed as its
 *     wiring says ({@code TALON_FX.can(32).pdh(4, Amps.of(40))}); with none, no device says one
 * @param joints the mechanisms' joints, each after its parent, each with its own motors, sensors,
 *     wiring and control in its block
 * @param couplings the joints that move as functions of others: an elevator's stages, a linkage
 * @param drives what moves the mechanisms beyond each joint's own: a differential's motors, turning
 *     two joints at once; each through its transmission
 * @param sensors the sensors beyond each joint's own: beam breaks, contact switches and
 *     rangefinders, each on its part and carrying its wiring
 * @param configurations the poses the robot takes by name, each some joints at their named
 *     positions or outputs: "stowed", "amp", "feeding"
 * @param interlocks what one joint may do by where another is, enforced beneath every command
 */
@ConfigRecord
public record RobotDefinition(
    String name,
    Body body,
    DrivetrainGeometry drivetrain,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<CameraConfig> cameras,
    Imus imus,
    Optional<Pdh> pdh,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Joint> joints,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Coupling> couplings,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Drive> drives,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Sensor> sensors,
    @RecordBuilder.Initializer(value = "of", source = List.class)
        List<Configuration> configurations,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Interlock> interlocks) {
  public RobotDefinition {
    cameras = List.copyOf(cameras);
    joints = List.copyOf(joints);
    couplings = List.copyOf(couplings);
    drives = List.copyOf(drives);
    sensors = List.copyOf(sensors);
    List<Joint> robots = joints;
    configurations = configurations.stream().map(each -> onOwn(robots, each)).toList();
    interlocks = interlocks.stream().map(each -> onOwn(robots, each)).toList();
    List<CameraConfig> allCameras = cameras;
    List<Joint> allJoints = joints;
    List<Coupling> allCouplings = couplings;
    List<Configuration> allConfigurations = configurations;
    List<Drive> everyDrive = everyDrive(joints, drives);
    List<Sensor> everySensor = everySensor(joints, sensors);
    List<Interlock> everyInterlock = everyInterlock(joints, interlocks);
    List<CanDevice> everyDevice = everyDevice(pdh, joints, drivetrain, everySensor);
    RobotMath whole = RobotMath.whole(body, joints, couplings, everyDrive, everySensor);
    Checks.on(RobotDefinition.class, name)
        .field("name", name, notBlank())
        .field("cameras", cameras, RobotDefinition::sharedNames)
        .check("drivetrain.corners", () -> drivetrain.whyWheelsDontFit(body))
        .check(
            "drivetrain.control.limits.maxLinearSpeed",
            () ->
                drivetrain.control().map(control -> control.whyTooFastFor(drivetrain)).orElse(null))
        .findings("cameras", () -> outside(allCameras, body))
        .findings("imus", () -> outside(imus, body))
        .findings(
            "joints",
            () ->
                RobotMath.problems(
                    name, body, allJoints, allCouplings, everyDrive, everySensor, allCameras))
        .findings("joints", () -> whole == null ? List.of() : Blocks.problems(whole))
        .findings("sensors", () -> Inputs.problems(allJoints, everySensor))
        .findings(
            "configurations",
            () ->
                whole == null
                    ? List.of()
                    : Places.configurationProblems(whole, allConfigurations, everyInterlock))
        .findings("interlocks", () -> twice(everyInterlock))
        .findings(
            "interlocks",
            () -> whole == null ? List.of() : Places.interlockProblems(whole, everyInterlock))
        .findings("devices", () -> CanDevice.sharedIds(everyDevice))
        .findings("pdh", () -> Pdh.problems(pdh, everyDevice))
        .orThrow();
  }

  /** Starts describing a robot, one named value at a time. */
  public static RobotDefinitionBuilder.NameStage builder() {
    return RobotDefinitionBuilder.builder();
  }

  /** Where each wheel touches the floor: front left, front right, back left, back right. */
  public Translation2d[] moduleTranslations() {
    return drivetrain.moduleTranslations();
  }

  /**
   * The same robot under another name: a practice robot of its design, {@code
   * COMPETITION.named("practice").with(...)}.
   */
  public RobotDefinition named(String name) {
    return RobotDefinitionBuilder.builder(this).name(name).build();
  }

  /**
   * The same robot with some of its joints wired, tuned or placed otherwise: a practice robot of
   * its design, {@code COMPETITION.with(PIVOT.motorsOn(TALON_FX.can(33)))}. Each joint given
   * replaces the robot's joint it's equal to, the same in all but how it's driven, its stops, where
   * it rests and where its places are, so everything that names that joint or its places, behavior
   * included, finds this one's. Each is the robot's joint changed one thing at a time ({@code
   * motorsOn}, {@code gearing}, {@code encoderOn}, {@code encoderOffset}, {@code limitSwitchOn},
   * {@code retuned}, {@code replaced}, {@code range}, {@code rest}), the rest kept, so a change to
   * the robot's own block reaches this one.
   *
   * @throws IllegalArgumentException for a joint the robot has no equal of, or one given twice
   */
  public RobotDefinition with(Joint... changed) {
    return RobotDefinitionBuilder.builder(this)
        .joints(
            replaced(
                joints,
                List.of(changed),
                joint -> joint,
                Joint::label,
                joint ->
                    name
                        + " has no joint "
                        + joint.label()
                        + " the same but for how it's driven, its stops, where it rests and"
                        + " where its places are"))
        .build();
  }

  /**
   * The same robot with some of its sensors wired otherwise: a beam on another CANdi, {@code
   * COMPETITION.with(CLAW_BEAM.on(CANDI.can(36).s1().normallyOpen()))}. Each replaces the robot's
   * sensor it's equal to, the same but for its wiring. A joint's own limit switch or absolute
   * encoder is re-wired with its joint ({@code limitSwitchOn}, {@code encoderOn}).
   *
   * @throws IllegalArgumentException for a sensor the robot has no equal of, or one given twice
   */
  public RobotDefinition with(Sensor... changed) {
    for (Sensor sensor : changed) {
      for (Joint joint : joints) {
        if (joint.ownSensors().contains(sensor)) {
          throw new IllegalArgumentException(
              sensor.label()
                  + " is "
                  + possessive(joint.label())
                  + " own: re-wire it with its joint, "
                  + (sensor instanceof Sensor.Switch onSwitch
                          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
                      ? "limitSwitchOn(End." + limit.end().name() + ", ...)"
                      : "encoderOn(...)"));
        }
      }
    }
    return RobotDefinitionBuilder.builder(this)
        .sensors(
            replaced(
                sensors,
                List.of(changed),
                sensor -> sensor,
                Sensor::label,
                sensor ->
                    name + " has no sensor " + sensor.label() + " the same but for its wiring"))
        .build();
  }

  /**
   * The same robot with some of its cameras mounted otherwise: a practice robot's back camera,
   * lower, {@code COMPETITION.with(CameraConfigBuilder.builder(BACK).up(Inches.of(20.8)).build())}.
   * Each replaces the robot's camera of its name, all of it: where it sits and points, the camera
   * itself, and how far its results are trusted.
   *
   * @throws IllegalArgumentException for a camera the robot has none of by its name, or one given
   *     twice
   */
  public RobotDefinition with(CameraConfig... changed) {
    return RobotDefinitionBuilder.builder(this)
        .cameras(
            replaced(
                cameras,
                List.of(changed),
                CameraConfig::name,
                CameraConfig::name,
                camera ->
                    name
                        + " has no camera named "
                        + camera.name()
                        + ": it has "
                        + (cameras.isEmpty()
                            ? "none"
                            : String.join(
                                ", ", cameras.stream().map(CameraConfig::name).toList()))))
        .build();
  }

  /**
   * The same robot, its drivetrain wired or driven otherwise: a practice robot's, its modules'
   * CANcoder offsets its own, {@code COMPETITION.with(DRIVETRAIN.wired(PRACTICE_SWERVE))}. Its
   * modules may differ in what only the simulator reads, their inertias, which it takes too.
   *
   * @throws IllegalArgumentException for a drivetrain that isn't this robot's: other modules, or
   *     placed otherwise
   */
  public RobotDefinition with(DrivetrainGeometry changed) {
    if (!changed.equals(drivetrain)) {
      throw new IllegalArgumentException(
          possessive(name)
              + " drivetrain isn't this one: their modules, or where they sit, differ. Give its"
              + " own, re-wired, .wired(...), or re-tuned, .control(...)");
    }
    return RobotDefinitionBuilder.builder(this).drivetrain(changed).build();
  }

  /**
   * Parts, some replaced by those the same as them, each given once: a second would drop the
   * first's change.
   *
   * @param same what two parts are the same by: the part itself, its equality, or its name
   * @param label what a part is called, for a message
   * @param none why the robot has no part the same as one given
   */
  private static <T> List<T> replaced(
      List<T> parts,
      List<? extends T> changed,
      Function<? super T, ?> same,
      Function<? super T, String> label,
      Function<? super T, String> none) {
    List<T> each = new ArrayList<>(parts);
    Set<Object> given = new HashSet<>();
    for (T part : changed) {
      int at = -1;
      for (int i = 0; i < each.size() && at < 0; i++) {
        if (same.apply(each.get(i)).equals(same.apply(part))) {
          at = i;
        }
      }
      if (at < 0) {
        throw new IllegalArgumentException(none.apply(part));
      }
      if (!given.add(same.apply(part))) {
        throw new IllegalArgumentException(
            label.apply(part)
                + " is given twice, and the second would drop the first's change: give it once,"
                + " its changes one after another");
      }
      each.set(at, part);
    }
    return each;
  }

  /**
   * What the mechanisms derive: their poses, ratios, ideal gains, gravity and reach, as pure
   * functions of their joints' positions.
   */
  public RobotMath math() {
    return RobotMath.of(
        body, joints, couplings, everyDrive(joints, drives), everySensor(joints, sensors));
  }

  /**
   * Every device on the robot's CAN buses, each once, as the definition wires it ({@link
   * CanDevice}): the PDH; the drivetrain's gyro, and each module's drive, steering and CANcoder;
   * each joint's motor controllers and CANcoder, in its block; and the CANdis and CANranges its
   * sensors are on. Each motor controller carries the PDH channel that feeds it, where its wiring
   * says. The one list the checks across the robot's wiring read: no two devices of a kind on one
   * bus at one ID, the drivetrain's included, and no channel feeding two devices.
   */
  public List<CanDevice> devices() {
    return everyDevice(pdh, joints, drivetrain, everySensor(joints, sensors));
  }

  /**
   * What the definition says about itself to the program that drives the robot ({@link Finding}):
   * what the program can't drive, which the definition itself allows; what's doubtful, though
   * allowed; and what it doesn't know, with what that costs and what reads it, the robot first and
   * then each joint in order. The program refuses the first, and says the rest at startup.
   *
   * <p>What it can't drive ({@link Finding.Level#REFUSED}): a joint with motors and no control, and
   * a drivetrain with no control. Localization, the simulator's physics and the views need neither.
   *
   * <p>Doubtful ({@link Finding.Level#DOUBT}): a named speed above nine tenths of what its joint's
   * motors turn it at, which a velocity loop can't hold under load ({@code RobotMath.freeSpeed});
   * one above it all is refused. A joint resting inside a region it's kept out of as the robot
   * starts, which it may only leave until the region opens; a configuration there is refused.
   *
   * <p>Not known ({@link Finding.Level#NOTE}): nothing here is wrong, only unknown. A value nobody
   * has is zero or the model's own stand-in, never a guess, and this says where. On the robot
   * ({@link Finding.Scope#ROBOT}):
   *
   * <ul>
   *   <li>the robot not weighed, its mass the season's most, or its inertia a box's: the drive's
   *       moves and feedforward read them;
   *   <li>the tread's grip not measured, PathPlanner's default: the drive's moves plan on it, and
   *       the slip check judges by it;
   *   <li>a joint gravity acts on, held at positions, with no kG written: none, or one seeded from
   *       its links' mass, never tuned;
   *   <li>a link with no shape;
   *   <li>a joint nothing drives or couples;
   *   <li>a joint with named speeds or voltages no motors turn it at, which can't be checked;
   *   <li>a sensor not placed on its part, or a limit switch whose position isn't known;
   *   <li>with a PDH declared, a motor controller whose wiring doesn't say which of its channels
   *       feeds it: the drivetrain's, then each joint's.
   * </ul>
   *
   * <p>In the simulator alone ({@link Finding.Scope#SIMULATOR}), since mass is optional and only
   * the simulator reads it: the body's pitch and roll a box's beside a given yaw; its center of
   * mass not given; the share of its mass in no link; the modules' drive or steering inertia not
   * measured; a link with no mass, or mass but no inertia.
   */
  public List<Finding> findings() {
    RobotMath math = math();
    List<Finding> found = new ArrayList<>();
    if (drivetrain.control().isEmpty()) {
      found.add(
          Finding.refused(
              "drivetrain",
              "the drivetrain has no control: give it its limits and gains,"
                  + " .control(DrivetrainControl.builder()...)"));
    }
    found.addAll(Blocks.uncontrolled(math));
    found.addAll(Places.doubts(math));
    found.addAll(Places.startDoubts(math, everyInterlock()));
    found.addAll(missing());
    return found;
  }

  /** What the definition doesn't know, each with what it costs and what reads it. */
  private List<Finding> missing() {
    RobotMath math = math();
    List<Finding> missing = new ArrayList<>();
    if (!body.weight().weighed()) {
      missing.add(
          missing(
              name,
              "the robot isn't weighed: its mass is the season's most, " + shown(body.mass()),
              "the drive's moves and feedforward, the simulator's acceleration and the links' mass"
                  + " budget rest on the rules, not the robot",
              Finding.Scope.ROBOT));
    }
    switch (body.inertia()) {
      case Body.Turning.Box nothing ->
          missing.add(
              missing(
                  name,
                  "the body's inertia is a box's, its mass spread through its bumpers' outline and"
                      + " height",
                  "how fast the drive's moves turn, its turning feedforward, and how the simulator"
                      + " tips and turns the robot are approximate",
                  Finding.Scope.ROBOT));
      case Body.Turning.Yaw yaw ->
          missing.add(
              missing(
                  name,
                  "the body's pitch and roll are a box's, beside its given yaw",
                  "how the simulator tips the robot is approximate",
                  Finding.Scope.SIMULATOR));
      case Body.Turning.Tensor tensor -> {}
    }
    // A center of mass not given isn't a doubt: nothing reads it but the mass model's own check.
    double robot = body.mass().in(Kilograms);
    double links = math.linksKilograms();
    missing.add(
        missing(
            name,
            String.format(
                Locale.ROOT, "%.2f kg of the robot's %.2f kg is in no link", robot - links, robot),
            "the chassis, its electronics, bumpers and battery: carried as one rigid body",
            Finding.Scope.SIMULATOR));
    unmeasuredModules(missing);
    unfedModules(missing);
    for (Joint joint : joints) {
      Link link = joint.link();
      // Held at positions against gravity: a joint that sags without its kG.
      boolean sags = joint.holdsPositions() && math.gravityActsOn(joint);
      if (link.mass().isMassless()) {
        boolean spins = !link.mass().inertia().isZero();
        missing.add(
            missing(
                joint.label(),
                link.label() + (spins ? " has no mass, only its inertia" : " has no mass"),
                (sags ? possessive(joint.label()) + " gravity comes from its kG, and " : "")
                    + link.label()
                    + (spins ? " adds only its own turning to " : " adds nothing to ")
                    + possessive(joint.label())
                    + " inertia",
                Finding.Scope.SIMULATOR));
      } else if (link.mass().inertia().isZero()) {
        missing.add(
            missing(
                joint.label(),
                link.label() + " has mass but no inertia",
                "its own turning adds nothing to " + possessive(joint.label()) + " inertia",
                Finding.Scope.SIMULATOR));
      }
      if (link.shapes().isEmpty()) {
        missing.add(
            missing(
                joint.label(),
                link.label() + " has no shape",
                "it isn't drawn, nor checked against the starting height and the extension limit",
                Finding.Scope.ROBOT));
      }
      if (!moved(joint)) {
        missing.add(
            missing(
                joint.label(),
                joint.label() + " has nothing moving it",
                "no drive turns it and no coupling leads it: no ideal gains, and the simulator"
                    + " leaves it where it is",
                Finding.Scope.ROBOT));
      }
      Finding kG = noKg(math, joint, sags);
      if (kG != null) {
        missing.add(kG);
      }
      Finding unfed = unfed(joint);
      if (unfed != null) {
        missing.add(unfed);
      }
    }
    for (Joint joint : Places.unchecked(math)) {
      missing.add(
          missing(
              joint.label(),
              joint.label() + " names outputs, and no motors turn it",
              "its speeds and voltages aren't checked against what its motors can do",
              Finding.Scope.ROBOT));
    }
    for (Sensor sensor : everySensor(joints, sensors)) {
      if (sensor instanceof Sensor.Placed placed && placed.pose().isEmpty()) {
        missing.add(
            missing(
                sensor.label(),
                sensor.label() + " isn't placed on its part",
                "it's drawn, and simulated, at its part's origin",
                Finding.Scope.ROBOT));
      }
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
          && limit.position().isEmpty()) {
        missing.add(
            missing(
                sensor.label(),
                possessive(sensor.label())
                    + " position on "
                    + limit.joint().label()
                    + " isn't known",
                "it can stop " + limit.joint().label() + " but not home it",
                Finding.Scope.ROBOT));
      }
    }
    return missing;
  }

  /**
   * What the drivetrain's modules leave to their defaults: the tread's grip, which the robot plans
   * and judges by; the drive's and the steering's inertia, which only the simulator reads. Each
   * once, if any corner's module leaves it out.
   */
  private void unmeasuredModules(List<Finding> missing) {
    List<DrivetrainGeometry.ModuleType> modules =
        drivetrain.corners().stream().map(DrivetrainGeometry.Corner::module).toList();
    if (modules.stream().anyMatch(module -> module.wheelCoefficientOfFriction().isEmpty())) {
      missing.add(
          missing(
              name,
              "the tread's grip isn't measured: it's "
                  + shown(DrivetrainGeometry.ModuleType.DEFAULT_WHEEL_COEFFICIENT_OF_FRICTION)
                  + ", PathPlanner's default",
              "drive moves plan their acceleration on it, and the slip check judges by it: measure"
                  + " it with a pull test (docs/characterization.md, tread friction)",
              Finding.Scope.ROBOT));
    }
    if (modules.stream().anyMatch(module -> module.driveInertia().isEmpty())) {
      missing.add(
          missing(
              name,
              "the drive's own inertia isn't measured: it's "
                  + shown(
                      DrivetrainGeometry.ModuleType.DEFAULT_DRIVE_INERTIA.in(KilogramSquareMeters))
                  + " kg m², an estimate for a 4 in wheel and a Kraken X60's rotor",
              "how fast the simulator's wheels spin up is a guess, though a small one beside the"
                  + " robot's mass they carry",
              Finding.Scope.SIMULATOR));
    }
    if (modules.stream().anyMatch(module -> module.steerInertia().isEmpty())) {
      missing.add(
          missing(
              name,
              "the steering's inertia isn't measured: it's "
                  + shown(
                      DrivetrainGeometry.ModuleType.DEFAULT_STEER_INERTIA.in(KilogramSquareMeters))
                  + " kg m², Phoenix Tuner X's simulation value",
              "how fast the simulator's modules steer at their current limit is a guess",
              Finding.Scope.SIMULATOR));
    }
  }

  /**
   * The drivetrain's motor controllers whose wiring doesn't say their PDH channel, once, if the
   * robot declares a PDH: their feeds can't be checked against it.
   */
  private void unfedModules(List<Finding> missing) {
    List<String> unfed =
        unfed(drivetrain.wiring().map(SwerveWiring::devices).orElse(List.of())).stream()
            .map(CanDevice::name)
            .toList();
    if (pdh.isEmpty() || unfed.isEmpty()) {
      return;
    }
    missing.add(
        missing(
            name,
            unfed.size() == 2 * DrivetrainGeometry.LABELS.size()
                ? "none of the drivetrain's motor controllers has a PDH channel"
                : "the drivetrain's " + String.join(", ", unfed) + " have no PDH channel",
            "which channel feeds each, and through what breaker, isn't checked: give each module's"
                + " wiring its feeds, .driveFeed(Pdh.feed(channel, breaker)) and .steerFeed(...)",
            Finding.Scope.ROBOT));
  }

  /**
   * A joint's motor controllers whose wiring doesn't say their PDH channel, if the robot declares a
   * PDH: null when each says it, the robot declares none, or the joint's motors aren't wired.
   */
  private @Nullable Finding unfed(Joint joint) {
    List<String> unfed =
        unfed(joint.devices()).stream().map(device -> String.valueOf(device.id())).toList();
    if (pdh.isEmpty() || unfed.isEmpty()) {
      return null;
    }
    return missing(
        joint.label(),
        possessive(joint.label())
            + (unfed.size() == 1
                ? " motor controller " + unfed.get(0) + " has"
                : " motor controllers " + String.join(", ", unfed) + " have")
            + " no PDH channel",
        "which channel feeds each, and through what breaker, isn't checked: give each its own where"
            + " it's wired, .pdh(channel, breaker)",
        Finding.Scope.ROBOT);
  }

  /**
   * One thing the definition doesn't know, and what not knowing it costs: a note, said at startup.
   *
   * @param subject what it's about: the robot, by its name, a joint, or a sensor
   * @param what what isn't known: {@code "pivot has no kG"}
   * @param costs what the robot's code, its derived values, or the simulator lose for it
   * @param scope what reads it: the robot, or only the simulator
   */
  private static Finding missing(String subject, String what, String costs, Finding.Scope scope) {
    return Finding.note(scope, subject, what + ": " + costs);
  }

  /** The motor controllers among some devices whose wiring doesn't say their PDH channel. */
  private static List<CanDevice> unfed(List<CanDevice> devices) {
    return devices.stream()
        .filter(device -> device.kind().motorController() && device.feed().isEmpty())
        .toList();
  }

  /**
   * A joint's kG, if it isn't written where it's needed: a joint held at positions that gravity
   * acts on, driven by its control, with no kG in it. Its links' mass may seed one, but only a
   * tuned kG is known to hold it; a counterbalanced joint writes 0. Null when it's written, or not
   * needed: a roller, a turret, a joint that follows another.
   */
  private static @Nullable Finding noKg(RobotMath math, Joint joint, boolean sags) {
    MechanismControl control = joint.driven().control().orElse(null);
    if (control == null
        || control.gains().kG().isPresent()
        || !sags
        || math.leaderOf(joint) != null) {
      return null;
    }
    if (math.gravityHeld(joint) > 1e-12) {
      return missing(
          joint.label(),
          possessive(joint.label()) + " kG is seeded from its mass, not tuned",
          "it holds only as well as its links' mass and where it sits are right: tune its kG and"
              + " write it, withKg(...)",
          Finding.Scope.ROBOT);
    }
    return missing(
        joint.label(),
        joint.label() + " has no kG",
        "it holds against gravity by kP alone, so it sits below its targets until its kG is tuned"
            + " and written, withKg(...), or written as 0 for a counterbalanced joint",
        Finding.Scope.ROBOT);
  }

  /**
   * The PDH, the drivetrain's devices, each joint's, then the sensors': a CANdi once however many
   * of its inputs are wired, since its two inputs are one device.
   */
  private static List<CanDevice> everyDevice(
      Optional<Pdh> pdh, List<Joint> joints, DrivetrainGeometry drivetrain, List<Sensor> sensors) {
    List<CanDevice> every = new ArrayList<>();
    pdh.ifPresent(
        hub ->
            every.add(
                new CanDevice(
                    "the PDH", CanDevice.Kind.PDH, hub.bus(), hub.id(), Optional.empty())));
    drivetrain.wiring().ifPresent(wiring -> every.addAll(wiring.devices()));
    joints.forEach(joint -> every.addAll(joint.devices()));
    for (Sensor sensor : sensors) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.wiring().map(SensorWiring.Connection::input).orElse(null)
              instanceof SensorWiring.Candi candi
          && every.stream()
              .noneMatch(
                  device ->
                      device.kind() == CanDevice.Kind.CANDI
                          && device.bus().equals(candi.bus())
                          && device.id() == candi.id())) {
        every.add(
            new CanDevice(
                "CANdi " + candi.id(),
                CanDevice.Kind.CANDI,
                candi.bus(),
                candi.id(),
                Optional.empty()));
      }
      if (sensor instanceof Sensor.Rangefinder rangefinder && rangefinder.wiring().isPresent()) {
        SensorWiring.CanRange range = rangefinder.wiring().get();
        every.add(
            new CanDevice(
                sensor.label(),
                CanDevice.Kind.CANRANGE,
                range.bus(),
                range.id(),
                Optional.empty()));
      }
    }
    return every;
  }

  /** Each joint's own drives, then the robot's. */
  private static List<Drive> everyDrive(List<Joint> joints, List<Drive> drives) {
    List<Drive> every = new ArrayList<>();
    joints.forEach(joint -> every.addAll(joint.ownDrives()));
    every.addAll(drives);
    return every;
  }

  /**
   * This robot's own joint, the one equal to a joint: the competition robot's {@code HOOD}, on its
   * practice robot, with the practice robot's stops and places. A subsystem of the team's own reads
   * its joint from it rather than restating it.
   *
   * @throws IllegalArgumentException for a joint the robot hasn't
   */
  public Joint own(Joint joint) {
    int at = joints.indexOf(joint);
    if (at < 0) {
      throw new IllegalArgumentException(
          name + " has no joint " + joint.label() + ": add it to the definition's joints");
    }
    return joints.get(at);
  }

  /**
   * A condition about this robot's own joint, each place it names where this robot puts it: the
   * competition robot's {@code PIVOT.atLeast(PIVOT_OUT)}, on its practice robot, from the practice
   * robot's out. One about a sensor, as it is; one about a joint the robot hasn't, as it is.
   */
  public Condition own(Condition condition) {
    return onOwn(joints, condition);
  }

  /**
   * Why a place a condition names isn't where this robot's joint has it, or null when each is: a
   * gate on it is refused as a move there is, and an interlock or a configuration on it is refused
   * here, as the definition is built. The competition robot's constant is its practice robot's own.
   */
  public @Nullable String whyNotOwn(Condition condition) {
    return Places.notOwn(onOwn(joints, condition));
  }

  /**
   * Every interlock: each joint's own, from its block, then the robot's list. What the robot
   * enforces, and what the definition checks.
   */
  public List<Interlock> everyInterlock() {
    return everyInterlock(joints, interlocks);
  }

  /** Each joint's own interlocks, then the robot's, each about the robot's own joints. */
  private static List<Interlock> everyInterlock(List<Joint> joints, List<Interlock> interlocks) {
    List<Interlock> every = new ArrayList<>();
    joints.forEach(
        joint -> joint.ownInterlocks().forEach(interlock -> every.add(onOwn(joints, interlock))));
    every.addAll(interlocks);
    return every;
  }

  /**
   * A configuration about the robot's own joints, each at its named place where the robot puts it:
   * a configuration written of the competition robot's joints, on its practice robot.
   */
  private static Configuration onOwn(List<Joint> joints, Configuration configuration) {
    return ConfigurationBuilder.builder(configuration)
        .targets(configuration.targets().stream().map(at -> onOwn(joints, at)).toList())
        .build();
  }

  /**
   * An interlock about the robot's own joints, its condition's places where the robot puts them.
   */
  private static Interlock onOwn(List<Joint> joints, Interlock interlock) {
    return switch (interlock) {
      case Interlock.RunsOnlyWhen runs ->
          InterlockRunsOnlyWhenBuilder.builder(runs)
              .joint(own(joints, runs.joint()))
              .condition(onOwn(joints, runs.condition()))
              .build();
      case Interlock.KeepsOut keeps ->
          InterlockKeepsOutBuilder.builder(keeps)
              .joint(own(joints, keeps.joint()))
              .condition(onOwn(joints, keeps.condition()))
              .build();
    };
  }

  /**
   * A condition about the robot's own joint, the one equal to its joint, a place it names where
   * that joint puts it: {@code ELEVATOR.atMost(ELEVATOR_CLEAR)} on a practice robot whose elevator
   * clears lower is at most its own. One about a sensor, as it is.
   */
  private static Condition onOwn(List<Joint> joints, Condition condition) {
    return switch (condition) {
      case Condition.At at -> onOwn(joints, at);
      case Condition.AtLeast atLeast -> {
        Joint joint = own(joints, atLeast.joint());
        yield ConditionAtLeastBuilder.builder(atLeast)
            .joint(joint)
            .position(placed(joint, atLeast.place(), atLeast.position()))
            .build();
      }
      case Condition.AtMost atMost -> {
        Joint joint = own(joints, atMost.joint());
        yield ConditionAtMostBuilder.builder(atMost)
            .joint(joint)
            .position(placed(joint, atMost.place(), atMost.position()))
            .build();
      }
      case Condition.Between between -> {
        Joint joint = own(joints, between.joint());
        yield ConditionBetweenBuilder.builder(between)
            .joint(joint)
            .from(placed(joint, between.fromPlace(), between.from()))
            .to(placed(joint, between.toPlace(), between.to()))
            .build();
      }
      case Condition.UpToSpeed upToSpeed ->
          ConditionUpToSpeedBuilder.builder(upToSpeed)
              .joint(own(joints, upToSpeed.joint()))
              .build();
      case Condition.OnSensor onSensor -> onSensor;
    };
  }

  /** A joint at a named place, the robot's own joint: where the robot puts it is its own. */
  private static Condition.At onOwn(List<Joint> joints, Condition.At at) {
    return ConditionAtBuilder.builder(at).joint(own(joints, at.joint())).build();
  }

  /**
   * The robot's own joint equal to one, the same but for how it's driven and where its places are;
   * or the joint, if the robot hasn't it, which the definition's checks refuse.
   */
  private static Joint own(List<Joint> joints, Joint joint) {
    int at = joints.indexOf(joint);
    return at < 0 ? joint : joints.get(at);
  }

  /**
   * A position, or where a joint puts its place, when it names one of the joint's own: one that
   * isn't stays as it was, for the definition's checks to refuse.
   */
  private static double placed(Joint joint, Optional<Position> place, double position) {
    return place
        .filter(written -> joint.whyNotOwn(written) == null)
        .map(written -> joint.own(written).value())
        .orElse(position);
  }

  /** An interlock written twice, in a block and in the robot's list, or twice in one. */
  private static List<Finding> twice(List<Interlock> interlocks) {
    Set<Interlock> seen = new HashSet<>();
    for (Interlock interlock : interlocks) {
      if (!seen.add(interlock)) {
        return List.of(
            Finding.refused(
                interlock.joint().label(),
                "\""
                    + interlock
                    + "\" is written twice: once is enough, in "
                    + possessive(interlock.joint().label())
                    + " block or the robot's interlocks"));
      }
    }
    return List.of();
  }

  /** Each joint's own sensors, then the robot's. */
  private static List<Sensor> everySensor(List<Joint> joints, List<Sensor> sensors) {
    List<Sensor> every = new ArrayList<>();
    joints.forEach(joint -> every.addAll(joint.ownSensors()));
    every.addAll(sensors);
    return every;
  }

  /** Whether a drive moves the joint, or it follows another. */
  private boolean moved(Joint joint) {
    for (Coupling coupling : couplings) {
      if (coupling.follower().equals(joint)) {
        return true;
      }
    }
    for (Drive drive : everyDrive(joints, drives)) {
      if (drive.transmission().joints().contains(joint)) {
        return true;
      }
    }
    return false;
  }

  /**
   * The names two cameras share, or null if none do: as written, or as ids ({@link
   * CameraConfig#id()}), where "Front Left" and "front-left" are one.
   */
  private static @Nullable String sharedNames(List<CameraConfig> cameras) {
    Map<String, String> names = new HashMap<>();
    List<String> shared = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      String before = names.putIfAbsent(camera.id(), camera.name());
      if (before == null) {
        continue;
      }
      shared.add(
          before.equals(camera.name())
              ? "two cameras are named " + camera.name()
              : "cameras \""
                  + before
                  + "\" and \""
                  + camera.name()
                  + "\" differ only in case or punctuation, so they're one id, \""
                  + camera.id()
                  + "\": name them apart");
    }
    return shared.isEmpty() ? null : String.join("; ", shared);
  }

  /**
   * The cameras on the chassis mounted outside the robot. A camera on a joint is checked with its
   * joint stowed ({@link RobotMath}).
   */
  private static List<Finding> outside(List<CameraConfig> cameras, Body body) {
    List<Finding> outside = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      if (camera.joint().isEmpty() && !body.contains(camera.position())) {
        outside.add(
            Finding.refused(
                camera.name(),
                "camera " + camera.name() + " is outside the robot, at " + camera.position()));
      }
    }
    return outside;
  }

  /** The IMUs mounted outside the robot. */
  private static List<Finding> outside(Imus imus, Body body) {
    List<Finding> outside = new ArrayList<>();
    outside(outside, "the gyro", imus.gyro(), body);
    imus.backup().ifPresent(backup -> outside(outside, "the backup IMU", backup, body));
    return outside;
  }

  private static void outside(List<Finding> outside, String name, ImuMount imu, Body body) {
    if (!body.contains(imu.position())) {
      outside.add(Finding.refused(name, name + " is outside the robot, at " + imu.position()));
    }
  }
}
