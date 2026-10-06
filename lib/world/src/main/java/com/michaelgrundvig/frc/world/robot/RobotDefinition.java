package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;

/**
 * What one robot is: its body, its drivetrain, where its cameras and IMUs are mounted, and its
 * mechanisms: their joints, how the joints are coupled, what drives them, and their sensors. Every
 * reader of the robot reads this one description: localization (the wheels' positions, the mounts),
 * the simulator (all of it), the mechanisms' own code (their ratios, ranges and ideal gains, {@link
 * #math}), and the views. How the robot is driven (gains, limits) and how it's wired (CAN IDs, an
 * encoder's offset) aren't here: they're the robot program's configuration around it.
 *
 * <p>It also names the robot's places: each joint's named positions and outputs, the configurations
 * the robot takes ({@link Configuration}), and the interlocks between joints ({@link Interlock}).
 * They're facts about the robot, read by everything that reads it; what to do with them is the
 * robot program's.
 *
 * <p>Each part checks its own values when built; the checks here are those across parts: a camera
 * mounted inside the body, a drive's joint among the robot's joints, the chassis left once the
 * links are taken off the body still a body, an absolute encoder telling its joint's whole range
 * apart, a named output within what its joint's motors can do, every configuration obeying every
 * interlock, and no interlocks holding joints by each other. Every reference between parts is the
 * part itself, never a name, so a joint built after its parent can't be its own ancestor; labels
 * are for logs and writers, and unique.
 *
 * @param name which robot this is, as its logs and its checks' messages name it
 * @param body the frame, bumpers, and mass
 * @param drivetrain the swerve drivetrain, as built
 * @param cameras the AprilTag cameras
 * @param imus where each IMU is mounted, and whether its accelerometer has been checked to read
 *     along the robot's axes
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
    List<CameraConfig> cameras,
    Imus imus,
    List<Joint> joints,
    List<Coupling> couplings,
    List<Drive> drives,
    List<Sensor> sensors,
    List<Configuration> configurations,
    List<Interlock> interlocks) {
  public RobotDefinition {
    joints = List.copyOf(joints);
    couplings = List.copyOf(couplings);
    drives = List.copyOf(drives);
    sensors = List.copyOf(sensors);
    configurations = List.copyOf(configurations);
    interlocks = List.copyOf(interlocks);
    List<Joint> allJoints = joints;
    List<Coupling> allCouplings = couplings;
    List<Configuration> allConfigurations = configurations;
    List<Interlock> allInterlocks = interlocks;
    List<Drive> everyDrive = everyDrive(joints, drives);
    List<Sensor> everySensor = everySensor(joints, sensors);
    RobotMath whole = RobotMath.whole(body, joints, couplings, everyDrive, everySensor);
    Checks.on(RobotDefinition.class, name)
        .field("name", name, notBlank())
        .field("cameras", cameras, RobotDefinition::sharedNames)
        .check("drivetrain.corners", () -> drivetrain.whyWheelsDontFit(body))
        .check("cameras", () -> outside(cameras, body))
        .check("imus", () -> outside(imus, body))
        .check(
            "joints",
            () ->
                RobotMath.problems(body, allJoints, allCouplings, everyDrive, everySensor, cameras))
        .check(
            "configurations",
            () ->
                whole == null
                    ? null
                    : Places.configurationProblems(whole, allConfigurations, allInterlocks))
        .check(
            "interlocks",
            () -> whole == null ? null : Places.interlockProblems(whole, allInterlocks))
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
   * The same robot with some of its joints driven or wired otherwise: a practice robot of the same
   * design, {@code COMPETITION.with(PIVOT.motor(Motor.KRAKEN_X60, Gearing.of(25),
   * TALON_FX.can(33)))}. Each joint given replaces the robot's joint it's equal to, the same in all
   * but how it's driven, so everything that names that joint, behavior included, finds this one.
   *
   * @throws IllegalArgumentException for a joint the robot has no equal of
   */
  public RobotDefinition with(Joint... changed) {
    List<Joint> each = new ArrayList<>(joints);
    for (Joint joint : changed) {
      int at = each.indexOf(joint);
      if (at < 0) {
        throw new IllegalArgumentException(
            name + " has no joint " + joint.label() + " the same but for how it's driven");
      }
      each.set(at, joint);
    }
    return RobotDefinitionBuilder.builder(this).joints(each).build();
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
   * What's doubtful in the definition, though allowed: each a line the robot program says at
   * startup. A named speed above nine tenths of what its joint's motors turn it at, which a
   * velocity loop can't hold under load ({@code RobotMath.freeSpeed}); one above it all is refused.
   */
  public List<String> notes() {
    return Places.notes(math());
  }

  /**
   * One thing the definition doesn't know, and what not knowing it costs: a line the robot program
   * says at startup.
   *
   * @param what what isn't known: {@code "hopper floor has no mass"}
   * @param costs what the derived values and the simulator lose for it
   */
  public record Missing(String what, String costs) {
    @Override
    public String toString() {
      return what + ": " + costs;
    }
  }

  /**
   * What the definition doesn't know, each with what it costs, robot first and then each link in
   * its joint's order. Nothing here is wrong, only unknown: a value nobody has is zero or the
   * model's own stand-in, never a guess, and this says where.
   *
   * <ul>
   *   <li>the robot not weighed, its mass the season's most;
   *   <li>its body's inertia a box's, or its pitch and roll beside a given yaw;
   *   <li>the share of its mass in no link: the chassis, its electronics, bumpers and battery;
   *   <li>a link with no mass, or mass but no inertia, or no shape;
   *   <li>a joint nothing drives or couples;
   *   <li>a joint with named speeds or voltages no motors turn it at, which can't be checked.
   * </ul>
   */
  public List<Missing> missing() {
    List<Missing> missing = new ArrayList<>();
    if (!body.weight().weighed()) {
      missing.add(
          new Missing(
              "the robot isn't weighed: its mass is the season's most, " + shown(body.mass()),
              "the simulator's acceleration and the links' mass budget rest on the rules, not the"
                  + " robot"));
    }
    switch (body.inertia()) {
      case Body.Turning.Box nothing ->
          missing.add(
              new Missing(
                  "the body's inertia is a box's, its mass spread through its bumpers' outline and"
                      + " height",
                  "how the simulator tips and turns the robot is approximate"));
      case Body.Turning.Yaw yaw ->
          missing.add(
              new Missing(
                  "the body's pitch and roll are a box's, beside its given yaw",
                  "how the simulator tips the robot is approximate"));
      case Body.Turning.Tensor tensor -> {}
    }
    double robot = body.mass().in(Kilograms);
    double links = math().linksKilograms();
    missing.add(
        new Missing(
            String.format(
                Locale.ROOT, "%.2f kg of the robot's %.2f kg is in no link", robot - links, robot),
            "the chassis, its electronics, bumpers and battery: carried as one rigid body"));
    for (Joint joint : joints) {
      Link link = joint.link();
      if (link.mass().isMassless()) {
        boolean spins = !link.mass().inertia().isZero();
        missing.add(
            new Missing(
                link.label() + (spins ? " has no mass, only its inertia" : " has no mass"),
                joint.label()
                    + " has no gravity seed (kG 0), no gravity torque, and no center of mass to"
                    + " shift"
                    + (spins ? "" : "; its kA seeds from its motors' rotors alone")));
      } else if (link.mass().inertia().isZero()) {
        missing.add(
            new Missing(
                link.label() + " has mass but no inertia",
                "its own turning adds nothing to "
                    + joint.label()
                    + "'s inertia: its kA seeds low"));
      }
      if (link.shapes().isEmpty()) {
        missing.add(
            new Missing(
                link.label() + " has no shape",
                "it isn't drawn, nor checked against the starting height and the extension limit"));
      }
      if (!moved(joint)) {
        missing.add(
            new Missing(
                joint.label() + " has nothing moving it",
                "no drive turns it and no coupling leads it: no ideal gains, and the simulator"
                    + " leaves it where it is"));
      }
    }
    for (Joint joint : Places.unchecked(math())) {
      missing.add(
          new Missing(
              joint.label() + " names outputs, and no motors turn it",
              "its speeds and voltages aren't checked against what its motors can do"));
    }
    for (Sensor sensor : everySensor(joints, sensors)) {
      if (sensor instanceof Sensor.Placed placed && placed.pose().isEmpty()) {
        missing.add(
            new Missing(
                sensor.label() + " isn't placed on its part",
                "it's drawn, and simulated, at its part's origin"));
      }
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
          && limit.position().isEmpty()) {
        missing.add(
            new Missing(
                sensor.label() + "'s position on " + limit.joint().label() + " isn't known",
                "it can stop " + limit.joint().label() + " but not home it"));
      }
    }
    return missing;
  }

  /** Each joint's own drives, then the robot's. */
  private static List<Drive> everyDrive(List<Joint> joints, List<Drive> drives) {
    List<Drive> every = new ArrayList<>();
    joints.forEach(joint -> every.addAll(joint.ownDrives()));
    every.addAll(drives);
    return every;
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
   * The cameras on the chassis mounted outside the robot, or null if none are. A camera on a joint
   * is checked with its joint stowed ({@link RobotMath}).
   */
  private static @Nullable String outside(List<CameraConfig> cameras, Body body) {
    List<String> outside = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      if (camera.joint().isEmpty() && !body.contains(camera.position())) {
        outside.add("camera " + camera.name() + " is outside the robot, at " + camera.position());
      }
    }
    return outside.isEmpty() ? null : String.join("; ", outside);
  }

  /** The IMUs mounted outside the robot, or null if none are. */
  private static @Nullable String outside(Imus imus, Body body) {
    List<String> outside = new ArrayList<>();
    outside(outside, "the gyro", imus.gyro(), body);
    imus.backup().ifPresent(backup -> outside(outside, "the backup IMU", backup, body));
    return outside.isEmpty() ? null : String.join("; ", outside);
  }

  private static void outside(List<String> outside, String name, ImuMount imu, Body body) {
    if (!body.contains(imu.position())) {
      outside.add(name + " is outside the robot, at " + imu.position());
    }
  }
}
