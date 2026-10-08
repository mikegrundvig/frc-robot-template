package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.config.Rules;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Shape;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.math.BigDecimal;
import java.math.MathContext;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;

/**
 * What a robot's mechanisms derive from their definition: the numbers robot code needs to drive
 * them (Phoenix's ratios, an absolute encoder's discontinuity, the ideal gains that seed tuning)
 * and the geometry it checks against (where each link is, how far the robot reaches, what gravity
 * does at each joint). Each is a pure function of the definition and the joints' positions, as
 * {@code DrivetrainGeometry.Modules.idealDriveKv()} is for the drive.
 *
 * <p>Positions are by joint, in its own units, radians or meters; a joint not given sits where it
 * rests, and a coupled joint follows its leader whatever it's given. Gravity is along the robot's
 * -z: the robot level.
 *
 * <p>The ideal gains are never the gains: they seed a mechanism's feedforward and Phoenix's slots
 * until SysId measures the real ones, and they're what SysId's results are compared with.
 */
public final class RobotMath {
  /** Standard gravity, in m/s². */
  public static final double GRAVITY = 9.80665;

  /**
   * How far a camera on a link may sit from the link's shapes: on it, give or take a bracket and
   * shapes simplified from CAD; farther is a typo or the wrong joint.
   */
  static final double CAMERA_ON_LINK_METERS = 0.1;

  /**
   * How near an absolute encoder's wrap may come to its joint's range, as a share of the turn that
   * wraps: a fortieth, 9° of an encoder on the joint's shaft. Nearer, and backlash, a hard stop's
   * give, or a range measured a little short carries a reading across the wrap, which then reads a
   * turn away. The definition refuses a joint whose encoders wrap this near its range ({@code
   * RobotDefinition}), and the Phoenix helper notes a discontinuity point that does, a team's own
   * included ({@link #wrapMargin}).
   */
  public static final double WRAP_MARGIN = 1.0 / 40;

  /**
   * A joint's ideal feedforward gains: what its motors would need with nothing lost but what the
   * gearing says, and no friction. In the joint's units: radians for a revolute joint, meters for a
   * prismatic one.
   *
   * @param kV volts per unit per second: WPILib's and Phoenix's kV, the motors' nominal voltage
   *     over their free speed through the gearing
   * @param kA volts per unit per second squared, with every link it moves where it rests and the
   *     motors' rotors through their gearing
   * @param kG volts to hold it against gravity, from its links' mass, so 0 for links without it:
   *     for a revolute joint, at its worst, which {@code ArmFeedforward} scales by the cosine of
   *     its angle from horizontal ({@link #gravityOffset}), and how well that cosine fits is {@link
   *     #gravityFit}; for a prismatic one, everywhere
   */
  public record Gains(double kV, double kA, double kG) {}

  /**
   * How well one cosine fits gravity's torque at a revolute joint across its range: Phoenix's
   * {@code Arm_Cosine} and WPILib's {@code ArmFeedforward} take gravity as {@code kG cos(angle +
   * offset)}, which is exact for one link on a level parent, but not for links that move with the
   * joint along their own curves (a hopper floor a linkage swings at a fraction of the intake's
   * angle), nor for a joint whose parent tilts. Past a few percent, feed {@link #gravityTorque} at
   * the joint's angle instead.
   *
   * @param offset the cosine's offset, {@link #gravityOffset}: Phoenix's {@code
   *     GravityArmPositionOffset}
   * @param peak the cosine's amplitude, gravity's torque at its worst, in newton meters: what the
   *     ideal kG holds, through the motors ({@link Gains#kG})
   * @param misfit the widest gap between the cosine and gravity's real torque over the range, as a
   *     share of the peak: 0 when one cosine is gravity, and when there's no gravity at all
   * @param worstAt where in the range the gap is widest, in radians
   */
  public record GravityFit(double offset, double peak, double misfit, double worstAt) {}

  /** How many steps across a joint's range the cosine is checked at. */
  private static final int FIT_STEPS = 360;

  private final Body body;
  private final List<Joint> joints;
  private final List<Coupling> couplings;
  private final List<Drive> drives;
  private final List<Sensor> sensors;
  private final Map<Joint, Integer> index = new HashMap<>();
  private final int[] parent;
  private final int[] leader;
  private final @Nullable Coupling[] coupling;

  private RobotMath(
      Body body,
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors) {
    this.body = body;
    this.joints = List.copyOf(joints);
    this.couplings = List.copyOf(couplings);
    this.drives = List.copyOf(drives);
    this.sensors = List.copyOf(sensors);
    for (int i = 0; i < this.joints.size(); i++) {
      index.put(this.joints.get(i), i);
    }
    int n = this.joints.size();
    parent = new int[n];
    leader = new int[n];
    coupling = new Coupling[n];
    for (int i = 0; i < n; i++) {
      parent[i] = this.joints.get(i).parent() instanceof Joint up ? indexOf(up) : -1;
      leader[i] = -1;
    }
    for (Coupling each : this.couplings) {
      int follower = indexOf(each.follower());
      leader[follower] = indexOf(each.leader());
      coupling[follower] = each;
    }
  }

  /**
   * A robot's mechanisms, from its parts: the joints, how they're coupled, and what drives and
   * senses them. A robot's definition gives its own ({@code RobotDefinition.math()}).
   *
   * @throws IllegalArgumentException if the parts don't make mechanisms: a label used twice, a
   *     joint or a part referred to that isn't among them, a joint following two others or itself
   */
  public static RobotMath of(
      Body body,
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors) {
    List<String> problems = structure(joints, couplings, drives, sensors, List.of());
    if (!problems.isEmpty()) {
      throw new IllegalArgumentException(String.join("; ", problems));
    }
    return new RobotMath(body, joints, couplings, drives, sensors);
  }

  /**
   * A robot's mechanisms, or null when its parts don't make them: what the definition's checks
   * across its named places read, once its parts' own checks have said what's wrong.
   */
  static @Nullable RobotMath whole(
      Body body,
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors) {
    return structure(joints, couplings, drives, sensors, List.of()).isEmpty()
        ? new RobotMath(body, joints, couplings, drives, sensors)
        : null;
  }

  /** The joints, in the order the definition lists them. */
  public List<Joint> joints() {
    return joints;
  }

  /** Whether a joint is one of the robot's. */
  boolean has(Joint joint) {
    return index.containsKey(joint);
  }

  /**
   * The joint a joint follows through a coupling, or null for one that moves on its own.
   *
   * @throws IllegalArgumentException for a joint that isn't the robot's
   */
  public @Nullable Joint leaderOf(Joint joint) {
    int i = indexOf(joint);
    return leader[i] < 0 ? null : joints.get(leader[i]);
  }

  /**
   * Whether a joint is turned by motors alone: one drive of motors, turning a shaft, and no other,
   * and it follows no joint. What its homing, a loop on its encoder, and the program's mechanism
   * for it are worked out for.
   *
   * @throws IllegalArgumentException for a joint that isn't the robot's
   */
  public boolean motorsAlone(Joint joint) {
    if (leaderOf(joint) != null) {
      return false;
    }
    List<Drive> drives = drivesOf(joint);
    return drives.size() == 1
        && drives.get(0).actuator() instanceof Actuator.Motors
        && !(drives.get(0).transmission() instanceof Transmission.Between);
  }

  /** The drives. */
  public List<Drive> drives() {
    return drives;
  }

  /** The sensors. */
  public List<Sensor> sensors() {
    return sensors;
  }

  /**
   * The absolute encoder on a joint alone, its block's or one of the robot's, or null when it has
   * none: what tells the joint where it is at power on.
   */
  public Sensor.@Nullable Encoder absoluteEncoder(Joint joint) {
    for (Sensor sensor : sensors) {
      if (sensor instanceof Sensor.Encoder encoder
          && encoder.absolute()
          && encoder.joints().equals(List.of(joint))) {
        return encoder;
      }
    }
    return null;
  }

  /**
   * A joint's limit switch at one end, its block's or one of the robot's, or null when it has none
   * there: what homing to a switch drives the joint onto.
   */
  public Sensor.@Nullable Switch limitSwitch(Joint joint, Sensor.Switch.Limit.End end) {
    for (Sensor sensor : sensors) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
          && limit.joint().equals(joint)
          && limit.end() == end) {
        return onSwitch;
      }
    }
    return null;
  }

  /**
   * Every joint that moves on its own where it rests: stowed, as the robot is weighed. The joints
   * that follow others aren't in it, since their positions are their leaders' to give, so every
   * method that takes positions takes it as it is; {@link #positions} with nothing given has them
   * all, followers too.
   */
  public Map<Joint, Double> rests() {
    Map<Joint, Double> rests = new LinkedHashMap<>();
    for (int i = 0; i < joints.size(); i++) {
      if (leader[i] < 0) {
        rests.put(joints.get(i), joints.get(i).restPosition());
      }
    }
    return rests;
  }

  /**
   * Every joint's position: the ones given, the rest where they rest, and each coupled joint where
   * its leader puts it.
   *
   * @throws IllegalArgumentException for a joint that isn't the robot's, or one that follows
   *     another, whose position is its leader's to give
   */
  public Map<Joint, Double> positions(Map<Joint, Double> given) {
    double[] q = resolve(given);
    Map<Joint, Double> positions = new LinkedHashMap<>();
    for (int i = 0; i < joints.size(); i++) {
      positions.put(joints.get(i), q[i]);
    }
    return positions;
  }

  /**
   * Where a part's frame is, in the robot's frame: the chassis's is the robot's own; a joint's is
   * its parent's, moved by its origin and then by its position.
   */
  public Pose3d pose(Part part, Map<Joint, Double> positions) {
    if (!(part instanceof Joint joint)) {
      return Pose3d.ZERO;
    }
    return frames(resolve(positions))[indexOf(joint)];
  }

  /**
   * Where a camera is and which way it looks, in the robot's frame: its mount, on its joint's link
   * where the joint's position puts it, or on the chassis.
   */
  public Pose3d cameraPose(CameraConfig camera, Map<Joint, Double> positions) {
    Pose3d on = camera.joint().map(joint -> pose(joint, positions)).orElse(Pose3d.ZERO);
    return on.transformBy(camera.robotToCamera());
  }

  /**
   * Where a switch or a rangefinder is, in the robot's frame: on its part where its joint's
   * position puts it, at its pose there, or the part's origin when that isn't known.
   */
  public Pose3d sensorPose(Sensor.Placed sensor, Map<Joint, Double> positions) {
    Pose3d on = pose(sensor.part(), positions);
    return sensor
        .pose()
        .map(where -> on.transformBy(new Transform3d(where.getTranslation(), where.getRotation())))
        .orElse(on);
  }

  /**
   * What the robot's mass is without its links: the body as weighed, less every link where its
   * joint rests. The simulator's chassis.
   *
   * @throws IllegalArgumentException if nothing's left, or what's left turns more easily than any
   *     body could
   */
  public MassProperties chassis() {
    MassProperties chassis = body.massProperties();
    Pose3d[] frames = frames(resolve(Map.of()));
    for (int i = 0; i < joints.size(); i++) {
      chassis = chassis.minus(placed(i, frames));
    }
    return chassis;
  }

  /**
   * The torque (or force) a joint must make to hold still against gravity, at a set of positions:
   * everything it carries, through the tree and every joint coupled to it. Positive pushes the
   * joint forward. A nested joint's feedforward, which Phoenix's onboard gravity term can't give: a
   * wrist's, as the arm under it turns.
   *
   * @return newton meters for a revolute joint, newtons for a prismatic one
   */
  public double gravityTorque(Joint joint, Map<Joint, Double> positions) {
    int j = independent(joint);
    double[] q = resolve(positions);
    return gravity(j, q, new Translation3d(0, 0, 1));
  }

  /**
   * How far a revolute joint's zero is from horizontal: the angle to add to its position to give
   * the angle {@code ArmFeedforward} takes, and Phoenix's {@code GravityArmPositionOffset}, with
   * the joint's parent level and every other joint where it rests. Zero for a joint gravity doesn't
   * turn, such as a turret, and for one whose links have no mass: its weight is then taken to pull
   * hardest at its zero, unless its control says where ({@code
   * MechanismControl.Gains.gravityAngle}).
   *
   * @throws IllegalArgumentException for a prismatic joint, which doesn't turn against gravity
   */
  public double gravityOffset(Joint joint) {
    int j = independent(joint);
    if (!(joint instanceof Joint.Revolute)) {
      throw new IllegalArgumentException(
          joint.label() + " slides: only a revolute joint turns against gravity");
    }
    double atZero = levelGravity(j, 0);
    double atQuarter = levelGravity(j, Math.PI / 2);
    if (Math.hypot(atZero, atQuarter) < 1e-12) {
      return 0;
    }
    return -Math.atan2(atQuarter, atZero);
  }

  /**
   * How well one cosine, the ideal kG and the gravity offset, fits gravity at a revolute joint
   * across its range (a turn about where it rests, for one that turns freely), its parent level and
   * every other joint where it rests.
   *
   * @throws IllegalArgumentException for a prismatic joint, or one that follows another
   */
  public GravityFit gravityFit(Joint joint) {
    double offset = gravityOffset(joint);
    int j = independent(joint);
    double peak = Math.hypot(levelGravity(j, 0), levelGravity(j, Math.PI / 2));
    double from = joint.range().map(Range::lower).orElse(joint.restPosition() - Math.PI);
    double to = joint.range().map(Range::upper).orElse(joint.restPosition() + Math.PI);
    double widest = 0;
    double worstAt = from;
    for (int i = 0; i <= FIT_STEPS; i++) {
      double angle = from + (to - from) * i / FIT_STEPS;
      double gap = Math.abs(peak * Math.cos(angle + offset) - levelGravity(j, angle));
      if (gap > widest) {
        widest = gap;
        worstAt = angle;
      }
    }
    double misfit = widest == 0 ? 0 : peak > 0 ? widest / peak : Double.POSITIVE_INFINITY;
    return new GravityFit(offset, peak, misfit, worstAt);
  }

  /**
   * Whether gravity can move a joint, whatever it carries: a revolute joint about an axis that
   * isn't vertical, or a prismatic one along an axis that isn't level, with the joints where they
   * rest. Not a turret or a spindexer, which turn about the vertical, nor a slide that runs level.
   * It needs no mass: it's what a tuned kG may hold against.
   */
  public boolean gravityActsOn(Joint joint) {
    Vector3 axis = joint.axis().unit();
    double up =
        Math.abs(
            new Translation3d(axis.x(), axis.y(), axis.z())
                .rotateBy(pose(joint, Map.of()).getRotation())
                .getZ());
    return joint instanceof Joint.Revolute ? up < 1 - 1e-9 : up > 1e-9;
  }

  /**
   * The drives of motors meant to move a joint, which its seeds and free speed come from: those
   * whose every joint moves with it (the joint itself, or one that follows it), where any do;
   * otherwise every drive whose motors turn as it moves. A swerve module's steering is its steering
   * motor's, though the drive motor turns with it through the coupling (the drive motor's other
   * joint, the wheel, stays still as the module steers); a wrist chained through the shoulder's
   * axle leaves the shoulder to the shoulder's motors; and a differential's two axes are both
   * motors', since neither moves either alone.
   *
   * @throws IllegalArgumentException if it follows another joint
   */
  public List<Drive> drivesOf(Joint joint) {
    return movers(sensitivity(independent(joint), resolve(Map.of())));
  }

  /**
   * The joint's ideal gains from its motors ({@link #drivesOf}), their gearing and what it moves,
   * pushing together. They seed a mechanism's feedforward until SysId measures it; they're never
   * the gains.
   *
   * @throws IllegalArgumentException if no motors move it, or it follows another joint
   */
  public Gains idealGains(Joint joint) {
    int j = independent(joint);
    double[] q = resolve(Map.of());
    double[] moved = sensitivity(j, q);
    double pushing = 0;
    double backEmf = 0;
    for (Drive drive : movers(moved)) {
      DCMotor model = ((Actuator.Motors) drive.actuator()).model();
      double g = shaftRate(drive, moved);
      double perVolt = model.stallTorque / model.nominalVoltage;
      double efficiency = drive.transmission().efficiency();
      // Volts per radian per second of the shaft, from the datasheet's free speed, as
      // idealDriveKv takes it: a motor's free current, its own friction, is in it.
      double voltsPerSpeed = model.nominalVoltage / model.freeSpeed;
      pushing += Math.abs(g) * efficiency * perVolt;
      backEmf += g * g * efficiency * perVolt * voltsPerSpeed;
    }
    if (pushing == 0) {
      throw new IllegalArgumentException("no motors move " + joint.label());
    }
    return new Gains(backEmf / pushing, inertia(j, q) / pushing, gravityHeld(joint) / pushing);
  }

  /**
   * What the ideal kG holds against, through the joint's motors: gravity's torque at the joint from
   * its links' mass, at its worst for a revolute joint, everywhere for a prismatic one, with its
   * parent level. Zero for links without mass.
   *
   * @return newton meters for a revolute joint, newtons for a prismatic one
   * @throws IllegalArgumentException if it follows another joint
   */
  double gravityHeld(Joint joint) {
    int j = independent(joint);
    return joint instanceof Joint.Revolute
        ? Math.hypot(levelGravity(j, 0), levelGravity(j, Math.PI / 2))
        : levelGravity(j, joint.restPosition());
  }

  /**
   * How fast the joint can move at its motors' nominal voltage ({@link #drivesOf}), with nothing on
   * it: the ceiling for its profile's cruise velocity, and for its named speeds. In its units per
   * second.
   *
   * @throws IllegalArgumentException if no motors move it, or it follows another joint
   */
  public double freeSpeed(Joint joint) {
    double nominal = Double.POSITIVE_INFINITY;
    for (Drive drive : drivesOf(joint)) {
      nominal = Math.min(nominal, ((Actuator.Motors) drive.actuator()).model().nominalVoltage);
    }
    return nominal / idealGains(joint).kV();
  }

  /**
   * How hard the joint is to accelerate at a set of positions: everything it moves, through the
   * tree and its couplings, and every motor's rotor that turns with it, through its gearing.
   *
   * @return kg·m² for a revolute joint, kg for a prismatic one
   */
  public double inertia(Joint joint, Map<Joint, Double> positions) {
    return inertia(independent(joint), resolve(positions));
  }

  /**
   * A drive's coordinate at a set of positions: the length between a line's two ends, in meters, or
   * its shaft's angle, in radians, the sum over the joints it turns.
   */
  public double coordinate(Drive drive, Map<Joint, Double> positions) {
    double[] q = resolve(positions);
    if (drive.transmission() instanceof Transmission.Between line) {
      Pose3d[] frames = frames(q);
      return point(line.b(), line.onB(), frames).getDistance(point(line.a(), line.onA(), frames));
    }
    double angle = 0;
    for (Joint each : drive.transmission().joints()) {
      angle += drive.transmission().shaftRadiansPer(each) * q[indexOf(each)];
    }
    return angle;
  }

  /**
   * How fast a drive's coordinate changes for a joint's speed, at a set of positions: a gearing's
   * ratio, or a line's lever arm about the joint, which changes as the joint turns.
   */
  public double leverage(Drive drive, Joint joint, Map<Joint, Double> positions) {
    int j = independent(joint);
    double[] q = resolve(positions);
    double[] moved = sensitivity(j, q);
    if (drive.transmission() instanceof Transmission.Between line) {
      Pose3d[] frames = frames(q);
      Translation3d a = point(line.a(), line.onA(), frames);
      Translation3d b = point(line.b(), line.onB(), frames);
      Translation3d along = b.minus(a).div(b.getDistance(a));
      Translation3d apart =
          velocity(line.b(), b, frames, moved).minus(velocity(line.a(), a, frames, moved));
      return along.dot(apart);
    }
    return shaftRate(drive, moved);
  }

  /**
   * The torque (or force) a drive makes at a joint, everything held still: a counterbalance's at an
   * angle, which its feedforward takes off the motors' work; a cylinder's, extending.
   *
   * @param command what the drive's told ({@link Actuator#forceOrTorque})
   */
  public double torque(Drive drive, Joint joint, Map<Joint, Double> positions, double command) {
    double force = drive.actuator().forceOrTorque(command, coordinate(drive, positions), 0);
    return force * leverage(drive, joint, positions) * drive.transmission().efficiency();
  }

  /**
   * Phoenix's {@code SensorToMechanismRatio} for an encoder that reads one joint: its turns for
   * each of its joint's turns, or for each meter of a prismatic joint, so the mechanism's position
   * reads in turns, or in meters. A drive's rotor is an encoder too ({@link Drive#rotor}).
   *
   * @throws IllegalStateException if it reads several joints: ask about one
   */
  public double sensorToMechanismRatio(Sensor.Encoder encoder) {
    return encoder.ratio(encoder.joint());
  }

  /**
   * Phoenix's {@code SensorToMechanismRatio} for an encoder reading several joints, in one of them:
   * a swerve drive motor's rotor in its wheel's turns, a differential's in its average axis.
   *
   * @throws IllegalArgumentException if it doesn't read the joint
   */
  public double sensorToMechanismRatio(Sensor.Encoder encoder, Joint joint) {
    double ratio = encoder.ratio(joint);
    if (ratio == 0) {
      throw new IllegalArgumentException(encoder.label() + " doesn't read " + joint.label());
    }
    return ratio;
  }

  /**
   * Phoenix's {@code RotorToSensorRatio} for a drive's motor fused with an encoder: the rotor's
   * turns for each of the encoder's.
   *
   * @throws IllegalArgumentException if the drive isn't motors, or doesn't turn the encoder's joint
   * @throws IllegalStateException if the encoder reads several joints
   */
  public double rotorToSensorRatio(Drive drive, Sensor.Encoder encoder) {
    if (!(drive.actuator() instanceof Actuator.Motors)) {
      throw new IllegalArgumentException(drive.label() + " has no rotor: it isn't motors");
    }
    Joint joint = encoder.joint();
    double radians = drive.transmission().shaftRadiansPer(joint);
    if (radians == 0) {
      throw new IllegalArgumentException(
          drive.label()
              + " doesn't turn "
              + joint.label()
              + ", which "
              + encoder.label()
              + " reads");
    }
    double turns = joint instanceof Joint.Revolute ? radians : radians / (2 * Math.PI);
    return turns / encoder.ratio(joint);
  }

  /**
   * Phoenix's {@code AbsoluteSensorDiscontinuityPoint} for an absolute encoder, in turns: where its
   * reading wraps, put as far from the joint's range as it can be, with the range centered in the
   * encoder's turn. The encoder's offset, measured per install, makes it read 0 with the joint at
   * 0. A joint that turns freely is centered on where it rests.
   *
   * @throws IllegalArgumentException if it isn't absolute, or the joint's range is more of its turn
   *     than one turn can hold
   */
  public double discontinuityPoint(Sensor.Encoder encoder) {
    if (!encoder.absolute()) {
      throw new IllegalArgumentException(encoder.label() + " isn't absolute: it doesn't wrap");
    }
    Joint joint = encoder.joint();
    double low;
    double high;
    if (joint.range().isPresent()) {
      double a = encoderTurns(encoder, joint.range().get().lower());
      double b = encoderTurns(encoder, joint.range().get().upper());
      low = Math.min(a, b);
      high = Math.max(a, b);
    } else {
      low = encoderTurns(encoder, joint.restPosition());
      high = low;
    }
    double point = wrapAt(low, high);
    if (low < point - 1 || high >= point) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s reads %.3f to %.3f turns over %s's range, which no one turn from -1 to 1 holds",
              encoder.label(),
              low,
              high,
              joint.label()));
    }
    return point;
  }

  /**
   * How far inside an absolute encoder's turn its joint's range sits when the encoder wraps at a
   * discontinuity point, in the encoder's turns, at the nearer end: negative when the range crosses
   * the wrap, and too near below {@link #WRAP_MARGIN}. The encoder reads 0 with the joint at 0, and
   * grows as its ratio's sign says, as {@link #discontinuityPoint} takes it. Infinite for a joint
   * that turns freely, which has no range to keep clear.
   *
   * @throws IllegalStateException if it reads several joints
   */
  public static double wrapMargin(Sensor.Encoder encoder, double discontinuityPoint) {
    Joint joint = encoder.joint();
    if (joint.range().isEmpty()) {
      return Double.POSITIVE_INFINITY;
    }
    double a = encoderTurns(encoder, joint.range().get().lower());
    double b = encoderTurns(encoder, joint.range().get().upper());
    return Math.min(Math.min(a, b) - (discontinuityPoint - 1), discontinuityPoint - Math.max(a, b));
  }

  /**
   * How far a joint can move with its absolute encoders still telling every position apart, as a
   * decoder reads them, at their resolution. In the joint's units, radians or meters.
   *
   * <p>One encoder tells its own turn apart, through its ratio. Two decode one of two ways:
   *
   * <ul>
   *   <li>Coarse then fine, when one turns at least twice as fast as the other: the coarse one
   *       picks which of the fine one's turns the joint is in, and the fine one says where in it.
   *       The span is the coarse one's alone; the fine one adds precision, never reach. Readings
   *       repeating together farther out don't count: one count off, or the backlash between them,
   *       picks the wrong turn.
   *   <li>A pair geared nearly alike, decoded by their difference (a vernier): it turns as slowly
   *       as their ratios differ, so it tells apart far more than either, as two encoders on 17 and
   *       18 teeth against a turret's ring tell more than three of its turns apart. Only if their
   *       resolution lets the difference pick the right turn of one of them: a count off on each
   *       must move it less than half that turn.
   * </ul>
   *
   * With more than two, the best one or pair is what a decoder would read.
   *
   * @throws IllegalArgumentException if there are none, they aren't all absolute and on one joint,
   *     or a pair geared nearly alike doesn't give each one's counts per turn
   */
  public double absoluteSpan(List<Sensor.Encoder> encoders) {
    return decoding(encoders).span();
  }

  /**
   * Where a joint is from one absolute encoder's reading: in the joint's units, wrapped into the
   * window the encoder tells apart ({@link #absoluteWindow}). What a motor controller is told at
   * startup, from an encoder read once, as a turret's coarse one is.
   *
   * @param turns the encoder's reading in its turns, its offset applied so it reads zero with the
   *     joint at zero, and growing as the joint moves forward
   * @throws IllegalArgumentException as {@link #absoluteWindow} does
   */
  public double fromAbsolute(Sensor.Encoder encoder, double turns) {
    Range window = absoluteWindow(List.of(encoder));
    Joint joint = encoder.joint();
    double perTurn = joint instanceof Joint.Revolute ? 2 * Math.PI : 1;
    double position = turns / Math.abs(encoder.ratio(joint)) * perTurn;
    double span = window.upper() - window.lower();
    double into = (position - window.lower()) % span;
    return window.lower() + (into < 0 ? into + span : into);
  }

  /**
   * Where a decoder reading these absolute encoders has its window: the joint's positions it tells
   * apart, from lower to upper, in the joint's units, as wide as their span ({@link
   * #absoluteSpan}). It wraps everything it reads into this window. For one encoder, its
   * discontinuity point puts it ({@link #discontinuityPoint}): its turn centered on the joint's
   * range, as far as Phoenix's point, from 0 to 1, can go. For a pair, centered on the range,
   * wherever that is: where their difference wraps is the decoder's own to choose. A joint that
   * turns freely is centered on where it rests.
   *
   * @throws IllegalArgumentException as {@link #absoluteSpan} does
   */
  public Range absoluteWindow(List<Sensor.Encoder> encoders) {
    Decoding decoding = decoding(encoders);
    Joint joint = encoders.get(0).joint();
    double perTurn = joint instanceof Joint.Revolute ? 2 * Math.PI : 1;
    Sensor.Encoder encoder = decoding.single();
    if (encoder != null) {
      double low;
      double high;
      if (joint.range().isPresent()) {
        double a = encoderTurns(encoder, joint.range().get().lower());
        double b = encoderTurns(encoder, joint.range().get().upper());
        low = Math.min(a, b);
        high = Math.max(a, b);
      } else {
        low = encoderTurns(encoder, joint.restPosition());
        high = low;
      }
      double point = wrapAt(low, high);
      double ratio = encoder.ratio(joint);
      double from = (point - 1) / ratio * perTurn;
      double to = point / ratio * perTurn;
      return Range.in(joint, Math.min(from, to), Math.max(from, to));
    }
    double middle =
        joint
            .range()
            .map(range -> (range.lower() + range.upper()) / 2)
            .orElse(joint.restPosition());
    double span = decoding.span();
    return Range.in(joint, middle - span / 2, middle + span / 2);
  }

  /**
   * How a decoder reads absolute encoders: the span it tells apart, in the joint's units, and the
   * one encoder that tells it, or null for a pair.
   */
  private record Decoding(double span, Sensor.@Nullable Encoder single) {}

  private static Decoding decoding(List<Sensor.Encoder> encoders) {
    if (encoders.isEmpty()) {
      throw new IllegalArgumentException("no encoders tell anything apart");
    }
    Joint joint = encoders.get(0).joint();
    for (Sensor.Encoder encoder : encoders) {
      if (!encoder.absolute() || !encoder.joints().equals(List.of(joint))) {
        throw new IllegalArgumentException(
            encoder.label() + ": a span is of absolute encoders on one joint");
      }
    }
    // In the joint's turns (or meters): each encoder's readings repeat every 1 / |ratio|.
    double best = 0;
    Sensor.Encoder single = null;
    for (Sensor.Encoder encoder : encoders) {
      double own = 1 / Math.abs(encoder.ratio(joint));
      if (own > best) {
        best = own;
        single = encoder;
      }
    }
    for (int i = 0; i < encoders.size(); i++) {
      for (int j = i + 1; j < encoders.size(); j++) {
        double beat = vernier(encoders.get(i), encoders.get(j), joint);
        if (beat > best) {
          best = beat;
          single = null;
        }
      }
    }
    double perTurn = joint instanceof Joint.Revolute ? 2 * Math.PI : 1;
    return new Decoding(best * perTurn, single);
  }

  /**
   * What a pair tells apart decoded by their difference, in the joint's turns: the turn of their
   * difference, if it's longer than either's own and a count off on each still picks the right turn
   * of the slower one; 0 otherwise, for a pair a decoder reads coarse then fine.
   *
   * @throws IllegalArgumentException for a pair geared nearly alike without each one's counts per
   *     turn, which is all that says whether their difference picks the right turn
   */
  private static double vernier(Sensor.Encoder a, Sensor.Encoder b, Joint joint) {
    double ra = Math.abs(a.ratio(joint));
    double rb = Math.abs(b.ratio(joint));
    double apart = Math.abs(ra - rb);
    if (!(apart < Math.min(ra, rb))) {
      return 0;
    }
    if (a.countsPerTurn().isEmpty() || b.countsPerTurn().isEmpty()) {
      throw new IllegalArgumentException(
          "decoding "
              + a.label()
              + " and "
              + b.label()
              + " by their difference needs each encoder's counts per turn: give them");
    }
    // The difference is off by up to a count of each, in their turns; through the beat, that's
    // off by this much of the joint's turn, which must be under half the slower one's turn.
    double counts = 1.0 / a.countsPerTurn().get() + 1.0 / b.countsPerTurn().get();
    return 2 * Math.min(ra, rb) * counts < apart ? 1 / apart : 0;
  }

  /**
   * Where an encoder's reading wraps, in its turns, as far as can be from readings low to high: the
   * middle of the turn on the middle of them, held to Phoenix's 0 to 1.
   */
  private static double wrapAt(double low, double high) {
    return Math.max(0, Math.min(1, (low + high) / 2 + 0.5));
  }

  /**
   * How far the robot's links reach beyond its frame perimeter at a set of positions, seen from
   * above: what the game's extension limit is checked against. Zero within it.
   */
  public double extension(Map<Joint, Double> positions) {
    double halfLength = body.frameLength().in(Meters) / 2;
    double halfWidth = body.frameWidth().in(Meters) / 2;
    double farthest = 0;
    for (Translation3d point : points(positions)) {
      double x = Math.max(0, Math.abs(point.getX()) - halfLength);
      double y = Math.max(0, Math.abs(point.getY()) - halfWidth);
      farthest = Math.max(farthest, Math.hypot(x, y));
    }
    return farthest;
  }

  /**
   * The highest any link reaches at a set of positions, above the floor; zero without links. The
   * starting height limit is checked against it with the joints where they rest, and against the
   * body's height.
   */
  public double top(Map<Joint, Double> positions) {
    double top = 0;
    for (Translation3d point : points(positions)) {
      top = Math.max(top, point.getZ());
    }
    return top;
  }

  /** Every point of every link's shapes, in the robot's frame. */
  private List<Translation3d> points(Map<Joint, Double> positions) {
    Pose3d[] frames = frames(resolve(positions));
    List<Translation3d> points = new ArrayList<>();
    for (int i = 0; i < joints.size(); i++) {
      Pose3d frame = frames[i];
      for (Shape shape : joints.get(i).link().shapes()) {
        for (Translation3d point : shape.points()) {
          points.add(frame.getTranslation().plus(point.rotateBy(frame.getRotation())));
        }
      }
    }
    return points;
  }

  /**
   * What the definition's parts make together, checked: nothing when they're one robot. What's
   * about a joint or a camera is about it; the rest, the robot's.
   *
   * @param robot the robot's name: what's about the whole of it is about this
   */
  static List<Finding> problems(
      String robot,
      Body body,
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors,
      List<CameraConfig> cameras) {
    List<String> structure = structure(joints, couplings, drives, sensors, cameras);
    if (!structure.isEmpty()) {
      return structure.stream().map(problem -> Finding.refused(robot, problem)).toList();
    }
    RobotMath math = new RobotMath(body, joints, couplings, drives, sensors);
    List<Finding> problems = new ArrayList<>();
    String budget = math.overBudget();
    if (budget != null) {
      problems.add(Finding.refused(robot, budget));
    } else if (!joints.isEmpty()) {
      String chassis;
      try {
        chassis = math.chassis().inertia().whyNotSolid();
      } catch (IllegalArgumentException e) {
        chassis = String.valueOf(e.getMessage()).replace('\n', ' ');
      }
      if (chassis != null) {
        problems.add(
            Finding.refused(
                robot, "the chassis, the body less its links at rest, isn't a body: " + chassis));
      }
    }
    for (Joint joint : joints) {
      List<String> encoders = new ArrayList<>();
      math.encoderProblems(joint, encoders);
      encoders.forEach(problem -> problems.add(Finding.refused(joint.label(), problem)));
    }
    Places.jointProblems(math, problems);
    for (CameraConfig camera : cameras) {
      List<String> mounted = new ArrayList<>();
      camera.joint().ifPresent(joint -> math.cameraProblems(camera, joint, mounted));
      mounted.forEach(problem -> problems.add(Finding.refused(camera.name(), problem)));
    }
    return problems;
  }

  /**
   * Why the links weigh too much for the robot they're on, or null when they don't: their masses
   * together, all a part of what the robot weighs, must leave some for the chassis, which carries
   * the drivetrain, the electronics, the bumpers and the battery.
   */
  private @Nullable String overBudget() {
    double links = linksKilograms();
    double robot = body.mass().in(Kilograms);
    return links < robot
        ? null
        : String.format(
            Locale.ROOT,
            "its links weigh %.2f kg together, %s the robot's %.2f kg%s: nothing is left for the"
                + " chassis",
            links,
            links > robot ? "more than" : "as much as",
            robot,
            body.weight().weighed() ? " as weighed" : ", the season's most");
  }

  /** What the links weigh together, in kilograms: the ones given; none given weigh nothing. */
  double linksKilograms() {
    double total = 0;
    for (Joint joint : joints) {
      total += joint.link().mass().mass().in(Kilograms);
    }
    return total;
  }

  /**
   * A joint's absolute encoders must tell its whole range apart, and wrap outside it by at least
   * {@link #WRAP_MARGIN} of the turn that wraps: one encoder's own, or a pair's difference's.
   */
  private void encoderProblems(Joint joint, List<String> problems) {
    List<Sensor.Encoder> absolute = new ArrayList<>();
    for (Sensor sensor : sensors) {
      if (sensor instanceof Sensor.Encoder encoder
          && encoder.absolute()
          && encoder.joints().equals(List.of(joint))) {
        absolute.add(encoder);
      }
    }
    if (absolute.isEmpty() || joint.range().isEmpty()) {
      return;
    }
    Range range = joint.range().get();
    double span;
    try {
      span = absoluteSpan(absolute);
    } catch (IllegalArgumentException e) {
      problems.add(String.valueOf(e.getMessage()));
      return;
    }
    if (absolute.size() == 1 && joint.ownSensors().contains(absolute.get(0))) {
      String own = ownEncoderProblem(joint, absolute.get(0));
      if (own != null) {
        problems.add(own);
      }
      return;
    }
    if (!(span > range.width())) {
      problems.add(
          String.format(
              Locale.ROOT,
              "%s's absolute encoders tell %s of its %s apart: gear one differently, or add one",
              joint.label(),
              joint.shown(span),
              joint.shown(range.width())));
      return;
    }
    Range window = absoluteWindow(absolute);
    double margin = WRAP_MARGIN * span;
    if (range.lower() < window.lower() || range.upper() >= window.upper()) {
      problems.add(
          String.format(
              Locale.ROOT,
              "%s's absolute encoders read it from %s to %s, and its range, %s to %s, crosses where"
                  + " they wrap",
              joint.label(),
              joint.shown(window.lower()),
              joint.shown(window.upper()),
              joint.shown(range.lower()),
              joint.shown(range.upper())));
    } else if (range.lower() < window.lower() + margin || range.upper() > window.upper() - margin) {
      problems.add(
          String.format(
              Locale.ROOT,
              "%s's absolute encoders read it from %s to %s, and its range, %s to %s, comes within"
                  + " %s of where they wrap, a fortieth of their turn: a reading a hair past the"
                  + " wrap reads a turn away",
              joint.label(),
              joint.shown(window.lower()),
              joint.shown(window.upper()),
              joint.shown(range.lower()),
              joint.shown(range.upper()),
              joint.shown(
                  Math.min(range.lower() - window.lower(), window.upper() - range.upper()))));
    }
  }

  /**
   * What's wrong with the absolute encoder in a joint's block, the joint's only one, as {@link
   * #encoderProblems} judges any, said in its block's terms with what to do, or null when nothing
   * is: its travel must be inside one of its turns, a fortieth of a turn clear of its wrap at each
   * end, and within a turn of where it reads 0, the joint's zero, where Phoenix can put its wrap.
   * The ratio that fits is its block's ratio, scaled until it does: its turns for each of the
   * joint's, or of a sliding joint's drum.
   */
  private static @Nullable String ownEncoderProblem(Joint joint, Sensor.Encoder encoder) {
    Range range = joint.range().orElseThrow();
    // In turns of its signal, which is the encoder's own turn but for a potentiometer's range.
    double a = encoderTurns(encoder, range.lower());
    double b = encoderTurns(encoder, range.upper());
    double low = Math.min(a, b);
    double high = Math.max(a, b);
    double turns = high - low;
    double point = wrapAt(low, high);
    if (low >= point - 1 + WRAP_MARGIN && high <= point - WRAP_MARGIN) {
      return null;
    }
    Driven.AbsoluteEncoder block = joint.driven().absoluteEncoder().orElseThrow();
    double perSignal = block.turnsPerSignal();
    double farthest = Math.max(Math.abs(low), Math.abs(high));
    // Geared slower, its readings shrink toward 0 by as much: by this, its turn holds the travel
    // with the margin at each end, and it reads within a turn of 0, less the margin, all along it.
    double scale = Math.min((1 - 2 * WRAP_MARGIN) / turns, (1 - WRAP_MARGIN) / farthest);
    String travel =
        String.format(
            Locale.ROOT,
            "its travel, %s to %s",
            joint.shown(range.lower()),
            joint.shown(range.upper()));
    String why;
    String or;
    if (turns >= 1 - 2 * WRAP_MARGIN) {
      // Where the joint's zero is far from its travel, reading within a turn of 0 from it bounds
      // the gearing more than the travel does: said, or the bound can't be worked out from this.
      boolean zeroBounds = (1 - WRAP_MARGIN) / farthest < (1 - 2 * WRAP_MARGIN) / turns;
      why =
          String.format(
              Locale.ROOT,
              "%s's absolute encoder turns %s times over %s: %s must cover the travel, with a"
                  + " fortieth of it to spare at each end,%s to tell where %s is",
              joint.label(),
              Rules.shown(turns * perSignal),
              travel,
              perSignal == 1
                  ? "one of its turns"
                  : "its signal's range, " + Rules.shown(perSignal) + " of its turns,",
              zeroBounds
                  ? String.format(
                      Locale.ROOT,
                      " and it must read within %s turns of 0 all along it, counting from 0 at %s's"
                          + " zero,",
                      Rules.shown((1 - WRAP_MARGIN) * perSignal),
                      joint.label())
                  : "",
              joint.label());
      or =
          (zeroBounds
                  ? "move " + possessive(joint.label()) + " zero, its origin, to a hard stop, or "
                  : "")
              + "leave it out and give "
              + possessive(joint.label())
              + " control a homing recipe";
    } else {
      why =
          String.format(
              Locale.ROOT,
              "%s's absolute encoder reads %s to %s turns over %s, counting from 0 at %s's zero,"
                  + " and must read within %s turns of 0 all along it, where its wrap can be put",
              joint.label(),
              Rules.shown(low * perSignal),
              Rules.shown(high * perSignal),
              travel,
              joint.label(),
              Rules.shown((1 - WRAP_MARGIN) * perSignal));
      or = "move " + possessive(joint.label()) + " zero, its origin, to a hard stop";
    }
    return String.format(
        Locale.ROOT,
        "%s. Gear it to at most %s of its turns for each %s, or %s",
        why,
        floored(block.ratio() * scale),
        joint instanceof Joint.Revolute
            ? "of " + possessive(joint.label()) + " turns"
            : "turn of " + possessive(joint.label()) + " drum",
        or);
  }

  /** A positive number rounded down to three significant figures: a bound it's safe to meet. */
  private static String floored(double value) {
    return new BigDecimal(value)
        .round(new MathContext(3, RoundingMode.FLOOR))
        .stripTrailingZeros()
        .toPlainString();
  }

  /** A camera on a joint sits on its link, and inside the robot when stowed. */
  private void cameraProblems(CameraConfig camera, Joint joint, List<String> problems) {
    List<Shape> shapes = joint.link().shapes();
    Translation3d mount = camera.position();
    if (!shapes.isEmpty()) {
      double off = Double.POSITIVE_INFINITY;
      for (Shape shape : shapes) {
        off = Math.min(off, shape.distance(mount));
      }
      if (off > CAMERA_ON_LINK_METERS) {
        problems.add(
            String.format(
                Locale.ROOT,
                "camera %s is %.2f m off %s, the link it rides on",
                camera.name(),
                off,
                joint.link().label()));
      }
    }
    Translation3d stowed = cameraPose(camera, Map.of()).getTranslation();
    if (!body.contains(stowed)) {
      problems.add(
          "camera " + camera.name() + " is outside the robot with its joint stowed, at " + stowed);
    }
  }

  /** What's wrong with how the parts refer to each other: nothing when nothing is. */
  private static List<String> structure(
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors,
      List<CameraConfig> cameras) {
    List<String> problems = new ArrayList<>();
    unique("joints", joints.stream().map(Joint::label).toList(), problems);
    unique("links", joints.stream().map(joint -> joint.link().label()).toList(), problems);
    unique("drives", drives.stream().map(Drive::label).toList(), problems);
    unique("sensors", sensors.stream().map(Sensor::label).toList(), problems);
    Set<Joint> known = new HashSet<>(joints);
    for (Joint joint : joints) {
      known(joint.parent(), "joint " + possessive(joint.label()) + " parent", known, problems);
    }
    for (Coupling each : couplings) {
      known(each.leader(), "a coupling's leader", known, problems);
      known(each.follower(), "a coupling's follower", known, problems);
    }
    for (Drive drive : drives) {
      for (Joint joint : drive.transmission().joints()) {
        known(joint, "drive " + possessive(drive.label()) + " joint", known, problems);
      }
    }
    for (Sensor sensor : sensors) {
      List<? extends Part> parts =
          switch (sensor) {
            case Sensor.Encoder encoder -> encoder.joints();
            case Sensor.Switch onSwitch ->
                onSwitch.kind() instanceof Sensor.Switch.Limit limit
                    ? List.of(onSwitch.part(), limit.joint())
                    : List.of(onSwitch.part());
            case Sensor.Rangefinder rangefinder -> List.of(rangefinder.part());
          };
      for (Part part : parts) {
        known(part, "sensor " + possessive(sensor.label()) + " joint", known, problems);
      }
    }
    for (CameraConfig camera : cameras) {
      camera
          .joint()
          .ifPresent(
              joint ->
                  known(joint, "camera " + possessive(camera.name()) + " joint", known, problems));
    }
    if (!problems.isEmpty()) {
      return problems;
    }
    Map<Joint, Coupling> leading = new HashMap<>();
    for (Coupling each : couplings) {
      Coupling before = leading.put(each.follower(), each);
      if (before != null) {
        problems.add(each.follower().label() + " follows two joints: it may follow one");
      }
    }
    for (Coupling each : couplings) {
      Joint up = each.leader();
      for (int steps = 0; steps <= couplings.size(); steps++) {
        if (up.equals(each.follower())) {
          problems.add(each.follower().label() + " follows itself, through its couplings");
          break;
        }
        Coupling next = leading.get(up);
        if (next == null) {
          break;
        }
        up = next.leader();
      }
    }
    for (Coupling each : couplings) {
      double expected = each.followerAt(each.leader().restPosition());
      if (Math.abs(expected - each.follower().restPosition()) > 1e-6) {
        problems.add(
            String.format(
                Locale.ROOT,
                "%s's rest (%.4g) isn't where its coupling puts it with %s at its own (%.4g)",
                each.follower().label(),
                each.follower().restPosition(),
                each.leader().label(),
                expected));
      }
    }
    return problems;
  }

  private static void unique(String what, List<String> labels, List<String> problems) {
    Set<String> seen = new HashSet<>();
    for (String label : labels) {
      if (!seen.add(label)) {
        problems.add("two " + what + " are labeled " + label);
      }
    }
  }

  private static void known(Part part, String what, Set<Joint> known, List<String> problems) {
    if (part instanceof Joint joint && !known.contains(joint)) {
      problems.add(what + ", " + joint.label() + ", isn't one of the robot's joints: list it");
    }
  }

  // The machinery: joints by index, positions as an array in that order.

  private int indexOf(Joint joint) {
    Integer i = index.get(joint);
    if (i == null) {
      throw new IllegalArgumentException(joint.label() + " isn't one of the robot's joints");
    }
    return i;
  }

  /** A joint's index, for one that moves on its own rather than following another. */
  private int independent(Joint joint) {
    int j = indexOf(joint);
    if (leader[j] >= 0) {
      throw new IllegalArgumentException(
          joint.label()
              + " follows "
              + joints.get(leader[j]).label()
              + ": ask about "
              + joints.get(leader[j]).label());
    }
    return j;
  }

  /** Every joint's position, by index: given, where it rests, or following its leader. */
  private double[] resolve(Map<Joint, Double> given) {
    int n = joints.size();
    double[] q = new double[n];
    for (int i = 0; i < n; i++) {
      q[i] = joints.get(i).restPosition();
    }
    for (Map.Entry<Joint, Double> entry : given.entrySet()) {
      int i = independent(entry.getKey());
      if (!Double.isFinite(entry.getValue())) {
        throw new IllegalArgumentException(
            possessive(entry.getKey().label()) + " position must be a number: " + entry.getValue());
      }
      q[i] = entry.getValue();
    }
    boolean[] done = new boolean[n];
    for (int i = 0; i < n; i++) {
      follow(i, q, done);
    }
    return q;
  }

  private double follow(int i, double[] q, boolean[] done) {
    Coupling each = coupling[i];
    if (each == null || done[i]) {
      return q[i];
    }
    q[i] = each.followerAt(follow(leader[i], q, done));
    done[i] = true;
    return q[i];
  }

  /** Each joint's frame in the robot's, at positions by index. */
  private Pose3d[] frames(double[] q) {
    Pose3d[] frames = new Pose3d[joints.size()];
    for (int i = 0; i < frames.length; i++) {
      frame(i, q, frames);
    }
    return frames;
  }

  private Pose3d frame(int i, double[] q, @Nullable Pose3d[] frames) {
    Pose3d known = frames[i];
    if (known != null) {
      return known;
    }
    Joint joint = joints.get(i);
    Pose3d base = parent[i] < 0 ? Pose3d.ZERO : frame(parent[i], q, frames);
    Pose3d frame = base.transformBy(joint.origin()).transformBy(joint.motion(q[i]));
    frames[i] = frame;
    return frame;
  }

  /** A point on a part, in the robot's frame. */
  private Translation3d point(Part part, Translation3d on, Pose3d[] frames) {
    if (!(part instanceof Joint joint)) {
      return on;
    }
    Pose3d frame = frames[indexOf(joint)];
    return frame.getTranslation().plus(on.rotateBy(frame.getRotation()));
  }

  /** A joint's axis, a unit vector in the robot's frame. */
  private Translation3d axis(int i, Pose3d[] frames) {
    Vector3 axis = joints.get(i).axis().unit();
    return new Translation3d(axis.x(), axis.y(), axis.z()).rotateBy(frames[i].getRotation());
  }

  /**
   * How fast each joint moves for one joint's speed, by index: 1 for itself, its couplings' slopes
   * for the joints that follow it, 0 for the rest.
   */
  private double[] sensitivity(int j, double[] q) {
    double[] moved = new double[joints.size()];
    boolean[] done = new boolean[moved.length];
    for (int i = 0; i < moved.length; i++) {
      rate(i, j, q, moved, done);
    }
    return moved;
  }

  private double rate(int i, int j, double[] q, double[] moved, boolean[] done) {
    if (!done[i]) {
      Coupling each = coupling[i];
      moved[i] =
          i == j
              ? 1
              : each == null ? 0 : each.slopeAt(q[leader[i]]) * rate(leader[i], j, q, moved, done);
      done[i] = true;
    }
    return moved[i];
  }

  /** How fast a point on a part moves, in the robot's frame, for the joints' rates. */
  private Translation3d velocity(Part part, Translation3d at, Pose3d[] frames, double[] moved) {
    Translation3d velocity = Translation3d.ZERO;
    int i = part instanceof Joint joint ? indexOf(joint) : -1;
    for (; i >= 0; i = parent[i]) {
      if (moved[i] == 0) {
        continue;
      }
      Translation3d axis = axis(i, frames);
      Translation3d swing =
          joints.get(i) instanceof Joint.Revolute
              ? cross(axis, at.minus(frames[i].getTranslation()))
              : axis;
      velocity = velocity.plus(swing.times(moved[i]));
    }
    return velocity;
  }

  /** How fast a joint's link turns, in the robot's frame, for the joints' rates. */
  private Translation3d spin(int link, Pose3d[] frames, double[] moved) {
    Translation3d spin = Translation3d.ZERO;
    for (int i = link; i >= 0; i = parent[i]) {
      if (moved[i] != 0 && joints.get(i) instanceof Joint.Revolute) {
        spin = spin.plus(axis(i, frames).times(moved[i]));
      }
    }
    return spin;
  }

  /** A link's mass properties, in the robot's frame, at its joint's frame. */
  private MassProperties placed(int i, Pose3d[] frames) {
    Pose3d frame = frames[i];
    return joints
        .get(i)
        .link()
        .mass()
        .transformedBy(new Transform3d(frame.getTranslation(), frame.getRotation()));
  }

  /**
   * Gravity's generalized force at a joint: every link's weight times how fast its center rises for
   * the joint's speed, with "up" the direction given.
   */
  private double gravity(int j, double[] q, Translation3d up) {
    Pose3d[] frames = frames(q);
    double[] moved = sensitivity(j, q);
    double torque = 0;
    for (int i = 0; i < joints.size(); i++) {
      MassProperties link = placed(i, frames);
      Translation3d rise = velocity(joints.get(i), link.cg(), frames, moved);
      torque += link.mass().in(Kilograms) * GRAVITY * rise.dot(up);
    }
    return torque;
  }

  /**
   * Gravity's at a joint with its parent level: the other joints where they rest, up along its
   * parent's z.
   */
  private double levelGravity(int j, double position) {
    double[] q = resolve(Map.of());
    q[j] = position;
    boolean[] done = new boolean[q.length];
    for (int i = 0; i < q.length; i++) {
      follow(i, q, done);
    }
    Pose3d[] frames = frames(q);
    Translation3d up = new Translation3d(0, 0, 1);
    if (parent[j] >= 0) {
      up = up.rotateBy(frames[parent[j]].getRotation());
    }
    return gravity(j, q, up);
  }

  /** The generalized inertia at a joint, rotors included, by index. */
  private double inertia(int j, double[] q) {
    Pose3d[] frames = frames(q);
    double[] moved = sensitivity(j, q);
    double inertia = 0;
    for (int i = 0; i < joints.size(); i++) {
      MassProperties link = placed(i, frames);
      Translation3d velocity = velocity(joints.get(i), link.cg(), frames, moved);
      Translation3d spin = spin(i, frames, moved);
      inertia += link.mass().in(Kilograms) * velocity.getSquaredNorm();
      double rate = spin.getNorm();
      if (rate > 0) {
        inertia +=
            rate * rate * link.inertia().about(new Vector3(spin.getX(), spin.getY(), spin.getZ()));
      }
    }
    for (Drive drive : drives) {
      if (drive.actuator() instanceof Actuator.Motors motors) {
        double g = shaftRate(drive, moved);
        inertia += motors.count() * motors.motor().rotorInertia().in(KilogramSquareMeters) * g * g;
      }
    }
    return inertia;
  }

  /** The drives of motors meant to move a joint, for its moves' rates: {@link #drivesOf}. */
  private List<Drive> movers(double[] moved) {
    List<Drive> turning = new ArrayList<>();
    List<Drive> own = new ArrayList<>();
    for (Drive drive : drives) {
      if (drive.actuator() instanceof Actuator.Motors && shaftRate(drive, moved) != 0) {
        turning.add(drive);
        boolean allMove = true;
        for (Joint each : drive.transmission().joints()) {
          allMove &= moved[indexOf(each)] != 0;
        }
        if (allMove) {
          own.add(drive);
        }
      }
    }
    return own.isEmpty() ? turning : own;
  }

  /** How fast a rotary drive's shaft turns, in radians, for the joints' rates; 0 for a line. */
  private double shaftRate(Drive drive, double[] moved) {
    if (drive.transmission() instanceof Transmission.Between) {
      return 0;
    }
    double rate = 0;
    for (Joint each : drive.transmission().joints()) {
      rate += drive.transmission().shaftRadiansPer(each) * moved[indexOf(each)];
    }
    return rate;
  }

  /**
   * A one-joint encoder's reading, in its turns, at its joint's position, with its offset reading 0
   * at 0.
   */
  private static double encoderTurns(Sensor.Encoder encoder, double position) {
    Joint joint = encoder.joint();
    double turns = joint instanceof Joint.Revolute ? position / (2 * Math.PI) : position;
    return encoder.ratio(joint) * turns;
  }

  private static Translation3d cross(Translation3d a, Translation3d b) {
    return new Translation3d(
        a.getY() * b.getZ() - a.getZ() * b.getY(),
        a.getZ() * b.getX() - a.getX() * b.getZ(),
        a.getX() * b.getY() - a.getY() * b.getX());
  }
}
