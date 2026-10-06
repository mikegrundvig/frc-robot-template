package frc.robot.subsystems.mechanisms;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismWiring;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import com.michaelgrundvig.frc.world.robot.Transmission;
import frc.robot.lib.phoenix.PhoenixUnits;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;

/**
 * One driven joint as this program drives it: the joint, from the robot's definition; how it's
 * driven, its control; and how it's wired, on a robot. The template builds the joint's mechanism
 * from it ({@link Joints}), whose moves, gates and idle come from these three and nothing else.
 *
 * <p>It checks the joint against its control when it's built, at startup and in the test that
 * builds every robot: the control's soft limits inside the joint's hard stops, and every named
 * position inside the soft limits, since the joint is never driven past them; a homing recipe only
 * for a joint held at positions, and a hard stop to home to. A place it can't reach is a typo or a
 * stale number, found here rather than in a match. Against the robot's definition ({@link
 * #problems}), a joint held at positions homes by its absolute encoder, or names how it does.
 *
 * @param joint the joint, as the robot's definition has it
 * @param control how it's driven: its gains, soft limits, profile, current limits and neutral mode
 * @param wiring where its devices are on the CAN bus, on a robot; empty for one that only runs in
 *     the simulator, whose joint then stays still on real hardware and says so
 */
@ConfigRecord
public record JointSetup(Joint joint, MechanismControl control, Optional<MechanismWiring> wiring) {
  public JointSetup {
    Checks.on(JointSetup.class, joint.label())
        .check("control.softLimits", () -> outsideStops(joint, control))
        .check("joint.positions", () -> outsideSoftLimits(joint, control))
        .check("control.homing", () -> nothingToHome(joint, control))
        .orThrow();
  }

  /**
   * Each joint's setup, from its block in the definition: every joint with a control, its wiring
   * the block's, in the definition's order.
   */
  public static List<JointSetup> all(RobotDefinition definition) {
    List<JointSetup> setups = new ArrayList<>();
    for (Joint joint : definition.joints()) {
      joint
          .driven()
          .control()
          .ifPresent(
              control ->
                  setups.add(
                      JointSetupBuilder.builder()
                          .joint(joint)
                          .control(control)
                          .wiring(joint.driven().wiring())
                          .build()));
    }
    return setups;
  }

  /** Starts describing a joint's setup, one named value at a time. */
  public static JointSetupBuilder.JointStage builder() {
    return JointSetupBuilder.builder();
  }

  /**
   * A joint's setup without wiring, in a line: it runs in the simulator, and on a robot it stays
   * still and says so, until wired: {@code JointSetup.of(PIVOT, MechanismControl.position(30))}.
   */
  public static JointSetup of(Joint joint, MechanismControl control) {
    return builder().joint(joint).control(control).wiring(Optional.empty()).build();
  }

  /**
   * What's wrong with a robot's setups against its definition, or null when nothing is: each joint
   * the template drives set up once, and no setup for a joint it isn't the robot's, or one the
   * template doesn't drive ({@link #whyNotDriven}).
   */
  public static @Nullable String problems(RobotDefinition definition, List<JointSetup> setups) {
    RobotMath math = definition.math();
    List<String> problems = new ArrayList<>();
    Set<Joint> setUp = new HashSet<>();
    for (JointSetup setup : setups) {
      Joint joint = setup.joint();
      if (!definition.joints().contains(joint)) {
        problems.add(joint.label() + " isn't one of " + definition.name() + "'s joints");
        continue;
      }
      String why = whyNotDriven(math, joint);
      if (why != null) {
        problems.add(joint.label() + " is set up, but the template doesn't drive it: " + why);
      } else {
        String wired = wrongController(math, setup);
        if (wired != null) {
          problems.add(wired);
        }
      }
      if (!setUp.add(joint)) {
        problems.add(joint.label() + " is set up twice");
      }
      if (why == null) {
        String homing = cantHome(math, setup);
        if (homing != null) {
          problems.add(homing);
        }
        String toSwitch = noSwitch(definition, setup);
        if (toSwitch != null) {
          problems.add(toSwitch);
        }
      }
    }
    for (Joint joint : definition.joints()) {
      if (!setUp.contains(joint) && whyNotDriven(math, joint) == null) {
        problems.add(
            joint.label()
                + " is driven by motors and has no setup: give it its control, and on a robot its"
                + " wiring");
      }
    }
    return problems.isEmpty() ? null : String.join("; ", problems);
  }

  /**
   * Why the template doesn't drive a joint, or null when it does: one drive of motors turning a
   * shaft. Any motor of the catalog: in the simulator as it is, on a robot through the controller
   * its wiring names ({@link MechanismWiring.Controller}). A joint that follows another is carried
   * by its leader's moves; a joint driven by a servo, or by two drives (a differential's pair, a
   * hood's two servos), isn't driven by the template yet.
   *
   * @throws IllegalArgumentException for a joint that isn't the robot's
   */
  public static @Nullable String whyNotDriven(RobotMath math, Joint joint) {
    Joint leader = math.leaderOf(joint);
    if (leader != null) {
      return "it follows " + leader.label() + ", whose moves carry it";
    }
    List<Drive> drives = math.drivesOf(joint);
    if (drives.isEmpty()) {
      List<String> movers = new ArrayList<>();
      for (Drive drive : math.drives()) {
        if (math.leverage(drive, joint, math.references()) != 0) {
          movers.add(drive.label());
        }
      }
      return movers.isEmpty()
          ? "nothing drives it"
          : "it's moved by "
              + String.join(" and ", movers)
              + ", and the template drives only motors turning a shaft so far";
    }
    if (drives.size() > 1) {
      return "it's driven by "
          + String.join(" and ", drives.stream().map(Drive::label).toList())
          + " together, which the template doesn't drive yet";
    }
    Drive drive = drives.get(0);
    if (!(drive.actuator() instanceof Actuator.Motors motors)
        || drive.transmission() instanceof Transmission.Between) {
      return drive.label() + " isn't motors turning a shaft";
    }
    try {
      PhoenixUnits.of(joint, drive);
    } catch (IllegalArgumentException e) {
      return String.valueOf(e.getMessage());
    }
    return null;
  }

  /**
   * Why a joint's wiring names a controller that can't drive its motor, or an encoder on a data
   * port its controller doesn't have, or null when neither or the joint has no wiring.
   */
  private static @Nullable String wrongController(RobotMath math, JointSetup setup) {
    if (setup.wiring().isEmpty()
        || !(math.drivesOf(setup.joint()).get(0).actuator() instanceof Actuator.Motors motors)) {
      return null;
    }
    MechanismWiring.Controller controller = setup.wiring().get().controller();
    if (setup.wiring().get().dataPort().isPresent()
        && controller == MechanismWiring.Controller.TALON_FX) {
      return setup.joint().label()
          + "'s wiring names an encoder on its controller's data port, and a TalonFX has none:"
          + " wire it as a CANcoder";
    }
    if (setup.wiring().get().dataPort().isPresent()
        && controller == MechanismWiring.Controller.TALON_FXS) {
      return setup.joint().label()
          + "'s wiring names an encoder on a TalonFXS's data port, which the template doesn't read"
          + " yet: wire it as a CANcoder";
    }
    if (controller.drives(motors.motor())) {
      return null;
    }
    List<String> could = new ArrayList<>();
    for (MechanismWiring.Controller each : MechanismWiring.Controller.values()) {
      if (each.drives(motors.motor())) {
        could.add(each.shown());
      }
    }
    return setup.joint().label()
        + "'s wiring puts its "
        + motors.motor().name()
        + " on "
        + controller.shown()
        + ", which doesn't drive one: "
        + String.join(" or ", could)
        + " does";
  }

  /**
   * Whether a joint is held at positions: it has a range or named positions. One without either, a
   * roller, only runs at outputs, and stops when idle.
   */
  public static boolean holdsPositions(Joint joint) {
    return joint.range().isPresent() || !joint.positions().isEmpty();
  }

  /** The definition's absolute encoder on the joint alone, or null when it has none. */
  public static Sensor.@Nullable Encoder absoluteEncoder(RobotMath math, Joint joint) {
    for (Sensor sensor : math.sensors()) {
      if (sensor instanceof Sensor.Encoder encoder
          && encoder.absolute()
          && encoder.joints().equals(List.of(joint))) {
        return encoder;
      }
    }
    return null;
  }

  /**
   * The joint's absolute encoder, when the robot reads it itself and tells the motor where the
   * joint is from it: on a Systemcore port, or unwired, read only by the simulator. Null for none,
   * or one its motor controller reads, a CANcoder or a SPARK's data port, or for a joint whose
   * control names a homing recipe, which it homes by instead.
   */
  public static Driven.@Nullable AbsoluteEncoder readsAbsolute(JointSetup setup) {
    Driven.AbsoluteEncoder encoder = setup.joint().driven().absoluteEncoder().orElse(null);
    return encoder == null
            || encoder.readByController()
            || setup.control().homing().isPresent()
            || !holdsPositions(setup.joint())
        ? null
        : encoder;
  }

  /**
   * Why a joint can't find where it is, or null when it can: held at positions, it homes by its
   * absolute encoder, read through its CANcoder on a robot, or names a recipe.
   */
  private static @Nullable String cantHome(RobotMath math, JointSetup setup) {
    Joint joint = setup.joint();
    if (!holdsPositions(joint) || setup.control().homing().isPresent()) {
      return null;
    }
    if (absoluteEncoder(math, joint) == null) {
      return joint.label()
          + " has no absolute encoder, so where it is at power on is a guess: give its control a"
          + " homing recipe, Homing.toHardStop, Homing.toSwitch or Homing.assumeAtReference()";
    }
    boolean onPort =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port).isPresent();
    if (setup.wiring().isPresent()
        && setup.wiring().get().cancoder().isEmpty()
        && setup.wiring().get().dataPort().isEmpty()
        && !onPort) {
      return joint.label()
          + " homes by its absolute encoder, and its wiring names no CANcoder, data port or"
          + " Systemcore port to read it on";
    }
    return null;
  }

  /**
   * The definition's limit switch on a joint at one end, or null when it has none there: what
   * homing to a switch drives the joint onto.
   */
  public static Sensor.@Nullable Switch limitSwitch(
      RobotDefinition definition, Joint joint, MechanismControl.Homing.Stop end) {
    Sensor.Switch.Limit.End at =
        end == MechanismControl.Homing.Stop.LOWER
            ? Sensor.Switch.Limit.End.LOWER
            : Sensor.Switch.Limit.End.UPPER;
    for (Sensor sensor : definition.math().sensors()) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
          && limit.joint().equals(joint)
          && limit.end() == at) {
        return onSwitch;
      }
    }
    return null;
  }

  /**
   * Why a joint can't home to its switch, or null when it can, or doesn't: it has none at that end,
   * or where its switch trips isn't known.
   */
  private static @Nullable String noSwitch(RobotDefinition definition, JointSetup setup) {
    if (!(setup.control().homing().orElse(null) instanceof MechanismControl.Homing.ToSwitch to)) {
      return null;
    }
    Joint joint = setup.joint();
    String end = to.end() == MechanismControl.Homing.Stop.LOWER ? "lower" : "upper";
    Sensor.Switch onSwitch = limitSwitch(definition, joint, to.end());
    if (onSwitch == null) {
      return joint.label()
          + " homes to its "
          + end
          + " limit switch, and its definition has none: add it (Sensor.Switch.limit)";
    }
    if (((Sensor.Switch.Limit) onSwitch.kind()).position().isEmpty()) {
      return joint.label()
          + " homes to "
          + onSwitch.label()
          + ", whose position isn't known: measure where it trips, or home another way";
    }
    return null;
  }

  /**
   * Why a joint's homing recipe can't work, or null when it can: a roller has no position to find,
   * and a hard stop must be one the joint has.
   */
  private static @Nullable String nothingToHome(Joint joint, MechanismControl control) {
    if (control.homing().isEmpty()) {
      return null;
    }
    if (!holdsPositions(joint)) {
      return joint.label() + " only runs at outputs: it has no position to home";
    }
    if (control.homing().get() instanceof MechanismControl.Homing.ToHardStop
        && joint.range().isEmpty()) {
      return joint.label() + " turns freely: it has no hard stop to home to";
    }
    return null;
  }

  /** Why the control's soft limits reach past the joint's hard stops, or null when they don't. */
  private static @Nullable String outsideStops(Joint joint, MechanismControl control) {
    if (control.softLimits().isEmpty() || joint.range().isEmpty()) {
      return null;
    }
    MechanismControl.SoftLimits soft = control.softLimits().get();
    Range range = joint.range().get();
    return soft.lower() >= range.lower() && soft.upper() <= range.upper()
        ? null
        : String.format(
            Locale.ROOT,
            "%s to %s must be inside %s's hard stops, %s to %s",
            shown(joint, soft.lower()),
            shown(joint, soft.upper()),
            joint.label(),
            shown(joint, range.lower()),
            shown(joint, range.upper()));
  }

  /**
   * The joint's named positions outside its control's soft limits, or null when they're all inside:
   * where it would be sent, and never get.
   */
  private static @Nullable String outsideSoftLimits(Joint joint, MechanismControl control) {
    if (control.softLimits().isEmpty()) {
      return null;
    }
    MechanismControl.SoftLimits soft = control.softLimits().get();
    List<String> outside = new ArrayList<>();
    for (Position position : joint.positions()) {
      if (position.value() < soft.lower() || position.value() > soft.upper()) {
        outside.add(
            String.format(
                Locale.ROOT,
                "%s, %s, is outside its soft limits, %s to %s: it's never driven there",
                position.name(),
                shown(joint, position.value()),
                shown(joint, soft.lower()),
                shown(joint, soft.upper())));
      }
    }
    return outside.isEmpty() ? null : String.join("; ", outside);
  }

  /** A position in the joint's units as people read it: degrees, or meters and inches. */
  static String shown(Joint joint, double position) {
    return joint instanceof Joint.Revolute
        ? String.format(Locale.ROOT, "%.1f°", Math.toDegrees(position))
        : String.format(Locale.ROOT, "%.3f m (%.2f in)", position, position / 0.0254);
  }
}
