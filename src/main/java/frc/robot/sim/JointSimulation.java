package frc.robot.sim;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import com.michaelgrundvig.frc.world.robot.Sensor;
import frc.robot.subsystems.mechanisms.AbsoluteEncoderIO;
import frc.robot.subsystems.mechanisms.JointIO;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The robot's driven joints in the simulator, each a {@link JointIOSim} the joints' mechanisms read
 * and drive as they would real motor controllers. Stepped once a loop, before the robot reads its
 * sensors, as the drivetrain's physics is ({@link DriveSimulation}): gravity on each joint is
 * worked out from where every joint really is as the step starts, so a wrist feels its arm's angle
 * through its links' weight (a tuned kG holds it as one cosine, {@code JointPlant}).
 *
 * <p>Closed form, from the robot's definition: no joint pushes on another as it moves.
 */
public final class JointSimulation {
  private final RobotMath math;
  private final List<JointIOSim> joints = new ArrayList<>();
  private final Map<Joint, AbsoluteEncoderIOSim> encoders = new HashMap<>();

  /** Where each joint really is, as the step starts: what gravity is worked out from. */
  private final Map<Joint, Double> truth = new LinkedHashMap<>();

  /**
   * @param math what the robot's definition derives: each joint's inertia, gravity and motors
   */
  public JointSimulation(RobotMath math) {
    this.math = math;
  }

  /**
   * A driven joint, simulated: its motor controller, for its mechanism to read and drive. Its
   * controller runs the control its block gives it; gravity on it is the definition's ({@link
   * JointIOSim}).
   */
  public JointIO add(Joint joint) {
    JointIOSim simulated = new JointIOSim(math, joint);
    joints.add(simulated);
    return simulated;
  }

  /**
   * A joint's absolute encoder, simulated, where the robot reads it itself: on a Systemcore port,
   * or unwired ({@code Joints.readsAbsolute}). Asked for after the joint itself.
   */
  public AbsoluteEncoderIO absoluteEncoder(Joint joint) {
    Sensor.Encoder model = Objects.requireNonNull(math.absoluteEncoder(joint));
    AbsoluteEncoderIOSim encoder =
        new AbsoluteEncoderIOSim(
            sim(joint),
            model,
            joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port).orElse(null));
    encoders.put(joint, encoder);
    return encoder;
  }

  /** Unplugs a joint's absolute encoder, or plugs it back in: it reads nothing while unplugged. */
  public void unplug(Joint joint, boolean unplugged) {
    Objects.requireNonNull(encoders.get(joint), joint.label() + " has no simulated encoder")
            .connected =
        !unplugged;
  }

  /** One loop: every joint runs for a time, with gravity where the joints stand as it starts. */
  public void step(double seconds) {
    truth.clear();
    for (JointIOSim joint : joints) {
      truth.put(joint.joint(), joint.position());
    }
    for (JointIOSim joint : joints) {
      joint.step(seconds, joint.gravityTorque(truth));
    }
  }

  /** Where a camera on a joint really is in the robot's frame, the joints as they really stand. */
  public Transform3d cameraMount(CameraConfig camera) {
    Map<Joint, Double> now = new LinkedHashMap<>();
    for (JointIOSim joint : joints) {
      now.put(joint.joint(), joint.position());
    }
    Pose3d pose = math.cameraPose(camera, now);
    return new Transform3d(pose.getTranslation(), pose.getRotation());
  }

  /** Whether a joint is simulated: one the robot drives. */
  public boolean simulates(Joint joint) {
    for (JointIOSim each : joints) {
      if (each.joint().equals(joint)) {
        return true;
      }
    }
    return false;
  }

  /** Where a joint really is, in its units. */
  public double position(Joint joint) {
    return sim(joint).position();
  }

  /**
   * The current each of a joint's motors draws from the battery, in amps: what its PDH channel
   * reads.
   */
  public double supplyAmps(Joint joint) {
    return sim(joint).supplyAmps();
  }

  /** How fast a joint really moves, in its units a second. */
  public double velocity(Joint joint) {
    return sim(joint).velocity();
  }

  /** Puts a joint somewhere, still: where a test starts it. */
  public void place(Joint joint, double position) {
    sim(joint).place(position);
  }

  /**
   * Powers a joint on somewhere other than its reference, still: its motor controller reads it at
   * its reference until it's homed, as a joint read by its motor alone does.
   */
  public void powerOnAt(Joint joint, double position) {
    sim(joint).powerOnAt(position);
  }

  private JointIOSim sim(Joint joint) {
    for (JointIOSim each : joints) {
      if (each.joint().equals(joint)) {
        return each;
      }
    }
    throw new IllegalArgumentException(
        joint.label() + " isn't simulated: the template doesn't drive it");
  }
}
