package frc.robot.sim;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotMath;
import frc.robot.subsystems.mechanisms.JointIO;
import frc.robot.subsystems.mechanisms.JointSetup;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The robot's driven joints in the simulator, each a {@link JointIOSim} the joints' mechanisms read
 * and drive as they would real motor controllers. Stepped once a loop, before the robot reads its
 * sensors, as the drivetrain's physics is ({@link DriveSimulation}): gravity on each joint is
 * worked out from where every joint really is as the step starts, so a wrist feels its arm's angle.
 *
 * <p>Closed form, from the robot's definition: no joint pushes on another as it moves, which the
 * simulator's rigid-body engine will add.
 */
public final class JointSimulation {
  private final RobotMath math;
  private final List<JointIOSim> joints = new ArrayList<>();

  /** Where each joint really is, as the step starts: what gravity is worked out from. */
  private final Map<Joint, Double> truth = new LinkedHashMap<>();

  /**
   * @param math what the robot's definition derives: each joint's inertia, gravity and motors
   */
  public JointSimulation(RobotMath math) {
    this.math = math;
  }

  /** A driven joint, simulated: its motor controller, for its mechanism to read and drive. */
  public JointIO add(JointSetup setup) {
    JointIOSim joint = new JointIOSim(math, setup);
    joints.add(joint);
    return joint;
  }

  /** One loop: every joint runs for a time, with gravity where the joints stand as it starts. */
  public void step(double seconds) {
    truth.clear();
    for (JointIOSim joint : joints) {
      truth.put(joint.joint(), joint.position());
    }
    for (JointIOSim joint : joints) {
      joint.step(seconds, -math.gravityTorque(joint.joint(), truth));
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
    throw new IllegalArgumentException(joint.label() + " isn't simulated: it has no setup");
  }
}
