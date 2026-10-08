package com.michaelgrundvig.frc.pitwall.scene;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * What a layer describes once ({@link Layer#describe}): the frames it draws in, the values its
 * shapes' sizes follow, and the shapes that stay until it's described again.
 *
 * <p><b>Frames</b> are named, each with a parent and a pose in it. The scene's root is {@value
 * #ROOT}; a layer adds its own beneath it (a robot under the field, a camera under the robot), and
 * may draw in any frame another layer of the scene adds. A frame named here moves when {@code
 * update} gives it a pose ({@link SceneState#pose}); until then it stands where it was put here.
 *
 * <p><b>Values</b> are numbers a shape's size follows (an arrow's length, an ellipse's radii), set
 * in {@code update} and sent only once they move by their resolution.
 *
 * {@snippet :
 * scene
 *     .frame("robot", "field")
 *     .frame(
 *         "camera",
 *         "robot",
 *         new Transform3d(0.3, 0, 0.5, new Rotation3d(0, Math.toRadians(-10), 0)))
 *     .value("speed", 0.05);
 * scene.box("chassis", "robot").size(0.7, 0.6, 0.2).at(0, 0, 0.1).style(Style.ACCENT);
 * }
 */
public final class SceneBuilder extends ShapeList {
  /** Every scene's root frame: the world it's drawn in, such as the field. */
  public static final String ROOT = "field";

  /** A frame as described: its parent, and its pose there as the page reads it. */
  record Frame(String parent, Map<String, Object> wire) {}

  /**
   * How far a frame must move before it's sent again.
   *
   * @param meters along any axis
   * @param radians about any axis
   */
  record Deadband(double meters, double radians) {
    /** What the page shows: a millimeter, and a tenth of a degree. */
    static final Deadband RESOLUTION =
        new Deadband(Wire.METER_STEP, Math.toRadians(Wire.DEGREE_STEP));
  }

  private final Map<String, Frame> frames = new LinkedHashMap<>();
  private final Map<String, Double> values = new LinkedHashMap<>();
  private final Map<String, Deadband> deadbands = new LinkedHashMap<>();

  SceneBuilder() {}

  /** A frame that starts at its parent's origin, and moves as {@code update} says. */
  public SceneBuilder frame(String name, String parent) {
    return frame(name, parent, 0, 0, 0, 0, 0, 0);
  }

  /**
   * A frame at a pose in its parent: where a camera is mounted on the robot, say.
   *
   * @param roll about x, in radians, as WPILib's {@code Rotation3d} turns
   * @param pitch about y: positive tips x down
   * @param yaw about z: positive turns x toward y
   */
  public SceneBuilder frame(
      String name,
      String parent,
      double x,
      double y,
      double z,
      double roll,
      double pitch,
      double yaw) {
    Wire.name("A frame", name);
    Wire.name("A frame", parent);
    if (name.equals(ROOT)) {
      throw new IllegalArgumentException("every scene has its root frame, " + ROOT + ", already");
    }
    if (name.equals(parent)) {
      throw new IllegalArgumentException("frame " + name + " can't be its own parent");
    }
    Map<String, Object> wire = new LinkedHashMap<>();
    wire.put("parent", parent);
    if (x != 0 || y != 0 || z != 0 || roll != 0 || pitch != 0 || yaw != 0) {
      wire.put("pose", Wire.pose(x, y, z, roll, pitch, yaw));
    }
    if (frames.putIfAbsent(name, new Frame(parent, Collections.unmodifiableMap(wire))) != null) {
      throw new IllegalArgumentException("two frames are named " + name);
    }
    return this;
  }

  /** A frame at a transform from its parent: a camera's {@code robotToCamera()}, say. */
  public SceneBuilder frame(String name, String parent, Transform3d pose) {
    return frame(
        name,
        parent,
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getX(),
        pose.getRotation().getY(),
        pose.getRotation().getZ());
  }

  /** A frame at a pose in its parent: an AprilTag on the field, say. */
  public SceneBuilder frame(String name, String parent, Pose3d pose) {
    return frame(
        name,
        parent,
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getX(),
        pose.getRotation().getY(),
        pose.getRotation().getZ());
  }

  /**
   * How far one of this layer's frames must move before it's sent again: by default a millimeter
   * and a tenth of a degree, what the page shows. A frame whose pose is an estimate that wanders
   * while the robot stands still takes a wider one, so a still robot sends nothing.
   *
   * @param meters along any axis
   * @param radians about any axis
   */
  public SceneBuilder deadband(String frame, double meters, double radians) {
    if (!frames.containsKey(frame)) {
      throw new IllegalArgumentException("a deadband is for a frame this layer adds: " + frame);
    }
    if (!(meters >= Wire.METER_STEP) || !(radians >= Math.toRadians(Wire.DEGREE_STEP))) {
      throw new IllegalArgumentException(
          frame + ": a deadband is at least a millimeter and a tenth of a degree");
    }
    deadbands.put(frame, new Deadband(meters, radians));
    return this;
  }

  /**
   * A value a shape's size can follow, zero until {@code update} sets it.
   *
   * @param resolution the smallest change worth sending, in its unit (meters for a length)
   */
  public SceneBuilder value(String name, double resolution) {
    Wire.name("A value", name);
    if (!(resolution > 0) || Double.isInfinite(resolution)) {
      throw new IllegalArgumentException(
          "value " + name + ": a resolution is a positive number: " + resolution);
    }
    if (values.putIfAbsent(name, resolution) != null) {
      throw new IllegalArgumentException("two values are named " + name);
    }
    return this;
  }

  /** The frames described, by name. */
  Map<String, Frame> frames() {
    return frames;
  }

  /** The values described, by name, with their resolutions. */
  Map<String, Double> values() {
    return values;
  }

  /** The frames given a deadband wider than the page's resolution. */
  Map<String, Deadband> deadbands() {
    return deadbands;
  }
}
