package frc.robot.studies;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import frc.robot.RobotConfig;
import frc.robot.lib.robot.Body;
import frc.robot.lib.robot.CameraConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * The camera layouts the study tries when CAD hasn't said where cameras could go ({@link
 * MountOptions}): the robot's own, and a family built around its frame. The family covers the
 * choices CAD makes: how many cameras, where on the frame, which way each faces, how high, and how
 * far up it tilts. Change the lists here to try others.
 */
public final class Candidates {
  /** Off the floor: just above the bumpers, the middle, and high. */
  static final List<Distance> HEIGHTS = List.of(Inches.of(8), Inches.of(16), Inches.of(24));

  /** Tilted up from level. */
  static final List<Angle> TILTS =
      List.of(Degrees.of(0), Degrees.of(10), Degrees.of(20), Degrees.of(30));

  /** How far a corner camera turns out, from straight ahead or straight back. */
  static final List<Angle> SPLAYS = List.of(Degrees.of(15), Degrees.of(30), Degrees.of(45));

  /** How far in from the frame's outside a lens sits: room for the camera's case. */
  static final Distance INSET = Inches.of(1.5);

  private Candidates() {}

  /**
   * Where a camera sits on the frame, and which way it faces.
   *
   * @param ahead 1 at the frame's front, -1 at its back, 0 midway
   * @param left 1 at its left side, -1 at its right, 0 midway
   */
  private record Spot(String name, int ahead, int left, double yawDegrees) {}

  private static final Spot FRONT = new Spot("front", 1, 0, 0);
  private static final Spot BACK = new Spot("back", -1, 0, 180);
  private static final Spot LEFT = new Spot("left", 0, 1, 90);
  private static final Spot RIGHT = new Spot("right", 0, -1, -90);

  /** A corner camera, turned out by the splay from straight ahead (or back) toward its side. */
  private static Spot corner(int ahead, int left, double splay) {
    String name = (ahead > 0 ? "front" : "back") + "-" + (left > 0 ? "left" : "right");
    double yaw = ahead > 0 ? splay : 180 - splay;
    return new Spot(name, ahead, left, left > 0 ? yaw : -yaw);
  }

  private record Layout(String name, List<Spot> spots) {}

  /**
   * The robot's own cameras first, then the family of up to four, which uses the sensor of the
   * robot's first camera.
   */
  public static List<Candidate> of(RobotConfig robot) {
    List<Candidate> all = new ArrayList<>(List.of(own(robot)));
    all.addAll(family(robot.definition().body(), robot.definition().cameras().get(0).sensor(), 4));
    return all;
  }

  /**
   * The robot's own cameras, as {@code Robots} configures them.
   *
   * @throws IllegalArgumentException if the robot has no camera, whose sensor every layout uses
   */
  public static Candidate own(RobotConfig robot) {
    if (robot.definition().cameras().isEmpty()) {
      throw new IllegalArgumentException(
          "robot " + robot.name() + " has no cameras: give it one, the sensor for every layout");
    }
    return new Candidate("The robot's own: " + robot.name(), robot.definition().cameras());
  }

  /**
   * One to four cameras at the frame's corners and the middles of its sides, facing out, at every
   * height the robot reaches and every tilt.
   *
   * @param mostCameras layouts of more cameras than this are left out
   */
  public static List<Candidate> family(Body body, CameraConfig.Sensor sensor, int mostCameras) {
    List<Layout> layouts = new ArrayList<>();
    layouts.add(new Layout("1 camera: ahead", List.of(FRONT)));
    for (Angle splay : SPLAYS) {
      double s = splay.in(Degrees);
      layouts.add(
          new Layout(
              format("2 cameras: front corners, splayed %.0f°", s),
              List.of(corner(1, 1, s), corner(1, -1, s))));
    }
    layouts.add(new Layout("2 cameras: ahead and back", List.of(FRONT, BACK)));
    layouts.add(new Layout("2 cameras: left and right", List.of(LEFT, RIGHT)));
    for (Angle splay : SPLAYS) {
      double s = splay.in(Degrees);
      layouts.add(
          new Layout(
              format("3 cameras: front corners, splayed %.0f°, and back", s),
              List.of(corner(1, 1, s), corner(1, -1, s), BACK)));
    }
    for (Angle splay : SPLAYS) {
      double s = splay.in(Degrees);
      layouts.add(
          new Layout(
              format("4 cameras: corners, splayed %.0f°", s),
              List.of(corner(1, 1, s), corner(1, -1, s), corner(-1, 1, s), corner(-1, -1, s))));
    }
    layouts.add(
        new Layout("4 cameras: ahead, back, left, right", List.of(FRONT, BACK, LEFT, RIGHT)));

    Distance forward = body.frameLength().div(2).minus(INSET);
    Distance left = body.frameWidth().div(2).minus(INSET);
    List<Candidate> family = new ArrayList<>();
    for (Layout layout : layouts) {
      if (layout.spots().size() > mostCameras) {
        continue;
      }
      for (Distance height : HEIGHTS) {
        if (height.gt(body.height())) {
          continue;
        }
        for (Angle tilt : TILTS) {
          List<CameraConfig> cameras = new ArrayList<>();
          for (Spot spot : layout.spots()) {
            cameras.add(
                CameraConfig.builder()
                    .name(spot.name())
                    .forward(forward.times(spot.ahead()))
                    .left(left.times(spot.left()))
                    .up(height)
                    .roll(Degrees.of(0))
                    .tiltUp(tilt)
                    .yaw(Degrees.of(spot.yawDegrees()))
                    .sensor(sensor)
                    .noiseScale(1.0)
                    .build());
          }
          family.add(
              new Candidate(
                  format(
                      "%s, %.0f in up, tilted %.0f°",
                      layout.name(), height.in(Inches), tilt.in(Degrees)),
                  cameras));
        }
      }
    }
    return family;
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }
}
