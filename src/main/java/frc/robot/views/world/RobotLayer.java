package frc.robot.views.world;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.pitwall.scene.SceneState;
import com.michaelgrundvig.frc.pitwall.scene.Style;
import frc.robot.lib.config.Body;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.field.FieldModel;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * The robot where localization believes it is, as its configuration describes it: its frame and
 * bumpers with an arrow out the front; each wheel at its module, turned to its measured angle, with
 * an arrow where it's driving; each camera at its exact mount, an arrow along its optical axis and
 * the first meter of its view, named off the robot. A click on a camera says how it's mounted,
 * offers its calibration, and shows its full view ({@link CameraViews}) until the details close.
 */
final class RobotLayer implements Layer {
  /** The robot's frame on the page: the frame the page's Follow follows. */
  static final String FRAME = "robot";

  /** How tall a bumper is, about: the rules allow 5 in. */
  private static final double BUMPER_HEIGHT = 0.127;

  /** How far above the floor bumpers ride. */
  private static final double BUMPER_CLEARANCE = 0.02;

  /** A swerve wheel's width, about. */
  private static final double WHEEL_WIDTH = 0.038;

  /** The arrow along a camera's optical axis. */
  private static final double CAMERA_ARROW = 0.3;

  /**
   * How far a camera's view is always drawn: past the bumpers, to show where it points and how wide
   * it sees, and far short of the 7 m that covered the field four times over.
   */
  static final double VIEW_STUB_METERS = 1.0;

  /** How far a camera's name stays clear of the bumpers. */
  private static final double LABEL_CLEARANCE = 0.15;

  /** How far apart two cameras' names stay, center to center. */
  private static final double LABEL_SPACING = 0.35;

  /** How far above its camera a camera's name is drawn. */
  private static final double LABEL_RISE = 0.1;

  /**
   * How far the estimate must move before the page hears of it: a still robot's estimate wanders a
   * few millimeters as each image corrects it, which is no news, and a centimeter is less than a
   * pixel of a field on a laptop.
   */
  static final double DEADBAND_METERS = 0.01;

  static final double DEADBAND_RADIANS = Math.toRadians(0.5);

  /**
   * How far a wheel must turn before the page hears of it: a wheel at rest reads a few tenths of a
   * degree either way as its encoder settles, and a degree is less than a pixel of its drawing.
   */
  static final double STEER_DEADBAND_RADIANS = Math.toRadians(1);

  /** A wheel's frame only turns: the least deadband for its place, a millimeter. */
  private static final double WHEEL_DEADBAND_METERS = 0.001;

  private final FieldModel field;
  private final Body body;
  private final SwerveConfig drivetrain;
  private final List<CameraConfig> cameras;
  private final double sightRange;
  private final WorldView.Readings readings;
  private final Function<String, Optional<String>> cameraAction;
  private final Translation2d[] modules;

  /** Each module's frame and speed on the page, named once, as the loop reads them often. */
  private final String[] moduleFrames;

  private final String[] moduleSpeeds;

  RobotLayer(
      FieldModel field,
      Body body,
      SwerveConfig drivetrain,
      List<CameraConfig> cameras,
      double sightRange,
      WorldView.Readings readings,
      Function<String, Optional<String>> cameraAction) {
    this.field = field;
    this.body = body;
    this.drivetrain = drivetrain;
    this.cameras = List.copyOf(cameras);
    this.sightRange = sightRange;
    this.readings = readings;
    this.cameraAction = cameraAction;
    modules = drivetrain.moduleTranslations(body);
    int count = Math.min(modules.length, WorldView.MODULES.size());
    moduleFrames = new String[count];
    moduleSpeeds = new String[count];
    for (int i = 0; i < count; i++) {
      moduleFrames[i] = "module." + WorldView.MODULES.get(i);
      moduleSpeeds[i] = "speed." + WorldView.MODULES.get(i);
    }
  }

  @Override
  public String id() {
    return "robot";
  }

  @Override
  public String title() {
    return "Robot";
  }

  @Override
  public void describe(SceneBuilder scene) {
    double height = body.height().in(Meters);
    double bumperLength = body.bumperLength().in(Meters);
    scene.frame(FRAME, SceneBuilder.ROOT).deadband(FRAME, DEADBAND_METERS, DEADBAND_RADIANS);
    scene
        .box("robot.frame", FRAME)
        .size(body.frameLength().in(Meters), body.frameWidth().in(Meters), height)
        .at(0, 0, height / 2)
        .outline()
        .style(Style.ACCENT);
    scene
        .box("robot.bumpers", FRAME)
        .size(bumperLength, body.bumperWidth().in(Meters), BUMPER_HEIGHT)
        .at(0, 0, BUMPER_CLEARANCE + BUMPER_HEIGHT / 2)
        .style(Style.ACCENT);
    scene
        .arrow("robot.front", FRAME)
        .from(0, 0, height + 0.02)
        .to(bumperLength / 2 + 0.2, 0, height + 0.02)
        .style(Style.ACCENT);
    double radius = drivetrain.modules().wheelRadius().in(Meters);
    for (int i = 0; i < moduleFrames.length; i++) {
      String module = WorldView.MODULES.get(i);
      String frame = moduleFrames[i];
      scene
          .frame(frame, FRAME, modules[i].getX(), modules[i].getY(), 0, 0, 0, 0)
          .deadband(frame, WHEEL_DEADBAND_METERS, STEER_DEADBAND_RADIANS)
          .value(moduleSpeeds[i], 0.01);
      scene
          .cylinder("wheel." + module, frame)
          .radius(radius)
          .length(WHEEL_WIDTH)
          .at(0, 0, radius, Math.PI / 2, 0, 0)
          .style(Style.ACCENT);
      scene
          .arrow("drive." + module, frame)
          .from(0, 0, 2 * radius + 0.03)
          .to(1, 0, 2 * radius + 0.03)
          .length(moduleSpeeds[i])
          .style(Style.ACCENT);
    }
    List<Translation3d> labels = labelPlaces(cameras, body);
    for (int i = 0; i < cameras.size(); i++) {
      CameraConfig camera = cameras.get(i);
      String frame = WorldView.cameraFrame(camera.id());
      String fullView = CameraViews.view(camera);
      scene.frame(frame, FRAME, camera.robotToCamera());
      scene
          .arrow(frame + ".axis", frame)
          .to(CAMERA_ARROW, 0, 0)
          .style(Style.ACCENT)
          .inspect(() -> WorldView.camera(camera, sightRange, cameraAction))
          .reveal(fullView);
      double[] fov = fieldOfView(camera.sensor());
      scene
          .frustum(frame + ".stub", frame)
          .fov(fov[0], fov[1])
          .range(0, VIEW_STUB_METERS)
          .inspect(() -> WorldView.camera(camera, sightRange, cameraAction))
          .reveal(fullView);
      Translation3d label = labels.get(i);
      scene
          .label(frame + ".name", FRAME)
          .text(camera.name())
          .at(label.getX(), label.getY(), label.getZ());
    }
  }

  /**
   * Where each camera's name is drawn, in the robot's frame: where its view's stub ends, seen from
   * above, then on along the way it faces while that's on the robot or on a name placed before, so
   * the names stay off the robot and off each other.
   */
  static List<Translation3d> labelPlaces(List<CameraConfig> cameras, Body body) {
    double halfLength = body.bumperLength().in(Meters) / 2 + LABEL_CLEARANCE;
    double halfWidth = body.bumperWidth().in(Meters) / 2 + LABEL_CLEARANCE;
    List<Translation3d> places = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      double facing = camera.yaw().in(Radians);
      double x = camera.forward().in(Meters);
      double y = camera.left().in(Meters);
      double out = VIEW_STUB_METERS;
      for (int step = 0; step < 100; step++, out += 0.1) {
        double atX = x + out * Math.cos(facing);
        double atY = y + out * Math.sin(facing);
        boolean onRobot = Math.abs(atX) < halfLength && Math.abs(atY) < halfWidth;
        boolean onAnother =
            places.stream()
                .anyMatch(
                    place -> Math.hypot(place.getX() - atX, place.getY() - atY) < LABEL_SPACING);
        if (!onRobot && !onAnother) {
          break;
        }
      }
      places.add(
          new Translation3d(
              x + out * Math.cos(facing),
              y + out * Math.sin(facing),
              camera.up().in(Meters) + LABEL_RISE));
    }
    return places;
  }

  @Override
  public void update(SceneState state) {
    Pose2d pose = readings.estimate().pose();
    // On a bump the robot stands as high as the ground under it, tilted as its IMU says.
    state.pose(
        FRAME,
        pose.getX(),
        pose.getY(),
        field.terrain().heightAt(pose.getTranslation()),
        readings.rollRadians(),
        readings.pitchRadians(),
        pose.getRotation().getRadians());
    SwerveModuleVelocity[] measured = readings.modules();
    for (int i = 0; i < measured.length && i < moduleFrames.length; i++) {
      state.pose(
          moduleFrames[i], modules[i].getX(), modules[i].getY(), measured[i].angle.getRadians());
      state.value(moduleSpeeds[i], measured[i].velocity * WorldView.ARROW_SECONDS);
    }
  }

  /**
   * A camera's field of view, side to side and top to bottom, in radians, from its diagonal and its
   * image's shape.
   */
  static double[] fieldOfView(CameraConfig.Sensor sensor) {
    double width = sensor.widthPixels();
    double height = sensor.heightPixels();
    double diagonal = Math.hypot(width, height);
    double focal = diagonal / 2 / Math.tan(sensor.diagonalFieldOfView().in(Radians) / 2);
    return new double[] {2 * Math.atan(width / 2 / focal), 2 * Math.atan(height / 2 / focal)};
  }
}
