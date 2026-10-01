package frc.robot.views.world;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import com.michaelgrundvig.frc.pitwall.scene.SceneState;
import com.michaelgrundvig.frc.pitwall.scene.Style;
import frc.robot.lib.config.Body;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.field.FieldModel;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.function.Function;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * The robot where localization believes it is, as its configuration describes it: its frame and
 * bumpers with an arrow out the front; each wheel at its module, turned to its measured angle, with
 * an arrow where it's driving; each camera at its exact mount, an arrow along its optical axis and
 * a wedge out as far as it reads tags. A click on a camera says how it's mounted, and offers its
 * calibration.
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
    for (CameraConfig camera : cameras) {
      String frame = WorldView.cameraFrame(camera.name());
      scene.frame(frame, FRAME, camera.robotToCamera());
      scene
          .arrow(frame + ".axis", frame)
          .to(CAMERA_ARROW, 0, 0)
          .style(Style.ACCENT)
          .inspect(() -> camera(camera));
      double[] fov = fieldOfView(camera.sensor());
      scene
          .frustum(frame + ".view", frame)
          .fov(fov[0], fov[1])
          .range(0, sightRange)
          .inspect(() -> camera(camera));
      scene.label(frame + ".name", frame).text(camera.name()).at(0, 0, 0.1);
    }
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

  /** What a click on a camera says: where it's mounted, what it sees, and its calibration. */
  private Inspection camera(CameraConfig camera) {
    Inspection said =
        Inspection.of("Camera " + camera.name())
            .detail(
                "Mounted",
                String.format(
                    Locale.ROOT,
                    "%.3f m forward, %.3f m left, %.3f m up",
                    camera.forward().in(Meters),
                    camera.left().in(Meters),
                    camera.up().in(Meters)))
            .detail(
                "Turned",
                String.format(
                    Locale.ROOT,
                    "yaw %.1f°, tilted up %.1f°, roll %.1f°",
                    camera.yaw().in(Degrees),
                    camera.tiltUp().in(Degrees),
                    camera.roll().in(Degrees)))
            .detail(
                "Sees",
                String.format(
                    Locale.ROOT,
                    "%.0f° across its diagonal, %d × %d, out to %.0f m",
                    camera.sensor().diagonalFieldOfView().in(Degrees),
                    camera.sensor().widthPixels(),
                    camera.sensor().heightPixels(),
                    sightRange))
            .detail("From", "the robot's configuration, in Robots.java");
    Optional<String> calibrate = cameraAction.apply(camera.name());
    return calibrate.isPresent() ? said.action(calibrate.get()) : said;
  }
}
