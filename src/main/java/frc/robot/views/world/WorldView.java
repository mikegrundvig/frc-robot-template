package frc.robot.views.world;

import com.michaelgrundvig.frc.pitwall.scene.Scene;
import frc.robot.lib.config.Body;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.SwerveConfig;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.localization.VisionDecision;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * The robot's world on Pitwall's World view: the field it plays on, the robot where localization
 * believes it is, its wheels and cameras as configured, how sure the estimate is, and each camera's
 * sight lines to the tags it saw, solid where the estimator accepted the measurement and dashed
 * where it refused it. A click on a sight line says why; a click on a camera says how it's mounted.
 * In the simulator, the simulator's truth stands beside the estimate as a dashed ghost.
 *
 * <p>It draws the code's own beliefs, not a log: a camera's wedge pointing the wrong way is a typo
 * in {@code Robots.java}, found at a glance. Pitwall draws it; this only says what's there.
 */
public final class WorldView {
  /** The scene's id: the page's address for it is {@code #world/world}. */
  public static final String ID = "world";

  /**
   * How far ahead a wheel's drive arrow reaches: where the wheel will be in this long at its speed.
   */
  static final double ARROW_SECONDS = 0.2;

  /**
   * The swerve modules' names on the page, in the order the drivetrain lists them ({@link
   * SwerveConfig#moduleTranslations}).
   */
  static final List<String> MODULES =
      List.of("front-left", "front-right", "back-left", "back-right");

  /** What the view reads in the robot loop, while a page shows it. */
  public interface Readings {
    /** The localization estimate, as of the last loop. */
    LocalizationEstimate estimate();

    /** The robot's pitch from the IMU: positive is nose down. */
    double pitchRadians();

    /** The robot's roll from the IMU. */
    double rollRadians();

    /**
     * Each module's measured wheel speed and angle, in {@link SwerveConfig#moduleTranslations}'s
     * order.
     */
    SwerveModuleVelocity[] modules();

    /** What the estimator did with each camera measurement in the last loop. */
    List<VisionDecision> decisions();
  }

  private WorldView() {}

  /**
   * The world as a scene for Pitwall: {@code pitwall.scenes().add(WorldView.scene(...))}.
   *
   * @param sightRangeMeters how far a camera's wedge reaches: as far as it reads tags
   * @param truth where the robot really is, in the simulator; null elsewhere
   * @param cameraAction the Pitwall action that calibrates a camera's mount, by the camera's name,
   *     to end a camera's or a sight line's details in
   */
  public static Scene scene(
      FieldModel field,
      Body body,
      SwerveConfig drivetrain,
      List<CameraConfig> cameras,
      double sightRangeMeters,
      Readings readings,
      @Nullable Supplier<Pose3d> truth,
      Function<String, Optional<String>> cameraAction) {
    Scene scene =
        Scene.named(ID, "World")
            .layer(new FieldLayer(field))
            .layer(
                new RobotLayer(
                    field, body, drivetrain, cameras, sightRangeMeters, readings, cameraAction))
            .layer(new Uncertainty(readings))
            .layer(new SightLines(field, cameras, readings, cameraAction));
    if (truth != null) {
      scene.layer(new TruthLayer(body, truth));
    }
    return scene.focus(RobotLayer.FRAME);
  }

  /** A camera's frame on the page, by its id ({@link CameraConfig#id()}). */
  static String cameraFrame(String cameraId) {
    return "camera." + cameraId;
  }

  /** A tag's frame on the page. */
  static String tagFrame(int id) {
    return "tag." + id;
  }

  /** A number to two places, for details. */
  static String meters(double meters) {
    return Double.isFinite(meters) ? String.format(Locale.ROOT, "%.2f m", meters) : "not measured";
  }

  /**
   * A distance that's often small, such as a camera's innovation: to the millimeter under 10 cm,
   * where two places of meters would show a few millimeters as 0.00 m.
   */
  static String offset(double meters) {
    return Double.isFinite(meters) && Math.abs(meters) < 0.1
        ? String.format(Locale.ROOT, "%.0f mm", meters * 1000)
        : meters(meters);
  }

  /** A χ² to three figures, or to the whole number from 100: a small one isn't shown as 0.0. */
  static String chiSquared(double value) {
    String format =
        Math.abs(value) >= 100
            ? "%.0f"
            : Math.abs(value) >= 10 ? "%.1f" : Math.abs(value) >= 1 ? "%.2f" : "%.3f";
    return String.format(Locale.ROOT, format, value);
  }
}
