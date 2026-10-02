package frc.robot.views.world;

import com.michaelgrundvig.frc.pitwall.scene.Layer;
import com.michaelgrundvig.frc.pitwall.scene.SceneBuilder;
import frc.robot.lib.config.CameraConfig;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;

/**
 * Each camera's full view, out as far as it reads tags: off until switched on, since four of them
 * reach across the field, and shown for one camera while it's clicked ({@link RobotLayer}'s camera
 * reveals its own). It draws in the cameras' frames, which the robot's layer adds.
 */
final class CameraViews implements Layer {
  private final List<CameraConfig> cameras;
  private final double sightRange;
  private final Function<String, Optional<String>> cameraAction;

  CameraViews(
      List<CameraConfig> cameras,
      double sightRange,
      Function<String, Optional<String>> cameraAction) {
    this.cameras = List.copyOf(cameras);
    this.sightRange = sightRange;
    this.cameraAction = cameraAction;
  }

  /** A camera's full view's shape. */
  static String view(CameraConfig camera) {
    return WorldView.cameraFrame(camera.id()) + ".view";
  }

  @Override
  public String id() {
    return "camera-views";
  }

  @Override
  public String title() {
    return "Camera views";
  }

  @Override
  public void describe(SceneBuilder scene) {
    for (CameraConfig camera : cameras) {
      double[] fov = RobotLayer.fieldOfView(camera.sensor());
      scene
          .frustum(view(camera), WorldView.cameraFrame(camera.id()))
          .fov(fov[0], fov[1])
          .range(0, sightRange)
          .inspect(() -> WorldView.camera(camera, sightRange, cameraAction));
    }
  }
}
