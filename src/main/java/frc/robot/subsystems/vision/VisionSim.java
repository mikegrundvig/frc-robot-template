package frc.robot.subsystems.vision;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Milliseconds;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The simulator's cameras: PhotonVision's own simulation, rendering each camera's view of the tags
 * from where the robot really is (never from the estimate), with the camera's resolution, field of
 * view, frame rate, latency, and a little lens-calibration error. Its results reach the robot code
 * through the same {@link CameraIOPhotonVision} a real camera uses.
 */
public final class VisionSim implements AutoCloseable {
  /** Tags farther than this are not detected: small in the image, and noisy. */
  private static final double MAX_SIGHT_METERS = 7.0;

  private final VisionSystemSim world = new VisionSystemSim("main");
  private final Field layout;
  private final List<PhotonCameraSim> cameras = new ArrayList<>();

  public VisionSim(FieldModel field) {
    layout = field.toField();
    world.addAprilTags(layout);
  }

  /** Adds a camera, mounted exactly as configured, and returns the IO the robot code reads. */
  public CameraIO camera(CameraConfig config) {
    return camera(config, Transform3d.ZERO);
  }

  /**
   * Adds a camera mounted off its configuration (knocked 4 in low, turned 2 degrees), and returns
   * the IO the robot code reads it through, which still believes the configuration.
   *
   * @param mountError in the robot's frame: added to the configured position, and turning the
   *     configured orientation
   */
  public CameraIO camera(CameraConfig config, Transform3d mountError) {
    CameraIOPhotonVision io = new CameraIOPhotonVision(config.name());
    CameraConfig.Sensor sensor = config.sensor();
    SimCameraProperties properties =
        new SimCameraProperties()
            .setCalibration(
                sensor.widthPixels(),
                sensor.heightPixels(),
                Rotation2d.fromDegrees(sensor.diagonalFieldOfView().in(Degrees)))
            .setCalibError(0.25, 0.08)
            .setFPS(sensor.frameRate().in(Hertz))
            .setAvgLatencyMs(sensor.latency().in(Milliseconds))
            .setLatencyStdDevMs(5)
            // Seeded, one seed per camera, so a simulated run repeats exactly.
            .setRandomSeed(cameras.size() + 1);
    PhotonCameraSim simulated = new PhotonCameraSim(io.camera(), properties, layout);
    simulated.setMaxSightRange(MAX_SIGHT_METERS);
    Transform3d configured = config.robotToCamera();
    world.addCamera(
        simulated,
        new Transform3d(
            configured.getTranslation().plus(mountError.getTranslation()),
            configured.getRotation().rotateBy(mountError.getRotation())));
    cameras.add(simulated);
    return io;
  }

  /**
   * Renders every camera's view from where the robot is, tilted as it is. Call once per loop,
   * before reading.
   */
  public void update(Pose3d truth) {
    world.update(truth);
  }

  @Override
  public void close() {
    world.clearCameras();
    for (PhotonCameraSim camera : cameras) {
      camera.close();
    }
  }
}
