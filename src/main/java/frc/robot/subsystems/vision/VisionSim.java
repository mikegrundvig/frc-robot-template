package frc.robot.subsystems.vision;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Milliseconds;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
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
 *
 * <p>A camera can be knocked on its mount ({@link #mount}) or made later than its configuration
 * says ({@link #addLatency}) while the simulation runs; the robot code still believes its
 * configuration.
 */
public final class VisionSim implements AutoCloseable {
  /** Tags farther than this are not detected: small in the image, and noisy. */
  public static final double MAX_SIGHT_METERS = 7.0;

  /**
   * How far each tag corner lands from where a perfect camera would put it, in pixels, on average
   * and its spread: a little lens-calibration error. PhotonVision's own example values.
   */
  public static final double CORNER_NOISE_MEAN_PIXELS = 0.25;

  public static final double CORNER_NOISE_STD_DEV_PIXELS = 0.08;

  private final VisionSystemSim world = new VisionSystemSim("main");
  private final Field layout;
  private final List<PhotonCameraSim> cameras = new ArrayList<>();

  /** A camera as added: PhotonVision's simulation of it, its configured mount and latency. */
  private record Added(PhotonCameraSim simulated, Transform3d configured, double latencyMillis) {}

  private final Map<String, Added> byName = new LinkedHashMap<>();

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
    // Seeded, one seed per camera, so a simulated run repeats exactly.
    PhotonCameraSim simulated =
        new PhotonCameraSim(io.camera(), properties(config.sensor(), cameras.size() + 1), layout);
    simulated.setMaxSightRange(MAX_SIGHT_METERS);
    Transform3d configured = config.robotToCamera();
    world.addCamera(simulated, mounted(configured, mountError));
    cameras.add(simulated);
    byName.put(
        config.name(),
        new Added(simulated, configured, config.sensor().latency().in(Milliseconds)));
    return io;
  }

  /**
   * Mounts a camera off its configuration from now on, as {@link #camera(CameraConfig,
   * Transform3d)} does when it's added: images taken from now on see from there.
   *
   * @param mountError in the robot's frame, from the configured mount; zero puts it back
   * @throws IllegalArgumentException if no camera has this name
   */
  public void mount(String camera, Transform3d mountError) {
    Added added = added(camera);
    world.adjustCamera(added.simulated(), mounted(added.configured(), mountError));
  }

  /**
   * Makes a camera this much later than its configuration says, from now on: its images arrive
   * later, still carrying when they were taken. PhotonVision's simulation then runs it at no more
   * frames a second than its latency allows.
   *
   * @param extraMillis zero for its configured latency
   * @throws IllegalArgumentException if no camera has this name
   */
  public void addLatency(String camera, double extraMillis) {
    Added added = added(camera);
    added.simulated().prop.setAvgLatencyMs(added.latencyMillis() + extraMillis);
  }

  private Added added(String camera) {
    Added added = byName.get(camera);
    if (added == null) {
      throw new IllegalArgumentException("no simulated camera is named " + camera);
    }
    return added;
  }

  /**
   * Where a camera really is, mounted off its configuration: the error added to the configured
   * position, and turning the configured orientation. The camera placement study mounts its cameras
   * off the same way.
   */
  public static Transform3d mounted(Transform3d configured, Transform3d mountError) {
    return new Transform3d(
        configured.getTranslation().plus(mountError.getTranslation()),
        configured.getRotation().rotateBy(mountError.getRotation()));
  }

  /**
   * A camera as PhotonVision's simulation models it: the sensor's resolution, field of view, frame
   * rate, and latency, and a little lens-calibration error, which moves each tag corner about a
   * quarter pixel. The camera placement study takes its images with the same model, and starts from
   * the same errors.
   *
   * @param seed where the noise starts, so a run repeats exactly
   */
  public static SimCameraProperties properties(CameraConfig.Sensor sensor, long seed) {
    return new SimCameraProperties()
        .setCalibration(
            sensor.widthPixels(),
            sensor.heightPixels(),
            Rotation2d.fromDegrees(sensor.diagonalFieldOfView().in(Degrees)))
        .setCalibError(CORNER_NOISE_MEAN_PIXELS, CORNER_NOISE_STD_DEV_PIXELS)
        .setFPS(sensor.frameRate().in(Hertz))
        .setAvgLatencyMs(sensor.latency().in(Milliseconds))
        .setLatencyStdDevMs(5)
        .setRandomSeed(seed);
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
