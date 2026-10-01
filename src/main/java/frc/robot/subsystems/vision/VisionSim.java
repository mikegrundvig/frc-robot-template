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
import java.util.Optional;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.system.Timer;

/**
 * The simulator's cameras: PhotonVision's own simulation, rendering each camera's view of the tags
 * from where the robot really is (never from the estimate), with the camera's resolution, field of
 * view, frame rate, latency, and a little lens-calibration error. Its results reach the robot code
 * through the same {@link CameraIOPhotonVision} a real camera uses, each image stamped with when it
 * was taken ({@link #update}).
 *
 * <p>A camera can be knocked on its mount ({@link #mount}) or made later than its configuration
 * says ({@link #addLatency}), and a tag moved off where the field was built ({@link #moveTag}),
 * while the simulation runs; the robot code still believes its configuration and the field's
 * layout.
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

  /** Each seed's cameras draw their noise from seeds this far apart. */
  private static final long SEEDS_PER_WORLD = 1000;

  /** How far back the robot's true poses are kept, for the images taken that long ago. */
  private static final double HISTORY_SECONDS = 1.5;

  private final VisionSystemSim world = new VisionSystemSim("main");
  private final Field layout;
  private final long seed;
  private final List<PhotonCameraSim> cameras = new ArrayList<>();

  /** Where the robot really was, loop by loop: what each image is taken from. */
  private final TimeInterpolatableBuffer<Pose3d> truths =
      TimeInterpolatableBuffer.createBuffer(HISTORY_SECONDS);

  /** A camera as added: PhotonVision's simulation of it, its configured mount and latency. */
  private record Added(PhotonCameraSim simulated, Transform3d configured, double latencyMillis) {}

  private final Map<String, Added> byName = new LinkedHashMap<>();

  /** The field's tags as the cameras see them, by ID, and where each was as the world was built. */
  private final Map<Integer, VisionTargetSim> tags = new LinkedHashMap<>();

  private final Map<Integer, Pose3d> builtTags = new LinkedHashMap<>();

  /** The tag moved from where it was built, if one is: its ID. */
  private int movedTag = NONE;

  private static final int NONE = Integer.MIN_VALUE;

  /**
   * @param tagErrors by tag ID, how far each tag really sits from where the field's layout says, in
   *     the field's frame, as the world is built: the cameras see it there, while their multi-tag
   *     solves, as the coprocessor's do, believe the layout ({@link #moveTag} moves one further)
   * @param seed where the cameras' noise starts: each camera draws its own from it, so a run
   *     repeats exactly, and a run with another seed sees other noise. Seed 1's cameras draw from
   *     1, 2, 3, and on; seed 2's from 1001, 1002, and on.
   */
  public VisionSim(FieldModel field, Map<Integer, Translation3d> tagErrors, long seed) {
    layout = field.toField();
    List<FieldTag> real = new ArrayList<>();
    for (FieldTag tag : layout.getTags()) {
      Translation3d off = tagErrors.getOrDefault(tag.getID(), Translation3d.ZERO);
      Pose3d pose = tag.getPose();
      Pose3d built = new Pose3d(pose.getTranslation().plus(off), pose.getRotation());
      real.add(new FieldTag(tag.getID(), built));
      builtTags.put(tag.getID(), built);
    }
    world.addAprilTags(
        new Field(
            layout.name,
            layout.season,
            layout.game,
            null,
            layout.length,
            layout.width,
            layout.program,
            real));
    for (VisionTargetSim target : world.getVisionTargets("apriltag")) {
      tags.put(target.fiducialID, target);
    }
    this.seed = seed;
  }

  /** The field's tags' IDs, as its layout lists them. */
  public List<Integer> tagIds() {
    return List.copyOf(builtTags.keySet());
  }

  /**
   * Moves one tag this far from where it was as the world was built, in the field's frame, from now
   * on, putting any other moved tag back: a tag knocked at an event. The robot code, and the
   * cameras' multi-tag solves, still believe the layout.
   *
   * @param by zero puts it back
   * @throws IllegalArgumentException if the field has no tag with this ID
   */
  public void moveTag(int id, Translation3d by) {
    VisionTargetSim target = tags.get(id);
    Pose3d built = builtTags.get(id);
    if (target == null || built == null) {
      throw new IllegalArgumentException("the field has no tag " + id);
    }
    putTagsBack();
    target.setPose(new Pose3d(built.getTranslation().plus(by), built.getRotation()));
    movedTag = by.equals(Translation3d.ZERO) ? NONE : id;
  }

  /** Puts every tag back where it was as the world was built. */
  public void putTagsBack() {
    for (Map.Entry<Integer, Pose3d> built : builtTags.entrySet()) {
      VisionTargetSim target = tags.get(built.getKey());
      if (target != null) {
        target.setPose(built.getValue());
      }
    }
    movedTag = NONE;
  }

  /** Whether a tag is moved from where it was as the world was built. */
  public boolean isTagMoved() {
    return movedTag != NONE;
  }

  /** Where a tag really is now, as the cameras see it; empty if the field has no such tag. */
  public Optional<Pose3d> tagPose(int id) {
    VisionTargetSim target = tags.get(id);
    return target == null ? Optional.empty() : Optional.of(target.getPose());
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
    long cameraSeed = (seed - 1) * SEEDS_PER_WORLD + cameras.size() + 1;
    PhotonCameraSim simulated =
        new PhotonCameraSim(io.camera(), properties(config.sensor(), cameraSeed), layout);
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
   *
   * <p>An image falls due on the camera's frame rate, and is taken its latency before then, from
   * where the robot was at that moment; it's stamped with that moment, as a real camera's time sync
   * stamps it. PhotonVision's {@code VisionSystemSim.update} takes it the same way but stamps it as
   * of the loop that renders it, up to a loop (20 ms) later. While the robot turns, every image
   * then shows it where it was a little before its stamp, turned a little less, the same way image
   * after image: just what a heading error does to a position solved with the gyro's heading. An
   * estimator that lets every fix correct the heading took it for one, and grew sure of a heading
   * 0.7° off after the gyro-scale procedure (docs/testing.md).
   */
  public void update(Pose3d truth) {
    truths.addSample(Timer.getMonotonicTimestamp(), truth);
    List<VisionTargetSim> targets = List.copyOf(world.getVisionTargets());
    for (PhotonCameraSim camera : cameras) {
      Optional<Long> due = camera.consumeNextEntryTime();
      if (due.isEmpty()) {
        continue;
      }
      long arrivedNanos = due.get();
      double latencyMillis = camera.prop.estLatencyMs();
      long takenNanos = arrivedNanos - Math.round(latencyMillis * 1e6);
      double taken = takenNanos / 1e9;
      Pose3d robot = truths.getSample(taken).orElse(truth);
      Pose3d from = robot.plus(world.getRobotToCamera(camera, taken).orElseThrow());
      PhotonPipelineResult image = camera.process(latencyMillis, from, targets);
      image.metadata.captureTimestampNanos = takenNanos;
      image.metadata.publishTimestampNanos = arrivedNanos;
      camera.submitProcessedFrame(image, arrivedNanos);
    }
    // Every image due is taken, so PhotonVision's own update only keeps its record of the robot.
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
