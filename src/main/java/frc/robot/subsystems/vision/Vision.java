package frc.robot.subsystems.vision;

import frc.robot.lib.calibration.CameraMountCalibration;
import frc.robot.lib.calibration.HeadingAverage;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.localization.AttitudeHistory;
import frc.robot.lib.localization.CameraLocalizer;
import frc.robot.lib.localization.CaptureContext;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.localization.TagDetections;
import frc.robot.lib.localization.Timestamps;
import frc.robot.lib.localization.VisionConfig;
import frc.robot.lib.localization.VisionDecision;
import frc.robot.lib.localization.VisionMeasurement;
import frc.robot.lib.localization.VisionPipeline;
import frc.robot.lib.localization.VisionResult;
import frc.robot.lib.timing.RobotLoop;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;

/**
 * The robot's cameras: reads each one's results, turns them into measurements through the {@link
 * VisionPipeline}, and hands those to the pose estimator. Call {@link #periodic} after the
 * drivetrain's odometry and before the estimator's update, then {@link #record} with the update's
 * decisions.
 *
 * <p>For the pit and the log, per camera: whether it's connected, what became of every image and
 * why, and each tag as the camera sees it (placed through the estimated pose and the camera's
 * configured mount) with how far that lands from the tag's real place. A camera whose tags land
 * consistently off is mounted differently than its configuration says: recalibrate it.
 *
 * <p>How far images are trusted is the robot's configuration ({@code RobotConfig.vision}), whose
 * main thresholds and uncertainties Pitwall tunes. Two switches are WPILib tunables, for the drive
 * team as much as the pit: one per camera to leave it out (a camera fogged over, knocked loose, or
 * blinded by stage lights), and how long to hold the cameras back when autonomous starts.
 */
public final class Vision implements AutoCloseable {
  /** How far back "lately" reaches, for how fast the robot has turned: 254 used 0.3 s. */
  private static final int YAW_RATE_LOOPS = RobotLoop.loops(0.3);

  private final List<Camera> cameras;
  private final CameraLocalizer localizer;
  private final FieldModel field;
  private final VisionPipeline pipeline;
  private final Supplier<VisionConfig> settings;
  private final double[] recentYawRates = new double[YAW_RATE_LOOPS];
  private int yawRateLoop;

  /** Half a second of the robot's attitude, for each image's capture time. */
  private final AttitudeHistory attitudes = new AttitudeHistory(RobotLoop.loops(0.5));

  /** The robot's pose is good enough to calibrate a camera against: within 5 cm. */
  private static final double CALIBRATION_POSE_STD_DEV_METERS = 0.05;

  /**
   * A camera is calibrated only from images taken standing still, this long: no blur, and no
   * question of exactly when the image was taken.
   */
  private static final long STILL_NANOS = 500_000_000L;

  /** Slower than this, the robot counts as standing still: 2 cm and about a degree a second. */
  private static final double STILL_METERS_PER_SECOND = 0.02;

  private static final double STILL_RADIANS_PER_SECOND = 0.02;

  private long movingUntil = Long.MIN_VALUE;

  // Calibration: headings the cameras measure, and one camera's sightings for its mount.
  private @Nullable HeadingAverage headings;
  private @Nullable CameraMountCalibration mountCalibration;
  private @Nullable Camera calibrating;

  /**
   * How long to hold the cameras' measurements back when autonomous starts, trusting the placed
   * start while the robot launches: 6328 held theirs 2 s. Off by default: the estimator gates every
   * measurement against the placed start anyway.
   */
  private final TunableDouble autoStartHoldSeconds = TunableDouble.create(0.0);

  /** Longer than any autonomous period has been. */
  private static final double MAX_HOLD_SECONDS = 30;

  private long holdUntilNanos = Long.MIN_VALUE;
  private final List<String> tunablePaths = new ArrayList<>();
  private VisionConfig current;

  /**
   * @param cameras each camera's configuration, in the same order as its IO
   * @param ios each camera's door to its vision software
   * @param localizer where the robot was when each image was taken, and where measurements go
   * @param field the field, whose tags the cameras see
   * @param settings how far images are trusted, as tuned: read once a loop
   */
  public Vision(
      List<CameraConfig> cameras,
      CameraIO[] ios,
      CameraLocalizer localizer,
      FieldModel field,
      Supplier<VisionConfig> settings) {
    if (cameras.size() != ios.length) {
      throw new IllegalArgumentException("one IO for each camera");
    }
    List<Camera> each = new ArrayList<>();
    for (int i = 0; i < ios.length; i++) {
      each.add(new Camera(cameras.get(i), ios[i]));
    }
    this.cameras = List.copyOf(each);
    this.localizer = localizer;
    this.field = field;
    pipeline = new VisionPipeline(field);
    this.settings = settings;
    current = settings.get();
    publish("Vision/autoStartHoldSeconds", autoStartHoldSeconds);
  }

  private void publish(String path, TunableBase tunable) {
    TunableRegistry.publish(path, tunable);
    tunablePaths.add(path);
  }

  /**
   * From now on, every heading the cameras measure (multi-tag solutions, standing still) goes into
   * this, until stopped: a heading reference for calibrating the gyro.
   */
  public void collectHeadings(HeadingAverage into) {
    headings = into;
  }

  public void stopCollectingHeadings() {
    headings = null;
  }

  /**
   * From now on, a camera's tag sightings go into this calibration of its mount, taken while the
   * robot stands still and the other cameras hold its pose within 5 cm; its measurements stay out
   * of the estimator, so the pose comes from the others. Until stopped.
   */
  public void calibrateMount(String camera, CameraMountCalibration into) {
    calibrating = camera(camera);
    mountCalibration = into;
  }

  public void stopCalibratingMount() {
    calibrating = null;
    mountCalibration = null;
  }

  /** Whether a calibration is collecting from the cameras: headings, or one camera's mount. */
  public boolean isCalibrating() {
    return calibrating != null || headings != null;
  }

  /**
   * Autonomous just started: hold the cameras' measurements back for the tuned time, if any. They
   * are still read and logged, marked held.
   */
  public void autonomousStarted(long nowNanos) {
    // A tuned value that isn't a sensible time holds nothing, and never throws in the loop.
    double seconds = autoStartHoldSeconds.get();
    holdUntilNanos =
        seconds > 0 && Double.isFinite(seconds)
            ? nowNanos + Timestamps.fromSeconds(Math.min(seconds, MAX_HOLD_SECONDS))
            : Long.MIN_VALUE;
  }

  /**
   * Reads every camera and hands the estimator a measurement for each usable image.
   *
   * @param estimate the estimate as of this loop's odometry: the robot's motion and how sure its
   *     heading is
   * @param pitchRadians the robot's pitch now, from the IMU: each image is solved with the robot
   *     tilted as it was when the image was taken, since a tag seen from a tilted camera as if
   *     level lands off by the tilt times its distance
   * @param rollRadians the robot's roll now
   */
  public void periodic(LocalizationEstimate estimate, double pitchRadians, double rollRadians) {
    attitudes.add(estimate.timestampNanos(), pitchRadians, rollRadians);
    if (estimate.fieldVelocity().getNorm() > STILL_METERS_PER_SECOND
        || Math.abs(estimate.yawRateRadiansPerSecond()) > STILL_RADIANS_PER_SECOND) {
      movingUntil = estimate.timestampNanos();
    }
    recentYawRates[yawRateLoop] = Math.abs(estimate.yawRateRadiansPerSecond());
    yawRateLoop = (yawRateLoop + 1) % YAW_RATE_LOOPS;
    double recentYawRate = 0;
    for (double rate : recentYawRates) {
      recentYawRate = Math.max(recentYawRate, rate);
    }
    current = settings.get();
    Motion motion =
        new Motion(
            estimate.fieldVelocity().getNorm(),
            recentYawRate,
            estimate.covariance().headingStdDev(),
            poseKnown(estimate));

    for (Camera camera : cameras) {
      camera.read();
      int next = 0;
      for (CameraIO.Frame frame : camera.inputs.frames) {
        int first = next;
        next += frame.tagCount();
        // One image's bad data costs that image, never the robot program: whatever a camera sends
        // that the checks miss ends here, logged as the image's outcome.
        try {
          processFrame(camera, frame, first, motion);
        } catch (RuntimeException e) {
          camera.outcome("unusable data from the camera: " + e.getMessage());
        }
      }
      camera.logLoop();
    }
  }

  /**
   * How the robot moves this loop, and how well its pose is known: what every image is solved with.
   *
   * @param speed meters per second, over the field
   * @param recentYawRate the fastest the robot turned lately, radians per second
   * @param headingStdDev how sure the estimate's heading is, radians
   * @param poseKnown whether the pose is held well enough to calibrate a camera against
   */
  private record Motion(
      double speed, double recentYawRate, double headingStdDev, boolean poseKnown) {}

  /**
   * One image: solved into a measurement for the estimator (unless it's held back), and its tags
   * placed on the field, for the camera's residual and any calibration collecting them.
   */
  private void processFrame(Camera camera, CameraIO.Frame frame, int first, Motion motion) {
    TagDetections detections = detections(frame, camera.inputs.tags, first, camera.name());
    if (!camera.enabled.get()) {
      camera.outcome("camera turned off");
      return;
    }
    if (!camera.isNew(frame.timestampNanos())) {
      camera.outcome("the same image again");
      return;
    }
    Pose2d then = localizer.poseAt(frame.timestampNanos()).orElse(null);
    Rotation2d heading = then != null ? then.getRotation() : null;
    AttitudeHistory.Attitude tilted = attitudes.at(frame.timestampNanos());
    VisionResult result =
        pipeline.process(
            detections,
            camera.config,
            new CaptureContext(
                heading,
                motion.headingStdDev(),
                tilted.pitchRadians(),
                tilted.rollRadians(),
                motion.speed(),
                motion.recentYawRate()),
            current);
    VisionMeasurement measurement = result.measurement();
    if (measurement == null) {
      camera.outcome(result.outcome());
    } else if (camera == calibrating) {
      camera.outcome("withheld while calibrating this camera: " + result.outcome());
    } else if (frame.timestampNanos() < holdUntilNanos) {
      camera.outcome("held, autonomous just started: " + result.outcome());
    } else {
      localizer.addVision(measurement);
      camera.outcome(result.outcome());
    }
    HeadingAverage collecting = headings;
    if (collecting != null && measurement != null && measurement.covariance().hasHeading()) {
      collecting.add(measurement.pose().getRotation());
    }
    if (then != null) {
      Pose3d robotThen =
          new Pose3d(
              then.getX(),
              then.getY(),
              0,
              new Rotation3d(
                  tilted.rollRadians(), tilted.pitchRadians(), then.getRotation().getRadians()));
      CameraMountCalibration sightings =
          camera == calibrating
                  && motion.poseKnown()
                  && frame.timestampNanos() - movingUntil >= STILL_NANOS
              ? mountCalibration
              : null;
      recordSightings(camera, robotThen, detections, sightings);
    }
  }

  /**
   * Places each tag an image saw on the field, through where the robot was and the camera's
   * configured mount, for the camera's residual; and gives it to the mount calibration collecting
   * this camera's sightings, if one is.
   */
  private void recordSightings(
      Camera camera,
      Pose3d robotThen,
      TagDetections detections,
      @Nullable CameraMountCalibration sightings) {
    for (TagDetections.Tag tag : detections.tags()) {
      Pose3d seen =
          robotThen.transformBy(camera.config.robotToCamera()).transformBy(tag.bestCameraToTag());
      Pose3d real = field.tag(tag.id()).orElse(null);
      camera.saw(seen, real);
      if (real != null && sightings != null) {
        sightings.add(robotThen, tag.bestCameraToTag(), tag.altCameraToTag(), real);
      }
    }
  }

  /** Whether the other cameras hold the robot's pose well enough to calibrate one against. */
  private static boolean poseKnown(LocalizationEstimate estimate) {
    return estimate.mode() == LocalizationEstimate.Mode.TRACKING
        && estimate.covariance().largestPositionStdDev() <= CALIBRATION_POSE_STD_DEV_METERS;
  }

  /** Counts and logs what the estimator decided about each camera's measurements. */
  public void record(List<VisionDecision> decisions) {
    for (Camera camera : cameras) {
      camera.record(decisions);
    }
  }

  /** One line for the pit: which cameras are connected, and how their measurements fare. */
  public String summary() {
    int connected = 0;
    int accepts = 0;
    int rejects = 0;
    for (Camera camera : cameras) {
      connected += camera.isConnected() ? 1 : 0;
      accepts += camera.accepted();
      rejects += camera.rejected();
    }
    String fate =
        accepts + rejects == 0
            ? "no measurements yet"
            : String.format(
                Locale.ROOT,
                "%.0f%% of %d measurements accepted",
                100.0 * accepts / (accepts + rejects),
                accepts + rejects);
    return connected + " of " + cameras.size() + " cameras connected, " + fate;
  }

  /**
   * How far, on average since the robot started, a camera's tags land from where they really are,
   * placed through the estimated pose and the camera's configured mount: the pre-event check that
   * each camera is still mounted as configured. A few centimeters is normal; one camera well above
   * the others has moved.
   *
   * @return NaN until the camera has seen a tag
   */
  public double meanTagResidualMeters(String camera) {
    return camera(camera).meanTagResidualMeters();
  }

  private Camera camera(String name) {
    for (Camera camera : cameras) {
      if (camera.name().equals(name)) {
        return camera;
      }
    }
    throw new IllegalArgumentException("no camera named " + name);
  }

  /** How long autonomous's start holds the cameras back, in seconds: zero is off. */
  TunableDouble autoStartHoldSeconds() {
    return autoStartHoldSeconds;
  }

  /** A camera, in the order of the cameras' configuration: its switch, alerts, and tallies. */
  Camera camera(int index) {
    return cameras.get(index);
  }

  /**
   * One frame's tags, from the flat list the IO logs, as the pipeline takes them. A tag whose pose
   * isn't a number is left out, and so is a multi-tag solution that isn't; an ambiguity that isn't
   * one counts as a coin toss.
   */
  private static TagDetections detections(
      CameraIO.Frame frame, CameraIO.SeenTag[] tags, int first, String camera) {
    List<TagDetections.Tag> seen = new ArrayList<>();
    List<Integer> usedByMultiTag = new ArrayList<>();
    for (int k = first; k < Math.min(first + frame.tagCount(), tags.length); k++) {
      CameraIO.SeenTag tag = tags[k];
      if (!isFinite(tag.bestCameraToTag()) || !isFinite(tag.altCameraToTag())) {
        continue;
      }
      seen.add(
          new TagDetections.Tag(
              tag.id(),
              List.of(
                  new Translation2d(tag.corner0X(), tag.corner0Y()),
                  new Translation2d(tag.corner1X(), tag.corner1Y()),
                  new Translation2d(tag.corner2X(), tag.corner2Y()),
                  new Translation2d(tag.corner3X(), tag.corner3Y())),
              tag.bestCameraToTag(),
              tag.altCameraToTag(),
              // PhotonVision reports -1 when it has no ambiguity to give: treat that as unknown.
              !(tag.ambiguity() >= 0) ? 1 : Math.min(tag.ambiguity(), 1)));
      if (tag.inMultiTag()) {
        usedByMultiTag.add(tag.id());
      }
    }
    TagDetections.MultiTag multiTag = null;
    if (frame.hasMultiTag()
        && !usedByMultiTag.isEmpty()
        && isFinite(frame.multiTagFieldToCamera())
        && Double.isFinite(frame.multiTagReprojectionErrorPixels())) {
      multiTag =
          new TagDetections.MultiTag(
              frame.multiTagFieldToCamera(),
              Math.max(0, frame.multiTagReprojectionErrorPixels()),
              usedByMultiTag);
    }
    return new TagDetections(frame.timestampNanos(), camera, seen, multiTag);
  }

  private static boolean isFinite(Transform3d transform) {
    Quaternion rotation = transform.getRotation().getQuaternion();
    return Double.isFinite(transform.getX())
        && Double.isFinite(transform.getY())
        && Double.isFinite(transform.getZ())
        && Double.isFinite(rotation.getW())
        && Double.isFinite(rotation.getX())
        && Double.isFinite(rotation.getY())
        && Double.isFinite(rotation.getZ());
  }

  @Override
  public void close() {
    for (Camera camera : cameras) {
      camera.close();
    }
    for (String path : tunablePaths) {
      TunableRegistry.remove(path);
    }
  }
}
