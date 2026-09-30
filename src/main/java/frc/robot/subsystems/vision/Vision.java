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
import frc.robot.lib.localization.VisionConfigBuilder;
import frc.robot.lib.localization.VisionDecision;
import frc.robot.lib.localization.VisionMeasurement;
import frc.robot.lib.localization.VisionPipeline;
import frc.robot.lib.localization.VisionResult;
import frc.robot.lib.timing.RobotLoop;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.util.Alert;

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
 * <p>Tunable while tuning is on: the main thresholds and uncertainties, a switch per camera to
 * leave it out (a camera fogged over, knocked loose, or blinded by stage lights), and how long to
 * hold the cameras back when autonomous starts.
 */
public final class Vision implements AutoCloseable {
  /** How far back "lately" reaches, for how fast the robot has turned: 254 used 0.3 s. */
  private static final int YAW_RATE_LOOPS = RobotLoop.loops(0.3);

  private final List<CameraConfig> cameras;
  private final CameraIO[] ios;
  private final CameraIOInputsAutoLogged[] inputs;
  private final CameraLocalizer localizer;
  private final FieldModel field;
  private final VisionPipeline pipeline;
  private final Alert[] disconnected;
  private final Alert[] unreadable;
  private final Alert badSettings =
      new Alert(
          "Vision",
          "badSettings",
          "Vision settings out of range: keeping the last good ones",
          Alert.Level.MEDIUM);
  private final long[] lastFrame;
  private final int[] accepted;
  private final int[] rejected;
  private final double[] residualSums;
  private final int[] residualCounts;
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

  private long movingUntil = Long.MIN_VALUE;

  // Calibration: headings the cameras measure, and one camera's sightings for its mount.
  private @Nullable HeadingAverage headings;
  private @Nullable CameraMountCalibration mountCalibration;
  private int calibrating = -1;

  private final TunableBoolean[] enabled;
  private final TunableDouble maxAmbiguity =
      TunableDouble.create(VisionConfig.DEFAULTS.maxAmbiguity());
  private final TunableDouble maxHeightError =
      TunableDouble.create(VisionConfig.DEFAULTS.maxHeightErrorMeters());
  private final TunableDouble linearStdDevBaseline =
      TunableDouble.create(VisionConfig.DEFAULTS.linearStdDevBaseline());
  private final TunableDouble angularStdDevBaseline =
      TunableDouble.create(VisionConfig.DEFAULTS.angularStdDevBaseline());
  private final TunableDouble distanceExponent =
      TunableDouble.create(VisionConfig.DEFAULTS.distanceExponent());

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
  private VisionConfig settings = VisionConfig.DEFAULTS;

  /**
   * @param cameras each camera's configuration, in the same order as its IO
   * @param ios each camera's door to its vision software
   * @param localizer where the robot was when each image was taken, and where measurements go
   * @param field the field, whose tags the cameras see
   */
  public Vision(
      List<CameraConfig> cameras, CameraIO[] ios, CameraLocalizer localizer, FieldModel field) {
    if (cameras.size() != ios.length) {
      throw new IllegalArgumentException("one IO for each camera");
    }
    this.cameras = List.copyOf(cameras);
    this.ios = ios.clone();
    this.localizer = localizer;
    this.field = field;
    pipeline = new VisionPipeline(field);
    inputs = new CameraIOInputsAutoLogged[ios.length];
    disconnected = new Alert[ios.length];
    unreadable = new Alert[ios.length];
    enabled = new TunableBoolean[ios.length];
    lastFrame = new long[ios.length];
    accepted = new int[ios.length];
    rejected = new int[ios.length];
    residualSums = new double[ios.length];
    residualCounts = new int[ios.length];
    for (int i = 0; i < ios.length; i++) {
      String name = cameras.get(i).name();
      inputs[i] = new CameraIOInputsAutoLogged();
      disconnected[i] =
          new Alert(
              "Vision",
              name + "Disconnected",
              "Camera " + name + " disconnected",
              Alert.Level.HIGH);
      unreadable[i] =
          new Alert(
              "Vision", name + "Unreadable", "Camera " + name + " can't be read", Alert.Level.HIGH);
      enabled[i] = TunableBoolean.create(true);
      publish("Vision/" + name + "/enabled", enabled[i]);
      lastFrame[i] = Long.MIN_VALUE;
    }
    publish("Vision/maxAmbiguity", maxAmbiguity);
    publish("Vision/maxHeightErrorMeters", maxHeightError);
    publish("Vision/linearStdDevBaseline", linearStdDevBaseline);
    publish("Vision/angularStdDevBaseline", angularStdDevBaseline);
    publish("Vision/distanceExponent", distanceExponent);
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
    for (int i = 0; i < cameras.size(); i++) {
      if (cameras.get(i).name().equals(camera)) {
        calibrating = i;
        mountCalibration = into;
        return;
      }
    }
    throw new IllegalArgumentException("no camera named " + camera);
  }

  public void stopCalibratingMount() {
    calibrating = -1;
    mountCalibration = null;
  }

  /** Whether a calibration is collecting from the cameras: headings, or one camera's mount. */
  public boolean isCalibrating() {
    return calibrating >= 0 || headings != null;
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
    if (estimate.fieldVelocity().getNorm() > 0.02
        || Math.abs(estimate.yawRateRadiansPerSecond()) > 0.02) {
      movingUntil = estimate.timestampNanos();
    }
    recentYawRates[yawRateLoop] = Math.abs(estimate.yawRateRadiansPerSecond());
    yawRateLoop = (yawRateLoop + 1) % YAW_RATE_LOOPS;
    double recentYawRate = 0;
    for (double rate : recentYawRates) {
      recentYawRate = Math.max(recentYawRate, rate);
    }
    settings = currentSettings();
    double speed = estimate.fieldVelocity().getNorm();
    double headingStdDev = estimate.covariance().headingStdDev();

    for (int i = 0; i < ios.length; i++) {
      CameraConfig camera = cameras.get(i);
      String key = "Vision/" + camera.name();
      read(ios[i], inputs[i]);
      Logger.processInputs(key, inputs[i]);
      boolean readable = inputs[i].error.isEmpty();
      disconnected[i].set(!inputs[i].connected && readable);
      if (!readable) {
        unreadable[i].setText("Camera " + camera.name() + " can't be read: " + inputs[i].error);
      }
      unreadable[i].set(!readable);

      List<String> outcomes = new ArrayList<>();
      if (inputs[i].skippedFrames > 0) {
        outcomes.add(inputs[i].skippedFrames + " image(s) without a capture time, left out");
      }
      List<Pose3d> seenTags = new ArrayList<>();
      double residualSum = 0;
      int residualCount = 0;
      int next = 0;
      for (CameraIO.Frame frame : inputs[i].frames) {
        int first = next;
        next += frame.tagCount();
        // One image's bad data costs that image, never the robot program: whatever a camera sends
        // that the checks below miss ends here, logged as the image's outcome.
        try {
          TagDetections detections = detections(frame, inputs[i].tags, first, camera.name());
          if (!enabled[i].get()) {
            outcomes.add("camera turned off");
            continue;
          }
          if (frame.timestampNanos() <= lastFrame[i]) {
            outcomes.add("the same image again");
            continue;
          }
          lastFrame[i] = frame.timestampNanos();
          Pose2d then = localizer.poseAt(frame.timestampNanos()).orElse(null);
          Rotation2d heading = then != null ? then.getRotation() : null;
          AttitudeHistory.Attitude tilted = attitudes.at(frame.timestampNanos());
          VisionResult result =
              pipeline.process(
                  detections,
                  camera,
                  new CaptureContext(
                      heading,
                      headingStdDev,
                      tilted.pitchRadians(),
                      tilted.rollRadians(),
                      speed,
                      recentYawRate),
                  settings);
          VisionMeasurement measurement = result.measurement();
          if (measurement == null) {
            outcomes.add(result.outcome());
          } else if (i == calibrating) {
            outcomes.add("withheld while calibrating this camera: " + result.outcome());
          } else if (frame.timestampNanos() < holdUntilNanos) {
            outcomes.add("held, autonomous just started: " + result.outcome());
          } else {
            localizer.addVision(measurement);
            outcomes.add(result.outcome());
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
                        tilted.rollRadians(),
                        tilted.pitchRadians(),
                        then.getRotation().getRadians()));
            CameraMountCalibration sightings =
                i == calibrating
                        && poseKnown(estimate)
                        && frame.timestampNanos() - movingUntil >= STILL_NANOS
                    ? mountCalibration
                    : null;
            for (TagDetections.Tag tag : detections.tags()) {
              Pose3d seen =
                  robotThen.transformBy(camera.robotToCamera()).transformBy(tag.bestCameraToTag());
              seenTags.add(seen);
              Pose3d real = field.tag(tag.id()).orElse(null);
              if (real != null) {
                residualSum += seen.getTranslation().getDistance(real.getTranslation());
                residualCount++;
                if (sightings != null) {
                  sightings.add(robotThen, tag.bestCameraToTag(), tag.altCameraToTag(), real);
                }
              }
            }
          }
        } catch (RuntimeException e) {
          outcomes.add("unusable data from the camera: " + e.getMessage());
        }
      }
      Logger.recordOutput(key + "/Outcomes", outcomes.toArray(String[]::new));
      Logger.recordOutput(key + "/SeenTags", seenTags.toArray(Pose3d[]::new));
      if (residualCount > 0) {
        Logger.recordOutput(key + "/TagResidualMeters", residualSum / residualCount);
        residualSums[i] += residualSum;
        residualCounts[i] += residualCount;
        Logger.recordOutput(key + "/MeanTagResidualMeters", meanTagResidualMeters(camera.name()));
      }
    }
  }

  /**
   * Reads a camera. Nothing its IO throws gets past here: PhotonLib throws on purpose when the
   * coprocessor runs a different PhotonVision version than the robot's library, and again every few
   * seconds, which would end the robot program each time. Instead the camera reads as unreadable,
   * with the reason, and the robot drives on its other sensors.
   */
  private static void read(CameraIO io, CameraIOInputsAutoLogged into) {
    try {
      io.updateInputs(into);
      into.error = "";
    } catch (RuntimeException e) {
      into.connected = false;
      into.frames = new CameraIO.Frame[] {};
      into.tags = new CameraIO.SeenTag[] {};
      into.skippedFrames = 0;
      String why = e.getMessage();
      into.error = why != null ? why.strip() : e.toString();
    }
  }

  /** Whether the other cameras hold the robot's pose well enough to calibrate one against. */
  private static boolean poseKnown(LocalizationEstimate estimate) {
    return estimate.mode() == LocalizationEstimate.Mode.TRACKING
        && estimate.covariance().largestPositionStdDev() <= CALIBRATION_POSE_STD_DEV_METERS;
  }

  /** Counts and logs what the estimator decided about each camera's measurements. */
  public void record(List<VisionDecision> decisions) {
    for (int i = 0; i < ios.length; i++) {
      String name = cameras.get(i).name();
      List<String> reasons = new ArrayList<>();
      for (VisionDecision decision : decisions) {
        if (decision.measurement().camera().equals(name)) {
          reasons.add(decision.reason());
          if (decision.accepted()) {
            accepted[i]++;
          } else {
            rejected[i]++;
          }
        }
      }
      Logger.recordOutput("Vision/" + name + "/Decisions", reasons.toArray(String[]::new));
      Logger.recordOutput("Vision/" + name + "/Accepted", accepted[i]);
      Logger.recordOutput("Vision/" + name + "/Rejected", rejected[i]);
    }
  }

  /** One line for the pit: which cameras are connected, and how their measurements fare. */
  public String summary() {
    int connected = 0;
    int accepts = 0;
    int rejects = 0;
    for (int i = 0; i < ios.length; i++) {
      connected += inputs[i].connected ? 1 : 0;
      accepts += accepted[i];
      rejects += rejected[i];
    }
    String fate =
        accepts + rejects == 0
            ? "no measurements yet"
            : String.format(
                Locale.ROOT,
                "%.0f%% of %d measurements accepted",
                100.0 * accepts / (accepts + rejects),
                accepts + rejects);
    return connected + " of " + ios.length + " cameras connected, " + fate;
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
    for (int i = 0; i < cameras.size(); i++) {
      if (cameras.get(i).name().equals(camera)) {
        return residualCounts[i] == 0 ? Double.NaN : residualSums[i] / residualCounts[i];
      }
    }
    throw new IllegalArgumentException("no camera named " + camera);
  }

  /** How long autonomous's start holds the cameras back, in seconds: zero is off. */
  TunableDouble autoStartHoldSeconds() {
    return autoStartHoldSeconds;
  }

  /** A camera's switch, by the camera's order: off leaves it out. */
  TunableBoolean enabled(int camera) {
    return enabled[camera];
  }

  boolean isDisconnectedAlertActive(int camera) {
    return disconnected[camera].get();
  }

  boolean isUnreadableAlertActive(int camera) {
    return unreadable[camera].get();
  }

  /** How many of a camera's measurements the estimator accepted, by the camera's order. */
  int accepted(int camera) {
    return accepted[camera];
  }

  int rejected(int camera) {
    return rejected[camera];
  }

  /** The settings as tuned, or the last good ones if a tuned value is out of range. */
  private VisionConfig currentSettings() {
    try {
      VisionConfig tuned =
          VisionConfigBuilder.builder(VisionConfig.DEFAULTS)
              .maxAmbiguity(maxAmbiguity.get())
              .maxHeightErrorMeters(maxHeightError.get())
              .linearStdDevBaseline(linearStdDevBaseline.get())
              .angularStdDevBaseline(angularStdDevBaseline.get())
              .distanceExponent(distanceExponent.get())
              .build();
      badSettings.set(false);
      return tuned;
    } catch (IllegalArgumentException e) {
      badSettings.set(true);
      return settings;
    }
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
    for (int i = 0; i < ios.length; i++) {
      ios[i].close();
      disconnected[i].close();
      unreadable[i].close();
    }
    badSettings.close();
    for (String path : tunablePaths) {
      TunableRegistry.remove(path);
    }
  }
}
