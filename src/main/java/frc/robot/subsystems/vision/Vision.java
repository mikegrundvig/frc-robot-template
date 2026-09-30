package frc.robot.subsystems.vision;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.localization.CaptureContext;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.localization.PoseEstimator;
import frc.robot.lib.localization.TagDetections;
import frc.robot.lib.localization.VisionConfig;
import frc.robot.lib.localization.VisionDecision;
import frc.robot.lib.localization.VisionPipeline;
import frc.robot.lib.localization.VisionResult;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
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
 * <p>Tunable while tuning is on: the main thresholds and uncertainties, and a switch per camera to
 * leave it out (a camera fogged over, knocked loose, or blinded by stage lights).
 */
public final class Vision implements AutoCloseable {
  /** How far back "lately" reaches, for how fast the robot has turned: 254 used 0.3 s. */
  private static final int YAW_RATE_LOOPS = 15;

  private final List<CameraConfig> cameras;
  private final CameraIO[] ios;
  private final CameraIOInputsAutoLogged[] inputs;
  private final PoseEstimator estimator;
  private final FieldModel field;
  private final VisionPipeline pipeline;
  private final Alert[] disconnected;
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
  private final List<String> tunablePaths = new ArrayList<>();
  private VisionConfig settings = VisionConfig.DEFAULTS;

  /**
   * @param cameras each camera's configuration, in the same order as its IO
   * @param ios each camera's door to its vision software
   * @param estimator where measurements go
   * @param field the field, whose tags the cameras see
   */
  public Vision(
      List<CameraConfig> cameras, CameraIO[] ios, PoseEstimator estimator, FieldModel field) {
    if (cameras.size() != ios.length) {
      throw new IllegalArgumentException("one IO for each camera");
    }
    this.cameras = List.copyOf(cameras);
    this.ios = ios.clone();
    this.estimator = estimator;
    this.field = field;
    pipeline = new VisionPipeline(field);
    inputs = new CameraIOInputsAutoLogged[ios.length];
    disconnected = new Alert[ios.length];
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
      enabled[i] = TunableBoolean.create(true);
      publish("Vision/" + name + "/enabled", enabled[i]);
      lastFrame[i] = Long.MIN_VALUE;
    }
    publish("Vision/maxAmbiguity", maxAmbiguity);
    publish("Vision/maxHeightErrorMeters", maxHeightError);
    publish("Vision/linearStdDevBaseline", linearStdDevBaseline);
    publish("Vision/angularStdDevBaseline", angularStdDevBaseline);
    publish("Vision/distanceExponent", distanceExponent);
  }

  private void publish(String path, TunableBase tunable) {
    TunableRegistry.publish(path, tunable);
    tunablePaths.add(path);
  }

  /**
   * Reads every camera and hands the estimator a measurement for each usable image.
   *
   * @param estimate the estimate as of this loop's odometry: the robot's motion and how sure its
   *     heading is
   */
  public void periodic(LocalizationEstimate estimate) {
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
      ios[i].updateInputs(inputs[i]);
      Logger.processInputs(key, inputs[i]);
      disconnected[i].set(!inputs[i].connected);

      List<String> outcomes = new ArrayList<>();
      List<Pose3d> seenTags = new ArrayList<>();
      double residualSum = 0;
      int residualCount = 0;
      int next = 0;
      for (CameraIO.Frame frame : inputs[i].frames) {
        TagDetections detections = detections(frame, inputs[i].tags, next, camera.name());
        next += frame.tagCount();
        if (!enabled[i].get()) {
          outcomes.add("camera turned off");
          continue;
        }
        if (frame.timestampNanos() <= lastFrame[i]) {
          outcomes.add("the same image again");
          continue;
        }
        lastFrame[i] = frame.timestampNanos();
        Pose2d then = estimator.poseAt(frame.timestampNanos()).orElse(null);
        Rotation2d heading = then != null ? then.getRotation() : null;
        VisionResult result =
            pipeline.process(
                detections,
                camera,
                new CaptureContext(heading, headingStdDev, 0, 0, speed, recentYawRate),
                settings);
        if (result.measurement() != null) {
          estimator.addVision(result.measurement());
        }
        outcomes.add(result.outcome());
        if (then != null) {
          for (TagDetections.Tag tag : detections.tags()) {
            Pose3d seen =
                new Pose3d(then)
                    .transformBy(camera.robotToCamera())
                    .transformBy(tag.bestCameraToTag());
            seenTags.add(seen);
            Pose3d real = field.tag(tag.id()).orElse(null);
            if (real != null) {
              residualSum += seen.getTranslation().getDistance(real.getTranslation());
              residualCount++;
            }
          }
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

  /** A camera's switch, by the camera's order: off leaves it out. */
  TunableBoolean enabled(int camera) {
    return enabled[camera];
  }

  boolean isDisconnectedAlertActive(int camera) {
    return disconnected[camera].get();
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
    VisionConfig d = VisionConfig.DEFAULTS;
    try {
      VisionConfig tuned =
          new VisionConfig(
              maxAmbiguity.get(),
              maxHeightError.get(),
              d.maxDistanceMeters(),
              d.maxYawRateRadiansPerSecond(),
              linearStdDevBaseline.get(),
              angularStdDevBaseline.get(),
              distanceExponent.get(),
              d.headingConstrainedFactor(),
              d.speedStdDevFactor(),
              d.yawRateStdDevFactor(),
              d.maxHeadingStdDevToConstrain(),
              d.minTagsForHeading(),
              d.maxYawRateForHeading(),
              d.maxReprojectionErrorForHeadingPixels());
      badSettings.set(false);
      return tuned;
    } catch (IllegalArgumentException e) {
      badSettings.set(true);
      return settings;
    }
  }

  /** One frame's tags, from the flat list the IO logs, as the pipeline takes them. */
  private static TagDetections detections(
      CameraIO.Frame frame, CameraIO.SeenTag[] tags, int first, String camera) {
    List<TagDetections.Tag> seen = new ArrayList<>();
    List<Integer> usedByMultiTag = new ArrayList<>();
    for (int k = first; k < Math.min(first + frame.tagCount(), tags.length); k++) {
      CameraIO.SeenTag tag = tags[k];
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
              tag.ambiguity() < 0 ? 1 : Math.min(tag.ambiguity(), 1)));
      if (tag.inMultiTag()) {
        usedByMultiTag.add(tag.id());
      }
    }
    TagDetections.MultiTag multiTag = null;
    if (frame.hasMultiTag() && !usedByMultiTag.isEmpty()) {
      multiTag =
          new TagDetections.MultiTag(
              frame.multiTagFieldToCamera(),
              Math.max(0, frame.multiTagReprojectionErrorPixels()),
              usedByMultiTag);
    }
    return new TagDetections(frame.timestampNanos(), camera, seen, multiTag);
  }

  @Override
  public void close() {
    for (int i = 0; i < ios.length; i++) {
      ios[i].close();
      disconnected[i].close();
    }
    badSettings.close();
    for (String path : tunablePaths) {
      TunableRegistry.remove(path);
    }
  }
}
