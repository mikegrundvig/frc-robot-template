package frc.robot.lib.localization;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * Turns one camera image's AprilTags into a measurement for the estimator, or says why not.
 *
 * <p>How it solves, in order of preference:
 *
 * <ol>
 *   <li><b>Heading-constrained</b>, whenever the estimate is sure of the heading: each tag fixes
 *       only the position, using the robot's heading from the gyro. Of the two poses a single tag
 *       allows, the one whose heading agrees with the gyro is used, which removes single-tag
 *       ambiguity, and each tag's known height checks the view. The idea behind Limelight's
 *       MegaTag2 and PhotonVision's constrained solve.
 *   <li><b>Multi-tag</b>, the vision software's solution from every tag at once, when the heading
 *       isn't known yet (a robot not yet placed).
 *   <li><b>Single tag</b>, the least ambiguous one, only if its ambiguity is low.
 * </ol>
 *
 * <p>Uncertainty grows with the tags' distance (squared, by default), shrinks with their count, and
 * grows with the robot's speed and turning (blur). A result may correct heading only from a
 * multi-tag solution of several tags, with the robot barely turning and the solution fitting well;
 * otherwise the gyro keeps heading to itself. Everything thrown out says why, for the log.
 *
 * <p>Pure logic: the field's layout, the camera's mount, and the robot at the capture time all come
 * in as arguments.
 */
public final class VisionPipeline {
  private final FieldModel field;

  public VisionPipeline(FieldModel field) {
    this.field = field;
  }

  /** What to make of one image. */
  public VisionResult process(
      TagDetections seen, CameraConfig camera, CaptureContext robot, VisionConfig config) {
    List<TagDetections.Tag> known = new ArrayList<>();
    double distance = 0;
    for (TagDetections.Tag tag : seen.tags()) {
      if (field.tag(tag.id()).isPresent()) {
        known.add(tag);
        distance += tag.bestCameraToTag().getTranslation().getNorm();
      }
    }
    if (known.isEmpty()) {
      return rejected(seen, "no tags from this field's layout", Double.NaN, Double.NaN);
    }
    distance /= known.size();
    if (!Double.isFinite(distance)) {
      return rejected(seen, "a tag's pose from the camera isn't a number", Double.NaN, Double.NaN);
    }
    if (robot.recentYawRateRadiansPerSecond() > config.maxYawRateRadiansPerSecond()) {
      return rejected(
          seen,
          format("turning too fast: %.1f rad/s", robot.recentYawRateRadiansPerSecond()),
          distance,
          Double.NaN);
    }
    if (distance > config.maxDistanceMeters()) {
      return rejected(seen, format("too far: %.1f m", distance), distance, Double.NaN);
    }
    Rotation2d heading = robot.heading();
    if (heading != null && robot.headingStdDevRadians() <= config.maxHeadingStdDevToConstrain()) {
      return headingConstrained(seen, known, camera, robot, heading, distance, config);
    }
    TagDetections.MultiTag multiTag = seen.multiTag();
    if (multiTag != null) {
      Pose3d pose =
          TagSolver.robotFromCamera(
              new Pose3d(
                  multiTag.fieldToCamera().getTranslation(),
                  multiTag.fieldToCamera().getRotation()),
              camera.robotToCamera());
      return fullPose(
          seen, pose, known, camera, robot, distance, VisionMeasurement.Solver.MULTI_TAG, config);
    }
    TagDetections.Tag clearest = known.get(0);
    for (TagDetections.Tag tag : known) {
      if (tag.ambiguity() < clearest.ambiguity()) {
        clearest = tag;
      }
    }
    if (clearest.ambiguity() > config.maxAmbiguity()) {
      return rejected(
          seen,
          format("ambiguous single tag: %.2f > %.2f", clearest.ambiguity(), config.maxAmbiguity()),
          distance,
          Double.NaN);
    }
    Pose3d pose =
        TagSolver.robotFromTag(
            field.tag(clearest.id()).orElseThrow(),
            clearest.bestCameraToTag(),
            camera.robotToCamera());
    return fullPose(
        seen,
        pose,
        List.of(clearest),
        camera,
        robot,
        clearest.bestCameraToTag().getTranslation().getNorm(),
        VisionMeasurement.Solver.SINGLE_TAG,
        config);
  }

  private VisionResult headingConstrained(
      TagDetections seen,
      List<TagDetections.Tag> known,
      CameraConfig camera,
      CaptureContext robot,
      Rotation2d heading,
      double distance,
      VisionConfig config) {
    Transform3d robotToCamera = camera.robotToCamera();
    Rotation3d attitude =
        new Rotation3d(robot.rollRadians(), robot.pitchRadians(), heading.getRadians());
    double sumX = 0;
    double sumY = 0;
    double sumWeights = 0;
    double worstHeight = 0;
    for (TagDetections.Tag tag : known) {
      Pose3d fieldToTag = field.tag(tag.id()).orElseThrow();
      Transform3d cameraToTag = agreeingWithTheGyro(tag, fieldToTag, robotToCamera, heading);
      TagSolver.PositionFix fix =
          TagSolver.positionFromTag(fieldToTag, robotToCamera.plus(cameraToTag), attitude);
      double tagDistance = cameraToTag.getTranslation().getNorm();
      double weight = 1 / Math.max(tagDistance * tagDistance, 1e-6);
      sumX += weight * fix.position().getX();
      sumY += weight * fix.position().getY();
      sumWeights += weight;
      if (Math.abs(fix.heightErrorMeters()) > Math.abs(worstHeight)) {
        worstHeight = fix.heightErrorMeters();
      }
    }
    if (!(Math.abs(worstHeight) <= config.maxHeightErrorMeters())) {
      return rejected(
          seen, format("a tag's height is off by %.2f m", worstHeight), distance, worstHeight);
    }
    Translation2d position = new Translation2d(sumX / sumWeights, sumY / sumWeights);
    if (!Double.isFinite(position.getX()) || !Double.isFinite(position.getY())) {
      return rejected(seen, "the solve isn't a number", distance, worstHeight);
    }
    double spread =
        config.linearStdDevBaseline()
            * config.headingConstrainedFactor()
            * scale(known.size(), distance, camera, robot, config);

    // Heading, too, only under strict conditions; otherwise the gyro keeps it to itself.
    Rotation2d measuredHeading = heading;
    double headingStdDev = Double.POSITIVE_INFINITY;
    TagDetections.MultiTag multiTag = seen.multiTag();
    if (multiTag != null
        && multiTag.tagIds().size() >= config.minTagsForHeading()
        && robot.recentYawRateRadiansPerSecond() <= config.maxYawRateForHeading()
        && multiTag.reprojectionErrorPixels() <= config.maxReprojectionErrorForHeadingPixels()) {
      Pose3d full =
          TagSolver.robotFromCamera(
              new Pose3d(
                  multiTag.fieldToCamera().getTranslation(),
                  multiTag.fieldToCamera().getRotation()),
              robotToCamera);
      Rotation2d solved = full.toPose2d().getRotation();
      if (Double.isFinite(solved.getRadians())) {
        measuredHeading = solved;
        headingStdDev =
            config.angularStdDevBaseline() * scale(known.size(), distance, camera, robot, config);
      }
    }
    VisionMeasurement measurement =
        new VisionMeasurement(
            seen.timestampNanos(),
            new Pose2d(position, measuredHeading),
            PoseCovariance.fromStdDevs(spread, spread, headingStdDev),
            camera.name(),
            ids(known),
            VisionMeasurement.Solver.HEADING_CONSTRAINED);
    return new VisionResult(
        seen,
        measurement,
        format(
            "heading-constrained: %d %s at %.1f m%s",
            known.size(),
            known.size() == 1 ? "tag" : "tags",
            distance,
            Double.isFinite(headingStdDev) ? ", with heading" : ""),
        distance,
        worstHeight);
  }

  /** Of a tag's two possible poses, the one whose robot heading is closer to the gyro's. */
  private static Transform3d agreeingWithTheGyro(
      TagDetections.Tag tag, Pose3d fieldToTag, Transform3d robotToCamera, Rotation2d heading) {
    Transform3d best = tag.bestCameraToTag();
    Transform3d alternate = tag.altCameraToTag();
    if (alternate.getTranslation().getNorm() < 1e-6) {
      return best;
    }
    double bestMiss =
        Math.abs(
            TagSolver.robotFromTag(fieldToTag, best, robotToCamera)
                .toPose2d()
                .getRotation()
                .minus(heading)
                .getRadians());
    double alternateMiss =
        Math.abs(
            TagSolver.robotFromTag(fieldToTag, alternate, robotToCamera)
                .toPose2d()
                .getRotation()
                .minus(heading)
                .getRadians());
    return alternateMiss < bestMiss ? alternate : best;
  }

  private VisionResult fullPose(
      TagDetections seen,
      Pose3d pose,
      List<TagDetections.Tag> used,
      CameraConfig camera,
      CaptureContext robot,
      double distance,
      VisionMeasurement.Solver solver,
      VisionConfig config) {
    double height = pose.getZ();
    if (!Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getZ())) {
      return rejected(seen, "the solve isn't a number", distance, height);
    }
    if (!(Math.abs(height) <= config.maxHeightErrorMeters())) {
      return rejected(
          seen, format("the solve puts the robot %.2f m off the floor", height), distance, height);
    }
    double scale = scale(used.size(), distance, camera, robot, config);
    VisionMeasurement measurement =
        new VisionMeasurement(
            seen.timestampNanos(),
            pose.toPose2d(),
            PoseCovariance.fromStdDevs(
                config.linearStdDevBaseline() * scale,
                config.linearStdDevBaseline() * scale,
                config.angularStdDevBaseline() * scale),
            camera.name(),
            ids(used),
            solver);
    return new VisionResult(
        seen,
        measurement,
        format(
            "%s: %d %s at %.1f m",
            solver == VisionMeasurement.Solver.MULTI_TAG ? "multi-tag" : "single tag",
            used.size(),
            used.size() == 1 ? "tag" : "tags",
            distance),
        distance,
        height);
  }

  /**
   * How much a result's uncertainty grows from the one-tag-at-one-meter baseline: with distance,
   * fewer tags, the camera's own noise, and the robot's motion.
   */
  private static double scale(
      int tags, double distance, CameraConfig camera, CaptureContext robot, VisionConfig config) {
    double motion =
        1
            + config.speedStdDevFactor() * robot.speedMetersPerSecond()
            + config.yawRateStdDevFactor() * robot.recentYawRateRadiansPerSecond();
    return Math.pow(distance, config.distanceExponent()) / tags * camera.noiseScale() * motion;
  }

  private static List<Integer> ids(List<TagDetections.Tag> tags) {
    return tags.stream().map(TagDetections.Tag::id).toList();
  }

  private static VisionResult rejected(
      TagDetections seen, String why, double distance, double heightError) {
    return new VisionResult(seen, null, why, distance, heightError);
  }

  private static String format(String pattern, Object... values) {
    return String.format(Locale.ROOT, pattern, values);
  }
}
