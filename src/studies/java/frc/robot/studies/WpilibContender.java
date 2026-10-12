package frc.robot.studies;

import com.michaelgrundvig.frc.localization.DrivetrainReadings;
import com.michaelgrundvig.frc.localization.Localizer;
import com.michaelgrundvig.frc.localization.LoopContext;
import com.michaelgrundvig.frc.localization.PoseCovariance;
import com.michaelgrundvig.frc.localization.PoseReset;
import com.michaelgrundvig.frc.localization.VisionMeasurement;
import com.michaelgrundvig.frc.localization.WpilibLocalizer;
import com.michaelgrundvig.frc.runtime.subsystems.vision.CameraIO;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;

/**
 * WPILib's {@code SwerveDrivePoseEstimator} as one of the estimator study's contenders, fed beside
 * the robot's own localizer in the same loop: the same wheel readings, and the cameras' images one
 * of three ways ({@link Feed}).
 *
 * <p>Every contender is a {@link Localizer}, WPILib's estimator as a robot would run it ({@link
 * WpilibLocalizer}), and the study talks to it only through that interface: the loop's readings go
 * in through {@link Localizer#odometry}, a placement through {@link Localizer#place}, every
 * measurement through {@link Localizer#measurements}, and the pose is read from {@link
 * Localizer#pose}. So the study exercises exactly what a team's own localizer would be given.
 */
final class WpilibContender {
  /** How the cameras' images reach the estimator. */
  enum Feed {
    /**
     * Every measurement the robot's vision pipeline handed its own estimator, after the pipeline's
     * filtering, with the standard deviations the pipeline gave it: the estimator alone.
     */
    SAME_INPUTS,
    /** Each image solved and filtered by {@link Rules}, as strong teams' public code does. */
    TEAM,
    /** Every image solved and added, unfiltered, with WPILib's default standard deviations. */
    RAW,
    /** No images at all: the wheels and the yaw alone, for what the cameras are worth. */
    NONE
  }

  /**
   * The filtering and the standard deviations a team puts in front of WPILib's estimator. Each rule
   * is one found in strong teams' public robot code; docs/estimator-comparison.md says which came
   * from where. A limit that is infinite, or zero where it says so, turns its rule off.
   *
   * @param stateStdDevMeters the estimator's state standard deviation for x and y (WPILib's
   *     default: 0.1)
   * @param stateStdDevRadians and for heading (WPILib's default: 0.1)
   * @param maxAmbiguity an image of one tag is used only at or below this ambiguity
   * @param maxHeightMeters a solve this far above or below the floor is thrown out
   * @param fieldMarginMeters a solve farther outside the field than this is thrown out
   * @param maxSingleTagDistanceMeters an image of one tag farther than this is thrown out
   * @param maxDistanceMeters any image whose tags are farther than this, on average, is thrown out
   * @param maxYawRateRadiansPerSecond images are thrown out while the robot has turned faster than
   *     this in the last 0.3 s
   * @param maxTiltRadians and while it pitches or rolls more than this
   * @param linearBaseline x and y standard deviation, in meters, for one tag at one meter
   * @param angularBaseline heading standard deviation, in radians, for one tag at one meter
   * @param distanceExponent the standard deviations grow with the tags' distance to this power
   * @param tagCountExponent and shrink with the number of tags to this power
   * @param multiTagHeading whether an image of several tags may correct the heading; one tag's
   *     never does
   * @param sorted whether a loop's images are added oldest first, across the cameras; otherwise
   *     camera by camera, as they are read
   * @param pickByHeading whether, of the two poses one tag allows, the one whose heading is nearer
   *     the estimate's is used, whichever the camera called likelier
   */
  record Rules(
      double stateStdDevMeters,
      double stateStdDevRadians,
      double maxAmbiguity,
      double maxHeightMeters,
      double fieldMarginMeters,
      double maxSingleTagDistanceMeters,
      double maxDistanceMeters,
      double maxYawRateRadiansPerSecond,
      double maxTiltRadians,
      double linearBaseline,
      double angularBaseline,
      double distanceExponent,
      double tagCountExponent,
      boolean multiTagHeading,
      boolean sorted,
      boolean pickByHeading) {
    /**
     * The widely used open-source swerve template's vision filter and its constants (ambiguity 0.3,
     * height 0.75 m, on the field, 0.02 m and 0.06 rad at one meter growing with distance squared
     * over the tag count, one tag's heading ignored), on WPILib's default state standard
     * deviations, with three rules from top teams' code added: no image while the robot turned
     * faster than 5 rad/s in the last 0.3 s, none while it pitches or rolls past 5°, and none of
     * one tag past 3 m or of any past 6 m.
     */
    static final Rules TEMPLATE =
        new Rules(
            0.1,
            0.1,
            0.3,
            0.75,
            0,
            3.0,
            6.0,
            5.0,
            Math.toRadians(5),
            0.02,
            0.06,
            2,
            1,
            true,
            false,
            false);

    /** The same rules, one tag's pose picked by the estimate's heading: ambiguity up to 0.5. */
    Rules pickingByHeading() {
      return new Rules(
          stateStdDevMeters,
          stateStdDevRadians,
          0.5,
          maxHeightMeters,
          fieldMarginMeters,
          maxSingleTagDistanceMeters,
          maxDistanceMeters,
          maxYawRateRadiansPerSecond,
          maxTiltRadians,
          linearBaseline,
          angularBaseline,
          distanceExponent,
          tagCountExponent,
          multiTagHeading,
          sorted,
          true);
    }

    /** The same rules on other state standard deviations. */
    Rules withState(double meters, double radians) {
      return new Rules(
          meters,
          radians,
          maxAmbiguity,
          maxHeightMeters,
          fieldMarginMeters,
          maxSingleTagDistanceMeters,
          maxDistanceMeters,
          maxYawRateRadiansPerSecond,
          maxTiltRadians,
          linearBaseline,
          angularBaseline,
          distanceExponent,
          tagCountExponent,
          multiTagHeading,
          sorted,
          pickByHeading);
    }

    /** The same rules with another noise model: the baselines, and the tag count's power. */
    Rules withNoise(double linear, double angular, double tagPower) {
      return new Rules(
          stateStdDevMeters,
          stateStdDevRadians,
          maxAmbiguity,
          maxHeightMeters,
          fieldMarginMeters,
          maxSingleTagDistanceMeters,
          maxDistanceMeters,
          maxYawRateRadiansPerSecond,
          maxTiltRadians,
          linear,
          angular,
          distanceExponent,
          tagPower,
          multiTagHeading,
          sorted,
          pickByHeading);
    }

    /** The same rules with stricter or looser limits on ambiguity and height. */
    Rules withLimits(double ambiguity, double heightMeters) {
      return new Rules(
          stateStdDevMeters,
          stateStdDevRadians,
          ambiguity,
          heightMeters,
          fieldMarginMeters,
          maxSingleTagDistanceMeters,
          maxDistanceMeters,
          maxYawRateRadiansPerSecond,
          maxTiltRadians,
          linearBaseline,
          angularBaseline,
          distanceExponent,
          tagCountExponent,
          multiTagHeading,
          sorted,
          pickByHeading);
    }

    /** The same rules, the heading left to the gyro alone or not, and a loop's images sorted. */
    Rules with(boolean headingFromSeveralTags, boolean oldestFirst) {
      return new Rules(
          stateStdDevMeters,
          stateStdDevRadians,
          maxAmbiguity,
          maxHeightMeters,
          fieldMarginMeters,
          maxSingleTagDistanceMeters,
          maxDistanceMeters,
          maxYawRateRadiansPerSecond,
          maxTiltRadians,
          linearBaseline,
          angularBaseline,
          distanceExponent,
          tagCountExponent,
          headingFromSeveralTags,
          oldestFirst,
          pickByHeading);
    }
  }

  /**
   * One camera image as its IO gave it.
   *
   * @param camera the camera's name
   * @param robotToCamera its configured mount
   * @param frame the image
   * @param tags the image's tags
   */
  record Image(
      String camera, Transform3d robotToCamera, CameraIO.Frame frame, List<CameraIO.SeenTag> tags) {
    Image {
      tags = List.copyOf(tags);
    }
  }

  /**
   * A robot pose solved from an image the way PhotonVision's own pose estimator does.
   *
   * @param pose the robot's pose
   * @param tagCount how many tags it rests on
   * @param distanceMeters how far they are from the camera, on average
   * @param ambiguity the one tag's ambiguity; zero for several
   */
  record Solve(Pose3d pose, int tagCount, double distanceMeters, double ambiguity) {}

  /** How far back "lately" reaches for how fast the robot has turned, in loops: 0.3 s. */
  private static final int YAW_RATE_LOOPS = 15;

  /** WPILib's default standard deviation for a measurement given none: x, y and heading. */
  private static final double DEFAULT_VISION_STD_DEV = 0.9;

  /** How sure a placement on the robot's start is, as the robot's own localizer is told. */
  private static final PoseCovariance PLACED =
      PoseCovariance.fromStdDevs(0.05, 0.05, Math.toRadians(2));

  private final String name;
  private final Feed feed;
  private final Rules rules;
  private final FieldModel field;

  /** WPILib's estimator, which the study reaches only as a localizer. */
  private final WpilibLocalizer wpilib;

  private final Localizer localizer;
  private final double[] recentYawRates = new double[YAW_RATE_LOOPS];
  private int loop;
  private double tiltRadians;
  private final Map<String, Long> lastStampNanos = new HashMap<>();

  /** The images solved, or the pipeline's measurements, that the contender was offered. */
  private int offered;

  /**
   * @param name what the report calls it
   * @param feed how the cameras' images reach it
   * @param rules its state standard deviations, and for {@link Feed#TEAM} its filtering
   * @param field the field whose tags the images are solved against
   * @param robot the robot, for where its swerve modules are
   */
  WpilibContender(String name, Feed feed, Rules rules, FieldModel field, RobotDefinition robot) {
    this.name = name;
    this.feed = feed;
    this.rules = rules;
    this.field = field;
    if (feed == Feed.RAW) {
      // The documented defaults: state 0.1, vision 0.9.
      wpilib = new WpilibLocalizer(robot, field);
    } else {
      wpilib =
          new WpilibLocalizer(robot, field, rules.stateStdDevMeters(), rules.stateStdDevRadians());
    }
    localizer = wpilib;
  }

  String name() {
    return name;
  }

  Feed feed() {
    return feed;
  }

  /** Where it believes the robot is. */
  Pose2d pose() {
    return localizer.pose();
  }

  /** The drive team placed the robot here: the placement the robot's own localizer gets. */
  void place(Pose2d pose, long nowNanos) {
    localizer.place(new PoseReset(nowNanos, pose, PLACED, "placed by the study"));
  }

  /** One loop's readings of the wheels and the IMUs, as the robot's own localizer is given them. */
  void odometry(DrivetrainReadings readings, LoopContext context) {
    localizer.odometry(readings, context);
  }

  /** What the IMU read this loop, for the rules that go by it. */
  void imu(double yawRateRadiansPerSecond, double pitchRadians, double rollRadians) {
    recentYawRates[loop % YAW_RATE_LOOPS] = Math.abs(yawRateRadiansPerSecond);
    loop++;
    tiltRadians = Math.max(Math.abs(pitchRadians), Math.abs(rollRadians));
  }

  /** The loop's measurements as the robot's pipeline handed them to its own localizer. */
  void measurements(List<VisionMeasurement> taken) {
    offered += taken.size();
    localizer.measurements(taken);
  }

  /**
   * The loop's images as the cameras gave them, camera by camera. Each is solved and filtered here,
   * as a team's code in front of WPILib's estimator does, and what is left is given to the
   * localizer one measurement at a time, so each is applied before the next image is solved.
   */
  void images(List<Image> images) {
    if (feed == Feed.NONE) {
      return;
    }
    List<Image> order = new ArrayList<>(images);
    if (feed == Feed.TEAM && rules.sorted()) {
      order.sort(Comparator.comparingLong(image -> image.frame().timestampNanos()));
    }
    for (Image image : order) {
      long taken = image.frame().timestampNanos();
      Long last = lastStampNanos.get(image.camera());
      // PhotonVision's library hands each result over once.
      if (last != null && taken <= last) {
        continue;
      }
      lastStampNanos.put(image.camera(), taken);
      boolean pick = feed == Feed.TEAM && rules.pickByHeading();
      Optional<Solve> solved = solve(image, field, pick ? pose().getRotation() : null);
      if (solved.isEmpty()) {
        continue;
      }
      offered++;
      Solve solve = solved.get();
      if (feed == Feed.RAW) {
        give(image, solve, DEFAULT_VISION_STD_DEV, DEFAULT_VISION_STD_DEV, DEFAULT_VISION_STD_DEV);
        continue;
      }
      if (rejects(solve)) {
        continue;
      }
      double scale =
          Math.pow(solve.distanceMeters(), rules.distanceExponent())
              / Math.pow(solve.tagCount(), rules.tagCountExponent());
      double linear = rules.linearBaseline() * scale;
      boolean heading = rules.multiTagHeading() && solve.tagCount() > 1;
      // An infinite heading spread is how a measurement says it knows nothing of the heading.
      double angular = heading ? rules.angularBaseline() * scale : Double.POSITIVE_INFINITY;
      give(image, solve, linear, linear, angular);
    }
  }

  /** Whether the team's rules throw a solve out. */
  private boolean rejects(Solve solve) {
    Pose3d pose = solve.pose();
    boolean oneTag = solve.tagCount() == 1;
    if (oneTag && solve.ambiguity() > rules.maxAmbiguity()) {
      return true;
    }
    if (oneTag && solve.distanceMeters() > rules.maxSingleTagDistanceMeters()) {
      return true;
    }
    if (solve.distanceMeters() > rules.maxDistanceMeters()) {
      return true;
    }
    if (Math.abs(pose.getZ()) > rules.maxHeightMeters()) {
      return true;
    }
    double margin = rules.fieldMarginMeters();
    boolean onTheField =
        pose.getX() >= -margin
            && pose.getX() <= field.length() + margin
            && pose.getY() >= -margin
            && pose.getY() <= field.width() + margin;
    if (!onTheField) {
      return true;
    }
    double recentYawRate = 0;
    for (double rate : recentYawRates) {
      recentYawRate = Math.max(recentYawRate, rate);
    }
    return recentYawRate > rules.maxYawRateRadiansPerSecond()
        || tiltRadians > rules.maxTiltRadians();
  }

  /** Gives the localizer one solved image as a measurement, with its standard deviations. */
  private void give(Image image, Solve solve, double x, double y, double heading) {
    List<Integer> tags = new ArrayList<>();
    for (CameraIO.SeenTag tag : image.tags()) {
      tags.add(tag.id());
    }
    VisionMeasurement measurement =
        new VisionMeasurement(
            image.frame().timestampNanos(),
            solve.pose().toPose2d(),
            PoseCovariance.fromStdDevs(x, y, heading),
            VisionMeasurement.Lever.NONE,
            image.camera(),
            tags,
            VisionMeasurement.Solver.EXTERNAL);
    localizer.measurements(List.of(measurement));
  }

  /** Measurements it was offered: solved images, or the pipeline's measurements. */
  int offered() {
    return offered;
  }

  /** Measurements that reached WPILib's estimator and corrected it. */
  int added() {
    return wpilib.corrected();
  }

  /** Corrections WPILib discarded because an older measurement arrived after them. */
  int laterDiscarded() {
    return wpilib.discardedLater();
  }

  /**
   * The robot's pose from one image, as PhotonVision's pose estimator solves it: the coprocessor's
   * solution from all the tags at once when it made one, and otherwise the least ambiguous tag's
   * likelier pose. Empty for an image with no tag the field has, or whose solve isn't a number.
   *
   * @param heading when not null, the one tag's pose whose heading is nearer this is used
   */
  static Optional<Solve> solve(Image image, FieldModel field, @Nullable Rotation2d heading) {
    CameraIO.Frame frame = image.frame();
    Transform3d cameraToRobot = image.robotToCamera().inverse();
    if (frame.hasMultiTag()) {
      int count = 0;
      double distance = 0;
      for (CameraIO.SeenTag tag : image.tags()) {
        if (tag.inMultiTag()) {
          count++;
          distance += tag.bestCameraToTag().getTranslation().getNorm();
        }
      }
      if (count > 0) {
        Transform3d fieldToCamera = frame.multiTagFieldToCamera();
        Pose3d camera = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());
        return finite(new Solve(camera.transformBy(cameraToRobot), count, distance / count, 0));
      }
    }
    CameraIO.SeenTag clearest = null;
    for (CameraIO.SeenTag tag : image.tags()) {
      boolean known = field.tag(tag.id()).isPresent();
      if (known && (clearest == null || ambiguity(tag) < ambiguity(clearest))) {
        clearest = tag;
      }
    }
    if (clearest == null) {
      return Optional.empty();
    }
    Pose3d tag = field.tag(clearest.id()).orElseThrow();
    Pose3d robot = tag.transformBy(clearest.bestCameraToTag().inverse()).transformBy(cameraToRobot);
    boolean hasAlternate = clearest.altCameraToTag().getTranslation().getNorm() > 1e-6;
    if (heading != null && hasAlternate) {
      Pose3d other =
          tag.transformBy(clearest.altCameraToTag().inverse()).transformBy(cameraToRobot);
      if (Double.isFinite(other.getX()) && off(other, heading) < off(robot, heading)) {
        robot = other;
      }
    }
    return finite(
        new Solve(
            robot, 1, clearest.bestCameraToTag().getTranslation().getNorm(), ambiguity(clearest)));
  }

  /** How far a pose's heading is from another, in radians. */
  private static double off(Pose3d pose, Rotation2d heading) {
    return Math.abs(pose.toPose2d().getRotation().minus(heading).getRadians());
  }

  /** A tag's ambiguity; PhotonVision's -1 for "none to give" counts as a coin toss. */
  private static double ambiguity(CameraIO.SeenTag tag) {
    return tag.ambiguity() >= 0 ? tag.ambiguity() : 1;
  }

  private static Optional<Solve> finite(Solve solve) {
    Pose3d pose = solve.pose();
    boolean numbers =
        Double.isFinite(pose.getX())
            && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getZ())
            && Double.isFinite(pose.getRotation().getZ())
            && Double.isFinite(solve.distanceMeters());
    return numbers ? Optional.of(solve) : Optional.empty();
  }
}
