package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.localization.TagDetections;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.subsystems.vision.VisionSim;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.SplittableRandom;
import java.util.stream.Collectors;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The study's camera against PhotonVision's own simulation ({@code PhotonCameraSim.process}), image
 * for image with the same seeds, over a sample of 2026's field with the template robot's four
 * cameras: the same tags seen, their corners in the same places, and the same solves.
 */
@ExtendWith(RobotSim.class)
class CameraImageSimTest {
  private static final FieldModel FIELD = TagLayouts.named("FRC_2026_REBUILT_WELDED");
  private static final CameraConfig.Sensor SENSOR =
      Robots.SIM.definition().cameras().get(0).sensor();
  private static final Conditions NOMINAL = Conditions.nominal(CameraStudyConfig.DEFAULTS);

  /** The worst difference found, of each kind, for the summary line. */
  private final Map<String, Double> worst = new HashMap<>();

  private void differs(String what, double by) {
    worst.merge(what, by, Math::max);
  }

  private static double apart(Transform3d a, Transform3d b) {
    return a.getTranslation().getDistance(b.getTranslation());
  }

  private static double turned(Transform3d a, Transform3d b) {
    return a.getRotation().relativeTo(b.getRotation()).getAngle();
  }

  /** Ambiguity as {@code Vision} reads PhotonVision's: -1 (none to give) is a coin toss. */
  private static double ambiguity(double reported) {
    return !(reported >= 0) ? 1 : Math.min(reported, 1);
  }

  @Test
  void itSeesWhatPhotonVisionsSimulationSees() {
    CameraImageSim study = new CameraImageSim(FIELD, SENSOR, NOMINAL);
    List<VisionTargetSim> targets = new ArrayList<>();
    FIELD
        .tags()
        .forEach(
            (id, pose) -> targets.add(new VisionTargetSim(pose, TargetModel.kAprilTag36h11, id)));
    try (PhotonCamera camera = new PhotonCamera("camera-study-check")) {
      PhotonCameraSim photonVision =
          new PhotonCameraSim(camera, VisionSim.properties(SENSOR, 0), FIELD.toField());
      try {
        photonVision.setMaxSightRange(VisionSim.MAX_SIGHT_METERS);
        photonVision.enableRawStream(false);
        photonVision.enableProcessedStream(false);
        compare(study, photonVision, targets);
      } finally {
        photonVision.close();
      }
    }
  }

  private void compare(
      CameraImageSim study, PhotonCameraSim photonVision, List<VisionTargetSim> targets) {
    // Poses all over the field; each of PhotonVision's images costs a megabyte until collected.
    SplittableRandom random = new SplittableRandom(1);
    int images = 0;
    int tags = 0;
    int multiTags = 0;
    for (int i = 0; i < 40; i++) {
      Pose3d robot =
          new Pose3d(
              new Pose2d(
                  random.nextDouble(1, FIELD.length() - 1),
                  random.nextDouble(1, FIELD.width() - 1),
                  Rotation2d.fromRadians(random.nextDouble(-Math.PI, Math.PI))));
      for (CameraConfig mount : Robots.SIM.definition().cameras()) {
        Pose3d cameraPose = robot.transformBy(mount.robotToCamera());
        long seed = random.nextLong();
        photonVision.prop.setRandomSeed(seed);
        PhotonPipelineResult theirs = photonVision.process(0, cameraPose, targets);
        TagDetections ours = study.image(cameraPose, mount.name(), seed, List.of());
        images++;

        Map<Integer, PhotonTrackedTarget> byId = new HashMap<>();
        theirs.getTargets().forEach(target -> byId.put(target.getFiducialId(), target));
        assertThat(ours.tags())
            .extracting(TagDetections.Tag::id)
            .containsExactlyInAnyOrderElementsOf(byId.keySet());
        for (TagDetections.Tag tag : ours.tags()) {
          PhotonTrackedTarget target = Objects.requireNonNull(byId.get(tag.id()));
          for (int c = 0; c < 4; c++) {
            differs(
                "corner px",
                Math.hypot(
                    tag.corners().get(c).getX() - target.getDetectedCorners().get(c).x,
                    tag.corners().get(c).getY() - target.getDetectedCorners().get(c).y));
          }
          differs("tag m", apart(tag.bestCameraToTag(), target.getBestCameraToTarget()));
          differs("tag rad", turned(tag.bestCameraToTag(), target.getBestCameraToTarget()));
          differs("alternate m", apart(tag.altCameraToTag(), target.getAlternateCameraToTarget()));
          // The pipeline picks between the two by the heading each implies: their turns matter.
          differs(
              "alternate rad", turned(tag.altCameraToTag(), target.getAlternateCameraToTarget()));
          differs("ambiguity", Math.abs(tag.ambiguity() - ambiguity(target.getPoseAmbiguity())));
          tags++;
        }

        MultiTargetPNPResult solved = theirs.getMultiTagResult().orElse(null);
        TagDetections.MultiTag multiTag = ours.multiTag();
        assertThat(multiTag != null).isEqualTo(solved != null);
        if (multiTag != null && solved != null) {
          assertThat(multiTag.tagIds())
              .containsExactlyInAnyOrderElementsOf(
                  solved.fiducialIDsUsed.stream().map(Short::intValue).toList());
          differs("multi-tag m", apart(multiTag.fieldToCamera(), solved.estimatedPose.best));
          differs("multi-tag rad", turned(multiTag.fieldToCamera(), solved.estimatedPose.best));
          differs(
              "reprojection px",
              Math.abs(multiTag.reprojectionErrorPixels() - solved.estimatedPose.bestReprojErr));
          multiTags++;
        }
      }
    }
    System.out.printf(
        Locale.ROOT,
        "Against PhotonCameraSim: %d images, %d tags and %d multi-tag solves seen alike;"
            + " worst differences: %s%n",
        images,
        tags,
        multiTags,
        worst.entrySet().stream()
            .sorted(Map.Entry.comparingByKey())
            .map(entry -> String.format(Locale.ROOT, "%s %.1e", entry.getKey(), entry.getValue()))
            .collect(Collectors.joining(", ")));
    assertThat(tags).isGreaterThan(100);
    assertThat(multiTags).isGreaterThan(20);
    // OpenCV projects in single precision for PhotonVision; the study projects in double.
    assertThat(worst.get("corner px")).isLessThan(0.01);
    assertThat(worst.get("tag m")).isLessThan(0.001);
    assertThat(worst.get("tag rad")).isLessThan(0.001);
    assertThat(worst.get("alternate m")).isLessThan(0.001);
    assertThat(worst.get("alternate rad")).isLessThan(0.001);
    assertThat(worst.get("ambiguity")).isLessThan(0.01);
    assertThat(worst.get("multi-tag m")).isLessThan(0.001);
    assertThat(worst.get("multi-tag rad")).isLessThan(0.001);
    assertThat(worst.get("reprojection px")).isLessThan(0.01);
  }

  /**
   * In front of the blue hub's near face, its two tags in view. When one tag's own solve fails, the
   * robot reads only the other, as {@code Vision} leaves out a pose that isn't a number; the
   * multi-tag solve still uses both tags' corners, as PhotonVision's does.
   */
  @Test
  void aTagWhoseOwnSolveFailsIsLeftOutButItsCornersStillCount() {
    CameraImageSim study = new CameraImageSim(FIELD, SENSOR, NOMINAL);
    Pose3d cameraPose = new Pose3d(2.3, 4.2, 0.5, new Rotation3d(0, Math.toRadians(-10), 0));
    List<CameraImageSim.Seen> seen = new ArrayList<>(study.seen(cameraPose, 1, List.of()));
    assertThat(seen).hasSizeGreaterThanOrEqualTo(2);
    TagDetections whole = study.detections(seen, "front");
    int failed = seen.get(0).tag().id();

    seen.set(0, seen.get(0).unsolved());
    TagDetections without = study.detections(seen, "front");

    assertThat(without.tags())
        .extracting(TagDetections.Tag::id)
        .doesNotContain(failed)
        .hasSize(whole.tags().size() - 1);
    TagDetections.MultiTag solvedWith = Objects.requireNonNull(whole.multiTag());
    TagDetections.MultiTag solvedWithout = Objects.requireNonNull(without.multiTag());
    assertThat(solvedWithout.tagIds()).doesNotContain(failed);
    assertThat(solvedWithout.fieldToCamera()).isEqualTo(solvedWith.fieldToCamera());
  }

  /** Another robot between the camera and the hub's face hides its tags. */
  @Test
  void anotherRobotInTheWayHidesTheTags() {
    CameraImageSim study = new CameraImageSim(FIELD, SENSOR, NOMINAL);
    Pose3d cameraPose = new Pose3d(2.3, 4.2, 0.5, new Rotation3d(0, Math.toRadians(-10), 0));
    OtherRobots.Blocker between = new OtherRobots.Blocker(3.2, 4.2, 0, 0.45, 0.45, 1.5);
    OtherRobots.Blocker aside = new OtherRobots.Blocker(3.2, 1.0, 0, 0.45, 0.45, 1.5);

    assertThat(study.image(cameraPose, "front", 1, List.of()).tags())
        .hasSizeGreaterThanOrEqualTo(2);
    assertThat(study.image(cameraPose, "front", 1, List.of(between)).tags()).isEmpty();
    assertThat(study.image(cameraPose, "front", 1, List.of(aside)).tags())
        .isEqualTo(study.image(cameraPose, "front", 1, List.of()).tags());
  }

  /**
   * A post between the camera and one of the hub face's two tags hides that one alone. The other
   * tags' corners are exactly as in the clear image: the hidden tag's noise is still drawn, in
   * PhotonVision's order, so the rest of the image is the same.
   */
  @Test
  void aNarrowPostHidesOneTagAndLeavesTheRestAsTheyWere() {
    CameraImageSim study = new CameraImageSim(FIELD, SENSOR, NOMINAL);
    Pose3d cameraPose = new Pose3d(2.3, 4.2, 0.5, new Rotation3d(0, Math.toRadians(-10), 0));
    // Halfway to the face, where the line to tag 25 (y 4.39) passes y 4.30, and tag 26's, 4.11.
    OtherRobots.Blocker post = new OtherRobots.Blocker(3.2, 4.30, 0, 0.05, 0.03, 2);

    TagDetections clear = study.image(cameraPose, "front", 1, List.of());
    TagDetections hidden = study.image(cameraPose, "front", 1, List.of(post));

    assertThat(clear.tags()).extracting(TagDetections.Tag::id).contains(25, 26);
    assertThat(hidden.tags())
        .containsExactlyElementsOf(clear.tags().stream().filter(tag -> tag.id() != 25).toList());
  }

  @Test
  void anImageWithNoTagInViewHasNothingInIt() {
    CameraImageSim study = new CameraImageSim(FIELD, SENSOR, NOMINAL);

    // The middle of the field, looking straight up.
    TagDetections sky =
        study.image(new Pose3d(8, 4, 1, new Rotation3d(0, -Math.PI / 2, 0)), "up", 1, List.of());

    assertThat(sky.tags()).isEmpty();
    assertThat(sky.multiTag()).isNull();
  }
}
