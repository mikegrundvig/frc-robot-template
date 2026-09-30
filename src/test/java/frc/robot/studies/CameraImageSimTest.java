package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.localization.TagDetections;
import frc.robot.subsystems.vision.VisionSim;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;

/**
 * The study's camera against PhotonVision's own simulation ({@code PhotonCameraSim.process}), image
 * for image, over a sample of 2026's field with the template robot's four cameras: the same tags
 * seen, their corners in the same places, and the same solves.
 */
@ExtendWith(RobotSim.class)
class CameraImageSimTest {
  private static final FieldModel FIELD = TagLayouts.named("FRC_2026_REBUILT_WELDED");

  @Test
  void itSeesWhatPhotonVisionsSimulationSees() {
    CameraConfig.Sensor sensor = Robots.SIM.cameras().get(0).sensor();
    CameraImageSim study = new CameraImageSim(FIELD, sensor);
    List<VisionTargetSim> targets = new ArrayList<>();
    FIELD
        .tags()
        .forEach(
            (id, pose) -> targets.add(new VisionTargetSim(pose, TargetModel.kAprilTag36h11, id)));
    try (PhotonCamera camera = new PhotonCamera("camera-study-check")) {
      PhotonCameraSim photonVision =
          new PhotonCameraSim(camera, VisionSim.properties(sensor, 0), FIELD.toField());
      photonVision.setMaxSightRange(VisionSim.MAX_SIGHT_METERS);
      photonVision.enableRawStream(false);
      photonVision.enableProcessedStream(false);

      // Poses all over the field; each of PhotonVision's images costs a megabyte until collected.
      SplittableRandom random = new SplittableRandom(1);
      int images = 0;
      int tags = 0;
      int multiTags = 0;
      double worstCorner = 0;
      double worstTag = 0;
      double worstMultiTag = 0;
      for (int i = 0; i < 40; i++) {
        Pose3d robot =
            new Pose3d(
                new Pose2d(
                    random.nextDouble(1, FIELD.length() - 1),
                    random.nextDouble(1, FIELD.width() - 1),
                    Rotation2d.fromRadians(random.nextDouble(-Math.PI, Math.PI))));
        for (CameraConfig mount : Robots.SIM.cameras()) {
          Pose3d cameraPose = robot.transformBy(mount.robotToCamera());
          long seed = random.nextLong();
          photonVision.prop.setRandomSeed(seed);
          PhotonPipelineResult theirs = photonVision.process(0, cameraPose, targets);
          TagDetections ours = study.image(cameraPose, mount.name(), seed);
          images++;

          Map<Integer, PhotonTrackedTarget> byId = new HashMap<>();
          theirs.getTargets().forEach(target -> byId.put(target.getFiducialId(), target));
          assertThat(ours.tags())
              .extracting(TagDetections.Tag::id)
              .containsExactlyInAnyOrderElementsOf(byId.keySet());
          for (TagDetections.Tag tag : ours.tags()) {
            PhotonTrackedTarget target = Objects.requireNonNull(byId.get(tag.id()));
            for (int c = 0; c < 4; c++) {
              Point corner =
                  new Point(
                      target.getDetectedCorners().get(c).x, target.getDetectedCorners().get(c).y);
              worstCorner =
                  Math.max(
                      worstCorner,
                      Math.hypot(
                          tag.corners().get(c).getX() - corner.x,
                          tag.corners().get(c).getY() - corner.y));
            }
            worstTag =
                Math.max(worstTag, apart(tag.bestCameraToTag(), target.getBestCameraToTarget()));
            tags++;
          }
          assertThat(ours.multiTag() != null).isEqualTo(theirs.getMultiTagResult().isPresent());
          TagDetections.MultiTag multiTag = ours.multiTag();
          if (multiTag != null) {
            worstMultiTag =
                Math.max(
                    worstMultiTag,
                    apart(
                        multiTag.fieldToCamera(),
                        theirs.getMultiTagResult().get().estimatedPose.best));
            multiTags++;
          }
        }
      }
      System.out.printf(
          Locale.ROOT,
          "Against PhotonCameraSim: %d images, %d tags and %d multi-tag solves, all seen alike;"
              + " worst differences: corner %.1e px, tag %.1e m, multi-tag camera %.1e m%n",
          images,
          tags,
          multiTags,
          worstCorner,
          worstTag,
          worstMultiTag);
      assertThat(tags).isGreaterThan(100);
      assertThat(multiTags).isGreaterThan(20);
      // OpenCV projects in single precision for PhotonVision; the study projects in double.
      assertThat(worstCorner).isLessThan(0.01);
      assertThat(worstTag).isLessThan(0.001);
      assertThat(worstMultiTag).isLessThan(0.001);
      photonVision.close();
    }
  }

  private static double apart(Transform3d a, Transform3d b) {
    return a.getTranslation().getDistance(b.getTranslation());
  }

  @Test
  void aTagsAreaIsItsConvexHulls() {
    Point[] square = {new Point(0, 0), new Point(10, 0), new Point(10, 10), new Point(0, 10)};
    Point[] bowtie = {new Point(0, 0), new Point(10, 10), new Point(10, 0), new Point(0, 10)};
    Point[] dented = {new Point(0, 0), new Point(10, 0), new Point(5, 2), new Point(5, 10)};

    assertThat(CameraImageSim.area(square)).isCloseTo(100, within(1e-12));
    assertThat(CameraImageSim.area(bowtie)).as("in any order").isCloseTo(100, within(1e-12));
    assertThat(CameraImageSim.area(dented)).as("the dent filled").isCloseTo(50, within(1e-12));
  }
}
