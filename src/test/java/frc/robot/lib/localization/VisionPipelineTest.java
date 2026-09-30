package frc.robot.lib.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Milliseconds;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.field.Symmetry;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The vision pipeline, on 2026's field: images made exactly from a known robot pose, so every solve
 * can be checked against the truth, and then spoiled the ways real ones are.
 */
class VisionPipelineTest {
  private static final FieldModel FIELD =
      new FieldModel(
          Fields.FRC_2026_REBUILT_WELDED.loadField(),
          Symmetry.ROTATED,
          List.of(),
          List.of(),
          List.of());

  /** In front of the blue hub's near face (tags 25 and 26), facing it. */
  private static final Pose2d ROBOT = new Pose2d(2.5, 4.1, Rotation2d.fromDegrees(10));

  private final VisionPipeline pipeline = new VisionPipeline(FIELD);
  private final CameraConfig camera = camera(0.3, 1.0);

  /** A forward camera at the front of the robot, tilted up 15 degrees. */
  private static CameraConfig camera(double up, double noiseScale) {
    return CameraConfig.builder()
        .name("front")
        .forward(Inches.of(12))
        .left(Inches.of(0))
        .up(Inches.of(up / 0.0254))
        .roll(Degrees.of(0))
        .tiltUp(Degrees.of(15))
        .yaw(Degrees.of(0))
        .sensor(
            CameraConfig.Sensor.builder()
                .widthPixels(1280)
                .heightPixels(800)
                .diagonalFieldOfView(Degrees.of(80))
                .frameRate(Hertz.of(30))
                .latency(Milliseconds.of(30))
                .build())
        .noiseScale(noiseScale)
        .build();
  }

  /** The camera's exact view of some tags, with the robot where it is. */
  private static TagDetections seeing(
      Pose2d robot, CameraConfig mount, boolean multiTag, int... ids) {
    Pose3d cameraPose = new Pose3d(robot).transformBy(mount.robotToCamera());
    List<TagDetections.Tag> tags = new ArrayList<>();
    for (int id : ids) {
      Transform3d view = new Transform3d(cameraPose, FIELD.tag(id).orElseThrow());
      tags.add(new TagDetections.Tag(id, corners(), view, flipped(view), 0.05));
    }
    TagDetections.MultiTag solved =
        multiTag
            ? new TagDetections.MultiTag(
                new Transform3d(new Pose3d(), cameraPose),
                0.3,
                tags.stream().map(TagDetections.Tag::id).toList())
            : null;
    return new TagDetections(1_000_000_000L, mount.name(), tags, solved);
  }

  /**
   * The other pose a single tag allows: turned to face another way, and a little off to the side,
   * so choosing it would put the robot in the wrong place.
   */
  private static Transform3d flipped(Transform3d view) {
    Rotation3d aside = new Rotation3d(0, 0, Math.toRadians(8));
    return new Transform3d(
        view.getTranslation().rotateBy(aside),
        view.getRotation().rotateBy(new Rotation3d(0, 0, Math.toRadians(35))));
  }

  private static List<Translation2d> corners() {
    return List.of(
        new Translation2d(600, 380),
        new Translation2d(680, 380),
        new Translation2d(680, 420),
        new Translation2d(600, 420));
  }

  private static CaptureContext steady(@Nullable Rotation2d heading) {
    return new CaptureContext(heading, Math.toRadians(1), 0, 0, 0, 0);
  }

  /** The result's measurement, which the test expects there to be. */
  private static VisionMeasurement measured(VisionResult result) {
    return Objects.requireNonNull(result.measurement(), result.outcome());
  }

  private VisionResult process(TagDetections seen, CaptureContext robot) {
    return pipeline.process(seen, camera, robot, VisionConfig.DEFAULTS);
  }

  @Test
  void withTheGyrosHeadingEachTagFixesThePosition() {
    VisionResult result =
        process(seeing(ROBOT, camera, false, 25, 26), steady(ROBOT.getRotation()));

    VisionMeasurement measured = measured(result);
    assertThat(measured.solver()).isEqualTo(VisionMeasurement.Solver.HEADING_CONSTRAINED);
    assertThat(measured.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-9);
    assertThat(measured.covariance().hasHeading()).as("the gyro keeps heading").isFalse();
    assertThat(result.heightErrorMeters()).isCloseTo(0.0, within(1e-9));
    assertThat(result.outcome()).startsWith("heading-constrained: 2 tags at ");
  }

  /**
   * A single tag's two poses: the one that agrees with the gyro wins, even when it's the second.
   */
  @Test
  void theGyroSettlesWhichOfATagsTwoPosesIsRight() {
    TagDetections seen = seeing(ROBOT, camera, false, 26);
    TagDetections.Tag tag = seen.tags().get(0);
    TagDetections swapped =
        new TagDetections(
            seen.timestampNanos(),
            seen.camera(),
            List.of(
                new TagDetections.Tag(
                    26, corners(), tag.altCameraToTag(), tag.bestCameraToTag(), 0.4)),
            null);

    VisionResult result = process(swapped, steady(ROBOT.getRotation()));

    assertThat(measured(result).pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-9);
  }

  /** 2026's cameras were measured from three different origins: a wrong height shows up here. */
  @Test
  void aCameraMountedOtherThanItsConfigurationSaysFailsTheHeightCheck() {
    TagDetections seen = seeing(ROBOT, camera(0.6, 1.0), false, 25, 26); // really 30 cm higher

    VisionResult result = process(seen, steady(ROBOT.getRotation()));

    assertThat(result.measurement()).isNull();
    assertThat(result.outcome()).startsWith("a tag's height is off by");
    assertThat(result.heightErrorMeters()).isCloseTo(0.3, within(0.05));
  }

  @Test
  void headingIsMeasuredOnlyFromSeveralTagsWithTheRobotSteady() {
    Pose2d truth = ROBOT;
    Rotation2d driftedGyro = truth.getRotation().plus(Rotation2d.fromDegrees(1.5));

    VisionResult steady = process(seeing(truth, camera, true, 25, 26), steady(driftedGyro));
    VisionResult turning =
        process(
            seeing(truth, camera, true, 25, 26),
            new CaptureContext(driftedGyro, Math.toRadians(1), 0, 0, 0, 2.0));

    assertThat(measured(steady).covariance().hasHeading()).isTrue();
    assertThat(measured(steady).pose().getRotation().minus(truth.getRotation()).getDegrees())
        .as("the tags' heading, not the drifted gyro's")
        .isCloseTo(0.0, within(1e-6));
    assertThat(measured(turning).covariance().hasHeading()).as("turning: gyro only").isFalse();
  }

  /** Before the robot is placed, its heading is a guess: full solves find it instead. */
  @Test
  void withoutATrustedHeadingItSolvesTheWholePose() {
    CaptureContext lost = new CaptureContext(null, Math.PI, 0, 0, 0, 0);

    VisionResult multi = process(seeing(ROBOT, camera, true, 25, 26), lost);
    VisionResult single = process(seeing(ROBOT, camera, false, 26), lost);

    assertThat(measured(multi).solver()).isEqualTo(VisionMeasurement.Solver.MULTI_TAG);
    assertThat(measured(multi).pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
    assertThat(measured(multi).covariance().hasHeading()).isTrue();
    assertThat(measured(single).solver()).isEqualTo(VisionMeasurement.Solver.SINGLE_TAG);
  }

  @Test
  void anAmbiguousSingleTagIsThrownOutWhenNothingSettlesIt() {
    TagDetections seen = seeing(ROBOT, camera, false, 26);
    TagDetections.Tag tag = seen.tags().get(0);
    TagDetections ambiguous =
        new TagDetections(
            seen.timestampNanos(),
            seen.camera(),
            List.of(
                new TagDetections.Tag(
                    26, corners(), tag.bestCameraToTag(), tag.altCameraToTag(), 0.35)),
            null);

    VisionResult result = process(ambiguous, new CaptureContext(null, Math.PI, 0, 0, 0, 0));

    assertThat(result.measurement()).isNull();
    assertThat(result.outcome()).isEqualTo("ambiguous single tag: 0.35 > 0.20");
  }

  @Test
  void imagesTakenWhileTurningFastOrFromFarAwayOrOfOtherTagsAreThrownOut() {
    TagDetections seen = seeing(ROBOT, camera, false, 25, 26);
    Pose2d farAway = new Pose2d(0.5, 4.1, Rotation2d.ZERO); // 3.5 m from the tags

    VisionResult spinning =
        process(seen, new CaptureContext(ROBOT.getRotation(), 0.01, 0, 0, 0, 6.0));
    VisionResult tooFar =
        pipeline.process(
            seeing(farAway, camera, false, 25, 26),
            camera,
            steady(farAway.getRotation()),
            new VisionConfig(0.2, 0.2, 3.0, 5.0, 0.02, 0.03, 2, 0.5, 0.25, 0.5, 0.1, 2, 0.5, 1));
    VisionResult unknownTag =
        process(
            new TagDetections(
                1,
                "front",
                List.of(
                    new TagDetections.Tag(99, corners(), Transform3d.ZERO, Transform3d.ZERO, 0.1)),
                null),
            steady(ROBOT.getRotation()));

    assertThat(spinning.outcome()).isEqualTo("turning too fast: 6.0 rad/s");
    assertThat(tooFar.outcome()).startsWith("too far: 3.");
    assertThat(unknownTag.outcome()).isEqualTo("no tags from this field's layout");
    assertThat(List.of(spinning, tooFar, unknownTag))
        .allSatisfy(result -> assertThat(result.measurement()).isNull());
  }

  @Test
  void aFullSolveThatPutsTheRobotOffTheFloorIsThrownOut() {
    Pose3d lifted = new Pose3d(ROBOT).transformBy(new Transform3d(0, 0, 0.4, Rotation3d.ZERO));
    Pose3d cameraPose = lifted.transformBy(camera.robotToCamera());
    TagDetections.Tag tag =
        new TagDetections.Tag(
            26,
            corners(),
            new Transform3d(cameraPose, FIELD.tag(26).orElseThrow()),
            Transform3d.ZERO,
            0.05);
    TagDetections seen = new TagDetections(1, "front", List.of(tag), null);

    VisionResult result = process(seen, new CaptureContext(null, Math.PI, 0, 0, 0, 0));

    assertThat(result.outcome()).isEqualTo("the solve puts the robot 0.40 m off the floor");
  }

  @Test
  void uncertaintyGrowsWithDistanceNoiseAndMotionAndShrinksWithTags() {
    double oneTag = spread(seeing(ROBOT, camera, false, 26), steady(ROBOT.getRotation()), camera);
    double twoTags =
        spread(seeing(ROBOT, camera, false, 25, 26), steady(ROBOT.getRotation()), camera);
    double noisyCamera =
        spread(seeing(ROBOT, camera, false, 26), steady(ROBOT.getRotation()), camera(0.3, 2.0));
    double moving =
        spread(
            seeing(ROBOT, camera, false, 26),
            new CaptureContext(ROBOT.getRotation(), 0.01, 0, 0, 4.0, 0),
            camera);
    double distance =
        seeing(ROBOT, camera, false, 26).tags().get(0).bestCameraToTag().getTranslation().getNorm();

    assertThat(oneTag).isCloseTo(0.02 * 0.5 * distance * distance, within(1e-9));
    assertThat(twoTags).isLessThan(oneTag * 0.6);
    assertThat(noisyCamera).isCloseTo(2 * oneTag, within(1e-9));
    assertThat(moving).as("4 m/s doubles it").isCloseTo(2 * oneTag, within(1e-9));
  }

  private double spread(TagDetections seen, CaptureContext robot, CameraConfig mount) {
    return measured(pipeline.process(seen, mount, robot, VisionConfig.DEFAULTS))
        .covariance()
        .xStdDev();
  }
}
