package com.michaelgrundvig.frc.localization;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.tuple;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * The localization service on its own: readings, images and placements in, the estimate and its
 * verdicts out. No simulator and no clock: each loop's time is the test's, and the robot stands in
 * front of the blue hub's near face (tags 25 and 26), its camera seeing it exactly.
 */
class LocalizationServiceTest {
  private static final FieldModel FIELD =
      new FieldModel(
          Fields.FRC_2026_REBUILT_WELDED.loadField(),
          Symmetry.ROTATED,
          List.of(),
          List.of(),
          List.of());

  /** In front of the blue hub's near face, facing it. */
  private static final Pose2d ROBOT = new Pose2d(2.5, 4.1, Rotation2d.fromDegrees(10));

  private static final long T0 = 1_000_000_000L;
  private static final long LOOP_NANOS = 20_000_000L;

  /** A forward camera at the front of the robot, tilted up 15 degrees. */
  private static final CameraConfig FRONT =
      CameraConfig.builder()
          .name("front")
          .forward(Inches.of(12))
          .left(Inches.of(0))
          .up(Inches.of(12))
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
                  .sightRange(Meters.of(7))
                  .build())
          .noiseScale(1.0)
          .build();

  /** A 28 x 24 in swerve robot with the front camera, its IMUs not yet measured. */
  private static final RobotDefinition DEFINITION =
      RobotDefinition.builder()
          .name("test")
          .body(
              Body.builder()
                  .frameLength(Inches.of(28))
                  .frameWidth(Inches.of(24))
                  .height(Inches.of(30))
                  .bumperThickness(Inches.of(3.5))
                  .mass(Kilograms.of(60))
                  .yawMomentOfInertia(KilogramSquareMeters.of(6))
                  .cgHeight(Meters.of(0.2))
                  .build())
          .drivetrain(
              DrivetrainGeometry.builder()
                  .modules(
                      DrivetrainGeometry.Modules.builder()
                          .driveMotor(Motor.KRAKEN_X60)
                          .steerMotor(Motor.KRAKEN_X44)
                          .wheelRadius(Inches.of(2))
                          .driveGearing(Gearing.builder().ratio(6.75).efficiency(1.0).build())
                          .steerGearing(Gearing.builder().ratio(12.8).efficiency(1.0).build())
                          .wheelCoefficientOfFriction(1.2)
                          .driveInertia(KilogramSquareMeters.of(0.001))
                          .steerInertia(KilogramSquareMeters.of(0.004))
                          .build())
                  .wheelInset(Inches.of(2.625))
                  .build())
          .cameras(List.of(FRONT))
          .imus(Imus.UNCHECKED)
          .build();

  /** Localization with the robot placed in front of the hub, to 5 cm and half a degree. */
  private static LocalizationService placed() {
    return new LocalizationService(
        new PoseEstimator(
            EstimatorConfig.DEFAULTS,
            FIELD::contains,
            new PoseReset(0, ROBOT, PoseCovariance.fromStdDevs(0.05, 0.05, 0.01), "placed")),
        FIELD,
        FIELD.terrain(),
        DEFINITION,
        0.02);
  }

  /** The drivetrain standing still at a moment, its gyro tilted as given. */
  private static DrivetrainReadings still(long nanos, double pitch, double roll) {
    return DrivetrainReadings.builder()
        .wheels(List.of(new OdometrySample(nanos, new double[4], new double[4])))
        .gyro(
            ImuReadings.builder()
                .reporting(true)
                .yawRadians(0.3)
                .yawRateRadiansPerSecond(0)
                .pitchRadians(pitch)
                .rollRadians(roll)
                // Gravity, as an accelerometer tilted so feels it standing still.
                .forwardAccelerationMetersPerSecSq(-ImuReadings.STANDARD_GRAVITY * Math.sin(pitch))
                .leftAccelerationMetersPerSecSq(
                    ImuReadings.STANDARD_GRAVITY * Math.sin(roll) * Math.cos(pitch))
                .yawTimesNanos(List.of(nanos))
                .yawsRadians(List.of(0.3))
                .build())
        .backupImu(ImuReadings.NONE)
        .velocities(new ChassisVelocities())
        .build();
  }

  /** Loops of the robot standing level, from {@link #T0}: the time of the last one. */
  private static long standStill(LocalizationService localization, int loops) {
    long now = T0;
    for (int k = 0; k < loops; k++) {
      now = T0 + k * LOOP_NANOS;
      LoopContext loop = new LoopContext(now, true);
      localization.drivetrain(still(now, 0, 0), loop);
      localization.update(loop);
    }
    return now;
  }

  /** One loop with what the cameras send in between, as {@code during}. */
  private static LocalizationOutput loop(
      LocalizationService localization, long now, Runnable during) {
    LoopContext loop = new LoopContext(now, true);
    localization.drivetrain(still(now, 0, 0), loop);
    during.run();
    return localization.update(loop);
  }

  /** The front camera's exact view of the hub's near face, from the robot, taken then. */
  private static TagDetections hubFace(long takenNanos) {
    Pose3d cameraPose = new Pose3d(ROBOT).transformBy(FRONT.robotToCamera());
    List<TagDetections.Tag> tags = new ArrayList<>();
    for (int id : new int[] {25, 26}) {
      Transform3d view = new Transform3d(cameraPose, FIELD.tag(id).orElseThrow());
      tags.add(
          new TagDetections.Tag(
              id,
              List.of(
                  new Translation2d(600, 380),
                  new Translation2d(680, 380),
                  new Translation2d(680, 420),
                  new Translation2d(600, 420)),
              view,
              Transform3d.ZERO,
              0.05));
    }
    return new TagDetections(takenNanos, "front", tags, null);
  }

  /** A measurement from the front camera of tags 25 and 26, put at a pose and claiming 3 cm. */
  private static VisionMeasurement measured(long takenNanos, Pose2d pose) {
    return new VisionMeasurement(
        takenNanos,
        pose,
        PoseCovariance.fromStdDevs(0.03, 0.03, Double.POSITIVE_INFINITY),
        VisionMeasurement.Lever.NONE,
        "front",
        List.of(25, 26),
        VisionMeasurement.Solver.HEADING_CONSTRAINED);
  }

  @Test
  void anImageBecomesAVerdictOnEachOfItsTags() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);

    VisionResult[] result = new VisionResult[1];
    LocalizationOutput output =
        loop(localization, now + LOOP_NANOS, () -> result[0] = localization.camera(hubFace(now)));

    assertThat(result[0].refused()).isNull();
    assertThat(result[0].outcome()).startsWith("heading-constrained: 2 tags at ");
    assertThat(output.frames()).hasSize(1);
    FrameVerdict frame = output.frames().get(0);
    assertThat(frame.camera()).isEqualTo("front");
    assertThat(frame.captureNanos()).isEqualTo(now);
    assertThat(frame.multiTag()).as("solved from each tag, not the multi-tag").isEmpty();
    FrameVerdict.Measured measured = frame.measured().orElseThrow();
    assertThat(frame.verdict()).isEqualTo(Verdict.ACCEPTED);
    assertThat(measured.solver()).isEqualTo(VisionMeasurement.Solver.HEADING_CONSTRAINED);
    assertThat(measured.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
    assertThat(frame.tags()).extracting(TagVerdict::tag).containsExactly(25, 26);
    assertThat(frame.tags())
        .allSatisfy(
            tag -> {
              assertThat(tag.verdict()).isEqualTo(Verdict.ACCEPTED);
              assertThat(tag.innovationMeters()).isLessThan(1e-6);
              assertThat(tag.chiSquared()).isLessThanOrEqualTo(tag.threshold());
              assertThat(tag.threshold())
                  .isEqualTo(EstimatorConfig.DEFAULTS.gateChiSquaredPosition());
            });
    assertThat(output.estimate().health().accepted()).isEqualTo(1);
  }

  /** An image held back is solved and said, and never moves the estimate. */
  @Test
  void anImageSolvedButHeldBackIsNeverApplied() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);

    VisionResult[] result = new VisionResult[1];
    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () -> result[0] = localization.solve(hubFace(now), null));

    assertThat(result[0].measurement()).isNotNull();
    assertThat(output.frames()).isEmpty();
    assertThat(output.estimate().health().accepted()).isZero();
  }

  /**
   * Asking how the robot stood, or solving an image held back, before the drive team places the
   * robot that loop changes nothing: the loop's images are solved as the robot stands once placed,
   * here with a heading too unsure to solve with, and the loop is recorded once.
   */
  @Test
  void askingBeforeAPlacementChangesNothing() {
    LocalizationService asked = placed();
    LocalizationService notAsked = placed();
    long now = standStill(asked, 10) + LOOP_NANOS;
    standStill(notAsked, 10);
    PoseReset roughly =
        new PoseReset(now, ROBOT, PoseCovariance.fromStdDevs(0.05, 0.05, 0.5), "facing roughly");

    VisionResult[] results = new VisionResult[2];
    loop(
        asked,
        now,
        () -> {
          for (int k = 0; k < 5; k++) {
            asked.capture(now);
            asked.solve(hubFace(now), null);
          }
          asked.place(roughly);
          results[0] = asked.camera(hubFace(now));
        });
    loop(
        notAsked,
        now,
        () -> {
          notAsked.place(roughly);
          results[1] = notAsked.camera(hubFace(now));
        });

    assertThat(results[0]).isEqualTo(results[1]);
    assertThat(Objects.requireNonNull(results[1].measurement()).solver())
        .as("solved without the placement's unsure heading")
        .isEqualTo(VisionMeasurement.Solver.SINGLE_TAG);
    long later = now + LOOP_NANOS;
    loop(asked, later, () -> {});
    loop(notAsked, later, () -> {});
    assertThat(asked.capture(now)).isEqualTo(notAsked.capture(now));
  }

  /**
   * An image the pipeline refuses says why at once, typed, and never reaches the estimator; the
   * update says so too, among every image's verdicts, with its numbers and every tag it saw.
   */
  @Test
  void anImageThePipelineRefusesSaysWhyAtOnce() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);
    localization.vision(
        VisionConfigBuilder.builder(VisionConfig.DEFAULTS).maxDistanceMeters(0.5).build());

    VisionResult[] result = new VisionResult[1];
    LocalizationOutput output =
        loop(localization, now + LOOP_NANOS, () -> result[0] = localization.camera(hubFace(now)));

    assertThat(result[0].refused()).isEqualTo(Verdict.TOO_FAR);
    assertThat(result[0].outcome())
        .isEqualTo(Verdict.TOO_FAR.describe(result[0].distanceMeters(), 0.5))
        .startsWith("too far: ");
    FrameVerdict refused = output.frames().get(0);
    assertThat(output.frames()).hasSize(1);
    assertThat(refused.verdict()).isEqualTo(Verdict.TOO_FAR);
    assertThat(refused.measured()).as("never weighed").isEmpty();
    assertThat(refused.reason()).isEqualTo(result[0].outcome());
    assertThat(refused.captureNanos()).isEqualTo(now);
    assertThat(refused.tags())
        .extracting(TagVerdict::tag, TagVerdict::verdict, TagVerdict::reason)
        .containsExactly(
            tuple(25, Verdict.TOO_FAR, result[0].outcome()),
            tuple(26, Verdict.TOO_FAR, result[0].outcome()));
    assertThat(output.estimate().health().rejected()).as("the estimator never saw it").isZero();
  }

  /**
   * A measurement 1.5 m off and claiming 3 cm fails the gate: the frame and each of its tags say
   * so, with the χ² past the threshold.
   */
  @Test
  void aMeasurementFarFromTheEstimateIsGated() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);

    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () ->
                localization.camera(
                    measured(now, ROBOT.plus(new Transform2d(1.5, 0, Rotation2d.ZERO)))));

    FrameVerdict gated = output.frames().get(0);
    FrameVerdict.Measured measured = gated.measured().orElseThrow();
    assertThat(gated.verdict()).isEqualTo(Verdict.GATED);
    assertThat(measured.chiSquared()).isGreaterThan(gated.threshold()).isEqualTo(gated.value());
    assertThat(measured.innovationMeters()).isCloseTo(1.5, within(0.01));
    assertThat(gated.reason()).startsWith("gate: ").endsWith("> 13.8");
    assertThat(output.frames().get(0).tags())
        .allSatisfy(
            tag -> {
              assertThat(tag.accepted()).isFalse();
              assertThat(tag.reason()).as("each tag, as its frame").isEqualTo(gated.reason());
            });
    assertThat(output.estimate().pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
  }

  /** A measurement taken before the robot was placed says so, without numbers. */
  @Test
  void aMeasurementFromBeforeThePlacementIsRefused() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);

    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () -> {
              localization.place(
                  new PoseReset(
                      now + LOOP_NANOS,
                      ROBOT,
                      PoseCovariance.fromStdDevs(0.05, 0.05, 0.01),
                      "placed again"));
              localization.camera(measured(now, ROBOT));
            });

    FrameVerdict refused = output.frames().get(0);
    assertThat(refused.verdict()).isEqualTo(Verdict.BEFORE_RESET);
    assertThat(refused.measured().orElseThrow().chiSquared()).isNaN();
    assertThat(refused.reason()).isEqualTo("before the robot was last placed or recovered");
  }

  /**
   * Each camera's trust: its mount's doubt as the robot last said it, and whether the estimator
   * keeps its repeated errors apart, which it does for eight cameras and no more.
   */
  @Test
  void integritySaysHowFarEachCameraIsTrusted() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);
    localization.mountDoubt("front", 0.04, 3.0);

    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () -> {
              for (int camera = 1; camera <= 9; camera++) {
                localization.camera(
                    new VisionMeasurement(
                        now,
                        ROBOT,
                        PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY),
                        VisionMeasurement.Lever.NONE,
                        "camera " + camera,
                        List.of(26),
                        VisionMeasurement.Solver.HEADING_CONSTRAINED));
              }
            });

    Integrity integrity = output.integrity();
    assertThat(integrity.cameras()).containsEntry("front", new CameraTrust(0.04, 3.0, true));
    assertThat(integrity.cameras())
        .containsEntry("camera 8", new CameraTrust(0, Double.NaN, true))
        .containsEntry("camera 9", new CameraTrust(0, Double.NaN, false));
    assertThat(integrity.cameras().keySet())
        .as("as the estimator first heard from each, then the robot's others")
        .endsWith("camera 9", "front");
    assertThat(integrity.protection()).isEmpty();
    assertThat(integrity.excludedTags()).isEmpty();
  }

  /** The wheels' trust: what the IMU says of them, and that a disabled robot may be carried. */
  @Test
  void integritySaysHowFarTheWheelsAreTrusted() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10) + LOOP_NANOS;
    double pitch = Math.toRadians(-12);

    LoopContext disabled = new LoopContext(now, false);
    localization.drivetrain(still(now, pitch, 0), disabled);
    WheelTrust wheels = localization.update(disabled).integrity().wheels();

    assertThat(wheels.conditions().disabled()).isTrue();
    assertThat(wheels.conditions().tiltRadians()).isCloseTo(Math.abs(pitch), within(1e-9));
    assertThat(wheels.conditions().groundSlopeRadians()).as("no bump explains it").isZero();
    assertThat(wheels.accelerometerLeaksGravity()).isFalse();
    assertThat(localization.wheels()).isEqualTo(wheels);
  }

  /** An image is solved with the robot tilted as it was when the image was taken. */
  @Test
  void anImageIsSolvedAsTheRobotStoodWhenItWasTaken() {
    LocalizationService localization = placed();
    long tilted = standStill(localization, 10) + LOOP_NANOS;
    double pitch = Math.toRadians(-8);
    LoopContext loop = new LoopContext(tilted, true);
    localization.drivetrain(still(tilted, pitch, 0), loop);
    localization.update(loop);
    long level = tilted + LOOP_NANOS;
    loop = new LoopContext(level, true);
    localization.drivetrain(still(level, 0, 0), loop);

    CaptureContext then = localization.capture(tilted);

    assertThat(then.pitchRadians()).isEqualTo(pitch);
    assertThat(then.heading()).isEqualTo(ROBOT.getRotation());
    assertThat(localization.capture(level).pitchRadians()).isZero();
    assertThat(localization.capture(T0 - 10 * LOOP_NANOS).heading())
        .as("before the history")
        .isNull();
  }

  /** A landmark the robot crossed is weighed at the update, and counted. */
  @Test
  void aLandmarkIsWeighedAndCounted() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);

    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () ->
                localization.landmark(
                    new LandmarkEvent(
                        now, "crest", ROBOT.getTranslation(), Rotation2d.ZERO, 0.05)));

    assertThat(output.landmarks())
        .extracting(LandmarkDecision::verdict)
        .containsExactly(Verdict.ACCEPTED);
    assertThat(localization.landmarksAccepted()).isEqualTo(1);
  }

  @Test
  void anImageFromACameraTheRobotDoesntHaveIsAnError() {
    LocalizationService localization = placed();

    assertThatThrownBy(() -> localization.camera(new TagDetections(T0, "back", List.of(), null)))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining("back");
  }

  /** A loop happens at a time on the robot's clock; a protection level is a distance. */
  @Test
  void itsRecordsRefuseWhatTheyCantBe() {
    assertThatThrownBy(() -> new LoopContext(-1, true))
        .isInstanceOf(IllegalArgumentException.class);
    assertThat(new ProtectionLevel(0.07).meters()).isEqualTo(0.07);
    assertThatThrownBy(() -> new ProtectionLevel(Double.NaN))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(
            () ->
                new VisionResult(
                    hubFace(T0),
                    null,
                    null,
                    "neither",
                    Double.NaN,
                    Double.NaN,
                    Double.NaN,
                    Double.NaN))
        .isInstanceOf(IllegalArgumentException.class);
  }

  /**
   * The same inputs, in the same order, give the same outputs, record for record: what a replay of
   * a logged match relies on.
   */
  @Test
  void theSameInputsGiveTheSameOutputs() {
    assertThat(session()).isEqualTo(session());
  }

  /** Standing, an image, a misread, a placement, a tilt: every output along the way. */
  private static List<LocalizationOutput> session() {
    LocalizationService localization = placed();
    List<LocalizationOutput> outputs = new ArrayList<>();
    long now = T0;
    for (int k = 0; k < 30; k++) {
      long then = now;
      int step = k;
      LoopContext loop = new LoopContext(now, k < 25);
      localization.drivetrain(still(now, k >= 20 ? Math.toRadians(-6) : 0, 0), loop);
      if (step == 10) {
        localization.place(
            new PoseReset(now, ROBOT, PoseCovariance.fromStdDevs(0.05, 0.05, 0.01), "again"));
      }
      if (step % 3 == 0) {
        localization.camera(hubFace(then - LOOP_NANOS));
      }
      if (step == 14) {
        localization.camera(measured(then, ROBOT.plus(new Transform2d(1.5, 0, Rotation2d.ZERO))));
      }
      outputs.add(localization.update(loop));
      now += LOOP_NANOS;
    }
    assertThat(outputs.stream().mapToInt(output -> output.frames().size()).sum())
        .as("the session weighed images")
        .isGreaterThan(5);
    return outputs;
  }
}
