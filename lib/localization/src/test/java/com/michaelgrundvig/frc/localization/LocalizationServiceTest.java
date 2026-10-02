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

import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.Terrain;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.DrivetrainGeometry;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Imus;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Consumer;
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
    return placed(new SwerveKinematicsModel(DEFINITION.moduleTranslations()));
  }

  /** The same, with another drivetrain's wheels. */
  private static LocalizationService placed(DriveKinematicsModel kinematics) {
    return new LocalizationService(
        new PoseEstimator(
            EstimatorConfig.DEFAULTS,
            FIELD::contains,
            FIELD.tags().keySet(),
            new PoseReset(0, ROBOT, PoseCovariance.fromStdDevs(0.05, 0.05, 0.01), "placed")),
        kinematics,
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
      localization.update();
    }
    return now;
  }

  /** One loop with what the cameras send in between, as {@code during}. */
  private static LocalizationOutput loop(
      LocalizationService localization, long now, Runnable during) {
    LoopContext loop = new LoopContext(now, true);
    localization.drivetrain(still(now, 0, 0), loop);
    during.run();
    return localization.update();
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

  /**
   * Any drivetrain localizes the same way, through its own kinematics: a tank drive's two sides,
   * here, rolling a meter forward.
   */
  @Test
  void aTankDriveLocalizesThroughItsOwnKinematics() {
    LocalizationService localization = placed(new DifferentialKinematicsModel(0.6));
    long now = T0;
    for (int k = 0; k <= 10; k++) {
      now = T0 + k * LOOP_NANOS;
      double rolled = k * 0.1;
      LoopContext loop = new LoopContext(now, true);
      localization.drivetrain(
          DrivetrainReadings.builder()
              .wheels(
                  List.of(new OdometrySample(now, new double[] {rolled, rolled}, new double[2])))
              .gyro(still(now, 0, 0).gyro())
              .backupImu(ImuReadings.NONE)
              .velocities(new ChassisVelocities(5, 0, 0))
              .build(),
          loop);
      localization.update();
    }

    assertThat(localization.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isCloseTo(1.0, within(1e-9));
    assertThat(localization.pose().getRotation()).isEqualTo(ROBOT.getRotation());
  }

  /**
   * An image the robot holds back is solved and said, and never moves the estimate: the update says
   * it was held.
   */
  @Test
  void anImageSolvedButHeldBackIsNeverApplied() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);

    VisionResult[] result = new VisionResult[1];
    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () -> result[0] = localization.hold(hubFace(now), null, Verdict.HELD));

    assertThat(result[0].measurement()).isNotNull();
    assertThat(output.frames())
        .singleElement()
        .satisfies(
            held -> {
              assertThat(held.verdict()).isEqualTo(Verdict.HELD);
              assertThat(held.measured()).as("never weighed").isEmpty();
              assertThat(held.tags()).extracting(TagVerdict::verdict).containsOnly(Verdict.HELD);
            });
    assertThat(output.estimate().health().accepted()).isZero();
    assertThatThrownBy(() -> localization.hold(hubFace(now), null, Verdict.GATED))
        .as("the estimator's verdicts are its own")
        .isInstanceOf(IllegalArgumentException.class);
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
            asked.hold(hubFace(now), null, Verdict.HELD);
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
    assertThat(integrity.cameras())
        .containsEntry(
            "front", CameraTrust.builder().mountDoubtMeters(0.04).mountDistanceMeters(3.0).build());
    assertThat(integrity.cameras())
        .containsEntry("camera 8", CameraTrust.builder().build())
        .containsEntry("camera 9", CameraTrust.builder().keptApart(false).build());
    assertThat(integrity.cameras().keySet())
        .as("as the estimator first heard from each, then the robot's others")
        .endsWith("camera 9", "front");
    assertThat(integrity.protection()).isEmpty();
    assertThat(integrity.excludedTags()).isEmpty();
    assertThat(integrity.tagsWithoutPlaces()).as("every tag on the field's layout").isEmpty();
  }

  /**
   * A measurement from a tag the field's layout doesn't have (one solved elsewhere, say): the
   * estimator can't keep its placement apart, takes each image's as new, and says so.
   */
  @Test
  void integritySaysWhichTagsAreOffTheLayout() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10);
    int offTheLayout =
        1 + FIELD.tags().keySet().stream().mapToInt(Integer::intValue).max().orElse(0);

    LocalizationOutput output =
        loop(
            localization,
            now + LOOP_NANOS,
            () ->
                localization.camera(
                    new VisionMeasurement(
                        now,
                        ROBOT,
                        PoseCovariance.fromStdDevs(0.05, 0.05, Double.POSITIVE_INFINITY),
                        VisionMeasurement.Lever.NONE,
                        "front",
                        List.of(26, offTheLayout),
                        VisionMeasurement.Solver.EXTERNAL)));

    assertThat(output.frames().get(0).accepted()).isTrue();
    assertThat(output.integrity().tagsWithoutPlaces()).containsExactly(offTheLayout);
  }

  /** The wheels' trust: what the IMU says of them, and that a disabled robot may be carried. */
  @Test
  void integritySaysHowFarTheWheelsAreTrusted() {
    LocalizationService localization = placed();
    long now = standStill(localization, 10) + LOOP_NANOS;
    double pitch = Math.toRadians(-12);

    LoopContext disabled = new LoopContext(now, false);
    localization.drivetrain(still(now, pitch, 0), disabled);
    WheelTrust wheels = localization.update().integrity().wheels();

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
    localization.update();
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
   * The same inputs, in the same order, give the same outputs, to the last bit of every number:
   * what a replay of a logged match relies on. The session drives, turns, crosses a bump, tilts,
   * loses the gyro and then the backup IMU too, while the camera sends an image every other loop
   * and the drive team places the robot once; everything the service says along the way is held to
   * it, its outputs, each image's solve and capture, and every reading of its state.
   */
  @Test
  void theSameInputsGiveTheSameOutputs() {
    String first = session();

    assertThat(session()).isEqualTo(first);
  }

  /**
   * A camera whose mount is in doubt has its images widened by the doubt, as independent errors
   * add: what the robot says of a mount reaches every image after it.
   */
  @Test
  void aCamerasMountDoubtWidensItsImages() {
    LocalizationService doubted = placed();
    LocalizationService trusted = placed();
    long now = standStill(doubted, 10);
    standStill(trusted, 10);
    doubted.mountDoubt("front", 0.2, 3.0);

    VisionResult[] results = new VisionResult[2];
    LocalizationOutput output =
        loop(doubted, now + LOOP_NANOS, () -> results[0] = doubted.camera(hubFace(now)));
    loop(trusted, now + LOOP_NANOS, () -> results[1] = trusted.camera(hubFace(now)));

    PoseCovariance wide = Objects.requireNonNull(results[0].measurement()).covariance();
    PoseCovariance narrow = Objects.requireNonNull(results[1].measurement()).covariance();
    assertThat(wide.xx()).isCloseTo(narrow.xx() + 0.2 * 0.2, within(1e-12));
    assertThat(wide.yy()).isCloseTo(narrow.yy() + 0.2 * 0.2, within(1e-12));
    assertThat(output.integrity().cameras().get("front"))
        .isEqualTo(CameraTrust.builder().mountDoubtMeters(0.2).mountDistanceMeters(3.0).build());
  }

  /** The scripted robot's loops, each taken by the service: what each update said. */
  private static List<LocalizationOutput> drive(
      LocalizationService localization,
      ScriptedDrivetrain robot,
      int loops,
      ChassisVelocities velocity,
      boolean seeing) {
    List<LocalizationOutput> outputs = new ArrayList<>();
    for (int k = 0; k < loops; k++) {
      DrivetrainReadings readings = robot.loop(velocity);
      LoopContext loop = new LoopContext(robot.now(), true);
      localization.drivetrain(readings, loop);
      if (seeing) {
        localization.camera(hubFace(robot.truth3d(), robot.now()));
      }
      outputs.add(localization.update());
    }
    return outputs;
  }

  private static ScriptedDrivetrain scripted() {
    return new ScriptedDrivetrain(DEFINITION.moduleTranslations(), ROBOT, T0 - LOOP_NANOS, 0.3);
  }

  /**
   * The gyro turns odometry while it answers; lost, the backup IMU stands in, on its own zero,
   * without that taken as a turn; with both lost the wheels turn it, and the estimate says the gyro
   * is lost; back, the gyro takes over again.
   */
  @Test
  void theGyroThenTheBackupThenTheWheelsTurnOdometry() {
    LocalizationService localization = placed();
    ScriptedDrivetrain robot = scripted();
    ChassisVelocities turning = new ChassisVelocities(0.5, 0, 0.5);
    drive(localization, robot, 20, turning, false);
    assertThat(localization.yawSource()).isEqualTo(YawSource.GYRO);

    robot.gyroLost = true;
    LocalizationOutput backup = drive(localization, robot, 20, turning, false).getLast();

    assertThat(localization.yawSource()).isEqualTo(YawSource.BACKUP);
    assertThat(backup.estimate().mode()).isNotEqualTo(LocalizationEstimate.Mode.IMU_FAULT);
    assertThat(headingError(localization, robot)).as("degrees").isLessThan(0.5);

    robot.backupLost = true;
    LocalizationOutput wheels = drive(localization, robot, 10, turning, false).getLast();

    assertThat(localization.yawSource()).isEqualTo(YawSource.WHEELS);
    assertThat(wheels.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.IMU_FAULT);
    assertThat(localization.pitchRadians()).as("no IMU, no tilt").isZero();

    robot.gyroLost = false;
    robot.backupLost = false;
    drive(localization, robot, 5, turning, false);

    assertThat(localization.yawSource()).isEqualTo(YawSource.GYRO);
    assertThat(headingError(localization, robot)).as("degrees").isLessThan(1);
  }

  private static double headingError(LocalizationService localization, ScriptedDrivetrain robot) {
    return Math.abs(
        localization.pose().getRotation().minus(robot.truth().getRotation()).getDegrees());
  }

  /**
   * Wheels that turn the robot while the gyro says it doesn't are sliding: the slip check says so,
   * and the estimate grows less sure of where the robot is than a twin's whose wheels grip.
   */
  @Test
  void wheelsSlidingWhileDrivingAreTrustedLess() {
    LocalizationService slipping = placed();
    LocalizationService gripping = placed();
    ScriptedDrivetrain sliding = scripted();
    ScriptedDrivetrain rolling = scripted();
    sliding.gyroFrozen = true;
    ChassisVelocities turning = new ChassisVelocities(1.0, 0, 2.0);

    LocalizationOutput slid = drive(slipping, sliding, 50, turning, false).getLast();
    LocalizationOutput rolled = drive(gripping, rolling, 50, turning, false).getLast();

    assertThat(slipping.wheels().turnSlipMetersPerSecond()).isPositive();
    assertThat(slid.integrity().wheels().conditions().slipMetersPerSecond()).isPositive();
    assertThat(gripping.wheels().turnSlipMetersPerSecond()).isZero();
    assertThat(slid.estimate().covariance().largestPositionStdDev())
        .isGreaterThan(2 * rolled.estimate().covariance().largestPositionStdDev());
  }

  /**
   * Over a bump, the IMU's pitch says the robot climbed, passed the crest and came down: the crest
   * is a landmark, weighed at the update, across the bump's crest line.
   */
  @Test
  void aCrestCrossedBecomesALandmark() {
    LocalizationService localization = beforeTheBump();
    ScriptedDrivetrain robot = scripted();
    robot.bump = BUMP;

    List<LocalizationOutput> outputs =
        drive(localization, robot, 70, new ChassisVelocities(1.5, 0, 0), false);

    List<LandmarkDecision> landmarks =
        outputs.stream().flatMap(output -> output.landmarks().stream()).toList();
    assertThat(landmarks).hasSize(1);
    LandmarkDecision crest = landmarks.get(0);
    assertThat(crest.verdict()).isEqualTo(Verdict.ACCEPTED);
    assertThat(crest.landmark().name()).isEqualTo("Test bump");
    assertThat(crest.innovationMeters())
        .as("odometry on the ramps, short of the truth")
        .isLessThan(0.05);
    assertThat(localization.terrainCrossings()).isEqualTo(1);
    assertThat(localization.landmarksAccepted()).isEqualTo(1);
    assertThat(BUMP.acrossOffset(robot.truth().getTranslation()))
        .as("past the bump")
        .isGreaterThan(0.5);
  }

  /**
   * A camera that goes silent leaves the estimate on odometry: tracking while it sees, dead
   * reckoning a second after its last image, and not good enough to score from.
   */
  @Test
  void aCameraGoingSilentLeavesTheEstimateDeadReckoning() {
    LocalizationService localization = placed();
    ScriptedDrivetrain robot = scripted();

    LocalizationOutput seeing =
        drive(localization, robot, 25, new ChassisVelocities(), true).getLast();
    assertThat(seeing.estimate().mode()).isEqualTo(LocalizationEstimate.Mode.TRACKING);
    assertThat(seeing.frames()).hasSize(1);

    List<LocalizationOutput> silent =
        drive(localization, robot, 60, new ChassisVelocities(), false);

    assertThat(silent).allSatisfy(output -> assertThat(output.frames()).isEmpty());
    LocalizationEstimate estimate = silent.getLast().estimate();
    assertThat(estimate.mode()).isEqualTo(LocalizationEstimate.Mode.DEAD_RECKONING);
    assertThat(estimate.health().visionAgeSeconds()).isGreaterThan(1);
    assertThat(LocalizationGate.SCORING.whyNot(estimate)).isEqualTo("no camera measurement lately");
  }

  /** What the session compares by: every bit, where WPILib's geometry allows 1e-9. */
  @Test
  void aFingerprintTellsApartWhatWpilibCallsEqual() {
    Pose2d pose = new Pose2d(1, 2, Rotation2d.ZERO);
    Pose2d nearly = new Pose2d(1 + 1e-12, 2, Rotation2d.ZERO);

    assertThat(nearly).isEqualTo(pose);
    assertThat(Fingerprint.of(nearly)).isNotEqualTo(Fingerprint.of(pose));
    assertThatThrownBy(() -> Fingerprint.of(new Object()))
        .isInstanceOf(IllegalArgumentException.class);
  }

  /** A bump across the robot's way, its crest a meter ahead. */
  static final Bump BUMP =
      new Bump(
          "Test bump",
          new Translation2d(ROBOT.getX() + 1.0, 6.0),
          new Translation2d(ROBOT.getX() + 1.0, 2.0),
          0.165,
          1.0);

  /** The robot placed in front of the hub, with the test bump ahead of it. */
  private static LocalizationService beforeTheBump() {
    return new LocalizationService(
        new PoseEstimator(
            EstimatorConfig.DEFAULTS,
            FIELD::contains,
            FIELD.tags().keySet(),
            new PoseReset(0, ROBOT, PoseCovariance.fromStdDevs(0.05, 0.05, 0.01), "placed")),
        new SwerveKinematicsModel(DEFINITION.moduleTranslations()),
        FIELD,
        new Terrain(List.of(BUMP)),
        DEFINITION,
        0.02);
  }

  /** The front camera's exact view of the hub's near face, from where the robot really is. */
  private static TagDetections hubFace(Pose3d robot, long takenNanos) {
    Pose3d cameraPose = robot.transformBy(FRONT.robotToCamera());
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

  /** One leg of the session: how long, how fast, and what changes as it starts. */
  private record Leg(int loops, ChassisVelocities velocity, Consumer<ScriptedDrivetrain> setUp) {}

  /** The session, every number it said fingerprinted, loop by loop. */
  private static String session() {
    LocalizationService localization = beforeTheBump();
    ScriptedDrivetrain robot =
        new ScriptedDrivetrain(DEFINITION.moduleTranslations(), ROBOT, T0 - LOOP_NANOS, 0.3);
    robot.bump = BUMP;
    List<Leg> legs =
        List.of(
            new Leg(15, new ChassisVelocities(), drivetrain -> {}),
            new Leg(60, new ChassisVelocities(1.5, 0, 0), drivetrain -> {}), // over the bump
            new Leg(25, new ChassisVelocities(1.0, 0, 2.5), drivetrain -> {}), // turning
            new Leg(
                20,
                new ChassisVelocities(1.0, 0.3, 0),
                drivetrain -> drivetrain.pitch = Math.toRadians(-7)), // tilted
            new Leg(
                20,
                new ChassisVelocities(0.5, 0, 0.5),
                drivetrain -> {
                  drivetrain.pitch = 0;
                  drivetrain.gyroLost = true; // the backup stands in
                }),
            new Leg(
                15,
                new ChassisVelocities(0.5, 0, 0),
                drivetrain -> drivetrain.backupLost = true), // the wheels alone
            new Leg(
                20,
                new ChassisVelocities(),
                drivetrain -> {
                  drivetrain.gyroLost = false;
                  drivetrain.backupLost = false;
                }));
    StringBuilder said = new StringBuilder();
    Set<YawSource> sources = EnumSet.noneOf(YawSource.class);
    int loops = 0;
    for (Leg leg : legs) {
      leg.setUp().accept(robot);
      for (int k = 0; k < leg.loops(); k++, loops++) {
        DrivetrainReadings readings = robot.loop(leg.velocity());
        LoopContext loop = new LoopContext(robot.now(), loops < 160);
        localization.drivetrain(readings, loop);
        if (loops == 140) {
          localization.place(
              new PoseReset(
                  robot.now(),
                  robot.truth(),
                  PoseCovariance.fromStdDevs(0.05, 0.05, 0.02),
                  "placed again"));
        }
        if (loops % 2 == 0) {
          long taken = robot.now();
          said.append(Fingerprint.of(localization.capture(taken)));
          said.append(Fingerprint.of(localization.camera(hubFace(robot.truth3d(), taken))));
        }
        said.append(Fingerprint.of(localization.update()));
        said.append(
            Fingerprint.of(
                List.of(
                    localization.pose(),
                    localization.pose3d(localization.pose()),
                    localization.pitchRadians(),
                    localization.rollRadians(),
                    localization.yawSource(),
                    localization.conditions(),
                    localization.wheels(),
                    localization.terrainPhase(),
                    localization.terrainCrossings(),
                    localization.climbRadians(),
                    localization.crest(),
                    localization.imuDisagreementRadians(),
                    localization.imusDisagree(),
                    localization.landmarksAccepted())));
        said.append('\n');
        sources.add(localization.yawSource());
      }
    }
    assertThat(sources).as("every IMU, and none").containsExactlyInAnyOrder(YawSource.values());
    assertThat(localization.terrainCrossings()).as("over the bump").isEqualTo(1);
    assertThat(localization.landmarksAccepted()).as("its crest").isEqualTo(1);
    return said.toString();
  }
}
