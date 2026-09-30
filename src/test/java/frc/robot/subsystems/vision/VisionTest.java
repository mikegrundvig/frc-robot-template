package frc.robot.subsystems.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Milliseconds;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.config.CameraConfigBuilder;
import frc.robot.lib.field.Bump;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.field.Symmetry;
import frc.robot.lib.localization.BodyMotion;
import frc.robot.lib.localization.EstimatorConfig;
import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.PoseEstimator;
import frc.robot.lib.localization.PoseReset;
import frc.robot.lib.localization.TagAllowlist;
import frc.robot.lib.localization.VisionConfig;
import frc.robot.lib.localization.VisionConfigBuilder;
import frc.robot.lib.localization.VisionDecision;
import java.util.ArrayList;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/** The cameras' side of localization, with a scripted camera in place of PhotonVision. */
class VisionTest {
  private static final FieldModel FIELD =
      new FieldModel(
          Fields.FRC_2026_REBUILT_WELDED.loadField(),
          Symmetry.ROTATED,
          List.of(),
          List.of(),
          List.of());
  private static final Pose2d ROBOT = new Pose2d(2.5, 4.1, Rotation2d.fromDegrees(10));
  private static final long T0 = 1_000_000_000L;

  private final CameraConfig camera =
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
                  .build())
          .noiseScale(1.0)
          .build();

  /** A camera that reports whatever the test scripts. */
  private static final class ScriptedCamera implements CameraIO {
    boolean connected = true;
    @Nullable RuntimeException failure;
    List<Frame> frames = new ArrayList<>();
    List<SeenTag> tags = new ArrayList<>();

    @Override
    public void updateInputs(CameraIOInputs inputs) {
      if (failure != null) {
        throw failure;
      }
      inputs.connected = connected;
      inputs.frames = frames.toArray(Frame[]::new);
      inputs.tags = tags.toArray(SeenTag[]::new);
      frames = new ArrayList<>();
      tags = new ArrayList<>();
    }

    /** The exact view of the blue hub's near face from the robot. */
    void see(long timestampNanos, CameraConfig mount) {
      see(timestampNanos, mount, new Pose3d(ROBOT));
    }

    /** The exact view from the robot, tilted as it is. */
    void see(long timestampNanos, CameraConfig mount, Pose3d robot) {
      Pose3d cameraPose = robot.transformBy(mount.robotToCamera());
      frames.add(new Frame(timestampNanos, 2, false, Transform3d.ZERO, 0));
      for (int id : new int[] {25, 26}) {
        Transform3d view = new Transform3d(cameraPose, FIELD.tag(id).orElseThrow());
        tags.add(new SeenTag(id, 0, 0, 1, 0, 1, 1, 0, 1, view, Transform3d.ZERO, 0.05, false));
      }
    }
  }

  private final ScriptedCamera io = new ScriptedCamera();
  private final PoseEstimator estimator =
      new PoseEstimator(
          EstimatorConfig.DEFAULTS,
          FIELD::contains,
          new PoseReset(0, ROBOT, PoseCovariance.fromStdDevs(0.05, 0.05, 0.01), "placed"));
  private Vision vision;

  /** The vision settings as Pitwall would hand them over, tuned or not. */
  private VisionConfig tuned = VisionConfig.DEFAULTS;

  @BeforeEach
  void setUp() {
    TunableRegistry.registerBackend("", new MockTunableBackend());
    vision = new Vision(List.of(camera), new CameraIO[] {io}, estimator, FIELD, () -> tuned);
    for (int k = 0; k <= 10; k++) {
      estimator.addOdometry(T0 + k * 20_000_000L, new BodyMotion(0, 0, 0, 0), 0.3, true);
    }
    estimator.addOdometry(T0 + 220_000_000L, new BodyMotion(0, 0, 0, 0), 0.3, true);
  }

  @AfterEach
  void tearDown() {
    vision.close();
    TunableRegistry.reset();
  }

  private void loop() {
    loop(0, 0);
  }

  private void loop(double pitch, double roll) {
    vision.periodic(estimator.estimate().timestampNanos(), estimator.estimate(), pitch, roll);
    vision.record(estimator.update());
  }

  @Test
  void anImageBecomesAMeasurementTheEstimatorAccepts() {
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted()).isEqualTo(1);
    assertThat(vision.summary())
        .isEqualTo("1 of 1 cameras connected, 100% of 1 measurements accepted");
    assertThat(vision.meanTagResidualMeters("front")).isLessThan(1e-6);
  }

  /** A tuned setting applies from the next loop's images. */
  @Test
  void theSettingsAreReadEveryLoop() {
    tuned = VisionConfigBuilder.builder(VisionConfig.DEFAULTS).maxDistanceMeters(0.1).build();
    io.see(T0 + 200_000_000L, camera);
    loop();
    tuned = VisionConfig.DEFAULTS;
    io.see(T0 + 220_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted()).as("only the image after").isEqualTo(1);
  }

  @Test
  void theSameImageTwiceCountsOnce() {
    io.see(T0 + 200_000_000L, camera);
    loop();
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted() + vision.camera(0).rejected()).isEqualTo(1);
  }

  @Test
  void aCameraTurnedOffIsLeftOut() {
    vision.camera(0).enabled.set(false);
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted() + vision.camera(0).rejected()).isZero();
    assertThat(vision.summary()).endsWith("no measurements yet");
  }

  @Test
  void aDisconnectedCameraRaisesItsAlert() {
    io.connected = false;

    loop();

    assertThat(vision.camera(0).isDisconnectedAlertActive()).isTrue();
    assertThat(vision.summary()).startsWith("0 of 1 cameras connected");
  }

  @Test
  void aTiltedRobotsImagesAreSolvedTilted() {
    double pitch = Math.toRadians(-8); // nose up on a ramp
    double roll = Math.toRadians(3);
    io.see(
        T0 + 200_000_000L,
        camera,
        new Pose3d(
            ROBOT.getX(),
            ROBOT.getY(),
            0,
            new Rotation3d(roll, pitch, ROBOT.getRotation().getRadians())));

    loop(pitch, roll);

    assertThat(vision.camera(0).accepted()).isEqualTo(1);
    assertThat(estimator.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
    assertThat(vision.meanTagResidualMeters("front")).isLessThan(1e-6);
  }

  /** On a bump the attitude changes before an image arrives: it's solved as the robot was then. */
  @Test
  void anImageIsSolvedWithTheAttitudeWhenItWasTaken() {
    double pitch = Math.toRadians(-8);
    double roll = Math.toRadians(3);
    loop(pitch, roll); // tilted as of T0 + 220 ms, when the image is taken
    estimator.addOdometry(T0 + 240_000_000L, new BodyMotion(0, 0, 0, 0), 0.3, true);
    io.see(
        T0 + 220_000_000L,
        camera,
        new Pose3d(
            ROBOT.getX(),
            ROBOT.getY(),
            0,
            new Rotation3d(roll, pitch, ROBOT.getRotation().getRadians())));

    loop(0, 0); // level again by the time it arrives

    assertThat(vision.camera(0).accepted()).isEqualTo(1);
    assertThat(estimator.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
  }

  @Test
  void autonomousStartingCanHoldTheCamerasBack() {
    vision.autoStartHoldSeconds().set(2.0);
    vision.autonomousStarted(T0 + 100_000_000L);
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted() + vision.camera(0).rejected()).as("held").isZero();
    vision.autoStartHoldSeconds().set(0.0);
    vision.autonomousStarted(T0 + 100_000_000L);
    io.see(T0 + 210_000_000L, camera);
    loop();
    assertThat(vision.camera(0).accepted()).as("off: nothing held").isEqualTo(1);
  }

  /**
   * PhotonLib throws, every few seconds, when the coprocessor runs a different PhotonVision than
   * the robot's library: the camera reads as unreadable, and the robot program runs on.
   */
  @Test
  void aCameraThatThrowsIsUnreadableAndTheLoopGoesOn() {
    io.failure = new UnsupportedOperationException("Photonlib version mismatch\n");

    loop();

    assertThat(vision.camera(0).isUnreadableAlertActive()).isTrue();
    assertThat(vision.camera(0).isDisconnectedAlertActive())
        .as("unreadable says why instead")
        .isFalse();
    assertThat(vision.summary()).startsWith("0 of 1 cameras connected");

    io.failure = null;
    io.see(T0 + 200_000_000L, camera);
    loop();
    assertThat(vision.camera(0).isUnreadableAlertActive()).isFalse();
    assertThat(vision.camera(0).accepted()).isEqualTo(1);
  }

  /** Whatever a camera sends that isn't a number costs that image, never the robot program. */
  @Test
  void badNumbersFromACameraCostOnlyTheirImage() {
    Transform3d notANumber = new Transform3d(Double.NaN, 0, 0, Rotation3d.ZERO);
    io.frames.add(new CameraIO.Frame(T0 + 120_000_000L, 1, true, notANumber, Double.NaN));
    io.tags.add(
        new CameraIO.SeenTag(26, 0, 0, 1, 0, 1, 1, 0, 1, notANumber, notANumber, Double.NaN, true));
    io.frames.add(new CameraIO.Frame(-1, 0, false, Transform3d.ZERO, 0)); // from an old log
    io.see(T0 + 200_000_000L, camera);
    io.tags.set(io.tags.size() - 1, withAmbiguity(io.tags.getLast(), Double.NaN));

    loop();

    assertThat(vision.camera(0).accepted()).as("only the good image").isEqualTo(1);
    assertThat(estimator.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
  }

  @Test
  void anAutoStartHoldThatIsntATimeHoldsNothing() {
    vision.autoStartHoldSeconds().set(Double.POSITIVE_INFINITY);
    vision.autonomousStarted(T0 + 100_000_000L);
    vision.autoStartHoldSeconds().set(Double.NaN);
    vision.autonomousStarted(T0 + 100_000_000L);
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.camera(0).accepted()).isEqualTo(1);
  }

  private static CameraIO.SeenTag withAmbiguity(CameraIO.SeenTag tag, double ambiguity) {
    return new CameraIO.SeenTag(
        tag.id(),
        tag.corner0X(),
        tag.corner0Y(),
        tag.corner1X(),
        tag.corner1Y(),
        tag.corner2X(),
        tag.corner2Y(),
        tag.corner3X(),
        tag.corner3Y(),
        tag.bestCameraToTag(),
        tag.altCameraToTag(),
        ambiguity,
        tag.inMultiTag());
  }

  /**
   * On a bump the robot stands as high as the ground under it, and its tags are placed from there:
   * from the floor, every one would land as far off as the bump is tall.
   */
  @Test
  void onABumpTagsArePlacedFromAsHighAsTheGround() {
    double crest = 0.165;
    FieldModel bumpy =
        new FieldModel(
            Fields.FRC_2026_REBUILT_WELDED.loadField(),
            Symmetry.ROTATED,
            List.of(),
            List.of(),
            List.of(),
            List.of(
                new Bump(
                    "Test bump",
                    new Translation2d(ROBOT.getX(), 3.0),
                    new Translation2d(ROBOT.getX(), 5.0),
                    crest,
                    1.0)));
    vision.close();
    vision = new Vision(List.of(camera), new CameraIO[] {io}, estimator, bumpy, () -> tuned);
    io.see(
        T0 + 200_000_000L,
        camera,
        new Pose3d(
            ROBOT.getX(),
            ROBOT.getY(),
            crest,
            new Rotation3d(0, 0, ROBOT.getRotation().getRadians())));

    loop();

    assertThat(vision.meanTagResidualMeters("front")).isLessThan(1e-6);
  }

  @Test
  void aCameraWhoseTagsLandOffShowsItInItsResidual() {
    CameraConfig reallyHigher =
        CameraConfig.builder()
            .name("front")
            .forward(Inches.of(12))
            .left(Inches.of(0))
            .up(Inches.of(16))
            .roll(Degrees.of(0))
            .tiltUp(Degrees.of(15))
            .yaw(Degrees.of(0))
            .sensor(camera.sensor())
            .noiseScale(1.0)
            .build();
    io.see(T0 + 200_000_000L, reallyHigher);

    loop();

    assertThat(vision.meanTagResidualMeters("front")).isBetween(0.09, 0.11); // 4 in
  }

  /**
   * An image stamped far earlier than its camera's images take to arrive, or after the robot read
   * it, has the wrong capture time: the camera's clock and the robot's are out of step. It's left
   * out before the estimator sees it, and an alert says so until an image arrives in time again.
   */
  @Test
  void anImageOutsideItsCamerasLatencyBandIsLeftOut() {
    long now = estimator.estimate().timestampNanos();
    io.see(now - 300_000_000L, camera); // the camera takes 30 ms; 100 ms more is the margin

    loop();

    assertThat(vision.camera(0).accepted() + vision.camera(0).rejected()).isZero();
    assertThat(vision.camera(0).isOutOfStepAlertActive()).isTrue();
    assertThat(vision.camera(0).outOfStepAlertText())
        .isEqualTo(
            "Camera front's images arrive 300 ms after they're taken, outside 0 to 130 ms, and are"
                + " left out: are its clock and the robot's in step (time sync)? If it's just"
                + " slower, measure its latency");
    io.see(now + 5_000_000L, camera);
    loop();
    assertThat(vision.camera(0).outOfStepAlertText()).contains("arrive 5 ms before they're taken");
    assertThat(vision.camera(0).accepted() + vision.camera(0).rejected()).isZero();

    estimator.addOdometry(now + 40_000_000L, new BodyMotion(0, 0, 0, 0), 0.3, true);
    io.see(now + 20_000_000L, camera);
    loop();
    assertThat(vision.camera(0).accepted()).as("in time again").isEqualTo(1);
    assertThat(vision.camera(0).isOutOfStepAlertActive()).isFalse();
  }

  /** One loop, and the tags each of its measurements came from. */
  private List<List<Integer>> tagsMeasuredFrom() {
    vision.periodic(estimator.estimate().timestampNanos(), estimator.estimate(), 0, 0);
    List<VisionDecision> decisions = estimator.update();
    vision.record(decisions);
    return decisions.stream().map(decision -> decision.measurement().tagIds()).toList();
  }

  /**
   * A task lining up on the hub trusts only the hub's tags while it runs; when it ends, or is
   * canceled partway, the cameras trust every tag again.
   */
  @Test
  void aTaskTrustsOnlyItsTagsWhileItRuns() {
    Scheduler scheduler = Scheduler.createIndependentScheduler();
    Command approach =
        Command.noRequirements(
                coroutine -> {
                  coroutine.yield();
                  coroutine.yield();
                })
            .named("Approach the hub");
    Command task = vision.trustingOnly(TagAllowlist.of("Hub", List.of(26)), approach);
    scheduler.schedule(task);
    scheduler.run();
    io.see(T0 + 200_000_000L, camera);

    assertThat(tagsMeasuredFrom()).containsExactly(List.of(26));
    for (int k = 0; k < 5; k++) {
      scheduler.run();
    }
    assertThat(scheduler.isScheduledOrRunning(task)).as("done").isFalse();
    io.see(T0 + 210_000_000L, camera);
    assertThat(tagsMeasuredFrom()).containsExactly(List.of(25, 26));

    Command canceled = vision.trustingOnly(TagAllowlist.of("Hub", List.of(26)), approach);
    scheduler.schedule(canceled);
    scheduler.run();
    scheduler.cancel(canceled);
    io.see(T0 + 215_000_000L, camera);
    assertThat(tagsMeasuredFrom()).containsExactly(List.of(25, 26));
  }

  @Test
  void anImageWithNoTagTheTaskTrustsIsLeftOut() {
    vision.trustOnly(TagAllowlist.of("Loading", List.of(13)));
    io.see(T0 + 200_000_000L, camera);

    assertThat(tagsMeasuredFrom()).isEmpty();
    vision.trustEveryTag();
    io.see(T0 + 210_000_000L, camera);
    assertThat(tagsMeasuredFrom()).containsExactly(List.of(25, 26));
  }

  /** Images from the front camera, one a loop, with odometry moving on to each one's time. */
  private void imagesFrom(CameraConfig reallyMounted, int images) {
    for (int k = 0; k < images; k++) {
      long now = estimator.estimate().timestampNanos() + 20_000_000L;
      estimator.addOdometry(now, new BodyMotion(0, 0, 0, 0), 0.3, true);
      io.see(now, reallyMounted);
      loop();
    }
  }

  /**
   * A camera knocked 4 in down since it was calibrated: once it has seen enough tags, every one of
   * them 4 in high, it's out of calibration, and the alert says where to recalibrate it. Knocked
   * straight down, it still places the robot right on the floor, so it counts as much as before.
   */
  @Test
  void aCameraWhoseTagsLandOffIsOutOfCalibrationOnceJudged() {
    CameraConfig reallyLower = CameraConfigBuilder.builder(camera).up(Inches.of(8)).build();

    imagesFrom(reallyLower, 200); // two tags an image: 400 seen, not enough to judge

    assertThat(vision.mountOffsetMeters("front")).isNaN();
    assertThat(vision.camera(0).isOutOfCalibrationAlertActive()).isFalse();
    imagesFrom(reallyLower, 60);
    assertThat(vision.mountOffsetMeters("front")).isCloseTo(0.1016, within(0.002));
    assertThat(vision.camera(0).isOutOfCalibrationAlertActive()).isTrue();
    assertThat(vision.camera(0).outOfCalibrationAlertText())
        .isEqualTo(
            "Camera front is out of calibration: its tags land 10 cm off. Recalibrate its mount"
                + " (Calibrate the front camera's mount)");
    assertThat(vision.mountDoubtMeters("front")).as("still places the robot right").isZero();
    assertThat(estimator.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
  }

  /** The tolerance is the pit's to tune, and applies from the next loop. */
  @Test
  void theMountToleranceIsReadEveryLoop() {
    CameraConfig reallyLower = CameraConfigBuilder.builder(camera).up(Inches.of(8)).build();
    imagesFrom(reallyLower, 260);

    tuned = VisionConfigBuilder.builder(VisionConfig.DEFAULTS).mountToleranceMeters(0.15).build();
    loop();

    assertThat(vision.camera(0).isOutOfCalibrationAlertActive()).isFalse();
  }
}
