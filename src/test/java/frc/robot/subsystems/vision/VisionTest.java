package frc.robot.subsystems.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Milliseconds;

import frc.robot.lib.config.CameraConfig;
import frc.robot.lib.field.FieldModel;
import frc.robot.lib.field.Symmetry;
import frc.robot.lib.localization.BodyMotion;
import frc.robot.lib.localization.EstimatorConfig;
import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.PoseEstimator;
import frc.robot.lib.localization.PoseReset;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
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
    List<Frame> frames = new ArrayList<>();
    List<SeenTag> tags = new ArrayList<>();

    @Override
    public void updateInputs(CameraIOInputs inputs) {
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

  @BeforeEach
  void setUp() {
    TunableRegistry.registerBackend("", new MockTunableBackend());
    vision = new Vision(List.of(camera), new CameraIO[] {io}, estimator, FIELD);
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
    vision.periodic(estimator.estimate(), pitch, roll);
    vision.record(estimator.update());
  }

  @Test
  void anImageBecomesAMeasurementTheEstimatorAccepts() {
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.accepted(0)).isEqualTo(1);
    assertThat(vision.summary())
        .isEqualTo("1 of 1 cameras connected, 100% of 1 measurements accepted");
    assertThat(vision.meanTagResidualMeters("front")).isLessThan(1e-6);
  }

  @Test
  void theSameImageTwiceCountsOnce() {
    io.see(T0 + 200_000_000L, camera);
    loop();
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.accepted(0) + vision.rejected(0)).isEqualTo(1);
  }

  @Test
  void aCameraTurnedOffIsLeftOut() {
    vision.enabled(0).set(false);
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.accepted(0) + vision.rejected(0)).isZero();
    assertThat(vision.summary()).endsWith("no measurements yet");
  }

  @Test
  void aDisconnectedCameraRaisesItsAlert() {
    io.connected = false;

    loop();

    assertThat(vision.isDisconnectedAlertActive(0)).isTrue();
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

    assertThat(vision.accepted(0)).isEqualTo(1);
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

    assertThat(vision.accepted(0)).isEqualTo(1);
    assertThat(estimator.pose().getTranslation().getDistance(ROBOT.getTranslation()))
        .isLessThan(1e-6);
  }

  @Test
  void autonomousStartingCanHoldTheCamerasBack() {
    vision.autoStartHoldSeconds().set(2.0);
    vision.autonomousStarted(T0 + 100_000_000L);
    io.see(T0 + 200_000_000L, camera);

    loop();

    assertThat(vision.accepted(0) + vision.rejected(0)).as("held").isZero();
    vision.autoStartHoldSeconds().set(0.0);
    vision.autonomousStarted(T0 + 100_000_000L);
    io.see(T0 + 210_000_000L, camera);
    loop();
    assertThat(vision.accepted(0)).as("off: nothing held").isEqualTo(1);
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
}
