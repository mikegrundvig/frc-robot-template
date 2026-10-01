package frc.robot.views.world;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.push.Feed;
import com.michaelgrundvig.frc.pitwall.scene.Inspection;
import com.michaelgrundvig.frc.pitwall.scene.Scenes;
import frc.robot.RobotConfig;
import frc.robot.RobotSim;
import frc.robot.Robots;
import frc.robot.lib.localization.LocalizationEstimate;
import frc.robot.lib.localization.PoseCovariance;
import frc.robot.lib.localization.VisionDecision;
import frc.robot.lib.localization.VisionMeasurement;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * The template's robot on its field as Pitwall's World view gets it: what's described, what moves,
 * and what a click on a sight line says.
 */
class WorldViewTest {
  private static final JsonType<Object> JSON =
      Jsonb.builder().serializeNulls(true).build().type(Object.class);
  private static final RobotConfig ROBOT = Robots.SIM;

  private final AtomicLong clock = new AtomicLong(1_000_000_000L);
  private final List<String> said = new ArrayList<>();
  private final Scenes scenes = new Scenes(clock::get, said::add);

  private LocalizationEstimate estimate =
      estimate(new Pose2d(3, 4, Rotation2d.fromDegrees(30)), LocalizationEstimate.Mode.TRACKING);
  private SwerveModuleVelocity[] modules = still();
  private List<VisionDecision> decisions = List.of();

  private final WorldView.Readings readings =
      new WorldView.Readings() {
        @Override
        public LocalizationEstimate estimate() {
          return estimate;
        }

        @Override
        public double pitchRadians() {
          return 0;
        }

        @Override
        public double rollRadians() {
          return 0;
        }

        @Override
        public SwerveModuleVelocity[] modules() {
          return modules;
        }

        @Override
        public List<VisionDecision> decisions() {
          return decisions;
        }
      };

  private static LocalizationEstimate estimate(Pose2d pose, LocalizationEstimate.Mode mode) {
    return new LocalizationEstimate(
        0,
        pose,
        new PoseCovariance(0.0004, 0.0001, 0.0009, 0.0001),
        Translation2d.ZERO,
        0,
        mode,
        new LocalizationEstimate.Health(0.1, 12, 3, 0.02, 0.03, 0, 0, 0, false, 0, true));
  }

  private static SwerveModuleVelocity[] still() {
    SwerveModuleVelocity[] modules = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      modules[i] = new SwerveModuleVelocity(0, Rotation2d.ZERO);
    }
    return modules;
  }

  private Feed world(boolean simulated) {
    scenes.add(
        WorldView.scene(
            RobotSim.FIELD,
            ROBOT.body(),
            ROBOT.drivetrain(),
            ROBOT.cameras(),
            7,
            readings,
            simulated ? () -> new Pose3d(new Pose2d(3.05, 4, Rotation2d.fromDegrees(31))) : null,
            camera -> Optional.of("Calibrate " + camera)));
    return scenes.feeds().getFirst();
  }

  private String loop(Feed feed) {
    clock.addAndGet(60_000_000L);
    return JSON.toJson(feed.patch());
  }

  @Test
  void theFieldTheRobotAndItsCamerasAreDescribedFromTheModelAndTheConfiguration() {
    world(true);
    String described = JSON.toJson(scenes.descriptions());

    assertThat(described)
        .contains("\"focus\":\"robot\"")
        .contains("\"walls\"", "\"bump.0\"", "\"tag.7\":{\"parent\":\"field\",\"pose\":")
        .contains("\"id\":\"tag.7.id\"", "\"text\":\"7\"")
        .contains("\"camera.front-left\":{\"parent\":\"robot\",\"pose\":")
        .contains("\"camera.front-left.view\"", "\"range\":[0,7]")
        .contains("\"wheel.back-right\"", "\"length\":\"speed.back-right\"")
        .contains("\"radii\":[\"uncertainty.major\",\"uncertainty.minor\"]")
        .contains("\"truth.robot\"", "\"style\":\"truth dashed\"")
        .contains("\"sight-lines\":{\"title\":\"Sight lines (dashed: rejected)\"");
    assertThat(RobotSim.FIELD.tags()).hasSizeGreaterThan(20);
    for (int tag : RobotSim.FIELD.tags().keySet()) {
      assertThat(described).contains("\"tag." + tag + ".square\"");
    }
    assertThat(said).isEmpty();
  }

  @Test
  void theRobotIsDrawnWhereTheEstimateIsWithItsWheelsTurnedAndDriving() {
    Feed feed = world(false);
    modules = still();
    modules[0] = new SwerveModuleVelocity(2.0, Rotation2d.fromDegrees(45));
    String patch = loop(feed);

    assertThat(patch)
        .contains("\"robot\":[3,4,30]")
        .contains("\"module.front-left\":[")
        .contains(",45]")
        .contains("\"speed.front-left\":0.4")
        .contains("\"uncertainty\":[3,4,");
    assertThat(loop(feed)).as("a still robot sends nothing").isEqualTo("{}");
    modules[0] = new SwerveModuleVelocity(2.0, Rotation2d.fromDegrees(45.6));
    assertThat(loop(feed)).as("a wheel's jitter at rest is no news").doesNotContain("module.");
    modules[0] = new SwerveModuleVelocity(2.0, Rotation2d.fromDegrees(46.2));
    assertThat(loop(feed)).as("a degree's turn is").contains("\"module.front-left\":[");
    assertThat(said).isEmpty();
  }

  @Test
  void aSightLineSaysWhyItsMeasurementWasRefusedAndOffersTheCamerasCalibration() {
    Feed feed = world(false);
    VisionMeasurement measurement =
        new VisionMeasurement(
            1,
            new Pose2d(3.2, 4.1, Rotation2d.fromDegrees(30)),
            PoseCovariance.fromStdDevs(0.1, 0.1, 0.1),
            VisionMeasurement.Lever.NONE,
            "front-left",
            List.of(7),
            VisionMeasurement.Solver.SINGLE_TAG);
    decisions = List.of(new VisionDecision(measurement, false, "gate: 45.2 > 13.8", 0.8, 45.2));
    String patch = loop(feed);

    assertThat(patch)
        .contains(
            "\"sight-lines\":[{\"id\":\"ray.front-left.7.bad\",\"kind\":\"line\","
                + "\"frame\":\"field\",\"style\":\"bad dashed\",\"inspect\":true,"
                + "\"between\":[\"camera.front-left\",\"tag.7\"]}]");
    Inspection said = scenes.inspect(WorldView.ID, "ray.front-left.7.bad").orElseThrow();
    assertThat(said.title()).isEqualTo("front-left → tag 7");
    assertThat(said.details())
        .extracting(Inspection.Detail::value)
        .contains("rejected: gate: 45.2 > 13.8", "0.80 m from the estimate", "45.2");
    assertThat(said.actions()).containsExactly("Calibrate front-left");

    for (int i = 0; i < 10; i++) {
      loop(feed); // the line goes after a moment without a measurement
    }
    assertThat(JSON.toJson(feed.sent())).doesNotContain("ray.front-left.7");
    assertThat(scenes.inspect(WorldView.ID, "ray.front-left.7.bad"))
        .as("still answered for a while after")
        .isPresent();
  }

  @Test
  void aClickOnACameraSaysHowItsMountedAndOnTheEllipseHowSureTheEstimateIs() {
    Feed feed = world(false);
    loop(feed);

    Inspection camera = scenes.inspect(WorldView.ID, "camera.front-left.view").orElseThrow();
    assertThat(camera.title()).isEqualTo("Camera front-left");
    assertThat(camera.details()).extracting(Inspection.Detail::label).contains("Mounted", "Sees");
    assertThat(scenes.inspect(WorldView.ID, "uncertainty.ellipse").orElseThrow().details())
        .extracting(Inspection.Detail::value)
        .contains("tracking");
    assertThat(scenes.inspect(WorldView.ID, "tag.7.square")).isPresent();
    assertThat(scenes.inspect(WorldView.ID, "bump.0")).isPresent();

    estimate = estimate(estimate.pose(), LocalizationEstimate.Mode.LOST);
    assertThat(loop(feed)).contains("\"styles\":{\"uncertainty.ellipse\":\"bad dashed\"}");
  }

  @Test
  void aCamerasFieldOfViewComesFromItsDiagonal() {
    double[] fov = RobotLayer.fieldOfView(ROBOT.cameras().getFirst().sensor());
    // 80° across the diagonal of a 1280 x 800 image.
    assertThat(Math.toDegrees(fov[0])).isBetween(70.0, 71.0);
    assertThat(Math.toDegrees(fov[1])).isBetween(47.0, 48.5);
  }
}
