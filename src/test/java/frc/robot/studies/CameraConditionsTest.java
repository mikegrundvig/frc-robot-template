package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.ExampleRobot;
import com.michaelgrundvig.frc.simulation.SensorErrors;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Transform3d;

/**
 * The estimator study's camera conditions: the default leaves a scenario's world exactly as it is,
 * and the pessimistic ones change the cameras as the camera placement study's ranges say.
 */
class CameraConditionsTest {
  private static final long SEED = 3;

  private static final List<CameraConfig> CAMERAS = ExampleRobot.SIM.definition().cameras();

  /** A scenario with no mount error of its own, then one whose seeds draw some. */
  static Stream<Arguments> rows() {
    SensorErrors.CornerNoise usual = SensorErrors.CornerNoise.TYPICAL;
    SensorErrors.CornerNoise soft =
        SensorErrors.CornerNoise.builder().meanPixels(0.6).stdDevPixels(0.192).build();
    return Stream.of(
        Arguments.of(CameraConditions.NOMINAL, "ideal", usual, 7.0, 0.0, 0.0),
        Arguments.of(CameraConditions.NOMINAL, "clean-match-mounts", usual, 7.0, 0.0, 0.0),
        Arguments.of(CameraConditions.PESSIMISTIC, "ideal", soft, 5.0, 5.0, 1.0),
        Arguments.of(CameraConditions.PESSIMISTIC, "clean-match-mounts", soft, 5.0, 5.0, 1.0));
  }

  @ParameterizedTest(name = "{0} on {1}")
  @MethodSource("rows")
  void theCamerasErrAsTheConditionsSay(
      CameraConditions conditions,
      String scenario,
      SensorErrors.CornerNoise corners,
      double sightMeters,
      double millimeters,
      double degrees) {
    SensorErrors base = EstimatorScenarios.named(scenario).errors().apply(SEED);
    SensorErrors errors = conditions.on(base, SEED);

    assertThat(errors.cornerNoise()).isEqualTo(corners);
    assertThat(conditions.robot().definition().cameras())
        .allSatisfy(
            camera -> assertThat(camera.sensor().sightRange().in(Meters)).isEqualTo(sightMeters));
    assertThat(errors).isEqualTo(conditions.on(base, SEED));
    if (millimeters == 0) {
      assertThat(errors).as("a world left as it was").isEqualTo(base);
      assertThat(conditions.robot()).isEqualTo(ExampleRobot.SIM);
      return;
    }
    assertThat(errors.seed()).as("the images' own seed").isEqualTo(base.seed());
    for (int i = 0; i < CAMERAS.size(); i++) {
      String camera = CAMERAS.get(i).name();
      Transform3d before = base.cameraMountError(camera);
      Transform3d after = errors.cameraMountError(camera);
      // On top of the scenario's own error: the translations add, and the turns follow one another.
      Transform3d drawn =
          new Transform3d(
              after.getTranslation().minus(before.getTranslation()),
              before.getRotation().inverse().rotateBy(after.getRotation()));
      assertThat(Math.abs(drawn.getX())).isBetween(0.0, millimeters / 1000);
      assertThat(Math.abs(drawn.getY())).isBetween(0.0, millimeters / 1000);
      assertThat(Math.abs(drawn.getZ())).isBetween(0.0, millimeters / 1000);
      assertThat(Math.toDegrees(drawn.getRotation().getAngle()))
          .isBetween(0.0, degrees * Math.sqrt(3));
      assertThat(drawn).as("camera %d is off its mount", i).isNotEqualTo(Transform3d.ZERO);
      if (i > 0) {
        assertThat(after)
            .as("each camera its own")
            .isNotEqualTo(errors.cameraMountError(CAMERAS.get(i - 1).name()));
      }
    }
    // Another seed draws other errors, not the same ones shifted to the next camera.
    SensorErrors next = conditions.on(base, SEED + 1);
    assertThat(next.cameraMountError(CAMERAS.get(0).name()))
        .isNotEqualTo(errors.cameraMountError(CAMERAS.get(1).name()))
        .isNotEqualTo(errors.cameraMountError(CAMERAS.get(0).name()));
  }

  @Test
  void conditionsAreAskedForByName() {
    assertThat(CameraConditions.named("nominal")).isSameAs(CameraConditions.NOMINAL);
    assertThat(CameraConditions.named("pessimistic")).isSameAs(CameraConditions.PESSIMISTIC);
    assertThatThrownBy(() -> CameraConditions.named("kind"))
        .hasMessage("--cameras must be nominal or pessimistic: kind");
  }
}
