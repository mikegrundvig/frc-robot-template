package frc.robot.studies;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.runtime.subsystems.vision.VisionSim;
import frc.robot.Robots;
import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Transform3d;

/** How the cameras and robot err: the nominal conditions, and a robustness run's draws. */
class ConditionsTest {
  private static final CameraStudyConfig DEFAULTS = CameraStudyConfig.DEFAULTS;

  @Test
  void theNominalConditionsAreTheSimulatorsOwn() {
    Conditions nominal = Conditions.nominal(DEFAULTS);

    assertThat(nominal.cornerNoiseMeanPixels()).isEqualTo(VisionSim.CORNER_NOISE_MEAN_PIXELS);
    assertThat(nominal.cornerNoiseStdDevPixels()).isEqualTo(VisionSim.CORNER_NOISE_STD_DEV_PIXELS);
    assertThat(nominal.sightMeters())
        .isEqualTo(Robots.current().definition().cameras().get(0).sensor().sightRange().in(Meters));
    assertThat(nominal.mountError(42)).isEqualTo(Transform3d.ZERO);
    assertThat(nominal.described())
        .isEqualTo(
            "corners off 0.25 px (spread 0.08), tags seen to 7 m, the gyro off 0.25° typically,"
                + " cameras mounted exactly as configured");
  }

  @Test
  void eachTrialDrawsEveryErrorFromItsRange() {
    Set<Double> sights = new HashSet<>();
    for (int t = 0; t < 50; t++) {
      Conditions trial = Conditions.trial(DEFAULTS, t);

      assertThat(trial).isEqualTo(Conditions.trial(DEFAULTS, t));
      assertThat(trial.cornerNoiseMeanPixels()).isBetween(0.1, 0.6);
      assertThat(trial.cornerNoiseStdDevPixels() / trial.cornerNoiseMeanPixels())
          .as("the simulator's proportion")
          .isCloseTo(0.08 / 0.25, within(1e-12));
      assertThat(trial.sightMeters()).isBetween(5.0, 7.0);
      assertThat(trial.gyroErrorDegrees()).isBetween(0.1, 0.5);
      assertThat(trial.mountErrorMillimeters()).isBetween(0.0, 5.0);
      assertThat(trial.mountErrorDegrees()).isBetween(0.0, 1.0);
      sights.add(trial.sightMeters());
    }
    assertThat(sights).as("each trial its own").hasSize(50);
    CameraStudyConfig otherSeed = CameraStudyConfigBuilder.builder(DEFAULTS).seed(2).build();
    assertThat(Conditions.trial(otherSeed, 0)).isNotEqualTo(Conditions.trial(DEFAULTS, 0));
  }

  /** Another run's seed gives other trials, not the same ones shifted along. */
  @Test
  void runsWithDifferentSeedsShareNoTrial() {
    Set<Long> seeds = new HashSet<>();
    for (long seed = 1; seed <= 3; seed++) {
      CameraStudyConfig run = CameraStudyConfigBuilder.builder(DEFAULTS).seed(seed).build();
      for (int t = 0; t < CameraStudyConfig.ROBUST_TRIALS; t++) {
        assertThat(seeds.add(Conditions.trial(run, t).seed()))
            .as("seed %d trial %d", seed, t)
            .isTrue();
      }
    }
  }

  /** A camera's mount error is its own, the same at every pose, and within the bounds. */
  @Test
  void eachCameraIsOffItsMountItsOwnWay() {
    Conditions knocked = new Conditions(1, 0.25, 0.08, 7, 0.25, 5, 1);

    Transform3d one = knocked.mountError(1);
    Transform3d other = knocked.mountError(2);

    assertThat(knocked.mountError(1)).isEqualTo(one);
    assertThat(other).isNotEqualTo(one);
    for (Transform3d error : new Transform3d[] {one, other}) {
      assertThat(Math.abs(error.getX())).isLessThanOrEqualTo(0.005);
      assertThat(Math.abs(error.getY())).isLessThanOrEqualTo(0.005);
      assertThat(Math.abs(error.getZ())).isLessThanOrEqualTo(0.005);
      assertThat(Math.abs(error.getRotation().getX())).isLessThanOrEqualTo(Math.toRadians(1));
      assertThat(Math.abs(error.getRotation().getY())).isLessThanOrEqualTo(Math.toRadians(1));
      assertThat(Math.abs(error.getRotation().getZ())).isLessThanOrEqualTo(Math.toRadians(1));
    }
    assertThat(knocked.described()).endsWith("cameras mounted off by up to 5 mm and 1°");
  }

  @Test
  void conditionsCheckThemselves() {
    assertThatThrownBy(() -> new Conditions(1, -0.1, 0.08, 0, 0.25, 0, 0))
        .hasMessageContaining("Conditions.cornerNoiseMeanPixels: -0.1 must be zero or more")
        .hasMessageContaining("Conditions.sightMeters: 0 must be positive");
  }
}
