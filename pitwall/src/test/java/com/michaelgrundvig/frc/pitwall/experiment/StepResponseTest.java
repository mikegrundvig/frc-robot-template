package com.michaelgrundvig.frc.pitwall.experiment;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Seconds;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleUnaryOperator;
import org.junit.jupiter.api.Test;

/**
 * Scores checked against responses whose answers are known exactly: a first-order lag (rise τ ln 9,
 * settling τ ln 50, no overshoot) and an underdamped second-order system (overshoot
 * e^(-πζ/√(1-ζ²))).
 */
class StepResponseTest {
  private static final long EDGE = 1_000_000_000L; // the jump, 1 s into the robot's clock
  private static final double TAU = 0.1;
  private static final Profile.Edge UP = new Profile.Edge(0, 0, 1);

  /**
   * Samples {@code response(t)}, t in seconds after the jump, for {@code seconds} at {@code hz}.
   */
  private static Series sampled(DoubleUnaryOperator response, double seconds, double hz) {
    var series = new Series((int) (seconds * hz) + 2);
    for (int i = 0; i <= seconds * hz; i++) {
      double t = i / hz;
      series.add(EDGE + Math.round(t * 1e9), response.applyAsDouble(t));
    }
    return series;
  }

  private static long after(double seconds) {
    return EDGE + Math.round(seconds * 1e9);
  }

  private static StepResponse score(Series measured, Profile.Edge edge, double windowSeconds) {
    return StepResponse.of(measured, edge, EDGE, after(windowSeconds)).orElseThrow();
  }

  @Test
  void aFirstOrderLagRisesAndSettlesAsTheTextbookSays() {
    Series measured = sampled(t -> 1 - Math.exp(-t / TAU), 2, 1000);

    StepResponse response = score(measured, UP, 2);

    assertThat(response.riseSeconds().getAsDouble()).isCloseTo(TAU * Math.log(9), within(1e-4));
    assertThat(response.settlingSeconds().getAsDouble())
        .isCloseTo(TAU * Math.log(50), within(1e-4));
    assertThat(response.overshoot()).isZero();
    assertThat(response.steadyStateError()).isCloseTo(0, within(1e-6));
    assertThat(response.samples()).isEqualTo(2001);
  }

  @Test
  void interpolatingKeepsA50HzMeasurementClose() {
    Series measured = sampled(t -> 1 - Math.exp(-t / TAU), 2, 50);

    StepResponse response = score(measured, UP, 2);

    assertThat(response.riseSeconds().getAsDouble())
        .as("a sample every 20 ms, timed to about 2 ms")
        .isCloseTo(TAU * Math.log(9), within(0.002));
    assertThat(response.settlingSeconds().getAsDouble())
        .isCloseTo(TAU * Math.log(50), within(0.002));
  }

  @Test
  void anUnderdampedSystemOvershootsByTheTextbookAmount() {
    double zeta = 0.5;
    double omega = 20;
    double damped = omega * Math.sqrt(1 - zeta * zeta);
    Series measured =
        sampled(
            t ->
                1
                    - Math.exp(-zeta * omega * t)
                        * (Math.cos(damped * t)
                            + zeta / Math.sqrt(1 - zeta * zeta) * Math.sin(damped * t)),
            2,
            1000);

    StepResponse response = score(measured, UP, 2);

    assertThat(response.overshoot())
        .isCloseTo(Math.exp(-Math.PI * zeta / Math.sqrt(1 - zeta * zeta)), within(1e-4));
    assertThat(response.settlingSeconds().getAsDouble())
        .as("about 4/(ζω), the textbook estimate")
        .isBetween(0.3, 0.45);
  }

  @Test
  void aStepDownScoresLikeAStepUp() {
    Series down = sampled(t -> 90 * Math.exp(-t / TAU), 2, 1000);

    StepResponse response = score(down, new Profile.Edge(0, 90, 0), 2);

    assertThat(response.riseSeconds().getAsDouble()).isCloseTo(TAU * Math.log(9), within(1e-4));
    assertThat(response.settlingSeconds().getAsDouble())
        .isCloseTo(TAU * Math.log(50), within(1e-4));
    assertThat(response.overshoot()).isZero();
  }

  @Test
  void stoppingShortIsAPositiveErrorWithNoRiseOrSettling() {
    Series measured = sampled(t -> 0.8 * (1 - Math.exp(-t / TAU)), 2, 1000);

    StepResponse response = score(measured, UP, 2);

    assertThat(response.riseSeconds()).as("never 90% of the way").isEmpty();
    assertThat(response.settlingSeconds()).as("never within 2%").isEmpty();
    assertThat(response.steadyStateError()).isCloseTo(0.2, within(1e-6));
  }

  @Test
  void stayingPastTheTargetIsANegativeError() {
    Series measured = sampled(t -> 1.01 * (1 - Math.exp(-t / TAU)), 2, 1000);

    StepResponse response = score(measured, UP, 2);

    assertThat(response.steadyStateError()).isCloseTo(-0.01, within(1e-6));
    assertThat(response.overshoot()).isCloseTo(0.01, within(1e-6));
    assertThat(response.settlingSeconds()).as("within 2% of the target, for good").isPresent();
  }

  @Test
  void settlingIsTimedWhereTheLastExitFromTheBandEnds() {
    var measured = new Series(10);
    measured.add(after(0.0), 0.0);
    measured.add(after(0.1), 1.10); // 10% over
    measured.add(after(0.2), 1.00);
    measured.add(after(0.3), 0.90); // 10% under: the last time outside the band
    measured.add(after(0.4), 1.00);
    measured.add(after(1.0), 1.00);

    StepResponse response = score(measured, UP, 1);

    // From 0.90 at 0.3 s to 1.00 at 0.4 s, the band's edge (0.98) is crossed 80% of the way.
    assertThat(response.settlingSeconds().getAsDouble()).isCloseTo(0.38, within(1e-9));
    assertThat(response.overshoot()).isCloseTo(0.10, within(1e-9));
  }

  /**
   * A mechanism still buzzing about its target at the end of the window hasn't settled, even when
   * its last sample happens to land inside the band: it must stay in the band through the window's
   * last tenth.
   */
  @Test
  void aBuzzWhoseLastSampleLandsInsideTheBandHasNotSettled() {
    Series measured = sampled(t -> 1 + 0.03 * Math.sin(2 * Math.PI * 50 * t), 1, 1000);

    StepResponse response = score(measured, UP, 1);

    assertThat(measured.value(measured.count() - 1))
        .as("the last sample, inside the band")
        .isCloseTo(1, within(StepResponse.SETTLING_BAND));
    assertThat(response.settlingSeconds()).as("buzzing ±3% to the end").isEmpty();
  }

  /** A mechanism that left the band only before the window's last tenth has settled. */
  @Test
  void aLastExcursionBeforeTheWindowsLastTenthStillSettles() {
    var measured = new Series(10);
    measured.add(after(0.0), 0.0);
    measured.add(after(0.85), 1.0);
    measured.add(after(0.89), 1.05); // out of the band just before the last tenth begins
    measured.add(after(0.9), 1.0);
    measured.add(after(1.0), 1.0);

    StepResponse response = score(measured, UP, 1);

    assertThat(response.settlingSeconds().getAsDouble()).isCloseTo(0.896, within(1e-9));
  }

  @Test
  void aWindowWithFewerThanTwoSamplesHasNoScore() {
    var measured = new Series(10);
    measured.add(after(0.5), 1.0);

    assertThat(StepResponse.of(measured, UP, EDGE, after(1))).isEmpty();
    assertThat(StepResponse.of(new Series(1), UP, EDGE, after(1))).isEmpty();
  }

  @Test
  void aRunIsScoredOnEachOfItsJumps() {
    Profile profile =
        Profiles.stepAndBack().from(Degrees.of(0)).to(Degrees.of(90)).hold(Seconds.of(1));
    long start = 5_000_000_000L;
    // A first-order lag following the profile, sampled at 1 kHz.
    var measured = new Series(3000);
    double y = 0;
    double dt = 0.001;
    for (int i = 0; i <= profile.durationSeconds() / dt; i++) {
      double t = i * dt;
      measured.add(start + Math.round(t * 1e9), y);
      y += (profile.setpoint(t) - y) * (1 - Math.exp(-dt / TAU));
    }

    List<Optional<StepResponse>> scores = StepResponse.of(measured, profile, start);

    assertThat(scores).hasSize(2);
    for (Optional<StepResponse> score : scores) {
      assertThat(score.orElseThrow().riseSeconds().getAsDouble())
          .isCloseTo(TAU * Math.log(9), within(0.002));
      assertThat(score.orElseThrow().overshoot()).isCloseTo(0, within(1e-6));
    }
  }
}
