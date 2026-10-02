package com.michaelgrundvig.frc.world.config;

import static com.michaelgrundvig.frc.world.config.Rules.atLeast;
import static com.michaelgrundvig.frc.world.config.Rules.below;
import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatCode;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.catchThrowableOfType;
import static org.wpilib.units.Units.Seconds;

import java.util.List;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.wpilib.units.measure.Time;

class ChecksTest {
  /** A record that checks itself the way configuration records do. */
  record Gains(double steerKp, double steerKd, int samples, Time lookahead) {
    Gains {
      Checks.on(Gains.class)
          .field("steerKp", steerKp, nonNegative())
          .field("steerKd", steerKd, nonNegative())
          .field("samples", samples, atLeast(1))
          .field("lookahead", lookahead, between(Seconds.of(0), Seconds.of(0.2)))
          .check(
              "steerKd",
              () ->
                  steerKd <= steerKp
                      ? null
                      : shown(steerKd) + " must be at most steerKp (" + shown(steerKp) + ")")
          .orThrow();
    }
  }

  @Test
  void aValidRecordBuilds() {
    assertThatCode(() -> new Gains(16, 0.1, 3, Seconds.of(0.05))).doesNotThrowAnyException();
  }

  @Test
  void everyViolationIsReportedAtOnce() {
    assertThat(violations(() -> new Gains(-1, Double.NaN, 0, Seconds.of(0.5))))
        .containsExactly(
            new Violation("Gains.steerKp", "-1 must be zero or more"),
            new Violation("Gains.steerKd", "NaN must be a finite number"),
            new Violation("Gains.samples", "0 must be at least 1"),
            new Violation("Gains.lookahead", "0.5 s must be from 0 s to 0.2 s"));
  }

  @Test
  void theMessageIsOneViolationPerLine() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new Gains(16, 0, 0, Seconds.of(-1)))
        .withMessage(
            "Gains.samples: 0 must be at least 1\nGains.lookahead: -1 s must be from 0 s to 0.2 s");
  }

  @Test
  void aCheckAcrossValuesIsReportedUnderItsName() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new Gains(16, 40, 3, Seconds.of(0)))
        .withMessage("Gains.steerKd: 40 must be at most steerKp (16)");
  }

  @Test
  void checksAcrossValuesWaitForEveryValueToPassItsOwnRule() {
    // steerKd is more than steerKp, but steerKp is itself wrong: fixing it comes first.
    assertThat(violations(() -> new Gains(-1, 0.5, 3, Seconds.of(0))))
        .containsExactly(new Violation("Gains.steerKp", "-1 must be zero or more"));
  }

  @Test
  void aRecordWithANameOfItsOwnIsNamedByIt() {
    assertThat(
            violations(
                () -> Checks.on(Gains.class, "front").field("noise", 0.0, positive()).orThrow()))
        .containsExactly(new Violation("Gains[front].noise", "0 must be positive"));
  }

  @Test
  void aValueStopsAtTheFirstRuleItBreaks() {
    assertThat(
            violations(
                () ->
                    Checks.on(Gains.class)
                        .field("fieldOfView", Double.NaN, positive())
                        .field("fieldOfView", Double.NaN, below(180))
                        .field("fieldOfView", 200.0, below(180))
                        .orThrow()))
        .containsExactly(new Violation("Gains.fieldOfView", "NaN must be a finite number"));
  }

  @Test
  void aRuleThatThrowsIsReportedWithTheRest() {
    Rule<List<Integer>> firstPositive = list -> list.get(0) > 0 ? null : "must start positive";

    List<Violation> violations =
        violations(
            () ->
                Checks.on(Gains.class)
                    .field("first", List.<Integer>of(), firstPositive)
                    .field("second", -1.0, positive())
                    .orThrow());

    assertThat(violations).hasSize(2);
    assertThat(violations.get(0).where()).isEqualTo("Gains.first");
    assertThat(violations.get(0).what()).startsWith("couldn't be checked: ");
    assertThat(violations.get(1)).isEqualTo(new Violation("Gains.second", "-1 must be positive"));
  }

  @Test
  void anExceptionNeedsAViolation() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new ConfigException(List.of()))
        .withMessageContaining("at least one violation");
  }

  private static List<Violation> violations(ThrowingCallable building) {
    return catchThrowableOfType(ConfigException.class, building).violations();
  }
}
