package com.michaelgrundvig.frc.world.config;

import static com.michaelgrundvig.frc.world.config.Rules.atLeast;
import static com.michaelgrundvig.frc.world.config.Rules.atMost;
import static com.michaelgrundvig.frc.world.config.Rules.below;
import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.each;
import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.oneOf;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static com.michaelgrundvig.frc.world.config.Rules.size;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

class RulesTest {
  @Test
  void positiveAndNonNegativeTakeNumbersAndMeasures() {
    Rule<Double> positiveNumber = positive();
    Rule<Distance> positiveDistance = positive();
    Rule<Integer> nonNegativeCount = nonNegative();

    assertThat(positiveNumber.whyNot(0.5)).isNull();
    assertThat(positiveNumber.whyNot(0.0)).isEqualTo("0 must be positive");
    assertThat(positiveDistance.whyNot(Meters.of(0.1))).isNull();
    assertThat(positiveDistance.whyNot(Meters.of(-2))).isEqualTo("-2 m must be positive");
    assertThat(nonNegativeCount.whyNot(0)).isNull();
    assertThat(nonNegativeCount.whyNot(-1)).isEqualTo("-1 must be zero or more");
  }

  @Test
  void nanAndInfinityFailEveryRuleThatTakesANumber() {
    List<Rule<Double>> numberRules =
        List.of(
            positive(), nonNegative(), finite(), atLeast(0), atMost(10), below(10), between(-1, 1));
    for (Rule<Double> rule : numberRules) {
      assertThat(rule.whyNot(Double.NaN)).isEqualTo("NaN must be a finite number");
      assertThat(rule.whyNot(Double.POSITIVE_INFINITY))
          .isEqualTo("Infinity must be a finite number");
      assertThat(rule.whyNot(Double.NEGATIVE_INFINITY))
          .isEqualTo("-Infinity must be a finite number");
    }
    List<Rule<Angle>> angleRules =
        List.of(
            positive(),
            nonNegative(),
            finite(),
            atLeast(Degrees.of(0)),
            atMost(Degrees.of(90)),
            below(Degrees.of(90)),
            between(Degrees.of(-90), Degrees.of(90)));
    for (Rule<Angle> rule : angleRules) {
      assertThat(rule.whyNot(Degrees.of(Double.NaN))).isEqualTo("NaN ° must be a finite number");
    }
  }

  @Test
  void rangesIncludeTheirEnds() {
    assertThat(atLeast(2).whyNot(2.0)).isNull();
    assertThat(atLeast(2).whyNot(1.0)).isEqualTo("1 must be at least 2");
    assertThat(atMost(90).whyNot(90.0)).isNull();
    assertThat(atMost(90).whyNot(90.5)).isEqualTo("90.5 must be at most 90");
    assertThat(below(180).whyNot(179.9)).isNull();
    assertThat(below(180).whyNot(180.0)).isEqualTo("180 must be below 180");
    assertThat(between(-1, 1).whyNot(-1.0)).isNull();
    assertThat(between(-1, 1).whyNot(1.0)).isNull();
    assertThat(between(-1, 1).whyNot(1.25)).isEqualTo("1.25 must be from -1 to 1");
  }

  @Test
  void measuresCompareInRealUnitsAndShowTheirOwn() {
    Rule<Angle> tilt = between(Degrees.of(-90), Degrees.of(90));

    assertThat(tilt.whyNot(Radians.of(1.5))).as("86 degrees").isNull();
    assertThat(tilt.whyNot(Radians.of(1.6))).isEqualTo("1.6 rad must be from -90 ° to 90 °");
    assertThat(atMost(Amps.of(80)).whyNot(Amps.of(120))).isEqualTo("120 A must be at most 80 A");
    assertThat(below(Degrees.of(180)).whyNot(Radians.of(3.2)))
        .isEqualTo("3.2 rad must be below 180 °");
    assertThat(atLeast(Meters.of(1)).whyNot(Meters.of(0.5)))
        .isEqualTo("0.5 m must be at least 1 m");
  }

  @Test
  void textListsAndChoices() {
    assertThat(notBlank().whyNot("front")).isNull();
    assertThat(notBlank().whyNot("  ")).isEqualTo("must not be blank");
    assertThat(oneOf("left", "right").whyNot("right")).isNull();
    assertThat(oneOf("left", "right").whyNot("up")).isEqualTo("up must be one of left, right");
    assertThat(size(4).whyNot(List.of(1, 2, 3, 4))).isNull();
    assertThat(size(4).whyNot(List.of(1, 2, 3))).isEqualTo("must have 4, has 3");
  }

  @Test
  void eachNamesEveryItemThatFails() {
    Rule<List<Double>> scales = each(positive());

    assertThat(scales.whyNot(List.of(1.0, 1.01))).isNull();
    assertThat(scales.whyNot(List.of(1.0, -1.0, 1.0, Double.NaN)))
        .isEqualTo("[1] -1 must be positive; [3] NaN must be a finite number");
  }

  @Test
  void anOptionalValueIsCheckedOnlyWhenGiven() {
    Rule<Optional<Integer>> counts = given(positive());

    assertThat(counts.whyNot(Optional.empty())).isNull();
    assertThat(counts.whyNot(Optional.of(4096))).isNull();
    assertThat(counts.whyNot(Optional.of(0))).isEqualTo("0 must be positive");
  }

  @Test
  void aValueThatIsNotANumberSaysSo() {
    Rule<Object> positive = positive();

    assertThat(positive.whyNot("seven")).isEqualTo("can't be checked as a number: it's a String");
  }

  @Test
  void numbersAreShownAsPeopleWriteThem() {
    assertThat(shown(MetersPerSecond.of(4.728761))).isEqualTo("4.729 m/s");
    assertThat(shown(Amps.of(10))).isEqualTo("10 A");
    assertThat(shown(0.08)).isEqualTo("0.08");
    assertThat(shown(0)).isEqualTo("0");
    assertThat(shown(1234567)).isEqualTo("1235000");
    assertThat(shown(Double.NaN)).isEqualTo("NaN");
  }
}
