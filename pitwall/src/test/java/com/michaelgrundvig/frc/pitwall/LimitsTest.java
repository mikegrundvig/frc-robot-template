package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import org.junit.jupiter.api.Test;
import org.wpilib.tunable.TunableConfig;

class LimitsTest {
  @Test
  void aValueOutsideSaysWhichLimitAndWhat() {
    Limits limits = Limits.between(0, 40);

    assertThat(limits.whyNot(20)).isNull();
    assertThat(limits.whyNot(0)).as("the ends are in").isNull();
    assertThat(limits.whyNot(-0.5)).isEqualTo("-0.5 is below its limit of 0");
    assertThat(limits.whyNot(41)).isEqualTo("41 is above its limit of 40");
  }

  @Test
  void theyTravelAsWpilibPropertiesAndComeBackTheSame() {
    for (Limits limits :
        new Limits[] {
          Limits.between(-1.5, 2.25), Limits.atLeast(0.001), Limits.atMost(1e6), Limits.NONE
        }) {
      assertThat(Limits.of(limits.config())).as(limits.toString()).isEqualTo(limits);
    }
    assertThat(Limits.between(0, 30).config().getProperties())
        .contains("\"min\":0")
        .contains("\"max\":30");
  }

  @Test
  void theyKeepATunablesOtherProperties() {
    TunableConfig config = Limits.atMost(5).on(new TunableConfig().withProperty("unit", "\"A\""));

    assertThat(config.getProperties()).contains("\"unit\":\"A\"").contains("\"max\":5");
  }

  @Test
  void propertiesThatArentLimitsMeanNone() {
    assertThat(Limits.of(null)).isEqualTo(Limits.NONE);
    assertThat(Limits.of(new TunableConfig())).isEqualTo(Limits.NONE);
    assertThat(Limits.of(new TunableConfig().withProperty("min", "\"low\"")))
        .isEqualTo(Limits.NONE);
    assertThat(Limits.of(new TunableConfig().withProperty("min", "5").withProperty("max", "1")))
        .as("backwards")
        .isEqualTo(Limits.NONE);
  }

  @Test
  void aBackwardsRangeIsAMistake() {
    assertThatIllegalArgumentException().isThrownBy(() -> Limits.between(10, 0));
    assertThatIllegalArgumentException().isThrownBy(() -> Limits.atLeast(Double.NaN));
  }

  @Test
  void theyReadAsPeopleSayThem() {
    assertThat(Limits.between(0, 100)).hasToString("0 to 100");
    assertThat(Limits.atLeast(0.25)).hasToString("at least 0.25");
    assertThat(Limits.atMost(30)).hasToString("at most 30");
    assertThat(Limits.NONE).hasToString("any number");
  }
}
