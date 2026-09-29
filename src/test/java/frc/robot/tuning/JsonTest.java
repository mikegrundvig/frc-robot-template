package frc.robot.tuning;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.lib.config.ConfigTuner;
import java.util.List;
import org.junit.jupiter.api.Test;

class JsonTest {
  @Test
  void stringsAreQuotedAndEscaped() {
    assertThat(Json.string("plain")).isEqualTo("\"plain\"");
    assertThat(Json.string("say \"hi\" \\ bye")).isEqualTo("\"say \\\"hi\\\" \\\\ bye\"");
    assertThat(Json.string("line\nnext")).isEqualTo("\"line\\nnext\"");
    assertThat(Json.string("bell\u0007")).isEqualTo("\"bell\\u0007\"");
    assertThat(Json.string("10°")).isEqualTo("\"10°\"");
  }

  @Test
  void theViewHoldsEveryValueAndChange() {
    var view =
        new Tuning.View(
            "test",
            true,
            false,
            List.of(
                new Tuning.Value("a.b", "A", 30, 35, null),
                new Tuning.Value("a.c", "", 1, 1, "too big")),
            List.of(
                new ConfigTuner.Change("a.b", "30.0 A", "35.0 A", ".b(Amps.of(35.0))"),
                new ConfigTuner.Change("a.d", "1.0", "2.0", ".d(2.0)")));

    assertThat(Json.view(view))
        .isEqualTo(
            "{\"robot\":\"test\",\"enabled\":true,\"fieldConnected\":false,\"values\":["
                + "{\"path\":\"a.b\",\"unit\":\"A\",\"committed\":30.0,\"current\":35.0,"
                + "\"refused\":null},"
                + "{\"path\":\"a.c\",\"unit\":\"\",\"committed\":1.0,\"current\":1.0,"
                + "\"refused\":\"too big\"}],"
                + "\"changes\":[{\"path\":\"a.b\",\"committed\":\"30.0 A\",\"tuned\":\"35.0 A\","
                + "\"builderCall\":\".b(Amps.of(35.0))\"},"
                + "{\"path\":\"a.d\",\"committed\":\"1.0\",\"tuned\":\"2.0\","
                + "\"builderCall\":\".d(2.0)\"}]}");
  }

  @Test
  void numbersJsonCannotHoldBecomeNull() {
    var view =
        new Tuning.View(
            "t",
            false,
            false,
            List.of(new Tuning.Value("x", "", Double.NaN, Double.POSITIVE_INFINITY, null)),
            List.of());

    assertThat(Json.view(view)).contains("\"committed\":null,\"current\":null");
  }
}
