package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

class JsonTextTest {
  @Test
  void readsEveryKindOfValueKeepingTheFilesOrder() {
    Object value =
        JsonText.parse(
            " {\"b\": [1, -2.5, 3e2, 0.25E-1], \"a\": {\"t\": true, \"f\": false, \"n\": null},"
                + " \"s\": \"x\\\"\\\\\\/\\b\\f\\n\\r\\t\\u00e9\", \"e\": [], \"o\": {}}\n");
    assertThat(value).isInstanceOf(Map.class);
    @SuppressWarnings("unchecked")
    Map<String, Object> map = (Map<String, Object>) value;
    assertThat(map.keySet()).containsExactly("b", "a", "s", "e", "o");
    assertThat(map.get("b")).isEqualTo(List.of(1.0, -2.5, 300.0, 0.025));
    @SuppressWarnings("unchecked")
    Map<String, Object> constants = (Map<String, Object>) map.get("a");
    assertThat(constants)
        .containsEntry("t", true)
        .containsEntry("f", false)
        .containsEntry("n", JsonText.NULL);
    assertThat(map.get("s")).isEqualTo("x\"\\/\b\f\n\r\té");
    assertThat(map.get("e")).isEqualTo(List.of());
    assertThat(map.get("o")).isEqualTo(Map.of());
  }

  @Test
  void refusesWhatIsntStrictlyJsonSayingWhere() {
    assertThatThrownBy(() -> JsonText.parse("{\n  \"a\": tru}"))
        .hasMessageContaining("unexpected 't'")
        .hasMessageContaining("line 2");
    assertThatThrownBy(() -> JsonText.parse("[1 2]")).hasMessageContaining("expected ']'");
    assertThatThrownBy(() -> JsonText.parse("[1")).hasMessageContaining("ends before ']'");
    assertThatThrownBy(() -> JsonText.parse("")).hasMessageContaining("ends where a value");
    assertThatThrownBy(() -> JsonText.parse("01")).hasMessageContaining("more after");
    assertThatThrownBy(() -> JsonText.parse("1.")).hasMessageContaining("after the decimal point");
    assertThatThrownBy(() -> JsonText.parse("1e")).hasMessageContaining("in the exponent");
    assertThatThrownBy(() -> JsonText.parse("1e999")).hasMessageContaining("too large");
    assertThatThrownBy(() -> JsonText.parse("\"a")).hasMessageContaining("no closing quote");
    assertThatThrownBy(() -> JsonText.parse("\"\\q\"")).hasMessageContaining("unknown escape");
    assertThatThrownBy(() -> JsonText.parse("\"\\u12\"")).hasMessageContaining("\\u escape");
    assertThatThrownBy(() -> JsonText.parse("\"\\u12zz\"")).hasMessageContaining("hex digits");
    assertThatThrownBy(() -> JsonText.parse("\"a\tb\"")).hasMessageContaining("control character");
    assertThatThrownBy(() -> JsonText.parse("{\"a\" 1}")).hasMessageContaining("expected ':'");
    assertThatThrownBy(() -> JsonText.parse("[".repeat(JsonText.MAX_DEPTH + 2)))
        .hasMessageContaining("nested deeper");
  }
}
