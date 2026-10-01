package com.michaelgrundvig.frc.world.bridges;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** The writers' JSON: readable for configs a person may diff, exact for glTF's bounds. */
class JsonTest {
  private static final Map<String, Object> VALUE = value();

  private static Map<String, Object> value() {
    Map<String, Object> inner = new LinkedHashMap<>();
    inner.put("axis", "x");
    inner.put("degrees", 90.0);
    Map<String, Object> value = new LinkedHashMap<>();
    value.put("name", "a \"quoted\"\\ name\n\t\r\u0001");
    value.put("on", true);
    value.put("count", 3);
    value.put("big", 12_000_000_000L);
    value.put("position", List.of(0.1234567, -0.0, 1e-9));
    value.put("rotations", List.of(inner));
    value.put("empty", List.of());
    value.put("none", Map.of());
    return value;
  }

  @Test
  void prettyIsAValueToALineAndAListOfNumbersOnOne() {
    assertThat(Json.pretty(VALUE))
        .isEqualTo(
            """
            {
              "name": "a \\"quoted\\"\\\\ name\\n\\t\\r\\u0001",
              "on": true,
              "count": 3,
              "big": 12000000000,
              "position": [0.123457, 0, 0],
              "rotations": [
                {
                  "axis": "x",
                  "degrees": 90
                }
              ],
              "empty": [],
              "none": {}
            }
            """);
  }

  @Test
  void compactKeepsEveryDigit() {
    assertThat(Json.compact(VALUE))
        .startsWith("{\"name\":")
        .contains(
            "\"position\":[0.1234567,0,1.0E-9],\"rotations\":[{\"axis\":\"x\",\"degrees\":90}]");
    assertThat(Json.compact(List.of((double) 0.1f))).isEqualTo("[0.10000000149011612]");
  }

  @Test
  void onlyNumbersJsonHas() {
    assertThatThrownBy(() -> Json.pretty(List.of(Double.NaN)))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> Json.compact(List.of(Double.POSITIVE_INFINITY)))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> Json.compact(List.of('c')))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("no JSON for a Character");
  }
}
