package com.michaelgrundvig.frc.pitwall.push;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.junit.jupiter.api.Test;

class DeltaTest {
  /** A state with one section of status and one of values, as the robot loop would hand it over. */
  private static Map<String, Map<String, Object>> state(double battery, double steerKp) {
    Map<String, Map<String, Object>> state = new LinkedHashMap<>();
    state.put(
        "status",
        new LinkedHashMap<>(
            Map.of("Robot", "practice", "Battery", Reading.of(battery).toNearest(0.1))));
    state.put("values", new LinkedHashMap<>(Map.of("drivetrain.gains.steerKp", steerKp)));
    return state;
  }

  private static Map<String, Object> section(Map<String, Map<String, Object>> state, String name) {
    return Objects.requireNonNull(state.get(name), name);
  }

  @Test
  void theFirstPatchIsEverythingAndTheDocumentIsWhatWasSent() {
    var delta = new Delta();

    Map<String, Object> first = delta.patch(state(12.34, 20.0));

    assertThat(first).containsOnlyKeys("status", "values");
    assertThat(delta.document())
        .isEqualTo(
            Map.of(
                "status", Map.of("Robot", "practice", "Battery", 12.34),
                "values", Map.of("drivetrain.gains.steerKp", 20.0)));
  }

  @Test
  void nothingChangedSendsNothingLoopAfterLoop() {
    var delta = new Delta();
    delta.patch(state(12.3, 20.0));

    for (int loop = 0; loop < 500; loop++) {
      assertThat(delta.patch(state(12.3, 20.0))).as("loop %d", loop).isEmpty();
    }
  }

  @Test
  void oneChangeSendsExactlyThatChange() {
    var delta = new Delta();
    delta.patch(state(12.3, 20.0));

    assertThat(delta.patch(state(12.3, 55.0)))
        .isEqualTo(Map.of("values", Map.of("drivetrain.gains.steerKp", 55.0)));
  }

  @Test
  void noiseAtARoundingEdgeSendsNothingUntilTheReadingReallyMoves() {
    var delta = new Delta();
    delta.patch(state(12.35, 20.0));

    for (double noise : new double[] {0.04, -0.04, 0.02, -0.03, 0.049, -0.049}) {
      assertThat(delta.patch(state(12.35 + noise, 20.0))).as("noise of %s V", noise).isEmpty();
    }

    Map<String, Object> patch = delta.patch(state(12.46, 20.0));

    assertThat(patch).isEqualTo(Map.of("status", Map.of("Battery", 12.46)));
  }

  @Test
  void aSlowDriftIsSentOnceItAddsUp() {
    var delta = new Delta();
    delta.patch(state(12.30, 20.0));
    int sends = 0;
    for (int step = 1; step <= 30; step++) {
      if (!delta.patch(state(12.30 - step * 0.015, 20.0)).isEmpty()) {
        sends++;
      }
    }

    assertThat(sends)
        .as("a 0.45 V sag in 0.015 V steps, at 0.1 V resolution: once every 7 steps")
        .isEqualTo(4);
  }

  @Test
  void aRemovedKeyOrSectionIsNullAndANewSectionComesWhole() {
    var delta = new Delta();
    Map<String, Map<String, Object>> state = state(12.3, 20.0);
    delta.patch(state);

    section(state, "status").remove("Robot");
    state.put("actions", new LinkedHashMap<>());
    Map<String, Object> patch = delta.patch(state);

    Map<String, Object> removedRobot = new HashMap<>();
    removedRobot.put("Robot", null);
    assertThat(patch).isEqualTo(Map.of("status", removedRobot, "actions", Map.of()));

    state.remove("values");
    Map<String, Object> removedValues = new HashMap<>();
    removedValues.put("values", null);
    assertThat(delta.patch(state)).isEqualTo(removedValues);
  }

  @Test
  void numbersJsonCantCarryGoAsTheirNames() {
    var delta = new Delta();
    Map<String, Map<String, Object>> state = new LinkedHashMap<>();
    state.put(
        "status",
        new LinkedHashMap<>(
            Map.of("Gyro", Double.NaN, "Battery", Reading.of(Double.NaN).toNearest(0.1))));

    assertThat(delta.patch(state))
        .isEqualTo(Map.of("status", Map.of("Gyro", "NaN", "Battery", "NaN")));
    assertThat(delta.patch(state)).as("still not a number: unchanged").isEmpty();

    section(state, "status").put("Gyro", Double.POSITIVE_INFINITY);
    assertThat(delta.patch(state)).isEqualTo(Map.of("status", Map.of("Gyro", "Infinity")));
  }

  @Test
  void listsAndMapsCompareWhole() {
    var delta = new Delta();
    Map<String, Map<String, Object>> state = new LinkedHashMap<>();
    state.put(
        "actions",
        new LinkedHashMap<>(Map.of("Drive check", Map.of("phase", "IDLE", "detail", ""))));
    state.put("changes", new LinkedHashMap<>(Map.of("lines", List.of(".steerKp(55.0)"))));
    delta.patch(state);

    section(state, "actions").put("Drive check", Map.of("phase", "RUNNING", "detail", ""));

    assertThat(delta.patch(state))
        .isEqualTo(
            Map.of("actions", Map.of("Drive check", Map.of("phase", "RUNNING", "detail", ""))));
  }

  /** A merge patch merges a map into the one before, so a key a map lost must be sent as null. */
  @Test
  void aMapThatLosesAKeySaysSo() {
    var delta = new Delta();
    Map<String, Map<String, Object>> state = new LinkedHashMap<>();
    state.put("run", new LinkedHashMap<>(Map.of("gains", Map.of("kP", 16.0, "kD", 0.1), "id", 1)));
    delta.patch(state);

    section(state, "run").put("gains", Map.of("kP", 18.0));
    Map<String, Object> gains = new HashMap<>();
    gains.put("kP", 18.0);
    gains.put("kD", null);

    assertThat(delta.patch(state)).isEqualTo(Map.of("run", Map.of("gains", gains)));
    assertThat(delta.document())
        .isEqualTo(Map.of("run", Map.of("gains", Map.of("kP", 18.0), "id", 1)));
  }

  @Test
  void aReadingHasAPositiveResolution() {
    assertThatIllegalArgumentException().isThrownBy(() -> Reading.of(1).toNearest(0));
    assertThatIllegalArgumentException().isThrownBy(() -> Reading.of(1).toNearest(Double.NaN));
    assertThat(Reading.of(12.3).toNearest(0.1)).hasToString("12.3 (to the nearest 0.1)");
  }
}
