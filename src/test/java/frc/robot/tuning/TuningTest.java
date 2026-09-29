package frc.robot.tuning;

import static frc.robot.lib.config.ConfigFixtures.body;
import static frc.robot.lib.config.ConfigFixtures.swerve;
import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.RobotConfig;
import frc.robot.lib.config.ConfigTuner;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class TuningTest {
  private static final String STEER_KP = "drivetrain.gains.steerKp";

  /** What each loop logged: the request kinds, then their paths. */
  private final List<String> logged = new ArrayList<>();

  /** Each view the robot was told about. */
  private final List<Tuning.View> shown = new ArrayList<>();

  @Test
  void startsAtTheCommittedConfiguration() {
    Tuning tuning = tuning(true);

    assertThat(tuning.config()).isEqualTo(robot());
    assertThat(tuning.view().enabled()).isTrue();
    assertThat(tuning.view().changes()).isEmpty();
    assertThat(tuning.view().values())
        .extracting(Tuning.Value::path)
        .contains(STEER_KP, "drivetrain.limits.driveStatorCurrent");
    assertThat(tuning.report()).isEmpty();
  }

  @Test
  void changesWaitForTheRobotLoop() {
    Tuning tuning = tuning(true);

    tuning.requestSet(STEER_KP, 18.5);
    assertThat(tuning.config().drivetrain().gains().steerKp()).as("before the loop").isEqualTo(16);

    tuning.periodic(false);
    assertThat(tuning.config().drivetrain().gains().steerKp()).isEqualTo(18.5);
    assertThat(tuning.view().changes())
        .extracting(ConfigTuner.Change::builderCall)
        .containsExactly(".steerKp(18.5)");
  }

  @Test
  void aRefusedValueSaysWhyAndChangesNothing() {
    Tuning tuning = tuning(true);

    tuning.requestSet(STEER_KP, -1);
    tuning.periodic(false);
    assertThat(value(tuning, STEER_KP).refused()).startsWith("steerKp must be zero or more");
    assertThat(tuning.config()).isEqualTo(robot());

    tuning.requestSet(STEER_KP, 18.5);
    tuning.periodic(false);
    assertThat(value(tuning, STEER_KP).refused()).isNull();
  }

  @Test
  void nothingChangesWhileTuningIsOff() {
    Tuning tuning = tuning(false);

    tuning.requestSet(STEER_KP, 18.5);
    tuning.periodic(false);
    assertThat(value(tuning, STEER_KP).refused()).isEqualTo("tuning is off");

    tuning.requestEnabled(true);
    tuning.requestSet(STEER_KP, 18.5);
    tuning.periodic(false);
    assertThat(tuning.config().drivetrain().gains().steerKp()).isEqualTo(18.5);

    tuning.requestEnabled(false);
    tuning.periodic(false);
    assertThat(tuning.view().enabled()).isFalse();
  }

  @Test
  void theFieldLocksTuningOff() {
    Tuning tuning = tuning(true);

    tuning.periodic(true);
    assertThat(tuning.view().enabled()).isFalse();
    assertThat(tuning.view().fieldConnected()).isTrue();

    tuning.requestEnabled(true);
    tuning.requestSet(STEER_KP, 18.5);
    tuning.periodic(true);
    assertThat(tuning.view().enabled()).isFalse();
    assertThat(value(tuning, STEER_KP).refused()).isEqualTo("locked while the field is connected");

    // Once the field disconnects, tuning stays off until someone switches it on.
    tuning.periodic(false);
    assertThat(tuning.view().enabled()).isFalse();
  }

  @Test
  void resetReturnsToTheCommittedConfiguration() {
    Tuning tuning = tuning(true);
    tuning.requestSet(STEER_KP, 18.5);
    tuning.requestSet("drivetrain.gains.steerKd", -1); // refused
    tuning.periodic(false);

    tuning.requestReset();
    tuning.periodic(false);

    assertThat(tuning.config()).isEqualTo(robot());
    assertThat(tuning.view().changes()).isEmpty();
    assertThat(value(tuning, "drivetrain.gains.steerKd").refused()).isNull();
  }

  @Test
  void resetNeedsTuningOn() {
    Tuning tuning = tuning(true);
    tuning.requestSet(STEER_KP, 18.5);
    tuning.requestEnabled(false);
    tuning.requestReset();

    tuning.periodic(false);

    assertThat(tuning.config().drivetrain().gains().steerKp()).isEqualTo(18.5);
  }

  @Test
  void everyLoopLogsTheRequestsItApplies() {
    Tuning tuning = tuning(true);

    tuning.periodic(false);
    tuning.requestSet(STEER_KP, 18.5);
    tuning.requestReset();
    tuning.periodic(false);

    assertThat(logged).containsExactly("|", "SET,RESET|" + STEER_KP + ",");
  }

  @Test
  void replayAppliesTheLoggedRequests() {
    // In replay nothing is queued: the log supplies the requests, and whether tuning was on.
    Tuning tuning =
        new Tuning(
            robot(),
            false,
            inputs -> {
              inputs.enabled = true;
              inputs.kinds = new String[] {"SET"};
              inputs.paths = new String[] {STEER_KP};
              inputs.values = new double[] {18.5};
            },
            view -> {});

    tuning.periodic(false);

    assertThat(tuning.config().drivetrain().gains().steerKp()).isEqualTo(18.5);
  }

  @Test
  void saysWhenWhatThePageShowsChanges() {
    Tuning tuning = tuning(true);

    tuning.periodic(false);
    assertThat(shown).as("nothing happened").isEmpty();

    tuning.requestSet(STEER_KP, 18.5);
    tuning.periodic(false);
    tuning.periodic(true); // the field connects
    assertThat(shown).hasSize(2);
    assertThat(shown.get(0).changes()).hasSize(1);
    assertThat(shown.get(1).fieldConnected()).isTrue();
  }

  @Test
  void theViewSaysWhichValuesDiffer() {
    Tuning tuning = tuning(true);
    assertThat(tuning.view().hasChanges()).isFalse();

    tuning.requestSet(STEER_KP, 18.5);
    tuning.requestSet("drivetrain.limits.driveStatorCurrent", 35);
    tuning.periodic(false);

    assertThat(tuning.view().hasChanges()).isTrue();
    assertThat(tuning.view().changedPaths())
        .isEqualTo("drivetrain.limits.driveStatorCurrent, " + STEER_KP);
  }

  @Test
  void theReportGivesTheBuilderCallForEachChange() {
    Tuning tuning = tuning(true);
    tuning.requestSet(STEER_KP, 18.5);
    tuning.periodic(false);

    assertThat(tuning.report())
        .containsExactly(
            "Tuned values differ from the committed configuration of test. To keep them, write"
                + " these into Robots.java:",
            "  drivetrain.gains.steerKp: 16.0 -> 18.5   .steerKp(18.5)");
  }

  private Tuning tuning(boolean enabled) {
    return new Tuning(
        robot(),
        enabled,
        inputs -> logged.add(String.join(",", inputs.kinds) + "|" + String.join(",", inputs.paths)),
        shown::add);
  }

  private static RobotConfig robot() {
    return RobotConfig.builder()
        .name("test")
        .body(body())
        .drivetrain(swerve())
        .cameras(List.of())
        .build();
  }

  private static Tuning.Value value(Tuning tuning, String path) {
    return tuning.view().values().stream()
        .filter(value -> value.path().equals(path))
        .findFirst()
        .orElseThrow();
  }
}
