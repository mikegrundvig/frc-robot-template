package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableBackend;
import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;

/**
 * The tee between WPILib's tunables and a dashboard, with WPILib's mock backend as the dashboard.
 */
class TunableTeeTest {
  private final MockTunableBackend dashboard = new MockTunableBackend();
  private final List<String> logged = new ArrayList<>();
  private final List<String> tuned = new ArrayList<>();
  private @Nullable String whyLocked;
  private final TunableTee tee =
      new TunableTee(
          dashboard,
          () -> whyLocked,
          inputs ->
              logged.add(String.join(",", inputs.paths) + "=" + String.join(",", inputs.values)),
          false);
  private final TunableDouble kP =
      TunableDouble.createConfig(0.5, new TunableConfig().withOnTune(() -> tuned.add("kP")));
  private final TunableBoolean intake = TunableBoolean.create(false);

  @BeforeEach
  void publish() {
    tee.publish("Shooter/kP", kP);
    tee.publish("Intake/enabled", intake);
  }

  @AfterEach
  void reset() {
    TunableRegistry.reset();
  }

  @Test
  void dashboardsSeeEveryTunableThroughTheTee() {
    assertThat(dashboard.getDouble("Shooter/kP")).isEqualTo(0.5);

    kP.set(0.7);

    assertThat(dashboard.getDouble("Shooter/kP")).as("the real value").isEqualTo(0.7);
  }

  @Test
  void aDashboardChangeAppliesWhileTuningIsAllowedAndIsLogged() {
    dashboard.setDouble("Shooter/kP", 0.9);
    tee.update();

    assertThat(kP.get()).isEqualTo(0.9);
    assertThat(logged).containsExactly("/Shooter/kP=0.9");
    assertThat(tuned).as("its onTune ran").containsExactly("kP");
  }

  @Test
  void aDashboardChangeWhileLockedIsRefused() {
    whyLocked = "the field system is connected";

    dashboard.setDouble("Shooter/kP", 0.9);
    dashboard.setBoolean("Intake/enabled", true);
    tee.update();

    assertThat(kP.get()).isEqualTo(0.5);
    assertThat(intake.get()).isFalse();
    assertThat(dashboard.getDouble("Shooter/kP")).as("snaps back").isEqualTo(0.5);
    assertThat(logged).containsExactly("=");
    assertThat(tuned).isEmpty();
  }

  @Test
  void replayMakesTheLoggedChangesAndIgnoresDashboards() {
    var replayDashboard = new MockTunableBackend();
    var replay =
        new TunableTee(
            replayDashboard,
            () -> null,
            inputs -> {
              inputs.paths = new String[] {"/Shooter/kP"};
              inputs.values = new String[] {"0.8"};
            },
            true);
    var gain =
        TunableDouble.createConfig(0.5, new TunableConfig().withOnTune(() -> tuned.add("gain")));
    replay.publish("Shooter/kP", gain);

    replayDashboard.setDouble("Shooter/kP", 0.9);
    replay.update();

    assertThat(gain.get()).isEqualTo(0.8);
    assertThat(tuned).containsExactly("gain");
  }

  @Test
  void thePageListsEveryTunableAndChangesSeveralAtOnce() {
    tee.update();
    assertThat(tee.view())
        .containsExactly(
            new TunableTee.Listed("/Shooter/kP", "double", 0.5, true),
            new TunableTee.Listed("/Intake/enabled", "boolean", false, true));

    Map<String, Object> values = new LinkedHashMap<>();
    values.put("Shooter/kP", 1.5);
    values.put("Intake/enabled", true);
    var outcome = tee.requestChanges(values);
    assertThat(outcome).as("waits for the update").isNotDone();
    tee.update();

    assertThat(outcome.getNow(null)).isEqualTo(Outcome.applied());
    assertThat(kP.get()).isEqualTo(1.5);
    assertThat(intake.get()).isTrue();
    assertThat(logged).last().isEqualTo("/Shooter/kP,/Intake/enabled=1.5,true");
    assertThat(tee.isEditable("Shooter/kP")).isTrue();
  }

  @Test
  void thePageIsLockedLikeDashboards() {
    whyLocked = "tuning is off";

    var outcome = tee.requestChanges(Map.of("Shooter/kP", 1.5));
    tee.update();

    assertThat(outcome.getNow(null))
        .isEqualTo(new Outcome(Outcome.Result.LOCKED, "Tuning is locked: tuning is off"));
    assertThat(kP.get()).isEqualTo(0.5);
  }

  @Test
  void aValueOfTheWrongTypeChangesNothing() {
    Map<String, Object> values = new LinkedHashMap<>();
    values.put("Shooter/kP", 1.5);
    values.put("Intake/enabled", 3);
    var outcome = tee.requestChanges(values);
    tee.update();

    assertThat(outcome.getNow(null))
        .isEqualTo(new Outcome(Outcome.Result.REFUSED, "/Intake/enabled takes a boolean value"));
    assertThat(kP.get()).as("all or none").isEqualTo(0.5);
  }

  @Test
  void onlyTunablesThePageCanChangeAreChanged() {
    var readOnly = TunableDouble.createConfig(2.0, new TunableConfig().withMutable(false));
    tee.publish("Arm/ratio", readOnly);
    tee.update();

    var unknown = tee.requestChanges(Map.of("Nope", 1));
    var fixed = tee.requestChanges(Map.of("Arm/ratio", 3.0));
    tee.update();

    assertThat(unknown.getNow(null).detail())
        .isEqualTo("There is no tunable the page can change at /Nope");
    assertThat(fixed.getNow(null).result()).isEqualTo(Outcome.Result.REFUSED);
    assertThat(tee.isEditable("Arm/ratio")).isFalse();
    assertThat(readOnly.get()).isEqualTo(2.0);
  }

  @Test
  void pitwallsOwnTunablesPassStraightThroughAndStayOffTheList() {
    var config = TunableDouble.create(16.0);
    tee.passThrough(config);
    tee.publish("Config/steerKp", config);
    tee.update();

    assertThat(dashboard.getTunable("Config/steerKp")).isSameAs(config);
    assertThat(tee.view()).extracting(TunableTee.Listed::path).doesNotContain("/Config/steerKp");
  }

  @Test
  void otherKindsPassStraightThroughReadOnly() {
    TunableBase other = new TunableBase(null) {};
    tee.publish("Vision/layout", other);
    tee.update();

    assertThat(dashboard.getTunable("Vision/layout")).isSameAs(other);
    assertThat(tee.isEditable("Vision/layout")).isFalse();
  }

  @Test
  void movingToAnotherBackendHandsOverTheRealTunables() {
    List<TunableBackend.PublishedTunable> moved = tee.removePrefix("");

    assertThat(moved)
        .extracting(TunableBackend.PublishedTunable::tunable)
        .containsExactly(kP, intake);
  }

  @Test
  void aRemovedTunableLeavesTheList() {
    tee.remove("Shooter/kP");
    tee.update();

    assertThat(tee.view()).extracting(TunableTee.Listed::path).containsExactly("/Intake/enabled");
  }
}
