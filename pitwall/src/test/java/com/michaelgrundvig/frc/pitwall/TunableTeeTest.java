package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.tuple;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableBackend;
import org.wpilib.tunable.TunableBase;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableInt;
import org.wpilib.tunable.TunableLong;
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
            new TunableTee.Listed("/Shooter/kP", "double", 0.5, true, Limits.NONE),
            new TunableTee.Listed("/Intake/enabled", "boolean", false, true, Limits.NONE));

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
        .isEqualTo(
            Outcome.refused(
                List.of(
                    Outcome.Reason.of(
                        Outcome.Layer.VALUE,
                        "/Intake/enabled",
                        "",
                        "/Intake/enabled takes a boolean value",
                        "/Intake/enabled takes a boolean value"))));
    assertThat(outcome.getNow(null).detail()).isEqualTo("/Intake/enabled takes a boolean value");
    assertThat(kP.get()).as("all or none").isEqualTo(0.5);
  }

  /** A switch left changed in the pit (a camera off, say) is listed, for the field's alert. */
  @Test
  void listsWhatChangedSinceStartup() {
    assertThat(tee.changedSinceStartup()).isEmpty();

    dashboard.setBoolean("Intake/enabled", true);
    tee.update();

    assertThat(tee.changedSinceStartup()).containsExactly("/Intake/enabled: true (was false)");
  }

  /** A value that isn't a number would throw wherever the robot program uses it. */
  @Test
  void numbersThatArentNumbersAreRefused() {
    var page = tee.requestChanges(Map.of("Shooter/kP", Double.POSITIVE_INFINITY));
    dashboard.setDouble("Shooter/kP", Double.NaN);
    tee.update();

    assertThat(page.getNow(null).detail()).isEqualTo("/Shooter/kP takes a double value");
    assertThat(page.getNow(null).reasons())
        .singleElement()
        .satisfies(
            reason -> {
              assertThat(reason.layer()).isEqualTo(Outcome.Layer.VALUE);
              assertThat(reason.paths()).containsExactly("/Shooter/kP");
            });
    assertThat(kP.get()).isEqualTo(0.5);
    assertThat(dashboard.getDouble("Shooter/kP")).as("snaps back").isEqualTo(0.5);
  }

  @Test
  void aWholeNumberBeyondItsTypeIsRefused() {
    var count = TunableLong.create(3);
    tee.publish("Arm/count", count);

    var page = tee.requestChanges(Map.of("Arm/count", 1e30)); // would saturate to Long.MAX_VALUE
    tee.update();

    assertThat(page.getNow(null).result()).isEqualTo(Outcome.Result.REFUSED);
    assertThat(count.get()).isEqualTo(3);
  }

  /** A team's tunable whose setter throws refuses the value; the page's other changes go back. */
  @Test
  void aSetterThatThrowsRefusesAndTheOtherChangesGoBack() {
    double[] limit = {10};
    var guarded =
        TunableDouble.createConfig(
            () -> limit[0],
            value -> {
              if (value > 40) {
                throw new IllegalArgumentException("at most 40 A");
              }
              limit[0] = value;
            },
            new TunableConfig());
    tee.publish("Arm/limit", guarded);
    Map<String, Object> values = new LinkedHashMap<>();
    values.put("Intake/enabled", true);
    values.put("Arm/limit", 60.0);

    var page = tee.requestChanges(values);
    tee.update();
    dashboard.setDouble("Arm/limit", 80);
    tee.update();

    assertThat(page.getNow(null))
        .isEqualTo(
            Outcome.refused(
                List.of(
                    Outcome.Reason.of(
                        Outcome.Layer.TUNABLE,
                        "/Arm/limit",
                        "",
                        "at most 40 A",
                        "/Arm/limit refused the value: at most 40 A"))));
    assertThat(intake.get()).as("all or none").isFalse();
    assertThat(limit[0]).isEqualTo(10);
  }

  @Test
  void anOnTuneThatThrowsNeverStopsTheLoop() {
    var gain =
        TunableDouble.createConfig(
            1.0,
            new TunableConfig()
                .withOnTune(
                    () -> {
                      throw new IllegalStateException("the mechanism rejected it");
                    }));
    tee.publish("Arm/kP", gain);

    dashboard.setDouble("Arm/kP", 2.0);
    tee.update();

    assertThat(gain.get()).isEqualTo(2.0);
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
  void otherKindsPassStraightThroughReadOnly() {
    TunableBase other = new TunableBase(null) {};
    tee.publish("Vision/layout", other);
    tee.update();

    assertThat(dashboard.getTunable("Vision/layout")).isSameAs(other);
    assertThat(tee.isEditable("Vision/layout")).isFalse();
  }

  /** A unit-aware tunable, built like WPILib's own for measures: a wrapper around a number. */
  static final class Wrapped extends TunableBase implements Tunable.CustomTunable {
    final TunableDouble magnitude;

    Wrapped(double initial) {
      super(null);
      magnitude = TunableDouble.create(initial);
    }

    @Override
    public TunableBase getInnerTunable() {
      return magnitude;
    }
  }

  @Test
  void unitAwareTunablesGoThroughAsTheirNumberInBaseUnits() {
    // WPILib's registry unwraps them before any backend sees them.
    TunableRegistry.registerBackend("", tee);
    var limit = new Wrapped(30.0);
    TunableRegistry.publish("Drive/limit", limit);

    dashboard.setDouble("Drive/limit", 25.0);
    TunableRegistry.update();

    assertThat(limit.magnitude.get()).isEqualTo(25.0);
    assertThat(logged).last().isEqualTo("/Drive/limit=25.0");
    assertThat(tee.isEditable("Drive/limit")).isTrue();
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

  /** A tunable's limits ride on it as WPILib properties, which dashboards see through the tee. */
  @Test
  void dashboardsSeeATunablesLimits() {
    tee.publish("Vision/hold", TunableDouble.createConfig(0.0, Limits.between(0, 30).config()));

    assertThat(Limits.of(dashboard.getTunable("Vision/hold").getConfig()))
        .isEqualTo(Limits.between(0, 30));
  }

  @Test
  void aDashboardWriteOutsideItsLimitsIsRefusedAndSaysWhy() {
    TunableDouble hold = TunableDouble.createConfig(2.0, Limits.between(0, 30).config());
    tee.publish("Vision/hold", hold);

    dashboard.setDouble("Vision/hold", 45);
    tee.update();
    dashboard.setDouble("Vision/hold", 5);
    tee.update();

    assertThat(hold.get()).as("the second, within the limits").isEqualTo(5.0);
    assertThat(tee.refusals()).containsExactly("/Vision/hold = 45.0: 45 is above its limit of 30");
    assertThat(logged).as("only what was applied").containsExactly("=", "/Vision/hold=5.0");
  }

  @Test
  void aDashboardWriteWhileLockedSaysWhyItWasRefused() {
    whyLocked = "tuning is off";

    dashboard.setDouble("Shooter/kP", 0.9);
    tee.update();

    assertThat(tee.refusals())
        .containsExactly("/Shooter/kP = 0.9: tuning is locked: tuning is off");
  }

  /** The page's changes are checked against every limit before any of them applies. */
  @Test
  void thePageIsRefusedOutsideTheLimitsAndNothingChanges() {
    TunableInt count = TunableInt.createConfig(3, Limits.atLeast(1).config());
    tee.publish("Arm/count", count);
    Map<String, Object> values = new LinkedHashMap<>();
    values.put("Shooter/kP", 0.7);
    values.put("Arm/count", 0);

    var outcome = tee.requestChanges(values);
    tee.update();

    assertThat(outcome.getNow(null))
        .isEqualTo(
            Outcome.refused(
                List.of(
                    Outcome.Reason.of(
                        Outcome.Layer.LIMIT,
                        "/Arm/count",
                        "",
                        "0 is below its limit of 1",
                        "/Arm/count: 0 is below its limit of 1"))));
    assertThat(outcome.getNow(null).detail()).isEqualTo("/Arm/count: 0 is below its limit of 1");
    assertThat(kP.get()).as("all or none").isEqualTo(0.5);
    assertThat(count.get()).isEqualTo(3);
  }

  @Test
  void theListCarriesEachTunablesLimits() {
    tee.publish("Vision/hold", TunableDouble.createConfig(0.0, Limits.atMost(30).config()));
    tee.update();

    assertThat(tee.view())
        .filteredOn(listed -> listed.path().equals("/Vision/hold"))
        .singleElement()
        .extracting(TunableTee.Listed::limits)
        .isEqualTo(Limits.atMost(30));
  }

  /** The test machine's travel and gain, live, published through the tee as WPILib does. */
  private LiveConfig<Fixtures.Machine> publishedMachine() {
    var live =
        LiveConfig.of("machine", Fixtures.machine())
            .number("drive.gains.kP", Limits.between(0, 100))
            .number("drive.limits.maxSpeed", Limits.atMost(10))
            .number("travel.min", Limits.NONE)
            .number("travel.max", Limits.NONE);
    TunableRegistry.registerBackend("", tee);
    TunableRegistry.publish("Machine", live);
    tee.update();
    return live;
  }

  @Test
  void aLiveConfigurationIsKnownWithWhereItsValuesArePublished() {
    var live = publishedMachine();

    assertThat(tee.configs()).containsExactly(live);
    assertThat(tee.pathOf(live, "travel.min")).isEqualTo("/Machine/travel/min");
    assertThat(tee.view())
        .as("shown with their configuration instead")
        .extracting(TunableTee.Listed::path)
        .doesNotContain("/Machine/travel/min");
  }

  /** A dashboard's change to a configuration value is checked by the whole configuration. */
  @Test
  void aDashboardChangeTheConfigurationRefusesSaysWhy() {
    var live = publishedMachine();

    dashboard.setDouble("Machine/drive/limits/maxSpeed", 5);
    tee.update();

    assertThat(live.get()).isEqualTo(Fixtures.machine());
    assertThat(tee.refusals())
        .containsExactly(
            "/Machine/drive/limits/maxSpeed = 5.0: machine: maxSpeed is faster than the motor's"
                + " free speed");
  }

  /** From the page, a configuration's values change together: one rebuild, all or none. */
  @Test
  void thePageChangesAConfigurationsValuesTogether() {
    var live = publishedMachine();
    Map<String, Object> both = new LinkedHashMap<>();
    both.put("Machine/travel/min", 12);
    both.put("Machine/travel/max", 20);

    var outcome = tee.requestChanges(both);
    tee.update();

    assertThat(outcome.getNow(null)).isEqualTo(Outcome.applied());
    assertThat(live.get().travel()).isEqualTo(new Fixtures.Range(12, 20));
    assertThat(logged).last().isEqualTo("/Machine/travel/min,/Machine/travel/max=12.0,20.0");
  }

  @Test
  void thePageIsToldWhyTheConfigurationRefusedAChange() {
    var live = publishedMachine();

    var outcome = tee.requestChanges(Map.of("Machine/travel/min", 12));
    tee.update();

    // The range's check names no value, so it concerns the change's values in that record, and
    // the record, where it is in the configuration.
    assertThat(outcome.getNow(null))
        .isEqualTo(
            Outcome.refused(
                List.of(
                    new Outcome.Reason(
                        Outcome.Layer.RECORD,
                        "Fixtures.Range",
                        List.of("/Machine/travel/min"),
                        "machine",
                        "travel",
                        "min must be below max, got 12.0 and 10.0",
                        "machine: min must be below max, got 12.0 and 10.0"))));
    assertThat(live.get()).isEqualTo(Fixtures.machine());
  }

  @Test
  void thePageIsGivenEveryReasonARequestIsRefused() {
    var live = publishedMachine();
    Map<String, Object> values = new LinkedHashMap<>();
    values.put("Machine/drive/gains/kP", -1);
    values.put("Nowhere", 1);
    values.put("Machine/travel/min", 12);

    var outcome = tee.requestChanges(values);
    tee.update();

    assertThat(outcome.getNow(null).detail().lines())
        .containsExactly(
            "/Machine/drive/gains/kP: -1 is below its limit of 0",
            "There is no tunable the page can change at /Nowhere");
    assertThat(outcome.getNow(null).reasons())
        .extracting(Outcome.Reason::layer, Outcome.Reason::paths, Outcome.Reason::config)
        .containsExactly(
            tuple(Outcome.Layer.LIMIT, List.of("/Machine/drive/gains/kP"), "machine"),
            tuple(Outcome.Layer.VALUE, List.of("/Nowhere"), ""));
    assertThat(live.get()).isEqualTo(Fixtures.machine());
  }

  @Test
  void replayMakesAConfigurationsLoggedChangesTogether() {
    var replayDashboard = new MockTunableBackend();
    var replay =
        new TunableTee(
            replayDashboard,
            () -> null,
            inputs -> {
              inputs.paths = new String[] {"/Machine/travel/min", "/Machine/travel/max"};
              inputs.values = new String[] {"12.0", "20.0"};
            },
            true);
    var live =
        LiveConfig.of("machine", Fixtures.machine())
            .number("travel.min", Limits.NONE)
            .number("travel.max", Limits.NONE);
    TunableRegistry.registerBackend("", replay);
    TunableRegistry.publish("Machine", live);

    replay.update();

    assertThat(live.get().travel()).isEqualTo(new Fixtures.Range(12, 20));
  }

  /** Configuration values go back to committed on the field; they aren't switches. */
  @Test
  void aConfigurationsValuesAreNotSwitches() {
    publishedMachine();

    dashboard.setDouble("Machine/drive/gains/kP", 18);
    tee.update();

    assertThat(tee.changedSinceStartup()).isEmpty();
  }
}
