package com.michaelgrundvig.frc.pitwall;

import static com.michaelgrundvig.frc.pitwall.Fixtures.machine;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.pitwall.experiment.ByHand;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;

/** The page's state, section by section, as the robot loop builds it for the event stream. */
class PageTest {
  private final AtomicReference<String> mode = new AtomicReference<>("Teleop, disabled");
  private final Status status = new Status().add("Robot", () -> "test");
  private final Actions actions = new Actions(inputs -> {});
  private final TunableTee tunables =
      new TunableTee(new MockTunableBackend(), this::whyLocked, inputs -> {}, false);
  private final Tuning tuning =
      new Tuning("test", tunables::configs, tunables::pathOf, true, inputs -> {}, view -> {});
  private final ByHand byHand = new ByHand(System::nanoTime);
  private final Page page =
      new Page(status, actions, tuning, tunables, byHand.experiments(), mode::get);

  PageTest() {
    TunableRegistry.registerBackend("", tunables);
    TunableRegistry.publish(
        "Machine",
        LiveConfig.of("machine", machine())
            .number("drive.gains.kP", Limits.between(0, 100))
            .number("travel.max", Limits.atLeast(1)));
  }

  private @Nullable String whyLocked() {
    return tuning.whyLocked();
  }

  @AfterEach
  void reset() {
    TunableRegistry.reset();
  }

  /** The state bar says the mode on every view; read only while a page is open, as it's built. */
  @Test
  void theRobotSectionSaysItsMode() {
    assertThat(page.sections().get("robot")).containsEntry("mode", "Teleop, disabled");

    mode.set("Utility, enabled");

    assertThat(page.sections().get("robot")).containsEntry("mode", "Utility, enabled");
  }

  /**
   * A section that hasn't changed is handed over as the same map, so comparing it with what was
   * sent stops there; one that has is built again.
   */
  @Test
  void aSectionThatHasntChangedIsTheSameMap() {
    actions.add(
        "Home",
        new Actions.Routine() {
          @Override
          public @Nullable String start() {
            return null;
          }

          @Override
          public boolean isRunning() {
            return false;
          }

          @Override
          public void cancel() {}
        });
    actions.periodic(new Actions.Gate(true, true, false));
    tunables.update();
    Map<String, Map<String, Object>> first = page.sections();

    actions.periodic(new Actions.Gate(true, true, false));
    tunables.update();
    Map<String, Map<String, Object>> again = page.sections();
    for (String section : new String[] {"robot", "units", "actions", "tuning", "wpilibTunables"}) {
      assertThat(again.get(section)).as(section).isSameAs(first.get(section));
    }

    mode.set("Utility, enabled");
    actions.periodic(new Actions.Gate(false, true, false));
    Map<String, Map<String, Object>> changed = page.sections();
    assertThat(changed.get("robot")).isNotSameAs(first.get("robot"));
    assertThat(changed.get("actions")).isNotSameAs(first.get("actions"));
    assertThat(changed.get("robot")).containsEntry("mode", "Utility, enabled");
  }

  /** The page draws a value's limits and checks an edit against them before sending it. */
  @Test
  void eachValueCarriesItsLimitsAsNumbersAndAnOpenEndHasNone() {
    Map<String, Object> tunables = Objects.requireNonNull(page.sections().get("tunables"));

    assertThat(entry(tunables, "/Machine/drive/gains/kP"))
        .containsEntry("limits", "0 to 100")
        .containsEntry("min", 0.0)
        .containsEntry("max", 100.0);
    assertThat(entry(tunables, "/Machine/travel/max"))
        .containsEntry("limits", "at least 1")
        .containsEntry("min", 1.0)
        .doesNotContainKey("max");
  }

  /** Run is off beside its reason: the gate's own, as the robot would refuse a start with it. */
  @Test
  void theRobotSectionSaysWhyNothingCanRunNow() {
    actions.periodic(new Actions.Gate(true, false, false));
    byHand.experiments().periodic(new Actions.Gate(true, false, false));

    assertThat(page.sections().get("robot"))
        .containsEntry("actionsLocked", "actions run only with the robot enabled in utility mode")
        .containsEntry(
            "experimentsLocked", "experiments run only with the robot enabled in utility mode");

    actions.periodic(new Actions.Gate(true, true, false));
    byHand.experiments().periodic(new Actions.Gate(true, true, false));

    assertThat(page.sections().get("robot"))
        .containsEntry("actionsLocked", "")
        .containsEntry("experimentsLocked", "");

    actions.periodic(new Actions.Gate(true, true, true));

    assertThat(page.sections().get("robot"))
        .containsEntry("actionsLocked", "the field system is connected");
  }

  @Test
  void aPageThatDoesntKnowTheModeSaysNothing() {
    Page unknowing = new Page(status, actions, tuning, tunables, byHand.experiments());

    assertThat(unknowing.sections().get("robot")).containsEntry("mode", "");
  }

  @SuppressWarnings("unchecked") // a section's entry is a map, as the page reads it
  private static Map<String, Object> entry(Map<String, Object> section, String key) {
    return (Map<String, Object>) Objects.requireNonNull(section.get(key));
  }
}
