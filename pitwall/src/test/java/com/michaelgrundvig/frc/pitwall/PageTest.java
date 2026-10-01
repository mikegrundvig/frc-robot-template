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
