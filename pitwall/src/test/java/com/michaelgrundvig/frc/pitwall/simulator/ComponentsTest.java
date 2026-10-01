package com.michaelgrundvig.frc.pitwall.simulator;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import com.michaelgrundvig.frc.pitwall.Limits;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import org.assertj.core.api.InstanceOfAssertFactories;
import org.junit.jupiter.api.Test;

/**
 * The registry of what can fail in a simulated robot: declared by the robot's simulation, applied
 * and undone by name, each fault's values checked against its ranges, and scenarios as sets of
 * faults. The robot here is a lamp and a fan.
 */
class ComponentsTest {
  /** A simulated lamp: on or off, and how bright. */
  private boolean unplugged;

  private double brightness = 1;
  private int flickers;
  private final List<String> said = new ArrayList<>();
  private final Components components = new Components(said::add);

  ComponentsTest() {
    components
        .add(
            Component.of(
                "Lamp",
                "light",
                Fault.lasting("Unplugged")
                    .does("No light at all.")
                    .isOn(() -> unplugged)
                    .apply(values -> unplugged = true)
                    .undo("Plugs it back in.", () -> unplugged = false)
                    .build(),
                Fault.lasting("Dimmed")
                    .does("Only this bright.")
                    .parameter("Brightness", "", Limits.between(0, 1), 0.5)
                    .isOn(() -> brightness < 1)
                    .apply(values -> brightness = values.get("Brightness"))
                    .undo("Full brightness.", () -> brightness = 1)
                    .build(),
                Fault.event("Flickers")
                    .does("Flickers this many times.")
                    .parameter("Times", "", Limits.between(1, 10), 3)
                    .apply(values -> flickers += (int) values.get("Times"))
                    .undoneBy("It's over by the time anyone looks.")
                    .build()))
        .add(
            Component.of(
                "Fan",
                "fan",
                Fault.event("Jams")
                    .does("Stops dead.")
                    .apply(
                        values -> {
                          throw new IllegalStateException("the fan has no motor");
                        })
                    .undoneBy("Nothing.")
                    .build()));
  }

  @Test
  void aLastingFaultIsOnUntilUndoneAndTheWorldSaysSo() {
    assertThat(components.apply(Setting.of("Lamp", "Unplugged"))).isNull();
    assertThat(unplugged).isTrue();
    assertThat(components.on()).containsExactly("Lamp: Unplugged");

    assertThat(components.undo("Lamp", "Unplugged")).isNull();
    assertThat(unplugged).isFalse();
    assertThat(components.on()).isEmpty();
    assertThat(said)
        .containsExactly("Lamp: Unplugged, on", "Lamp: Unplugged, undone: Plugs it back in.");
  }

  /** On or off is the world's to say: a fault turned off some other way shows as off. */
  @Test
  void whetherAFaultIsOnIsReadFromTheWorld() {
    components.apply(Setting.of("Lamp", "Unplugged"));
    unplugged = false;

    assertThat(components.on()).isEmpty();
    assertThat(components.undo("Lamp", "Unplugged")).as("off already: nothing to do").isNull();
    assertThat(said).hasSize(1);
  }

  @Test
  void aParameterLeftOutTakesItsInitialValue() {
    components.apply(Setting.of("Lamp", "Dimmed"));
    assertThat(brightness).isEqualTo(0.5);

    components.apply(Setting.of("Lamp", "Dimmed", Map.of("Brightness", 0.2)));
    assertThat(brightness).isEqualTo(0.2);
  }

  /**
   * A lasting fault applied with the part's values as built changes nothing: refused, never on, and
   * not said. Among a scenario's faults it's passed over.
   */
  @Test
  void aLastingFaultThatChangesNothingIsRefused() {
    String refused = components.apply(Setting.of("Lamp", "Dimmed", Map.of("Brightness", 1.0)));

    assertThat(refused).isEqualTo("Lamp: Dimmed changes nothing: as built");
    assertThat(components.on()).isEmpty();
    assertThat(said).isEmpty();
    assertThat(
            components.apply(
                List.of(
                    Setting.of("Lamp", "Dimmed", Map.of("Brightness", 1.0)),
                    Setting.of("Lamp", "Unplugged"))))
        .as("passed over among others")
        .isNull();
    assertThat(components.on()).containsExactly("Lamp: Unplugged");
  }

  /** Undone, a fault's values are forgotten: turned on again some other way, it shows none. */
  @Test
  void undoingAFaultForgetsTheValuesItWasAppliedWith() {
    components.apply(Setting.of("Lamp", "Dimmed", Map.of("Brightness", 0.25)));
    components.undo("Lamp", "Dimmed");
    brightness = 0.4; // dimmed some other way

    assertThat(dimmed()).containsEntry("on", true).containsEntry("values", Map.of());
  }

  @SuppressWarnings("unchecked") // the view's shape, as documented
  private Map<String, Object> dimmed() {
    Map<String, Object> lamp =
        (Map<String, Object>) Objects.requireNonNull(components.view().get("Lamp"));
    Map<String, Object> faults = (Map<String, Object>) Objects.requireNonNull(lamp.get("faults"));
    return (Map<String, Object>) Objects.requireNonNull(faults.get("Dimmed"));
  }

  @Test
  void anEventHappensOnceAndCantBeUndone() {
    assertThat(components.apply(Setting.of("Lamp", "Flickers", Map.of("Times", 2.0)))).isNull();

    assertThat(flickers).isEqualTo(2);
    assertThat(components.on()).isEmpty();
    assertThat(components.undo("Lamp", "Flickers"))
        .isEqualTo(
            "Lamp: Flickers happens once, and nothing undoes it: It's over by the time anyone"
                + " looks.");
    assertThat(said).containsExactly("Lamp: Flickers: Times 2");
  }

  @Test
  void aNameOrValueThatIsntThereIsRefusedSayingWhatIs() {
    assertThat(components.whyNot(Setting.of("Lmap", "Unplugged")))
        .isEqualTo("There is no component named Lmap; there are Lamp, Fan");
    assertThat(components.whyNot(Setting.of("Lamp", "Unpluged")))
        .isEqualTo("Lamp has no fault named Unpluged; it has Unplugged, Dimmed, Flickers");
    assertThat(components.whyNot(Setting.of("Lamp", "Dimmed", Map.of("Brightness", 2.0))))
        .isEqualTo("Brightness: 2 is above its limit of 1");
    assertThat(components.whyNot(Setting.of("Lamp", "Dimmed", Map.of("Level", 0.5))))
        .isEqualTo("Dimmed has no parameter named Level; it takes Brightness");
    assertThat(components.whyNot(Setting.of("Lamp", "Unplugged", Map.of("Level", 0.5))))
        .isEqualTo("Unplugged has no parameter named Level; it takes none");
    assertThat(components.apply(Setting.of("Lamp", "Dimmed", Map.of("Brightness", -1.0))))
        .isEqualTo("Brightness: -1 is below its limit of 0");
    assertThat(brightness).as("nothing changed").isEqualTo(1);
  }

  /** A fault whose own code throws is refused, saying why, and the robot carries on. */
  @Test
  void aFaultThatFailsIsRefused() {
    assertThat(components.apply(Setting.of("Fan", "Jams")))
        .isEqualTo("Fan: Jams failed: the fan has no motor");
  }

  @Test
  void undoingEverythingUndoesEachLastingFaultThatsOn() {
    components.apply(Setting.of("Lamp", "Unplugged"));
    components.apply(Setting.of("Lamp", "Dimmed"));

    assertThat(components.undoAll()).isNull();

    assertThat(unplugged).isFalse();
    assertThat(brightness).isEqualTo(1);
  }

  @Test
  void aScenarioAppliesItsFaultsInOrderAndSaysWhichFailed() {
    components
        .scenario(
            "blackout",
            "The lamp goes out.",
            List.of(Setting.of("Lamp", "Flickers"), Setting.of("Lamp", "Unplugged")))
        .scenario("broken", "The fan jams.", List.of(Setting.of("Fan", "Jams")));

    assertThat(components.hasScenario("blackout")).isTrue();
    assertThat(components.applyScenario("blackout")).isNull();
    assertThat(flickers).isEqualTo(3);
    assertThat(unplugged).isTrue();
    assertThat(components.applyScenario("broken"))
        .isEqualTo("broken: Fan: Jams failed: the fan has no motor");
    assertThat(components.applyScenario("dark"))
        .isEqualTo("There is no scenario named dark; there are blackout, broken");
    assertThat(said).first().isEqualTo("scenario blackout: The lamp goes out.");
  }

  /** A scenario made of faults that aren't there is refused as the robot starts. */
  @Test
  void aScenarioMustBeMadeOfDeclaredFaults() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                components.scenario("typo", "Misspelled.", List.of(Setting.of("Lamp", "Unpluged"))))
        .withMessage(
            "the scenario typo: Lamp has no fault named Unpluged; it has Unplugged, Dimmed,"
                + " Flickers");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> components.add(Component.of("Lamp", "light")))
        .withMessage("two components are named Lamp");
  }

  /** The page's view: each component's faults, what they do, their ranges, and what's on. */
  @Test
  void thePageSeesEachFaultWithItsRangesAndWhetherItsOn() {
    components.apply(Setting.of("Lamp", "Dimmed", Map.of("Brightness", 0.25)));

    Map<String, Object> view = components.view();

    assertThat(view).containsOnlyKeys("Lamp", "Fan");
    @SuppressWarnings("unchecked") // the view's shape, as documented
    Map<String, Object> faults =
        (Map<String, Object>)
            Objects.requireNonNull((Map<String, Object>) view.get("Lamp")).get("faults");
    assertThat(Objects.requireNonNull(faults).get("Dimmed"))
        .isEqualTo(
            Map.of(
                "does",
                "Only this bright.",
                "lasting",
                true,
                "undo",
                "Full brightness.",
                "parameters",
                List.of(
                    Map.of(
                        "name", "Brightness",
                        "unit", "",
                        "min", 0.0,
                        "max", 1.0,
                        "initial", 0.5)),
                "on",
                true,
                "values",
                Map.of("Brightness", 0.25)));
    assertThat(faults.get("Flickers"))
        .asInstanceOf(InstanceOfAssertFactories.MAP)
        .doesNotContainKeys("on", "values")
        .containsEntry("lasting", false);
  }

  /** A component's view is rebuilt only when one of its faults turns on or off. */
  @Test
  void aComponentsViewIsRebuiltOnlyWhenItChanges() {
    Object before = components.view().get("Lamp");
    Object unchanged = components.view().get("Lamp");
    unplugged = true;
    Object changed = components.view().get("Lamp");

    assertThat(unchanged).isSameAs(before);
    assertThat(changed).isNotSameAs(before).isNotEqualTo(before);
  }

  @Test
  void aFaultSaysWhatItNeeds() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Fault.lasting("Off").does("Off.").apply(values -> {}).build())
        .withMessage("the fault Off needs how it's undone");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Fault.event("Bang")
                    .does("Bang.")
                    .apply(values -> {})
                    .isOn(() -> true)
                    .undoneBy("No.")
                    .build())
        .withMessage("the event Bang happens once: it isn't on or off, and nothing undoes it");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Fault.event("Bang").parameter("Loudness", "dB", Limits.between(0, 10), 20).build())
        .withMessage("Loudness starts outside its range: 20 is above its limit of 10");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> Fault.event("Bang").parameter("Loudness", "dB", Limits.atLeast(0), 20).build())
        .withMessage("Loudness needs a range with both ends, for the page to draw: at least 0");
  }
}
