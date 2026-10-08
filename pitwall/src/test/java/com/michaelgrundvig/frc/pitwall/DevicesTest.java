package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.pitwall.Devices.Card;
import com.michaelgrundvig.frc.pitwall.Devices.Device;
import com.michaelgrundvig.frc.pitwall.Devices.Level;
import com.michaelgrundvig.frc.pitwall.Devices.Line;
import java.time.Duration;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;

/** The devices' cards as the robot loop reads them, and what a device may be. */
class DevicesTest {
  private final AtomicLong clock = new AtomicLong();
  private final Devices devices = new Devices(clock::get, Duration.ofSeconds(1));
  private final AtomicInteger reads = new AtomicInteger();
  private static final Devices.Fetch NO_FILE =
      () -> Devices.File.of("empty.zip", "application/zip", new byte[0]);

  private Card card() {
    reads.incrementAndGet();
    return new Card(
        "Present", Level.OK, List.of(new Line("Version", "v1 (" + reads + ")", Level.OK)));
  }

  @Test
  void cardsAreReadOnlyWhileWatched() {
    devices.add(Device.named("vision-front", "Coprocessor", this::card));

    devices.periodic(false);

    assertThat(reads).hasValue(0);
    assertThat(devices.view()).isEmpty();
  }

  @Test
  void cardsAreReadAtMostOnceAnInterval() {
    devices.add(Device.named("vision-front", "Coprocessor", this::card));

    devices.periodic(true);
    clock.addAndGet(500_000_000L);
    devices.periodic(true);

    assertThat(reads).hasValue(1);
    clock.addAndGet(500_000_000L);
    devices.periodic(true);
    assertThat(reads).hasValue(2);
  }

  @Test
  void theViewHoldsWhatTheCardSaysAndWhatCanBeDone() {
    devices.add(
        Device.named("vision-front", "Coprocessor", this::card)
            .download("settings", "Download settings backup", NO_FILE)
            .upload("survey", "Upload venue", body -> "Saved"));

    devices.periodic(true);

    @SuppressWarnings("unchecked")
    Map<String, Object> shown =
        (Map<String, Object>) Objects.requireNonNull(devices.view().get("vision-front"));
    assertThat(shown)
        .containsEntry("kind", "Coprocessor")
        .containsEntry("state", "Present")
        .containsEntry("level", "ok");
    assertThat(shown.get("lines"))
        .isEqualTo(List.of(Map.of("label", "Version", "value", "v1 (1)", "level", "ok")));
    assertThat(shown.get("downloads"))
        .isEqualTo(List.of(Map.of("id", "settings", "label", "Download settings backup")));
    assertThat(shown.get("uploads"))
        .isEqualTo(List.of(Map.of("id", "survey", "label", "Upload venue")));
  }

  @Test
  void aCardThatThrowsSaysWhyRatherThanStopTheRobot() {
    devices.add(
        Device.named(
            "vision-front",
            "Coprocessor",
            () -> {
              throw new IllegalStateException("no table");
            }));

    devices.periodic(true);

    @SuppressWarnings("unchecked")
    Map<String, Object> shown =
        (Map<String, Object>) Objects.requireNonNull(devices.view().get("vision-front"));
    assertThat(shown).containsEntry("state", "Unknown").containsEntry("level", "warning");
    assertThat(String.valueOf(shown.get("lines"))).contains("Error: no table");
  }

  @Test
  void aDeviceAddedLaterIsReadAtTheNextChance() {
    devices.add(Device.named("vision-front", "Coprocessor", this::card));
    devices.periodic(true);

    devices.add(Device.named("vision-back", "Coprocessor", this::card));
    devices.periodic(true);

    assertThat(devices.view()).containsOnlyKeys("vision-front", "vision-back");
  }

  @Test
  void namesAreUniqueAndFitAnAddress() {
    devices.add(Device.named("vision-front", "Coprocessor", this::card));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> devices.add(Device.named("vision-front", "Coprocessor", this::card)))
        .withMessageContaining("two devices");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Device.named("Vision Front", "Coprocessor", this::card));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Device.named("vision-back", "Coprocessor", this::card)
                    .download("settings", "One", NO_FILE)
                    .download("settings", "Two", NO_FILE))
        .withMessageContaining("two downloads");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Device.named("vision-back", "Coprocessor", this::card)
                    .download("../etc", "Bad", NO_FILE));
    assertThat(devices.named("vision-front")).isPresent();
    assertThat(devices.named("nothing")).isEmpty();
    assertThat(devices.all()).hasSize(1);
  }

  @Test
  void filesAreChecked() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Devices.File.of("../passwd", "text/plain", new byte[0]));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Devices.File.of(".hidden", "text/plain", new byte[0]));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Devices.File.of("a.txt", "text/html\r\nX: y", new byte[0]));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                new Devices.File(
                    "a.txt", "text/plain", -2, new java.io.ByteArrayInputStream(new byte[0])));
  }

  /** Once on the page, the server's threads read a device: it can't change after. */
  @Test
  void aDeviceIsDescribedWholeBeforeItsAdded() {
    Device device = Device.named("vision-front", "Coprocessor", this::card);
    devices.add(device);

    assertThatThrownBy(() -> device.download("settings", "Settings", NO_FILE))
        .isInstanceOf(IllegalStateException.class)
        .hasMessageContaining("describe a device whole");
    assertThatThrownBy(() -> device.upload("survey", "Upload", body -> "Saved"))
        .isInstanceOf(IllegalStateException.class);
    assertThatThrownBy(() -> device.choose("venue", value -> "Chose"))
        .isInstanceOf(IllegalStateException.class);
  }
}
