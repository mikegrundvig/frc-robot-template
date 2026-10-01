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
            .link("Open PhotonVision", "http://10.0.0.11:5800")
            .download("settings", "Download settings backup", NO_FILE)
            .log(query -> new Devices.LogPage(List.of(), "", "")));

    devices.periodic(true);

    @SuppressWarnings("unchecked")
    Map<String, Object> shown =
        (Map<String, Object>) Objects.requireNonNull(devices.view().get("vision-front"));
    assertThat(shown)
        .containsEntry("kind", "Coprocessor")
        .containsEntry("state", "Present")
        .containsEntry("level", "ok")
        .containsEntry("log", true);
    assertThat(shown.get("lines"))
        .isEqualTo(List.of(Map.of("label", "Version", "value", "v1 (1)", "level", "ok")));
    assertThat(shown.get("links"))
        .isEqualTo(List.of(Map.of("label", "Open PhotonVision", "url", "http://10.0.0.11:5800")));
    assertThat(shown.get("downloads"))
        .isEqualTo(List.of(Map.of("id", "settings", "label", "Download settings backup")));
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
  void aCardLinksOnlyToAPage() {
    Device device = Device.named("vision-front", "Coprocessor", this::card);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> device.link("Run", "javascript:alert(1)"))
        .withMessageContaining("http or https");
    assertThatIllegalArgumentException().isThrownBy(() -> device.link("Odd", "not a link"));
    assertThatIllegalArgumentException().isThrownBy(() -> device.link("Odd", "http://a b"));
    device.link("Open", "https://example.org/");
    assertThat(device.links()).hasSize(1);
  }

  @Test
  void filesAndLogQueriesAreChecked() {
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
    assertThatIllegalArgumentException().isThrownBy(() -> Devices.LogQuery.latest(8, ""));
    assertThatIllegalArgumentException().isThrownBy(() -> Devices.LogQuery.latest(-1, ""));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Devices.LogQuery.latest(3, "photon vision"));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new Devices.LogQuery(Devices.From.AFTER, "a b", 3, ""));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new Devices.LogQuery(Devices.From.BEFORE, "", 3, ""))
        .withMessageContaining("needs one");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new Devices.LogQuery(Devices.From.START, "s=1", 3, ""))
        .withMessageContaining("takes none");
    assertThat(
            new Devices.LogQuery(Devices.From.AFTER, "s=1;i=2", 3, "photonvision.service")
                .priority())
        .isEqualTo(3);
  }

  /** Once on the page, the server's threads read a device: it can't change after. */
  @Test
  void aDeviceIsDescribedWholeBeforeItsAdded() {
    Device device = Device.named("vision-front", "Coprocessor", this::card);
    devices.add(device);

    assertThatThrownBy(() -> device.link("Open", "http://10.0.0.11:5800"))
        .isInstanceOf(IllegalStateException.class)
        .hasMessageContaining("describe a device whole");
    assertThatThrownBy(() -> device.download("settings", "Settings", NO_FILE))
        .isInstanceOf(IllegalStateException.class);
    assertThatThrownBy(() -> device.log(query -> new Devices.LogPage(List.of(), "", "")))
        .isInstanceOf(IllegalStateException.class);
  }
}
