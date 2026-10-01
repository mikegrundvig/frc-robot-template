package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import com.michaelgrundvig.frc.pitwall.Notices.Notice;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;

/** What the robot tells whoever is at the page: shown only while it has something to say. */
class NoticesTest {
  private final AtomicReference<@Nullable Notice> said = new AtomicReference<>();
  private final Notices notices = new Notices().add("Coprocessors", said::get);

  @Test
  void aNoticeShowsOnlyWhileItSaysSomething() {
    notices.periodic(true);
    assertThat(notices.view()).isEmpty();

    said.set(new Notice("Safe to switch off", Devices.Level.OK));
    notices.periodic(true);

    assertThat(notices.view())
        .isEqualTo(Map.of("Coprocessors", Map.of("text", "Safe to switch off", "level", "ok")));

    said.set(new Notice(" ", Devices.Level.OK));
    notices.periodic(true);
    assertThat(notices.view()).isEmpty();
  }

  @Test
  void noticesAreReadOnlyWhileWatched() {
    said.set(new Notice("Safe to switch off", Devices.Level.OK));

    notices.periodic(false);

    assertThat(notices.view()).isEmpty();
  }

  @Test
  void aNoticeThatThrowsSaysWhat() {
    notices.add(
        "Broken",
        () -> {
          throw new IllegalStateException("no table");
        });

    notices.periodic(true);

    assertThat(notices.view())
        .containsEntry("Broken", Map.of("text", "Error: no table", "level", "warning"));
  }

  @Test
  void namesAreUnique() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> notices.add("Coprocessors", () -> null))
        .withMessage("two notices are named Coprocessors");
    assertThatIllegalArgumentException().isThrownBy(() -> notices.add(" ", () -> null));
  }
}
