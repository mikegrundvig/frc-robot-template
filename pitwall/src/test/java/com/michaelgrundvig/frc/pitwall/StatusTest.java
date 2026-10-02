package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import com.michaelgrundvig.frc.pitwall.push.Reading;
import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;

class StatusTest {
  @Test
  void showsEachItemAsOfTheLastLoopInTheOrderAdded() {
    var homed = new AtomicBoolean(false);
    var status =
        new Status()
            .add("Robot", () -> "practice")
            .add("Arm homed", homed::get)
            .add("Battery", () -> 12.4);

    assertThat(status.view()).as("nothing read before the first loop").isEmpty();

    status.periodic(true);
    homed.set(true);

    assertThat(status.view())
        .containsExactly(
            new Status.Item("Robot", "practice"),
            new Status.Item("Arm homed", false),
            new Status.Item("Battery", 12.4));

    status.periodic(true);

    assertThat(status.view()).contains(new Status.Item("Arm homed", true));
  }

  @Test
  void anythingElseShowsAsText() {
    var status = new Status().add("Pose", () -> new StringBuilder("(3.0, 3.0)"));

    status.periodic(true);

    assertThat(status.view()).containsExactly(new Status.Item("Pose", "(3.0, 3.0)"));
  }

  @Test
  void anItemThatThrowsShowsWhatItThrew() {
    var status =
        new Status()
            .add(
                "Camera",
                () -> {
                  throw new IllegalStateException("not connected");
                })
            .add(
                "Gyro",
                () -> {
                  throw new UnsupportedOperationException();
                })
            .add("Robot", () -> "practice");

    status.periodic(true);

    assertThat(status.view())
        .containsExactly(
            new Status.Item("Camera", "Error: not connected"),
            new Status.Item("Gyro", "Error: java.lang.UnsupportedOperationException"),
            new Status.Item("Robot", "practice"));
  }

  /** Nobody looking, nothing read: the robot does as little as it can (audit PERF-003). */
  @Test
  void itemsAreReadOnlyWhileAPageWatchesOrARequestAsks() {
    var reads = new AtomicInteger();
    var status = new Status().add("Robot", () -> "practice" + reads.incrementAndGet());

    for (int i = 0; i < 10; i++) {
      status.periodic(false);
    }
    assertThat(reads).hasValue(0);
    assertThat(status.view()).isEmpty();

    var asked = status.requestRead();
    status.periodic(false);

    assertThat(asked.getNow(null)).containsExactly(new Status.Item("Robot", "practice1"));
    status.periodic(false);
    assertThat(reads).as("only the loop it was asked").hasValue(1);

    var first = status.requestRead();
    var second = status.requestRead();
    status.periodic(false);

    assertThat(second).as("however many ask, one read").isSameAs(first);
    assertThat(reads).hasValue(2);
  }

  @Test
  void aNumberWithAResolutionIsShownToItWithItsUnitAndStreamedAsAReading() {
    var status = new Status().add("Battery", () -> 12.34, "V", 0.1).add("Robot", () -> "practice");

    status.periodic(true);

    assertThat(status.view()).contains(new Status.Item("Battery", "12.3 V"));
    assertThat(status.values().get("Battery")).isInstanceOf(Reading.class);
    assertThat(status.values()).containsEntry("Robot", "practice");
    assertThat(status.units()).isEqualTo(Map.of("Battery", Map.of("unit", "V", "resolution", 0.1)));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> status.add("Pressure", () -> 1, "psi", 0));
  }

  /**
   * A line that moves faster than anyone reads it is read at most once an interval, so it changes
   * at most that often, and the loop doesn't make its text in between.
   */
  @Test
  void anItemWithAnIntervalIsReadAtMostOnceAnInterval() {
    var clock = new AtomicLong();
    var frames = new AtomicInteger();
    var reads = new AtomicInteger();
    var status =
        new Status(clock::get)
            .add(
                "Seen",
                () -> {
                  reads.incrementAndGet();
                  return frames.get() + " frames";
                },
                Duration.ofSeconds(1));
    List<Object> shown = new ArrayList<>();

    for (int loop = 0; loop < 150; loop++) {
      frames.incrementAndGet();
      status.periodic(true);
      shown.add(status.view().get(0).value());
      clock.addAndGet(Duration.ofMillis(20).toNanos());
    }

    assertThat(shown.stream().distinct()).as("three seconds: three changes").hasSize(3);
    assertThat(shown.get(49)).as("held a second").isEqualTo("1 frames");
    assertThat(shown.get(50)).as("then the latest").isEqualTo("51 frames");
    assertThat(reads).as("read once a second, not every loop").hasValue(3);
    assertThat(status.values()).containsEntry("Seen", shown.get(149));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> status.add("Other", () -> 1, Duration.ofSeconds(-1)));
  }

  @Test
  void everyItemHasItsOwnName() {
    var status = new Status().add("Robot", () -> "practice");

    assertThatIllegalArgumentException()
        .isThrownBy(() -> status.add("Robot", () -> "other"))
        .withMessage("two status items are named Robot");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> status.add(" ", () -> "blank"))
        .withMessage("a status item needs a name");
  }
}
