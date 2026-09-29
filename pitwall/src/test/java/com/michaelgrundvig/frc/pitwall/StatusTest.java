package com.michaelgrundvig.frc.pitwall;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import java.util.concurrent.atomic.AtomicBoolean;
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

    status.periodic();
    homed.set(true);

    assertThat(status.view())
        .containsExactly(
            new Status.Item("Robot", "practice"),
            new Status.Item("Arm homed", false),
            new Status.Item("Battery", 12.4));

    status.periodic();

    assertThat(status.view()).contains(new Status.Item("Arm homed", true));
  }

  @Test
  void anythingElseShowsAsText() {
    var status = new Status().add("Pose", () -> new StringBuilder("(3.0, 3.0)"));

    status.periodic();

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

    status.periodic();

    assertThat(status.view())
        .containsExactly(
            new Status.Item("Camera", "Error: not connected"),
            new Status.Item("Gyro", "Error: java.lang.UnsupportedOperationException"),
            new Status.Item("Robot", "practice"));
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
