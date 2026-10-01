package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

class PlacementTest {
  private final Place left = Place.at("Left", new Pose2d(1, 6, Rotation2d.ZERO));
  private final Place center = Place.at("Center", new Pose2d(1, 4, Rotation2d.ZERO));
  private final FieldModel field = TestFields.model(List.of(left, center), List.of(), List.of());

  private static final Optional<Alliance> RED = Optional.of(Alliance.RED);

  /** Red's center start: the image of blue's (1, 4, 0°). */
  private static Pose2d redCenter(double dx, double dy, double degrees) {
    return new Pose2d(15 + dx, 4 + dy, Rotation2d.fromDegrees(180 + degrees));
  }

  private Placement check(Pose2d pose, @Nullable Place expected) {
    return Placement.check(field, RED, pose, expected, 0.10, 5);
  }

  @Test
  void onTheExpectedStartItIsReady() {
    Placement placement = check(redCenter(0.02, 0, 1), center);

    assertThat(placement.status()).isEqualTo(Placement.Status.READY);
    assertThat(placement.start()).isEqualTo("Center");
    assertThat(placement.describe()).isEqualTo("Ready on Center: 2 cm, 1° off");
  }

  @Test
  void onAnotherStartItSaysWhichOne() {
    Placement placement = check(redCenter(0, -2, 0), center); // red's left is toward -y

    assertThat(placement.status()).isEqualTo(Placement.Status.WRONG_START);
    assertThat(placement.describe()).isEqualTo("On Left, not the expected start");
  }

  @Test
  void offEveryStartItPointsToTheExpectedOne() {
    Placement placement = check(redCenter(-1.0, -1.5, 12), center);

    assertThat(placement.status()).isEqualTo(Placement.Status.NOT_ON_A_START);
    assertThat(placement.describe()).isEqualTo("Not on a start: Center is 1.80 m and 12° away");
  }

  @Test
  void turnedTooFarItIsNotOnTheStart() {
    assertThat(check(redCenter(0, 0, 10), center).status())
        .isEqualTo(Placement.Status.NOT_ON_A_START);
  }

  @Test
  void withNothingExpectedAnyStartWillDoAndTheNearestIsNamed() {
    assertThat(check(redCenter(0, -2, 0), null).describe()).startsWith("Ready on Left");
    assertThat(check(redCenter(-1, -1.5, 0), null).describe())
        .isEqualTo("Not on a start: Left is 1.12 m and 0° away");
  }

  @Test
  void onTheBlueAllianceTheStartsAreBlues() {
    Placement placement =
        Placement.check(
            field, Optional.of(Alliance.BLUE), new Pose2d(1, 4, Rotation2d.ZERO), center, 0.1, 5);

    assertThat(placement.status()).isEqualTo(Placement.Status.READY);
  }

  @Test
  void untilTheAllianceIsKnownNeitherAreTheStarts() {
    Placement placement =
        Placement.check(field, Optional.empty(), redCenter(0, 0, 0), center, 0.1, 5);

    assertThat(placement.status()).isEqualTo(Placement.Status.ALLIANCE_UNKNOWN);
    assertThat(placement.describe()).isEqualTo("Alliance unknown");
  }

  @Test
  void aFieldWithoutStartsSaysSo() {
    Placement placement =
        Placement.check(TestFields.model(), RED, redCenter(0, 0, 0), null, 0.1, 5);

    assertThat(placement.describe()).isEqualTo("No start positions");
  }
}
