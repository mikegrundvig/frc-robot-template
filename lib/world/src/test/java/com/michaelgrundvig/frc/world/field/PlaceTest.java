package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.assertPose;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

class PlaceTest {
  private final FieldModel field = TestFields.model();

  @Test
  void aPlaceWrittenForBlueIsItsImageForRed() {
    Place start = Place.at("Start", new Pose2d(1, 2, Rotation2d.ZERO));

    assertPose(start.on(field, Alliance.BLUE), 1, 2, 0);
    assertPose(start.on(field, Alliance.RED), 15, 6, 180);
  }

  @Test
  void whereTheFieldIsNotSymmetricRedsIsWrittenToo() {
    Place spot =
        Place.at("Spot", new Pose2d(1, 2, Rotation2d.ZERO), new Pose2d(14, 1, Rotation2d.PI));

    assertPose(spot.on(field, Alliance.BLUE), 1, 2, 0);
    assertPose(spot.on(field, Alliance.RED), 14, 1, 180);
  }

  @Test
  void betweenTagsIsMidwayBetweenEachAlliancesOwnTags() {
    Place hub = Place.between("Hub", 2, 3);

    assertPose(hub.on(field, Alliance.BLUE), 4.5, 4, 0);
    assertPose(hub.on(field, Alliance.RED), 11.5, 4, 180);
  }

  /**
   * Fields differ from the published layout by centimeters, so events are measured. Each alliance's
   * places come from its own tags, so the measurement corrects both, and flipping blue's numbers
   * would have missed red's.
   */
  @Test
  void aMeasuredLayoutMovesEachAlliancesPlacesWithItsOwnTags() {
    List<FieldTag> measured = TestFields.tags();
    measured.set(2, TestFields.tag(12, 12.04, 4, 0)); // red's far face is 4 cm further out
    var model =
        new FieldModel(
            TestFields.layout(measured), Symmetry.ROTATED, List.of(), List.of(), List.of());
    Place hub = Place.between("Hub", 2, 3);

    assertPose(hub.on(model, Alliance.RED), 11.52, 4, 180);
    assertPose(hub.on(model, Alliance.BLUE), 4.5, 4, 0);
  }

  @Test
  void facingATagStandsOutInFrontOfItAndFacesIt() {
    Place loading = Place.facing("Loading", 4, 1.5);

    assertPose(loading.on(field, Alliance.BLUE), 1.5, 2, 180);
    assertPose(loading.on(field, Alliance.RED), 14.5, 6, 0);
  }

  /** A task lining up on a place trusts the tags it's found from: each alliance's own. */
  @Test
  void aPlaceNamesTheTagsItIsFoundFrom() {
    assertThat(Place.between("Hub", 2, 3).tags(field, Alliance.BLUE)).containsExactly(2, 3);
    assertThat(Place.between("Hub", 2, 3).tags(field, Alliance.RED)).containsExactly(12, 13);
    assertThat(Place.facing("Loading", 4, 1.5).tags(field, Alliance.RED)).containsExactly(14);
    assertThat(Place.at("Start", Pose2d.ZERO).tags(field, Alliance.BLUE)).isEmpty();
  }

  @Test
  void aPlaceNeedsANameAndSomethingToStandOn() {
    assertThatThrownBy(() -> Place.at(" ", Pose2d.ZERO)).hasMessage("a place needs a name");
    assertThatThrownBy(() -> Place.between("Hub")).hasMessage("Hub needs at least one tag");
    assertThatThrownBy(() -> Place.facing("Loading", 4, -1))
        .hasMessage("Loading: the distance must be zero or more");
  }
}
