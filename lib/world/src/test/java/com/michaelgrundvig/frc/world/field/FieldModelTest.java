package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.assertPose;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;

class FieldModelTest {
  /** A square opening, a meter on a side. */
  private final FieldModel field = TestFields.model();

  @Test
  void itKnowsTheLayoutsSizeAndTags() {
    assertThat(field.name()).isEqualTo("Test field");
    assertThat(field.length()).isEqualTo(16);
    assertThat(field.width()).isEqualTo(8);
    assertThat(field.symmetry()).isEqualTo(Symmetry.ROTATED);
    assertThat(field.tags()).containsOnlyKeys(2, 3, 12, 13, 4, 14);
    assertThat(field.tag(4).orElseThrow().getY()).isEqualTo(2);
    assertThat(field.tag(99)).isEmpty();
  }

  @Test
  void blueIsAsWrittenAndRedIsItsImage() {
    Pose2d blue = new Pose2d(1, 2, Rotation2d.fromDegrees(30));

    assertThat(field.forAlliance(Alliance.BLUE, blue)).isEqualTo(blue);
    assertPose(field.forAlliance(Alliance.RED, blue), 15, 6, 210);
    assertThat(field.forAlliance(Alliance.RED, new Translation2d(1, 2)))
        .isEqualTo(new Translation2d(15, 6));
  }

  @Test
  void anAreaTurnsWithTheField() {
    Polygon2d blue = Polygon2d.rectangle(new Translation2d(0, 0), new Translation2d(2, 3));

    Polygon2d red = field.forAlliance(Alliance.RED, blue);

    assertThat(field.forAlliance(Alliance.BLUE, blue)).isSameAs(blue);
    assertThat(red.contains(new Translation2d(15, 6))).isTrue();
    assertThat(red.contains(new Translation2d(1, 2))).isFalse();
  }

  @Test
  void driversFaceEachOtherWhateverTheSymmetry() {
    assertThat(FieldModel.driverForward(Alliance.BLUE)).isEqualTo(Rotation2d.ZERO);
    assertThat(FieldModel.driverForward(Alliance.RED)).isEqualTo(Rotation2d.PI);
  }

  @Test
  void eachTagIsPairedWithTheTagTheSymmetryPutsInItsPlace() {
    assertThat(field.tagFor(Alliance.BLUE, 2)).isEqualTo(2);
    assertThat(field.tagFor(Alliance.RED, 2)).isEqualTo(12);
    assertThat(field.tagFor(Alliance.RED, 3)).isEqualTo(13);
    assertThat(field.tagFor(Alliance.RED, 12)).as("and back").isEqualTo(2);
    assertThat(field.tagFor(Alliance.RED, 4)).isEqualTo(14);
  }

  /**
   * On 2026's field, trench tags stand back to back, 7.5 cm apart: only one faces the right way.
   */
  @Test
  void aTagFacingAwayIsNotAPartnerHoweverClose() {
    List<FieldTag> tags = TestFields.tags();
    tags.add(TestFields.tag(15, 12.05, 4, 180)); // right behind 12, facing the other way
    var model =
        new FieldModel(TestFields.layout(tags), Symmetry.ROTATED, List.of(), List.of(), List.of());

    assertThat(model.tagFor(Alliance.RED, 2)).isEqualTo(12);
  }

  @Test
  void aTagWithoutAPartnerServesBlueButNotRed() {
    List<FieldTag> tags = TestFields.tags();
    tags.add(TestFields.tag(7, 6, 1, 90));
    var model =
        new FieldModel(TestFields.layout(tags), Symmetry.ROTATED, List.of(), List.of(), List.of());

    assertThat(model.tagFor(Alliance.BLUE, 7)).isEqualTo(7);
    assertThatThrownBy(() -> model.tagFor(Alliance.RED, 7))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining("tag 7 has no partner");
    assertThatThrownBy(() -> model.tagFor(Alliance.BLUE, 99))
        .hasMessageContaining("the layout has no tag 99");
  }

  @Test
  void theSymmetryErrorShowsWhetherTheLayoutHasTheDeclaredSymmetry() {
    var mirrored =
        new FieldModel(TestFields.layout(), Symmetry.MIRRORED, List.of(), List.of(), List.of());

    assertThat(field.symmetryError()).isLessThan(1e-9);
    assertThat(mirrored.symmetryError()).as("the wrong symmetry").isGreaterThan(1);
  }

  @Test
  void aPlaceThatDoesNotFitTheLayoutFailsWhenTheModelIsBuilt() {
    assertThatThrownBy(
            () -> TestFields.model(List.of(), List.of(Place.between("Goal", 77)), List.of()))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("Goal: the layout has no tag 77");
  }

  @Test
  void namesAreUnique() {
    Place start = Place.at("Center", Pose2d.ZERO);
    Zone zone = Zone.ours("Zone", Polygon2d.rectangle(new Pose2d(1, 1, Rotation2d.ZERO), 1, 1));

    assertThatThrownBy(() -> TestFields.model(List.of(start), List.of(start), List.of()))
        .hasMessage("two places are named Center");
    assertThatThrownBy(() -> TestFields.model(List.of(), List.of(), List.of(zone, zone)))
        .hasMessage("two zones are named Zone");
  }

  @Test
  void aZoneOffTheFieldFails() {
    Zone zone =
        Zone.ours("Parking lot", Polygon2d.rectangle(new Pose2d(-5, 4, Rotation2d.ZERO), 1, 1));

    assertThatThrownBy(() -> TestFields.model(List.of(), List.of(), List.of(zone)))
        .hasMessage("Parking lot: an area is off the field");
  }

  @Test
  void aPoseIn3dTurnsWithTheField() {
    Pose3d blue = new Pose3d(1, 2, 1.5, new Rotation3d(0, 0.2, 0.5));

    assertThat(field.forAlliance(Alliance.BLUE, blue)).isSameAs(blue);
    assertThat(field.forAlliance(Alliance.RED, blue))
        .isEqualTo(Symmetry.ROTATED.apply(blue, TestFields.LENGTH, TestFields.WIDTH));
  }

  @Test
  void theFieldIsWithinItsWalls() {
    assertThat(field.contains(new Translation2d(8, 4))).isTrue();
    assertThat(field.contains(new Translation2d(0, 0))).isTrue();
    assertThat(field.contains(new Translation2d(-0.1, 4))).isFalse();
    assertThat(field.contains(new Translation2d(8, 8.1))).isFalse();
    assertThat(field.contains(new Translation2d(16.1, 4))).isFalse();
    assertThat(field.contains(new Translation2d(8, -1))).isFalse();
  }

  @Test
  void itKeepsItsStartsPlacesAndZones() {
    Place start = Place.at("Center", new Pose2d(1, 4, Rotation2d.ZERO));
    Place hub = Place.between("Hub", 2, 3);
    Zone zone = Zone.ours("Zone", Polygon2d.rectangle(new Pose2d(1, 4, Rotation2d.ZERO), 1, 1));

    var model = TestFields.model(List.of(start), List.of(hub), List.of(zone));

    assertThat(model.starts()).containsExactly(start);
    assertThat(model.places()).containsExactly(hub);
    assertThat(model.zones()).containsExactly(zone);
  }

  @Test
  void theBuilderNamesEachListAndLeavesOutWhatIsntGiven() {
    Place start = Place.at("Center", new Pose2d(1, 4, Rotation2d.ZERO));

    FieldModel model =
        FieldModel.builder(TagLayout.of(TestFields.layout()), Symmetry.ROTATED)
            .starts(List.of(start))
            .build();

    assertThat(model.starts()).containsExactly(start);
    assertThat(model.places()).isEmpty();
    assertThat(model.zones()).isEmpty();
    assertThat(model.terrain().surfaces()).isEmpty();
  }
}
