package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

/** Tags as known: where the official layout puts them, and where a survey found them. */
class TagTest {
  @Test
  void aTagIsWhereTheSurveyFoundItElseWhereTheLayoutPutsIt() {
    Pose3d nominal = new Pose3d(4, 4, 1, new Rotation3d(0, 0, Math.PI));
    Pose3d surveyed = new Pose3d(4.01, 3.98, 1.02, new Rotation3d(0, 0, Math.PI));

    Tag official = Tag.nominal(2, nominal);
    Tag measured = new Tag(2, nominal, surveyed);

    assertThat(official.pose()).isEqualTo(nominal);
    assertThat(official.surveyed()).isNull();
    assertThat(measured.pose()).isEqualTo(surveyed);
  }

  @Test
  void aTagMustMakeSense() {
    Pose3d pose = new Pose3d();
    assertThatThrownBy(() -> Tag.nominal(-1, pose)).hasMessageContaining("zero or more");
  }

  @Test
  void aLayoutHasEachTagOnceAndASize() {
    TagLayout layout = TagLayout.of(TestFields.layout());

    assertThat(layout.name()).isEqualTo("Test field");
    assertThat(layout.lengthMeters()).isEqualTo(16);
    assertThat(layout.tags()).allMatch(tag -> tag.surveyed() == null);
    List<Tag> twice = new ArrayList<>(layout.tags());
    twice.add(layout.tags().get(0));
    assertThatThrownBy(() -> new TagLayout("Twice", 16, 8, twice))
        .hasMessageContaining("tag 2 twice");
    assertThatThrownBy(() -> new TagLayout("Flat", 0, 8, List.of()))
        .hasMessageContaining("positive");
  }

  /**
   * A survey moves where the robot believes a tag is, and the places found from it, but never which
   * tag plays which part on the other half: the partners come from the official layout.
   */
  @Test
  void theFieldUsesTheSurveyButPairsByTheOfficialLayout() {
    // Tag 12 surveyed 30 cm off: past the partner distance, were partners found from the survey.
    List<Tag> tags = new ArrayList<>();
    for (FieldTag tag : TestFields.tags()) {
      Pose3d nominal = tag.getPose();
      Pose3d surveyed =
          tag.getID() == 12
              ? new Pose3d(
                  nominal.getX(), nominal.getY() + 0.3, nominal.getZ(), nominal.getRotation())
              : nominal;
      tags.add(new Tag(tag.getID(), nominal, surveyed));
    }
    FieldModel field =
        FieldModel.builder(new TagLayout("Surveyed", 16, 8, tags), Symmetry.ROTATED)
            .places(List.of(Place.between("Hub", 2, 3)))
            .build();

    assertThat(field.name()).isEqualTo("Surveyed");
    assertThat(field.tag(12).orElseThrow().getY()).isCloseTo(4.3, within(1e-12));
    assertThat(Objects.requireNonNull(field.tags().get(12)).nominal().getY()).isEqualTo(4);
    assertThat(field.toField().getTagPose(12).orElseThrow().getY()).isCloseTo(4.3, within(1e-12));
    assertThat(field.tagFor(Alliance.RED, 2)).isEqualTo(12);
    assertThat(field.symmetryError()).as("of the official layout").isLessThan(1e-9);
    // Red's hub is between red's tags as surveyed.
    assertThat(field.places().get(0).on(field, Alliance.RED).getY()).isCloseTo(4.15, within(1e-9));
  }
}
