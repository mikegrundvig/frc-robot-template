package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;

class VenueTest {
  private static final String BASE = "Test field";

  /** A venue file over the test field's tags, in WPILib's field format or its earlier one. */
  private static String file(
      String kind, String base, List<FieldTag> tags, boolean fieldFormat, String more) {
    StringBuilder list = new StringBuilder();
    for (FieldTag tag : tags) {
      Pose3d pose = tag.getPose();
      var q = pose.getRotation().getQuaternion();
      list.append(list.isEmpty() ? "" : ",")
          .append(
              String.format(
                  Locale.ROOT,
                  "{\"ID\": %d, \"pose\": {\"translation\": {\"x\": %s, \"y\": %s, \"z\": %s},"
                      + " \"rotation\": {\"quaternion\": {\"W\": %s, \"X\": %s, \"Y\": %s,"
                      + " \"Z\": %s}}}}",
                  tag.getID(),
                  pose.getX(),
                  pose.getY(),
                  pose.getZ(),
                  q.getW(),
                  q.getX(),
                  q.getY(),
                  q.getZ()));
    }
    String layout =
        fieldFormat
            ? "{\"name\": \"Test field\", \"season\": \"2027\", \"game\": \"Test\","
                + " \"field-dimensions\": {\"length\": 16, \"width\": 8}, \"program\": \"frc\","
                + " \"field-tags\": ["
                + list
                + "]}"
            : "{\"tags\": [" + list + "], \"field\": {\"length\": 16, \"width\": 8}}";
    return "{\"format\": 1, \"name\": \"Practice field, October\", \"kind\": \""
        + kind
        + "\", \"base\": \""
        + base
        + "\", \"layout\": "
        + layout
        + more
        + "}";
  }

  private static String file(List<FieldTag> tags, String more) {
    return file("practice", BASE, tags, true, more);
  }

  /** The test field's tags, with one moved by (dx, dy). */
  private static List<FieldTag> moved(int id, double dx, double dy) {
    List<FieldTag> tags = new ArrayList<>();
    for (FieldTag tag : TestFields.tags()) {
      Pose3d pose = tag.getPose();
      tags.add(
          tag.getID() == id
              ? new FieldTag(
                  id,
                  new Pose3d(pose.getX() + dx, pose.getY() + dy, pose.getZ(), pose.getRotation()))
              : tag);
    }
    return tags;
  }

  private static List<String> problems(String text) {
    return VenueCheck.problems(VenueFile.read(text), TestFields.layout(), Symmetry.ROTATED);
  }

  @Test
  void readsAVenueWhoseLayoutIsInWpilibsFieldFormat() {
    Venue venue = VenueFile.read(file(TestFields.tags(), ""));
    assertThat(venue.name()).isEqualTo("Practice field, October");
    assertThat(venue.kind()).isEqualTo(Venue.Kind.PRACTICE);
    assertThat(venue.base()).isEqualTo(BASE);
    assertThat(venue.layout().getTags()).hasSize(6);
    assertThat(venue.layout().getTagPose(14).orElseThrow().getY()).isCloseTo(6, within(1e-12));
    assertThat(venue.layout().getName()).isEqualTo("Practice field, October");
    assertThat(venue.method()).isEqualTo("unknown");
    assertThat(venue.wrapped()).isFalse();
    assertThat(venue.hash()).hasSize(64).matches("[0-9a-f]+");
  }

  @Test
  void readsALayoutInTheEarlierAprilTagFormat() {
    Venue venue = VenueFile.read(file("event", BASE, TestFields.tags(), false, ""));
    assertThat(venue.kind()).isEqualTo(Venue.Kind.EVENT);
    assertThat(venue.layout().getFieldLength()).isEqualTo(16);
    assertThat(venue.layout().getTagPose(2).orElseThrow().getX()).isCloseTo(4, within(1e-12));
  }

  @Test
  void readsReasonsAndProvenance() {
    Venue venue =
        VenueFile.read(
            file(
                TestFields.tags(),
                ", \"moved\": {\"4\": \"rehung after the survey\"},"
                    + " \"method\": \"hand\", \"date\": \"2026-10-12\""));
    assertThat(venue.moved()).containsEntry(4, "rehung after the survey");
    assertThat(venue.method()).isEqualTo("hand");
    assertThat(venue.date()).isEqualTo("2026-10-12");
  }

  @Test
  void aBareLayoutIsNotAVenueFileAndSaysHowToMakeOne() {
    assertThatThrownBy(
            () -> VenueFile.read("{\"tags\": [], \"field\": {\"length\": 16, \"width\": 8}}"))
        .hasMessageContaining("bare WPILib layout")
        .hasMessageContaining("Pitwall's upload");
  }

  @Test
  void aBareLayoutIsWrappedWithStatedDefaults() {
    // The test field's layout as WPIcal writes one: the earlier format, on its own.
    String bare = file("event", BASE, TestFields.tags(), false, "");
    String embedded = bare.substring(bare.indexOf("\"layout\": ") + 10, bare.length() - 1);
    Venue venue = VenueFile.wrap(embedded, "Event \"A\"\nfield", Venue.Kind.EVENT, BASE);
    assertThat(venue.wrapped()).isTrue();
    assertThat(venue.name()).isEqualTo("Event \"A\"\nfield");
    assertThat(venue.method()).isEqualTo("unknown");
    assertThat(venue.layout().getTags()).hasSize(6);
    assertThat(VenueFile.read(venue.text()).name())
        .as("its text reads back")
        .isEqualTo(venue.name());
    assertThatThrownBy(() -> VenueFile.wrap("{\"a\": 1}", "x", Venue.Kind.EVENT, BASE))
        .hasMessageContaining("not a WPILib layout");
  }

  @Test
  void everyProblemInTheFileIsReportedAtOnce() {
    String text =
        file("party", BASE, TestFields.tags(), true, ", \"surveyor\": \"Sam\"")
            .replace("\"base\": \"Test field\", ", "");
    assertThatThrownBy(() -> VenueFile.read(text))
        .hasMessageContaining("unknown key \"surveyor\"")
        .hasMessageContaining("kind \"party\"")
        .hasMessageContaining("base is missing");
  }

  @Test
  void anotherFormatIsRefused() {
    assertThatThrownBy(
            () ->
                VenueFile.read(
                    file(TestFields.tags(), "").replace("\"format\": 1", "\"format\": 2")))
        .hasMessageContaining("this code reads format 1");
  }

  @Test
  void jsonThatIsntStrictlyJsonIsRefusedSayingWhere() {
    assertThatThrownBy(() -> VenueFile.read("{\"name\": \"a\", \"name\": \"b\"}"))
        .hasMessageContaining("appears twice")
        .hasMessageContaining("line 1");
    assertThatThrownBy(() -> VenueFile.read("{} {}")).hasMessageContaining("more after");
    assertThatThrownBy(() -> VenueFile.read("{\"a\": 1,}")).hasMessageContaining("expected a key");
  }

  @Test
  void aSurveyMatchingTheOfficialLayoutFits() {
    assertThat(problems(file(TestFields.tags(), ""))).isEmpty();
  }

  @Test
  void aSmallCorrectionNeedsNoReason() {
    assertThat(problems(file(moved(4, 0.02, -0.01), ""))).isEmpty();
  }

  @Test
  void aTagFarFromItsPlaceNeedsAReason() {
    assertThat(problems(file(moved(4, 0.2, 0), "")))
        .singleElement()
        .asString()
        .contains("tag 4 sits 0.200 m")
        .contains("\"moved\"");
    assertThat(problems(file(moved(4, 0.2, 0), ", \"moved\": {\"4\": \"rehung\"}"))).isEmpty();
  }

  @Test
  void aSurveyOfAnotherFieldIsCaught() {
    assertThat(problems(file("event", "2026 FRC Rebuilt Welded", TestFields.tags(), true, "")))
        .singleElement()
        .asString()
        .contains("measured against \"2026 FRC Rebuilt Welded\"");
    List<FieldTag> fewer = new ArrayList<>(TestFields.tags());
    fewer.removeIf(tag -> tag.getID() == 3);
    fewer.add(TestFields.tag(99, 8, 4, 0));
    assertThat(problems(file(fewer, "")))
        .anyMatch(problem -> problem.contains("missing tags [3]"))
        .anyMatch(problem -> problem.contains("has no tags [99]"));
  }

  @Test
  void anOfficialVenueMustBeWpilibsLayoutExactly() {
    assertThat(problems(file("official", BASE, moved(4, 0.005, 0), true, "")))
        .singleElement()
        .asString()
        .contains("an official venue, but tag 4 sits 5.0 mm");
  }

  @Test
  void aSurveyThatBreaksTheFieldsSymmetryIsCaught() {
    assertThat(problems(file(moved(14, 0, -1), ", \"moved\": {\"14\": \"a typo\"}")))
        .containsExactlyInAnyOrder(
            "tag 4 no longer pairs with tag 14 under the field's symmetry",
            "tag 14 no longer pairs with tag 4 under the field's symmetry");
  }

  @Test
  void aReasonMustNameATagOfTheLayout() {
    assertThat(problems(file(TestFields.tags(), ", \"moved\": {\"78\": \"why\"}")))
        .anyMatch(problem -> problem.contains("moved names tags the layout doesn't have: [78]"));
  }

  @Test
  void aSurveysOldSectionsAreRefused() {
    assertThatThrownBy(() -> VenueFile.read(file(TestFields.tags(), ", \"sigma\": {}")))
        .hasMessageContaining("sigma");
  }

  @Test
  void aSurveyGivesItsTagsWhereTheOfficialLayoutPutsThemAndWhereItFoundThem() {
    Venue venue = VenueFile.read(file(moved(4, 0.02, -0.01), ""));

    List<Tag> tags = venue.tags(TestFields.layout());

    assertThat(tags).extracting(Tag::id).containsExactly(2, 3, 12, 13, 4, 14);
    Tag four = tags.get(4);
    assertThat(four.nominal().getX()).isEqualTo(0);
    assertThat(four.surveyed()).isNotNull();
    assertThat(four.pose().getX()).isCloseTo(0.02, within(1e-12));
    assertThat(four.pose().getY()).isCloseTo(1.99, within(1e-12));
    TagLayout layout = venue.tagLayout(TestFields.layout());
    assertThat(layout.name()).isEqualTo("Practice field, October");
    assertThat(layout.lengthMeters()).isEqualTo(16);
    assertThat(layout.tags()).isEqualTo(tags);
  }

  @Test
  void anOfficialVenueIsNominalWithNothingSurveyed() {
    Venue venue = VenueFile.read(file("official", BASE, TestFields.tags(), true, ""));

    assertThat(venue.tags(TestFields.layout())).allMatch(tag -> tag.surveyed() == null);
  }

  @Test
  void aVenueWithATagItsBaseDoesntHaveHasNoTags() {
    List<FieldTag> more = new ArrayList<>(TestFields.tags());
    more.add(TestFields.tag(99, 8, 4, 0));
    Venue venue = VenueFile.read(file(more, ""));

    assertThatThrownBy(() -> venue.tags(TestFields.layout())).hasMessageContaining("no tag 99");
  }
}
