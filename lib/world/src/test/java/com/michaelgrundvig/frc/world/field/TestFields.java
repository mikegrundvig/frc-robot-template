package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.List;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * A small made-up field, 16 x 8 m and turned half a turn from blue's half to red's, with a few tags
 * that are easy to check by hand:
 *
 * <ul>
 *   <li>2 and 3: a "hub" on blue's half, faces at x = 4 (facing blue's wall) and x = 5.
 *   <li>12 and 13: red's hub, their images: faces at x = 12 and x = 11.
 *   <li>4 on blue's wall at y = 2, and 14 on red's wall at y = 6, its image.
 * </ul>
 */
final class TestFields {
  static final double LENGTH = 16;
  static final double WIDTH = 8;

  private TestFields() {}

  static List<FieldTag> tags() {
    List<FieldTag> tags = new ArrayList<>();
    tags.add(tag(2, 4, 4, 180));
    tags.add(tag(3, 5, 4, 0));
    tags.add(tag(12, 12, 4, 0));
    tags.add(tag(13, 11, 4, 180));
    tags.add(tag(4, 0, 2, 0));
    tags.add(tag(14, 16, 6, 180));
    return tags;
  }

  static Field layout(List<FieldTag> tags) {
    return new Field("Test field", "2027", "Test", null, LENGTH, WIDTH, "frc", tags);
  }

  static Field layout() {
    return layout(tags());
  }

  /** A tag standing upright, facing the given way. */
  static FieldTag tag(int id, double x, double y, double degrees) {
    return new FieldTag(
        id, new Pose3d(new Translation3d(x, y, 1), new Rotation3d(0, 0, Math.toRadians(degrees))));
  }

  static FieldModel model(List<Place> starts, List<Place> places, List<Zone> zones) {
    return new FieldModel(layout(), Symmetry.ROTATED, starts, places, zones);
  }

  static FieldModel model() {
    return model(List.of(), List.of(), List.of());
  }

  static void assertPose(Pose2d actual, double x, double y, double degrees) {
    assertThat(actual.getX()).as("x of " + actual).isCloseTo(x, within(1e-6));
    assertThat(actual.getY()).as("y of " + actual).isCloseTo(y, within(1e-6));
    assertThat(Math.IEEEremainder(actual.getRotation().getDegrees() - degrees, 360))
        .as("heading of " + actual + ", expected " + degrees + "°")
        .isCloseTo(0.0, within(1e-6));
  }
}
