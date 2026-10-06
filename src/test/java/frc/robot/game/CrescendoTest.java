package frc.robot.game;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagLayout;
import frc.robot.Robots;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * 2024's field checked against WPILib's tag layout and the game manual: a fixture for the field
 * model, a mirrored one, so these check that it can say what the field is, on both halves.
 */
class CrescendoTest {
  private final FieldModel field = Crescendo.field(Robots.SIM.definition().body());

  @Test
  void everyTagLandsOnAPartnerReflectedAcrossTheCenterLine() {
    assertThat(field.tags()).hasSize(16);
    assertThat(field.symmetryError()).as("meters, worst tag").isLessThan(0.01);
  }

  @Test
  void theFieldIsNotTurnedHalfATurn() {
    FieldModel rotated =
        FieldModel.builder(TagLayout.of(Crescendo.LAYOUT.loadField()), Symmetry.ROTATED).build();

    assertThat(rotated.symmetryError()).as("the amps' tags have no partner").isGreaterThan(1);
  }

  @Test
  void startsAreInTheStartingZoneFacingTheField() {
    double halfLength = Robots.SIM.definition().body().bumperLength().in(Meters) / 2;
    double halfWidth = Robots.SIM.definition().body().bumperWidth().in(Meters) / 2;
    assertThat(field.starts())
        .extracting(Place::name)
        .containsExactly("Amp side", "Subwoofer", "Center");
    for (Place start : field.starts()) {
      for (Alliance alliance : Alliance.values()) {
        Pose2d pose = start.on(field, alliance);

        // Bumpers within the zone, its edges included: a start backs up to them.
        for (Translation2d corner : corners(pose, halfLength, halfWidth)) {
          assertThat(Crescendo.STARTING_ZONE.distance(field, alliance, corner))
              .as(start.name() + " for " + alliance)
              .isLessThan(1e-9);
        }
        assertThat(pose.getRotation()).isEqualTo(FieldModel.driverForward(alliance));
      }
    }
  }

  /**
   * Each alliance's source zone is in front of its own source, by the opposing alliance's wall, and
   * its tape cuts the far corner off that alliance's robot starting zone.
   */
  @Test
  void theOpponentsSourceZoneCutsTheStartingZonesFarCorner() {
    for (int tag : new int[] {1, 2}) {
      Pose2d face = field.tag(tag).orElseThrow().toPose2d();
      Translation2d near = face.getTranslation().plus(new Translation2d(0.1, face.getRotation()));
      Translation2d past =
          face.getTranslation()
              .plus(new Translation2d(Crescendo.SOURCE_ZONE_DEPTH + 0.01, face.getRotation()));

      assertThat(Crescendo.SOURCE_ZONE.contains(field, Alliance.BLUE, near))
          .as("blue's, in front of tag " + tag)
          .isTrue();
      assertThat(Crescendo.SOURCE_ZONE.contains(field, Alliance.RED, near)).isFalse();
      assertThat(Crescendo.SOURCE_ZONE.contains(field, Alliance.BLUE, past)).isFalse();
    }
    for (Alliance alliance : Alliance.values()) {
      Alliance other = alliance == Alliance.BLUE ? Alliance.RED : Alliance.BLUE;
      // Along the tape between them, a centimeter either side, halfway out.
      Pose2d tag9 = field.tag(9).orElseThrow().toPose2d();
      Translation2d tape =
          tag9.getTranslation()
              .plus(new Translation2d(Crescendo.SOURCE_ZONE_DEPTH, tag9.getRotation()));
      for (double off : new double[] {-0.01, 0.01}) {
        Translation2d spot =
            field.forAlliance(alliance, tape.plus(new Translation2d(off, tag9.getRotation())));
        boolean starting = Crescendo.STARTING_ZONE.contains(field, alliance, spot);
        boolean source = Crescendo.SOURCE_ZONE.contains(field, other, spot);

        assertThat(starting).as(alliance + ", " + off).isEqualTo(off > 0);
        assertThat(source).as(alliance + ", " + off).isEqualTo(off < 0);
      }
    }
    // The far edge, along the black tape: the manual's 23 ft 8⅛ in, to within the tags' word.
    Translation2d corner = Crescendo.STARTING_ZONE.blueAreas().get(0).corner(1);
    assertThat(corner.getX()).isCloseTo(Crescendo.STARTING_ZONE_DEPTH, within(1e-9));
    double low = corner.getY();
    assertThat(low).as("from the guardrail up to the zone").isCloseTo(0.52, within(0.01));
    double length = field.width() - Crescendo.AMP_ZONE_WIDTH - low;
    assertThat(length).isCloseTo(Crescendo.STARTING_ZONE_LENGTH, within(0.03));
  }

  @Test
  void theFieldIsFlatAndKeepsItsPlacesAndZones() {
    assertThat(field.terrain().surfaces()).isEmpty();
    assertThat(field.places()).containsExactly(Crescendo.STAGE);
    assertThat(field.zones())
        .containsExactly(
            Crescendo.STARTING_ZONE,
            Crescendo.AMP_ZONE,
            Crescendo.SOURCE_ZONE,
            Crescendo.STAGE_ZONE);
    assertThat(field.name()).isEqualTo("2024 FRC Crescendo");
  }

  /** Which way a pose's x axis points: an opening's way in. */
  private static Translation3d axis(Pose3d pose) {
    return new Translation3d(1, 0, 0).rotateBy(pose.getRotation());
  }

  /** A point on one half, on the other: reflected across the center line, at the same height. */
  private Translation3d image(Translation3d point) {
    Translation2d image =
        field.symmetry().apply(point.toTranslation2d(), field.length(), field.width());
    return new Translation3d(image.getX(), image.getY(), point.getZ());
  }

  /** A robot's four bumper corners. */
  private static List<Translation2d> corners(Pose2d pose, double halfLength, double halfWidth) {
    List<Translation2d> corners = new ArrayList<>();
    for (double[] sign : new double[][] {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}}) {
      corners.add(
          pose.getTranslation()
              .plus(
                  new Translation2d(sign[0] * halfLength, sign[1] * halfWidth)
                      .rotateBy(pose.getRotation())));
    }
    return corners;
  }

  /** Points around a center, inside and outside a shape. */
  private static List<Translation3d> around(Translation3d center) {
    List<Translation3d> points = new ArrayList<>();
    for (double dx = -0.3; dx <= 0.3; dx += 0.15) {
      for (double dy = -0.3; dy <= 0.3; dy += 0.15) {
        for (double dz = -0.3; dz <= 0.3; dz += 0.15) {
          points.add(center.plus(new Translation3d(dx, dy, dz)));
        }
      }
    }
    return points;
  }
}
