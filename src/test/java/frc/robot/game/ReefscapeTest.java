package frc.robot.game;

import static frc.robot.game.Manuals.IN;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagLayout;
import frc.robot.Robots;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * 2025's field checked against WPILib's tag layouts and the game manual: a fixture for the field
 * model, so these check that it can say what the field is, on both halves.
 */
class ReefscapeTest {
  private final FieldModel field = Reefscape.field(Robots.SIM.definition().body());

  @ParameterizedTest
  @EnumSource(
      value = Fields.class,
      names = {"FRC_2025_REEFSCAPE_WELDED", "FRC_2025_REEFSCAPE_ANDY_MARK"})
  void everyTagLandsOnAPartnerTurnedHalfATurn(Fields layout) {
    FieldModel model = Reefscape.field(Robots.SIM.definition().body(), layout);

    assertThat(model.tags()).hasSize(22);
    assertThat(model.symmetryError()).as("meters, worst tag").isLessThan(0.01);
  }

  @Test
  void theFieldIsNotAMirrorImage() {
    FieldModel mirrored =
        FieldModel.builder(TagLayout.of(Reefscape.LAYOUT.loadField()), Symmetry.MIRRORED).build();

    assertThat(mirrored.symmetryError())
        .as("the processors' tags have no partner")
        .isGreaterThan(1);
  }

  @Test
  void theReefIsTwelveFeetOutAndCenteredOnItsTags() {
    Translation2d blue = Reefscape.REEF.on(field, Alliance.BLUE).getTranslation();
    Translation2d red = Reefscape.REEF.on(field, Alliance.RED).getTranslation();

    assertThat(blue.getDistance(reefCenter(Alliance.BLUE))).isLessThan(0.001);
    assertThat(red.getDistance(reefCenter(Alliance.RED))).isLessThan(0.001);
    assertThat(field.tag(18).orElseThrow().getX()).isCloseTo(12 * 12 * IN, within(0.001));
  }

  /** The reef zone is the reef's own hexagon grown to 7 ft 9½ in across, around each alliance's. */
  @Test
  void theReefZoneSurroundsEachAlliancesReef() {
    double flat = Reefscape.REEF_ZONE_ACROSS / 2;
    double corner = flat / Math.cos(Math.PI / 6);
    for (Alliance alliance : Alliance.values()) {
      Translation2d reef = reefCenter(alliance);
      for (int face = 0; face < 6; face++) {
        Rotation2d out = Rotation2d.fromDegrees(60 * face);
        Rotation2d toCorner = out.plus(Rotation2d.fromDegrees(30));

        assertThat(inReefZone(alliance, reef.plus(new Translation2d(flat - 0.001, out)))).isTrue();
        assertThat(inReefZone(alliance, reef.plus(new Translation2d(flat + 0.001, out)))).isFalse();
        assertThat(inReefZone(alliance, reef.plus(new Translation2d(corner - 0.001, toCorner))))
            .isTrue();
        assertThat(inReefZone(alliance, reef.plus(new Translation2d(corner + 0.001, toCorner))))
            .isFalse();
      }
      Alliance other = alliance == Alliance.BLUE ? Alliance.RED : Alliance.BLUE;
      assertThat(Reefscape.REEF_ZONE.contains(field, other, reef)).as("theirs").isFalse();
    }
  }

  @Test
  void startsOverlapTheStartingLineFacingTheField() {
    double halfLength = Robots.SIM.definition().body().bumperLength().in(Meters) / 2;
    assertThat(field.starts()).extracting(Place::name).containsExactly("Left", "Center", "Right");
    for (Place start : field.starts()) {
      Pose2d blue = start.on(field, Alliance.BLUE);

      assertThat(blue.getX() + halfLength)
          .isCloseTo(Reefscape.STARTING_LINE_X + Reefscape.TAPE_WIDTH, within(1e-9));
      assertThat(start.on(field, Alliance.RED).getRotation()).isEqualTo(Rotation2d.PI);
    }
  }

  @Test
  void theFieldIsFlatAndKeepsItsPlacesAndZones() {
    assertThat(field.terrain().surfaces()).isEmpty();
    assertThat(field.places()).containsExactly(Reefscape.REEF);
    assertThat(field.zones()).containsExactly(Reefscape.REEF_ZONE, Reefscape.BARGE_ZONE);
    assertThat(field.name()).isEqualTo("2025 FRC Reefscape Welded");
  }

  /** Where an alliance's reef is drawn: blue's from the manual, red's its image. */
  private Translation2d reefCenter(Alliance alliance) {
    return field.forAlliance(alliance, new Translation2d(Reefscape.REEF_X, Reefscape.REEF_Y));
  }

  private boolean inReefZone(Alliance alliance, Translation2d point) {
    return Reefscape.REEF_ZONE.contains(field, alliance, point);
  }
}
