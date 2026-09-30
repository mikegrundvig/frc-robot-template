package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import frc.robot.lib.field.FieldModel;
import frc.robot.lib.field.Place;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.wpilib.driverstation.Alliance;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The season's field model checked against WPILib's tag layouts. A wrong field model is wrong for
 * every feature at once, so these are the checks that keep it honest: the declared symmetry holds
 * tag for tag, and places derived from tags land where the tags say.
 */
class RebuiltTest {
  private final FieldModel field = Rebuilt.field(Robots.SIM.body());

  @ParameterizedTest
  @EnumSource(
      value = Fields.class,
      names = {"FRC_2026_REBUILT_WELDED", "FRC_2026_REBUILT_ANDY_MARK"})
  void everyTagLandsOnAPartnerUnderTheDeclaredSymmetry(Fields layout) {
    FieldModel model = Rebuilt.field(Robots.SIM.body(), layout);

    assertThat(model.tags()).hasSize(32);
    assertThat(model.symmetryError()).as("meters, worst tag").isLessThan(0.01);
  }

  @Test
  void theHubIsTheCenterOfEachAlliancesHubTags() {
    Translation2d blue = Rebuilt.HUB.on(field, Alliance.BLUE).getTranslation();
    Translation2d red = Rebuilt.HUB.on(field, Alliance.RED).getTranslation();

    assertThat(blue.getDistance(center(18, 19, 20, 21, 24, 25, 26, 27))).isLessThan(0.001);
    assertThat(red.getDistance(center(2, 3, 4, 5, 8, 9, 10, 11))).isLessThan(0.001);
  }

  @Test
  void theTowerSpotFacesTheTowerFromJustClearOfIt() {
    Pose2d blue = Rebuilt.TOWER.on(field, Alliance.BLUE);
    Pose2d red = Rebuilt.TOWER.on(field, Alliance.RED);
    double halfLength = Robots.SIM.body().bumperLength().in(Meters) / 2;

    assertThat(blue.getX() - halfLength)
        .as("front bumper beyond the tower's face, 43.51 in from the wall")
        .isBetween(1.105, 1.2);
    assertThat(blue.getRotation().getDegrees()).isCloseTo(180, within(1e-9));
    assertThat(
            red.getTranslation()
                .getDistance(field.forAlliance(Alliance.RED, blue).getTranslation()))
        .as("red's, from red's tags, is blue's image")
        .isLessThan(0.01);
  }

  @Test
  void startsAreInTheAllianceZoneWithBumpersOnTheStartingLineFacingTheField() {
    double halfLength = Robots.SIM.body().bumperLength().in(Meters) / 2;
    assertThat(field.starts()).extracting(Place::name).containsExactly("Left", "Center", "Right");
    for (Place start : field.starts()) {
      for (Alliance alliance : Alliance.values()) {
        Pose2d pose = start.on(field, alliance);

        assertThat(Rebuilt.ALLIANCE_ZONE.contains(field, alliance, pose.getTranslation()))
            .as(start.name() + " for " + alliance)
            .isTrue();
        assertThat(Rebuilt.ALLIANCE_ZONE.distance(field, alliance, pose.getTranslation())).isZero();
        assertThat(pose.getRotation()).isEqualTo(FieldModel.driverForward(alliance));
      }
      Pose2d blue = start.on(field, Alliance.BLUE);
      assertThat(blue.getX() + halfLength).isCloseTo(Rebuilt.STARTING_LINE_X, within(1e-9));
    }
  }

  @Test
  void eachTrenchIsUnderItsTags() {
    for (int tag : new int[] {1, 6, 7, 12, 17, 22, 23, 28}) {
      Translation2d under = field.tag(tag).orElseThrow().toPose2d().getTranslation();

      assertThat(Rebuilt.TRENCH.contains(field, Alliance.BLUE, under)).as("tag " + tag).isTrue();
    }
  }

  @Test
  void theModelKeepsTheSeasonsPlacesAndZones() {
    assertThat(field.places()).containsExactly(Rebuilt.HUB, Rebuilt.TOWER);
    assertThat(field.zones()).containsExactly(Rebuilt.ALLIANCE_ZONE, Rebuilt.TRENCH);
    assertThat(field.name()).isEqualTo("2026 FRC Rebuilt Welded");
  }

  /** The middle of the box around some tags, on the floor. */
  private Translation2d center(int... tags) {
    List<Pose3d> poses = Arrays.stream(tags).mapToObj(id -> field.tag(id).orElseThrow()).toList();
    double minX = poses.stream().mapToDouble(Pose3d::getX).min().orElseThrow();
    double maxX = poses.stream().mapToDouble(Pose3d::getX).max().orElseThrow();
    double minY = poses.stream().mapToDouble(Pose3d::getY).min().orElseThrow();
    double maxY = poses.stream().mapToDouble(Pose3d::getY).max().orElseThrow();
    return new Translation2d((minX + maxX) / 2, (minY + maxY) / 2);
  }
}
