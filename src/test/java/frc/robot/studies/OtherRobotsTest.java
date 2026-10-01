package frc.robot.studies;

import static frc.robot.studies.StudyFixtures.WALL;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import frc.robot.Robots;
import frc.robot.lib.config.Body;
import java.util.List;
import java.util.SplittableRandom;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;

/** Other robots in the cameras' way: boxes on the floor, placed near the robot. */
class OtherRobotsTest {
  /** A meter-square box a meter tall, centered 2 m ahead. */
  private static final OtherRobots.Blocker BOX = new OtherRobots.Blocker(2, 0, 0, 0.5, 0.5, 1);

  private static final Body BODY = Robots.SIM.body();

  /** The middle of the field. */
  private static final double MIDDLE_X = WALL.length() / 2;

  private static final double MIDDLE_Y = WALL.width() / 2;

  /** How far from the robot to place others. */
  private static final double REACH = 8;

  @Test
  void aLineThroughTheBoxIsBlocked() {
    assertThat(BOX.blocks(new Translation3d(0, 0, 0.5), new Translation3d(4, 0, 0.5))).isTrue();
    assertThat(BOX.blocks(new Translation3d(0, 0, 1.5), new Translation3d(4, 0, 1.5)))
        .as("over it")
        .isFalse();
    assertThat(BOX.blocks(new Translation3d(0, 0, 0.5), new Translation3d(1.4, 0, 0.5)))
        .as("ending before it")
        .isFalse();
    assertThat(BOX.blocks(new Translation3d(0, 0.8, 0.5), new Translation3d(4, 0.8, 0.5)))
        .as("beside it")
        .isFalse();
    assertThat(BOX.blocks(new Translation3d(0, 0, 0.2), new Translation3d(4, 0, 2.0)))
        .as("rising out of it")
        .isTrue();
  }

  /** Turned 45°, the box reaches 0.707 m to its sides: a line 0.6 m off center now hits it. */
  @Test
  void aTurnedBoxBlocksWhereItsCornersReach() {
    OtherRobots.Blocker turned = new OtherRobots.Blocker(2, 0, Math.PI / 4, 0.5, 0.5, 1);

    assertThat(BOX.blocks(new Translation3d(0, 0.6, 0.5), new Translation3d(4, 0.6, 0.5)))
        .isFalse();
    assertThat(turned.blocks(new Translation3d(0, 0.6, 0.5), new Translation3d(4, 0.6, 0.5)))
        .isTrue();
  }

  /**
   * Outlines overlap only when they do: side by side a centimeter apart, two boxes are clear,
   * though their centers are closer than either's corners reach; one turned 45° into the gap isn't.
   */
  @Test
  void outlinesOverlapOnlyWhenTheyDo() {
    OtherRobots.Blocker beside = new OtherRobots.Blocker(3.01, 0, 0, 0.5, 0.5, 1);
    OtherRobots.Blocker turnedBeside = new OtherRobots.Blocker(3.01, 0, Math.PI / 4, 0.5, 0.5, 1);
    OtherRobots.Blocker diagonal = new OtherRobots.Blocker(2.9, 0.9, Math.PI / 4, 0.5, 0.5, 1);

    assertThat(BOX.overlaps(beside)).isFalse();
    assertThat(beside.overlaps(BOX)).isFalse();
    assertThat(BOX.overlaps(turnedBeside)).isTrue();
    assertThat(BOX.overlaps(diagonal)).as("corner to corner, not touching").isFalse();
    assertThat(BOX.overlaps(BOX)).isTrue();
  }

  /** As far as a seen tag, plus how far a camera and another robot's outline reach. */
  @Test
  void theReachIsTheSightAndBothOutlines() {
    double halfDiagonal = Math.hypot(35.5, 35.5) / 2 * 0.0254;

    assertThat(OtherRobots.reachMeters(7, BODY)).isCloseTo(7 + 2 * halfDiagonal, within(1e-9));
  }

  @Test
  void robotsArePlacedWithinReachClearOfTheRobotEachOtherAndTheWalls() {
    Pose2d robot = new Pose2d(MIDDLE_X, MIDDLE_Y, Rotation2d.fromDegrees(30));
    OtherRobots.Blocker self =
        new OtherRobots.Blocker(MIDDLE_X, MIDDLE_Y, Math.toRadians(30), 0.45085, 0.45085, 1);

    for (long seed = 1; seed <= 20; seed++) {
      List<OtherRobots.Blocker> placed =
          OtherRobots.around(
              robot, WALL, BODY, 1.2, 5, REACH, spot -> true, new SplittableRandom(seed));

      assertThat(placed).hasSize(5);
      assertThat(
              OtherRobots.around(
                  robot, WALL, BODY, 1.2, 5, REACH, spot -> true, new SplittableRandom(seed)))
          .isEqualTo(placed);
      for (OtherRobots.Blocker other : placed) {
        assertThat(Math.hypot(other.x() - MIDDLE_X, other.y() - MIDDLE_Y))
            .isLessThanOrEqualTo(REACH);
        assertThat(other.onField(WALL)).isTrue();
        assertThat(other.overlaps(self)).isFalse();
        assertThat(other.height()).isEqualTo(1.2);
        assertThat(other.halfLength()).isCloseTo(0.45085, within(1e-9));
        for (OtherRobots.Blocker another : placed) {
          assertThat(another == other || !other.overlaps(another)).isTrue();
        }
      }
    }
  }

  /** Nowhere the robots may stand, none stands; where only one side is open, all stand there. */
  @Test
  void robotsStandOnlyWhereTheyMay() {
    Pose2d robot = new Pose2d(MIDDLE_X, MIDDLE_Y, Rotation2d.ZERO);

    assertThat(
            OtherRobots.around(
                robot, WALL, BODY, 1, 5, REACH, spot -> false, new SplittableRandom(1)))
        .isEmpty();
    assertThat(
            OtherRobots.around(
                robot,
                WALL,
                BODY,
                1,
                5,
                REACH,
                spot -> spot.getX() > MIDDLE_X + 1,
                new SplittableRandom(1)))
        .hasSize(5)
        .allMatch(other -> other.x() > MIDDLE_X + 1);
  }
}
