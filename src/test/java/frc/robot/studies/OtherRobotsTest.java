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

  @Test
  void robotsArePlacedNearTheRobotClearOfItEachOtherAndTheWalls() {
    Body body = Robots.SIM.body();
    Pose2d robot = new Pose2d(8, 4, Rotation2d.ZERO);

    List<OtherRobots.Blocker> placed =
        OtherRobots.around(robot, WALL, body, 5, new SplittableRandom(1));

    assertThat(placed).hasSize(5);
    assertThat(OtherRobots.around(robot, WALL, body, 5, new SplittableRandom(1))).isEqualTo(placed);
    double clear = 2 * Math.hypot(placed.get(0).halfLength(), placed.get(0).halfWidth());
    for (OtherRobots.Blocker other : placed) {
      double distance = Math.hypot(other.x() - 8, other.y() - 4);
      assertThat(distance).isBetween(clear, OtherRobots.REACH_METERS);
      assertThat(other.x()).isBetween(clear / 2, 16 - clear / 2);
      assertThat(other.y()).isBetween(clear / 2, 8 - clear / 2);
      assertThat(other.height()).isCloseTo(0.762, within(1e-9));
      for (OtherRobots.Blocker another : placed) {
        if (another != other) {
          assertThat(Math.hypot(other.x() - another.x(), other.y() - another.y()))
              .isGreaterThanOrEqualTo(clear);
        }
      }
    }
  }
}
