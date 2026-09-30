package frc.robot.lib.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.shape.Rectangle2d;

class ZoneTest {
  private final FieldModel field = TestFields.model();

  /** The blue alliance's end of the field, 3 m deep. */
  private final Rectangle2d blueEnd =
      new Rectangle2d(new Translation2d(0, 0), new Translation2d(3, 8));

  private final Zone ours = Zone.ours("Alliance zone", blueEnd);

  @Test
  void ourZoneIsAtOurEndAndTheirsAtTheOther() {
    Zone theirs = Zone.theirs("Their zone", blueEnd);
    Translation2d nearBlueWall = new Translation2d(1, 1);
    Translation2d nearRedWall = new Translation2d(15, 7);

    assertThat(ours.contains(field, Alliance.BLUE, nearBlueWall)).isTrue();
    assertThat(ours.contains(field, Alliance.RED, nearRedWall)).isTrue();
    assertThat(ours.contains(field, Alliance.RED, nearBlueWall)).isFalse();
    assertThat(theirs.contains(field, Alliance.BLUE, nearRedWall)).isTrue();
    assertThat(theirs.contains(field, Alliance.RED, nearBlueWall)).isTrue();
  }

  @Test
  void aZoneOnBothHalvesIsEachAreaAndItsImage() {
    Zone trench =
        Zone.both("Trench", new Rectangle2d(new Translation2d(4, 7), new Translation2d(5, 8)));

    assertThat(trench.areas(field, Alliance.BLUE)).hasSize(2);
    for (Alliance alliance : Alliance.values()) {
      assertThat(trench.contains(field, alliance, new Translation2d(4.5, 7.5))).isTrue();
      assertThat(trench.contains(field, alliance, new Translation2d(11.5, 0.5))).isTrue();
      assertThat(trench.contains(field, alliance, new Translation2d(8, 4))).isFalse();
    }
  }

  @Test
  void distanceIsZeroInsideAndMetersToTheEdgeOutside() {
    assertThat(ours.distance(field, Alliance.BLUE, new Translation2d(1, 4))).isZero();
    assertThat(ours.distance(field, Alliance.BLUE, new Translation2d(5, 4)))
        .isCloseTo(2.0, within(1e-9));
  }

  @Test
  void itLooksAheadAlongTheRobotsVelocity() {
    Translation2d at = new Translation2d(5, 4);
    Translation2d towardIt = new Translation2d(-2, 0);

    assertThat(ours.reachedWithin(field, Alliance.BLUE, at, towardIt, 0.5)).as("4 m out").isFalse();
    assertThat(ours.reachedWithin(field, Alliance.BLUE, at, towardIt, 1.0))
        .as("at the edge")
        .isTrue();
    assertThat(ours.reachedWithin(field, Alliance.BLUE, at, towardIt.unaryMinus(), 5)).isFalse();
    assertThat(
            ours.reachedWithin(
                field, Alliance.BLUE, new Translation2d(1, 4), Translation2d.ZERO, 0))
        .as("already inside, standing still")
        .isTrue();
  }

  @Test
  void aPathCanCutThroughATurnedArea() {
    // A 1 m square turned 45°: a diamond reaching 0.71 m from its center along each axis.
    Rectangle2d diamond = new Rectangle2d(new Pose2d(8, 4, Rotation2d.fromDegrees(45)), 1, 1);

    assertThat(Zone.crosses(diamond, new Translation2d(7, 4), new Translation2d(9, 4))).isTrue();
    assertThat(Zone.crosses(diamond, new Translation2d(7, 4.8), new Translation2d(9, 4.8)))
        .as("passes above its top corner")
        .isFalse();
    assertThat(Zone.crosses(diamond, new Translation2d(8, 2), new Translation2d(8, 3.2)))
        .as("stops short")
        .isFalse();
    assertThat(Zone.crosses(diamond, new Translation2d(7, 4.8), new Translation2d(7, 5.8)))
        .as("beside it, straight up")
        .isFalse();
  }

  @Test
  void aZoneNeedsANameAndAnArea() {
    assertThatThrownBy(() -> Zone.ours(" ", blueEnd)).hasMessage("a zone needs a name");
    assertThatThrownBy(() -> new Zone("Nowhere", Zone.Side.OURS, List.of()))
        .hasMessage("Nowhere needs at least one area");
  }

  @Test
  void aTriggerTurnsOnBeforeArrivingAndOffOnlyPastTheMargin() {
    ZoneTrigger trigger = new ZoneTrigger(ours, 0.4, 0.1);
    Translation2d toward = new Translation2d(-2, 0);
    Translation2d still = Translation2d.ZERO;

    assertThat(trigger.update(field, Alliance.BLUE, new Translation2d(4, 4), toward))
        .as("1 m out, 0.5 s away")
        .isFalse();
    assertThat(trigger.update(field, Alliance.BLUE, new Translation2d(3.7, 4), toward))
        .as("0.35 s away")
        .isTrue();
    assertThat(trigger.update(field, Alliance.BLUE, new Translation2d(3.05, 4), still))
        .as("stopped just outside, within the margin")
        .isTrue();
    assertThat(trigger.update(field, Alliance.BLUE, new Translation2d(3.2, 4), still))
        .as("past the margin")
        .isFalse();
    assertThat(trigger.isActive()).isFalse();
    assertThat(trigger.update(field, Alliance.BLUE, new Translation2d(3.05, 4), still))
        .as("the margin only holds it on")
        .isFalse();
    assertThat(trigger.zone()).isEqualTo(ours);
  }

  @Test
  void aTriggerNeedsALookaheadAndMarginOfZeroOrMore() {
    assertThatThrownBy(() -> new ZoneTrigger(ours, -1, 0))
        .isInstanceOf(IllegalArgumentException.class);
    assertThatThrownBy(() -> new ZoneTrigger(ours, 0, Double.NaN))
        .isInstanceOf(IllegalArgumentException.class);
  }
}
