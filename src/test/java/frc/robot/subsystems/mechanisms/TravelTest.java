package frc.robot.subsystems.mechanisms;

import static frc.robot.subsystems.mechanisms.JointFixtures.ELEVATOR;
import static frc.robot.subsystems.mechanisms.JointFixtures.PIVOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.ROBOT;
import static frc.robot.subsystems.mechanisms.JointFixtures.TURRET;
import static frc.robot.subsystems.mechanisms.JointFixtures.TURRET_FRONT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.MechanismControlBuilder;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * Where a joint is driven, worked out from its block: its limits, a turret's turn of a heading, a
 * free joint's short way round, and where it stops braking. No simulator: each is a function of
 * what it's given.
 */
class TravelTest {
  /** A turret whose range spans more than a turn, -200° to 200°. */
  private static final Joint.Revolute WIDE =
      JointRevoluteBuilder.builder(TURRET)
          .range(Range.degrees(-200, 200))
          .positions(List.of(TURRET_FRONT, Position.degrees("back", -180)))
          .build();

  private static final RobotDefinition WIDE_ROBOT =
      RobotDefinitionBuilder.builder(ROBOT)
          .joints(List.of(PIVOT, WIDE))
          .configurations(List.of())
          .build();

  private static Travel travel(Joint joint) {
    RobotDefinition robot = joint.equals(WIDE) ? WIDE_ROBOT : ROBOT;
    return Travel.of(robot.math(), joint, ControllerSettings.control(joint));
  }

  private static double radians(double degrees) {
    return Math.toRadians(degrees);
  }

  @Test
  void itsLimitsAreItsSoftLimitsElseItsStopsElseNone() {
    MechanismControl soft =
        ControllerSettings.control(PIVOT).withSoftLimits(Range.of(Radians.of(0.1), Radians.of(2)));

    assertThat(Travel.of(ROBOT.math(), PIVOT, soft).lower()).isEqualTo(0.1);
    assertThat(Travel.of(ROBOT.math(), PIVOT, soft).upper()).isEqualTo(2);
    assertThat(travel(PIVOT).lower()).isEqualTo(0);
    assertThat(travel(PIVOT).upper()).isCloseTo(radians(135), within(1e-12));
    assertThat(travel(TURRET).lower()).isEqualTo(Double.NEGATIVE_INFINITY);
    assertThat(travel(TURRET).upper()).isEqualTo(Double.POSITIVE_INFINITY);
  }

  /**
   * The fixtures' profiles, and a quarter of its free speed, reached in a quarter second, without.
   */
  @Test
  void itCruisesAtItsProfileElseAQuarterOfItsFreeSpeed() {
    MechanismControl unprofiled =
        MechanismControlBuilder.builder(ControllerSettings.control(ELEVATOR))
            .profile(Optional.empty())
            .build();
    double free = ROBOT.math().freeSpeed(ELEVATOR);

    assertThat(travel(ELEVATOR).cruise()).isEqualTo(1.2);
    assertThat(travel(ELEVATOR).acceleration()).isEqualTo(5);
    assertThat(Travel.of(ROBOT.math(), ELEVATOR, unprofiled).cruise()).isEqualTo(0.25 * free);
    assertThat(Travel.of(ROBOT.math(), ELEVATOR, unprofiled).acceleration())
        .isEqualTo(0.25 * free / 0.25);
  }

  @Test
  void aPositionIsHeldInsideItsLimits() {
    assertThat(travel(PIVOT).clamp(-1)).isEqualTo(0);
    assertThat(travel(PIVOT).clamp(1)).isEqualTo(1);
    assertThat(travel(PIVOT).clamp(3)).isCloseTo(radians(135), within(1e-12));
    assertThat(travel(TURRET).clamp(100)).isEqualTo(100);
  }

  /** A joint that turns freely takes a goal the short way round; one with stops, as it is. */
  @Test
  void aFreeJointsGoalIsTheShortWayRound() {
    assertThat(travel(TURRET).wraps()).isTrue();
    assertThat(travel(TURRET).goalNear(radians(-170), radians(170)))
        .isCloseTo(radians(190), within(1e-9));
    assertThat(travel(PIVOT).wraps()).isFalse();
    assertThat(travel(PIVOT).goalNear(radians(200), 0)).isCloseTo(radians(135), within(1e-12));
  }

  /** At 170°, -170° is 20° on, at 190°; from 0°, -170° as it is; 370° from 0° is 10°. */
  @Test
  void aTurretTakesAHeadingAtItsNearestTurn() {
    Travel wide = travel(WIDE);

    assertThat(wide.choosesTurn()).isTrue();
    assertThat(wide.turnNear(radians(-170), radians(170))).isCloseTo(radians(190), within(1e-9));
    assertThat(wide.turnNear(radians(-170), 0)).isCloseTo(radians(-170), within(1e-9));
    assertThat(wide.turnNear(radians(370), 0)).isCloseTo(radians(10), within(1e-9));
    assertThat(wide.whyUnreachable(radians(370), 0)).isNull();
  }

  /**
   * Soft limits narrower than a turn leave headings with no turn inside: one is refused, saying so,
   * and taken at the limit nearer it either way round.
   */
  @Test
  void aHeadingWithNoTurnInsideIsRefusedAndHeldAtTheNearerLimit() {
    Travel soft =
        Travel.of(
            WIDE_ROBOT.math(),
            WIDE,
            ControllerSettings.control(WIDE).withSoftLimits(Range.degrees(-190, 160)));

    assertThat(soft.turnNear(radians(163), 0)).isCloseTo(radians(160), within(1e-12));
    assertThat(soft.turnNear(radians(-193), 0)).isCloseTo(radians(-190), within(1e-12));
    assertThat(soft.whyUnreachable(radians(165), 0))
        .isEqualTo("165°, at every turn of it, is outside turret's limits, -190° to 160°");
  }

  /** A joint narrower than a turn takes an angle as it is: one off by whole turns is a slip. */
  @Test
  void aJointNarrowerThanATurnTakesItsAngleAsItIs() {
    assertThat(travel(PIVOT).choosesTurn()).isFalse();
    assertThat(travel(PIVOT).turnNear(radians(370), 0)).isEqualTo(radians(370));
    assertThat(travel(PIVOT).whyUnreachable(radians(370), 0))
        .isEqualTo("370° is outside pivot's limits, 0° to 135°");
    assertThat(travel(PIVOT).whyUnreachable(radians(90), 0)).isNull();
  }

  /** An angle far too big to tell its turns apart still ends its search, inside the limits. */
  @Test
  void anAngleTooBigToTurnStillEnds() {
    assertThat(travel(WIDE).turnNear(1e300, 0)).isBetween(radians(-200), radians(200));
  }

  /** Braking at 30 rad/s², from 1 rad at 3 rad/s it stops 0.15 rad on; back the other way, too. */
  @Test
  void itStopsWhereItsAccelerationBrakesIt() {
    assertThat(travel(PIVOT).stoppingPoint(1, 3)).isCloseTo(1.15, within(1e-12));
    assertThat(travel(PIVOT).stoppingPoint(1, -3)).isCloseTo(0.85, within(1e-12));
    assertThat(travel(PIVOT).stoppingPoint(0.05, -3)).as("inside its limits").isEqualTo(0);
  }

  /**
   * Held a loop on, the pivot goes on a loop at its speed, and faster by what it's speeding up, and
   * then brakes at its 30 rad/s²: from 1 rad at 3 rad/s, 1.06 rad on and 0.15 more; speeding up at
   * 50 rad/s², 4 rad/s a loop on, 1.07 rad on and 0.267 more. Slowing down counts as going on.
   */
  @Test
  void heldALoopOnItStopsWhereItsSpeedAndSpeedingUpTakeIt() {
    Travel pivot = travel(PIVOT);

    assertThat(pivot.stoppingPointLoopOn(1, 3, 0)).isCloseTo(1.21, within(1e-12));
    assertThat(pivot.stoppingPointLoopOn(1, 3, 50)).isCloseTo(1.07 + 16 / 60.0, within(1e-12));
    assertThat(pivot.stoppingPointLoopOn(1, 3, -50)).isCloseTo(1.21, within(1e-12));
    assertThat(pivot.stoppingPointLoopOn(1, -3, -50)).isCloseTo(0.93 - 16 / 60.0, within(1e-12));
    assertThat(pivot.stoppingPointLoopOn(1, 0, 50)).as("still, it stays").isEqualTo(1);
  }
}
