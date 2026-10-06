package frc.robot.game;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Pounds;

import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Shape;
import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.BodyBuilder;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Coupling;
import com.michaelgrundvig.frc.world.robot.Driven;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.LinkBuilder;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import frc.robot.RobotConfig;
import frc.robot.Robots;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The season's limits on what a robot is: every robot's definition fits them, and one that doesn't
 * is told each way it doesn't. An arm on the template's robot, 0.7 m long and pivoting 5 cm up at
 * its center, stands in for a mechanism: it starts straight up, inside the frame and under the
 * starting height, and reaches 0.35 m out laid flat, more than the 12 in allowed.
 */
class SeasonLimitsTest {
  private static final double UPRIGHT = Math.PI / 2;

  @Test
  void everyRobotFitsTheSeason() {
    for (RobotConfig robot : Robots.all()) {
      assertThat(Rebuilt.LIMITS.problems(robot.definition())).as(robot.name()).isEmpty();
    }
  }

  @Test
  void aRobotWithAnArmStartingUprightFits() {
    RobotDefinition robot = withArm(arm(0.7, UPRIGHT));

    assertThat(Rebuilt.LIMITS.problems(robot)).isEmpty();
  }

  /**
   * A robot with a joint that follows another is checked with each where its references put it: the
   * follower's position is its leader's to give, not the check's.
   */
  @Test
  void aRobotWithACoupledJointIsCheckedStowed() {
    Joint.Revolute arm = arm(0.7, UPRIGHT);
    Joint.Revolute forearm =
        JointRevoluteBuilder.builder(arm)
            .label("forearm")
            .link(LinkBuilder.builder(arm.link()).label("forearm").build())
            .build();
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(Robots.SIM.definition())
            .joints(List.of(arm, forearm))
            .couplings(
                List.of(
                    Coupling.Linear.builder()
                        .leader(arm)
                        .follower(forearm)
                        .ratio(1)
                        .offset(0)
                        .build()))
            .build();

    assertThat(Rebuilt.LIMITS.problems(robot)).isEmpty();
  }

  /**
   * A robot not yet weighed is the most the season allows, with its battery: the template's own,
   * which fits exactly.
   */
  @Test
  void anUnweighedRobotIsTheSeasonsMostWithABattery() {
    Body.Weight unweighed = Rebuilt.LIMITS.unweighed();

    assertThat(unweighed.weighed()).isFalse();
    assertThat(unweighed.mass().in(Pounds)).isCloseTo(149.5, within(1e-9));
    assertThat(Robots.SIM.definition().body().weight()).isEqualTo(unweighed);
    assertThat(Rebuilt.LIMITS.problems(Robots.SIM.definition())).isEmpty();
  }

  /** Everything wrong is told at once. */
  @Test
  void aRobotThatDoesntFitIsToldEveryWay() {
    Body weighed = Robots.SIM.definition().body();
    Body big =
        BodyBuilder.builder(weighed)
            .frameLength(Inches.of(30))
            .height(Inches.of(31))
            .bumperBottom(Inches.of(2))
            .weight(Body.Weight.weighed(Pounds.of(155)))
            .build();
    RobotDefinition robot = RobotDefinitionBuilder.builder(withArm(arm(0.7, 0))).body(big).build();

    assertThat(Rebuilt.LIMITS.problems(robot))
        .containsExactly(
            "its frame is 115 in around, more than the 110 in allowed",
            "it starts 0.787 m tall, more than the 0.762 m allowed",
            "it starts reaching 0.319 m beyond its frame perimeter: it must start inside it",
            "its bumpers, 2 in to 5.75 in up, aren't within the bumper zone, 2.5 in to 5.75 in",
            "it weighs 140.5 lb. without its battery, more than the 135 lb. allowed");
  }

  /** A link above the body at the start is too tall, as a body that tall would be. */
  @Test
  void aLinkStartingAboveTheStartingHeightIsTooTall() {
    RobotDefinition robot = withArm(arm(0.8, UPRIGHT));

    assertThat(Rebuilt.LIMITS.problems(robot))
        .containsExactly("it starts 0.850 m tall, more than the 0.762 m allowed");
  }

  /** What a mechanism's guard asks before a move: how far out the robot would reach. */
  @Test
  void aGuardAsksHowFarTheRobotWouldReach() {
    Joint.Revolute arm = arm(0.7, UPRIGHT);
    RobotDefinition robot = withArm(arm);

    assertThat(Rebuilt.LIMITS.whyTooFar(robot, Map.of(arm, UPRIGHT))).isNull();
    assertThat(Rebuilt.LIMITS.whyTooFar(robot, Map.of(arm, Math.PI / 4))).isNull();
    assertThat(Rebuilt.LIMITS.whyTooFar(robot, Map.of(arm, 0.0)))
        .isEqualTo(
            "it would reach 0.351 m beyond its frame perimeter, more than the 0.305 m allowed");
  }

  /**
   * Each configuration is checked against the extension limit, with the joints it doesn't name at
   * their references: the arm laid flat reaches 0.351 m out, more than the 12 in allowed, and
   * raised halfway, it's inside.
   */
  @Test
  void eachConfigurationIsCheckedAgainstTheExtensionLimit() {
    Joint.Revolute arm =
        JointRevoluteBuilder.builder(arm(0.7, UPRIGHT))
            .positions(
                List.of(
                    Position.degrees("flat", 0),
                    Position.degrees("halfway", 45),
                    Position.degrees("upright", 90)))
            .build();
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(withArm(arm))
            .configurations(
                List.of(
                    Configuration.of("flat", arm.at(arm.position("flat"))),
                    Configuration.of("halfway", arm.at(arm.position("halfway")))))
            .build();

    assertThat(Rebuilt.LIMITS.problems(robot))
        .containsExactly(
            "configuration flat reaches 0.351 m beyond its frame perimeter, more than the 0.305 m"
                + " allowed");
  }

  @Test
  void theBumperZoneHasATopAboveItsBottom() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> SeasonLimitsBuilder.builder(Rebuilt.LIMITS).bumperZoneTop(Inches.of(2.5)).build())
        .withMessageStartingWith("SeasonLimits.bumperZoneTop: ");
  }

  /** The template's robot with one arm, the arm's own reference its start. */
  private static RobotDefinition withArm(Joint arm) {
    return RobotDefinitionBuilder.builder(Robots.SIM.definition()).joints(List.of(arm)).build();
  }

  /**
   * A 3 kg arm of a length, 5 cm wide and deep, pivoting 5 cm up at the robot's center, raised as
   * its position grows, from flat forward at 0 to straight up at a quarter turn.
   */
  private static Joint.Revolute arm(double length, double reference) {
    Translation3d center = new Translation3d(length / 2, 0, 0);
    Translation3d size = new Translation3d(length, 0.05, 0.05);
    return Joint.Revolute.builder()
        .label("arm")
        .parent(Part.CHASSIS)
        .origin(new Transform3d(0, 0, 0.05, Rotation3d.ZERO))
        .axis(new Vector3(0, -1, 0))
        .range(Optional.of(Range.degrees(-10, 100)))
        .reference(reference)
        .link(
            Link.builder()
                .label("arm")
                .mass(
                    MassProperties.builder()
                        .mass(Kilograms.of(3))
                        .cg(center)
                        .inertia(Inertia.box(Kilograms.of(3), size))
                        .build())
                .shapes(
                    List.of(
                        Shape.Box.builder()
                            .pose(new Pose3d(center, Rotation3d.ZERO))
                            .size(size)
                            .material(Material.builder().name("aluminum").build())
                            .build()))
                .build())
        .positions(List.of())
        .outputs(List.of())
        .driven(Driven.NONE)
        .build();
  }
}
