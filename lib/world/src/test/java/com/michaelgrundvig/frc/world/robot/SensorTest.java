package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.definition;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.gearing;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.offset;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Switches and rangefinders as the physical devices they are, and the conditions over them that say
 * what they mean.
 */
class SensorTest {
  private static final Joint.Revolute PIVOT =
      Joint.pivot(
          "pivot",
          Part.CHASSIS,
          new Translation3d(0.2, 0, 0.3),
          Joint.PITCH_UP,
          Range.degrees(0, 120),
          Degrees.of(0),
          Link.unseen("arm"));

  private static final Joint.Revolute ROLLER =
      Joint.roller("roller", PIVOT, new Translation3d(0.3, 0, 0), Link.unseen("roller"));

  /**
   * A limit switch at the bottom of the pivot's travel trips at or below its position; the top one
   * at or above; one whose position isn't known never says it's tripped, since nothing says where.
   */
  @Test
  void aLimitSwitchTripsAtItsEndOfTheTravel() {
    Sensor.Switch bottom =
        Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, 0);
    Sensor.Switch top =
        Sensor.Switch.limit("pivot up", PIVOT, Sensor.Switch.Limit.End.UPPER, Math.toRadians(120));
    Sensor.Switch.Limit unplaced =
        (Sensor.Switch.Limit)
            Sensor.Switch.limit("pivot somewhere", PIVOT, Sensor.Switch.Limit.End.LOWER).kind();

    Sensor.Switch.Limit lower = (Sensor.Switch.Limit) bottom.kind();
    Sensor.Switch.Limit upper = (Sensor.Switch.Limit) top.kind();
    assertThat(lower.trippedAt(0)).isTrue();
    assertThat(lower.trippedAt(-0.01)).isTrue();
    assertThat(lower.trippedAt(0.01)).isFalse();
    assertThat(upper.trippedAt(Math.toRadians(121))).isTrue();
    assertThat(upper.trippedAt(Math.toRadians(119))).isFalse();
    assertThat(unplaced.trippedAt(0)).isFalse();
    assertThat(bottom.part())
        .as("mounted where it's pressed: the pivot's parent")
        .isEqualTo(Part.CHASSIS);
  }

  /**
   * What a switch or a rangefinder means is a condition over its reading, either way: a beam
   * tripped or clear, a range nearer or farther than a distance it can read.
   */
  @Test
  void conditionsReadASwitchEitherWayAndARangeAgainstADistance() {
    Sensor.Switch claw = Sensor.Switch.beam("claw beam", PIVOT);
    Sensor.Rangefinder hopper =
        Sensor.Rangefinder.builder()
            .label("hopper range")
            .part(Part.CHASSIS)
            .pose(Optional.empty())
            .range(Meters.of(1.3))
            .fieldOfView(Degrees.of(27))
            .build();

    assertThat(claw.tripped()).hasToString("claw beam tripped");
    assertThat(claw.clear()).hasToString("claw beam clear");
    assertThat(claw.tripped().sensor()).isEqualTo(claw);
    assertThat(hopper.nearerThan(Inches.of(6))).hasToString("hopper range nearer than 0.152 m");
    assertThat(hopper.fartherThan(Meters.of(0.5))).hasToString("hopper range farther than 0.500 m");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> hopper.nearerThan(Meters.of(2)))
        .withMessageContaining("2.000 m isn't inside hopper range's range, 1.300 m");
  }

  /**
   * A switch on a part the robot doesn't have is refused; one not placed on its part, and a limit
   * switch whose position isn't known, are in the missing-data report, as a link's mass is.
   */
  @Test
  void theDefinitionKnowsWhereItsSwitchesAreAndSaysWhatItDoesnt() {
    Drive pivotMotor = PIVOT.motors(Motor.KRAKEN_X60, 1, gearing(30, 1));
    Drive rollerMotor = ROLLER.motors(Motor.KRAKEN_X60, 1, gearing(3, 1));
    Sensor.Switch placed =
        Sensor.Switch.beam("claw beam", PIVOT)
            .at(new Pose3d(new Translation3d(0.35, 0, 0), Rotation3d.ZERO));
    Sensor.Switch unplaced = Sensor.Switch.contact("cage", Part.CHASSIS);
    Sensor.Switch unknown = Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER);

    RobotDefinition robot =
        definition(
            List.of(PIVOT, ROLLER),
            List.of(),
            List.of(pivotMotor, rollerMotor),
            List.of(placed, unplaced, unknown));
    Joint.Revolute stranger =
        Joint.roller("stranger", Part.CHASSIS, new Translation3d(), Link.unseen("stranger"));

    assertThat(robot.math().sensorPose(placed, Map.of(PIVOT, Math.PI / 2)).getTranslation())
        .as("on the arm, near its end, raised")
        .satisfies(
            where -> {
              assertThat(where.getX()).isCloseTo(0.2, offset(1e-9));
              assertThat(where.getZ()).isCloseTo(0.65, offset(1e-9));
            });
    assertThat(robot.math().sensorPose(unplaced, Map.of()))
        .as("at its part's origin")
        .isEqualTo(Pose3d.ZERO);
    assertThat(robot.missing())
        .extracting(RobotDefinition.Missing::what)
        .contains(
            "cage isn't placed on its part",
            "pivot down isn't placed on its part",
            "pivot down's position on pivot isn't known")
        .doesNotContain("claw beam isn't placed on its part");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                definition(
                    List.of(PIVOT, ROLLER),
                    List.of(),
                    List.of(pivotMotor, rollerMotor),
                    List.of(Sensor.Switch.beam("stray beam", stranger))))
        .withMessageContaining(
            "sensor stray beam's joint, stranger, isn't one of the robot's joints");
  }
}
