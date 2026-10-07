package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.gearing;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.motors;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * A joint's rotations, fixed in one place: a revolute joint's own turns, a prismatic joint's
 * drum's, and the gains with them.
 */
class MechanismRotationsTest {
  private static final Joint.Revolute ARM =
      Joint.Revolute.builder()
          .label("arm")
          .parent(Part.CHASSIS)
          .origin(Transform3d.ZERO)
          .axis(Vector3.UP)
          .link(link("arm link"))
          .build();

  private static final Joint.Prismatic LIFT =
      Joint.Prismatic.builder()
          .label("lift")
          .parent(Part.CHASSIS)
          .origin(Transform3d.ZERO)
          .axis(Vector3.UP)
          .link(link("hook"))
          .build();

  /** 100:1 on a ½ in spool: a quarter of an inch of radius. */
  private static final Drive WINCH =
      motors(LIFT, Motor.KRAKEN_X60, 1, gearing(100, 1), Inches.of(0.25));

  @Test
  void aRevoluteJointsRotationIsItsTurn() {
    MechanismRotations arm =
        MechanismRotations.of(ARM, motors(ARM, Motor.KRAKEN_X60, 1, gearing(100, 1)));

    assertThat(arm.joint()).isEqualTo(ARM);
    assertThat(arm.perRotation()).isEqualTo(2 * Math.PI);
    assertThat(arm.toRotations(Math.PI / 2)).isCloseTo(0.25, within(1e-15));
    assertThat(arm.fromRotations(0.25)).isCloseTo(Math.PI / 2, within(1e-15));
    assertThat(arm.ratioPerRotation(100)).as("turns per turn already").isEqualTo(100);
  }

  /** 2 V per radian of error is 4π V per rotation: the same push for the same miss. */
  @Test
  void aRevoluteJointsGainsArePerRotation() {
    MechanismRotations arm =
        MechanismRotations.of(ARM, motors(ARM, Motor.KRAKEN_X60, 1, gearing(100, 1)));

    assertThat(arm.gainPerRotation(2)).isCloseTo(4 * Math.PI, within(1e-12));
    assertThat(arm.gainPerJointUnit(4 * Math.PI)).isCloseTo(2, within(1e-12));
  }

  /**
   * A winch's rotation is its spool's, 2π × 0.25 in of rope: its ratio the gearing alone, 100,
   * where the rotor's turns per meter, 2506, are past Phoenix's 1000. Its meters convert here, so a
   * soft limit of 0.5 m is 12.5 spool turns.
   */
  @Test
  void aDrumsRotationIsItsTurn() {
    MechanismRotations winch = MechanismRotations.of(LIFT, WINCH);
    double circumference = 2 * Math.PI * 0.25 * 0.0254;
    double rotorPerMeter = WINCH.rotor().ratio(LIFT);

    assertThat(rotorPerMeter).isCloseTo(2506.4, within(0.1));
    assertThat(winch.perRotation()).isCloseTo(circumference, within(1e-15));
    assertThat(winch.ratioPerRotation(rotorPerMeter)).isCloseTo(100, within(1e-9));
    assertThat(winch.toRotations(0.5)).isCloseTo(0.5 / circumference, within(1e-12));
    assertThat(winch.fromRotations(1)).isCloseTo(circumference, within(1e-15));
    assertThat(winch.gainPerRotation(30)).isCloseTo(30 * circumference, within(1e-12));
    assertThat(winch.gainPerJointUnit(30 * circumference)).isCloseTo(30, within(1e-12));
  }

  /**
   * A slide moved through a combined transmission, as a differential deploy's motors move theirs,
   * names its pinion in its term, so it's in the pinion's rotations, as a drum's: 5:1 onto a 1 in
   * pinion is 5 turns of the motor for each.
   */
  @Test
  void aSlideThroughACombinedTransmissionIsInItsPinionsRotations() {
    Drive combined =
        Drive.builder()
            .label("slide motor")
            .actuator(Actuator.Motors.builder().motor(Motor.KRAKEN_X60).count(1).build())
            .transmission(
                Transmission.Combined.builder()
                    .terms(
                        List.of(
                            Transmission.Term.builder()
                                .joint(LIFT)
                                .ratio(5)
                                .pitchRadius(Inches.of(0.5))
                                .build()))
                    .efficiency(1)
                    .build())
            .build();
    double circumference = 2 * Math.PI * 0.5 * 0.0254;

    MechanismRotations slide = MechanismRotations.of(LIFT, combined);

    assertThat(slide.perRotation()).isCloseTo(circumference, within(1e-15));
    assertThat(slide.ratioPerRotation(combined.rotor().ratio(LIFT))).isCloseTo(5, within(1e-12));
  }

  /** A slide nothing turns through a drum has no rotation: a linear servo's, say. */
  @Test
  void aSlideNoShaftTurnsHasNoRotation() {
    Drive cylinder =
        Drive.builder()
            .label("cylinder")
            .actuator(Actuator.Servo.builder().from(0.1).to(0.2).force(50).speed(0.03).build())
            .transmission(
                Transmission.Between.builder()
                    .a(Part.CHASSIS)
                    .onA(Translation3d.ZERO)
                    .b(LIFT)
                    .onB(Translation3d.ZERO)
                    .build())
            .build();

    assertThatIllegalArgumentException()
        .isThrownBy(() -> MechanismRotations.of(LIFT, cylinder))
        .withMessage("cylinder doesn't slide lift through a drum, a sprocket or a pinion");
  }

  private static Link link(String label) {
    return Link.builder().label(label).mass(MassProperties.NONE).build();
  }
}
