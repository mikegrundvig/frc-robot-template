package frc.robot.lib.phoenix;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Actuator;
import com.michaelgrundvig.frc.world.robot.Drive;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Transmission;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Phoenix's units, fixed in one place: a revolute joint's rotations, a prismatic joint's drum's
 * rotations, and the gains with them.
 */
class PhoenixUnitsTest {
  private static final Joint.Revolute ARM =
      Joint.Revolute.builder()
          .label("arm")
          .parent(Part.CHASSIS)
          .origin(Transform3d.ZERO)
          .axis(Vector3.UP)
          .range(Optional.empty())
          .reference(0)
          .link(link("arm link"))
          .positions(List.of())
          .outputs(List.of())
          .build();

  private static final Joint.Prismatic LIFT =
      Joint.Prismatic.builder()
          .label("lift")
          .parent(Part.CHASSIS)
          .origin(Transform3d.ZERO)
          .axis(Vector3.UP)
          .range(Optional.empty())
          .reference(0)
          .link(link("hook"))
          .positions(List.of())
          .outputs(List.of())
          .build();

  private static final Gearing GEARING = Gearing.builder().ratio(100).efficiency(1).build();

  /** 100:1 on a ½ in spool: a quarter of an inch of radius. */
  private static final Drive WINCH = LIFT.motors(Motor.KRAKEN_X60, 1, GEARING, Inches.of(0.25));

  @Test
  void aRevoluteJointsPhoenixUnitIsARotation() {
    PhoenixUnits arm = PhoenixUnits.of(ARM, ARM.motors(Motor.KRAKEN_X60, 1, GEARING));

    assertThat(arm.joint()).isEqualTo(ARM);
    assertThat(arm.perPhoenixUnit()).isEqualTo(2 * Math.PI);
    assertThat(arm.toPhoenix(Math.PI / 2)).isCloseTo(0.25, within(1e-15));
    assertThat(arm.fromPhoenix(0.25)).isCloseTo(Math.PI / 2, within(1e-15));
    assertThat(arm.ratioToPhoenix(100)).as("turns per turn already").isEqualTo(100);
    assertThat(arm.name()).isEqualTo("rotations of arm");
    assertThat(arm.symbol()).isEqualTo("rot");
  }

  /** 2 V per radian of error is 4π V per rotation: the same push for the same miss. */
  @Test
  void aRevoluteJointsGainsArePerRotation() {
    PhoenixUnits arm = PhoenixUnits.of(ARM, ARM.motors(Motor.KRAKEN_X60, 1, GEARING));

    assertThat(arm.gainToPhoenix(2)).isCloseTo(4 * Math.PI, within(1e-12));
    assertThat(arm.gainFromPhoenix(4 * Math.PI)).isCloseTo(2, within(1e-12));
  }

  /**
   * A winch's Phoenix unit is its spool's rotation, 2π × 0.25 in of rope: its ratio the gearing
   * alone, 100, where the rotor's turns per meter, 2506, are past Phoenix's 1000. Its meters
   * convert here, so a soft limit of 0.5 m is 12.5 spool turns.
   */
  @Test
  void aDrumsPhoenixUnitIsItsRotation() {
    PhoenixUnits winch = PhoenixUnits.of(LIFT, WINCH);
    double circumference = 2 * Math.PI * 0.25 * 0.0254;
    double rotorPerMeter = WINCH.rotor().ratio(LIFT);

    assertThat(rotorPerMeter).isCloseTo(2506.4, within(0.1));
    assertThat(winch.perPhoenixUnit()).isCloseTo(circumference, within(1e-15));
    assertThat(winch.ratioToPhoenix(rotorPerMeter)).isCloseTo(100, within(1e-9));
    assertThat(winch.toPhoenix(0.5)).isCloseTo(0.5 / circumference, within(1e-12));
    assertThat(winch.fromPhoenix(1)).isCloseTo(circumference, within(1e-15));
    assertThat(winch.gainToPhoenix(30)).isCloseTo(30 * circumference, within(1e-12));
    assertThat(winch.gainFromPhoenix(30 * circumference)).isCloseTo(30, within(1e-12));
    assertThat(winch.name()).isEqualTo("rotations of lift's drum");
    assertThat(winch.symbol()).isEqualTo("drum rot");
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
                                .pitchRadius(Optional.of(Inches.of(0.5)))
                                .build()))
                    .efficiency(1)
                    .build())
            .build();
    double circumference = 2 * Math.PI * 0.5 * 0.0254;

    PhoenixUnits slide = PhoenixUnits.of(LIFT, combined);

    assertThat(slide.perPhoenixUnit()).isCloseTo(circumference, within(1e-15));
    assertThat(slide.ratioToPhoenix(combined.rotor().ratio(LIFT))).isCloseTo(5, within(1e-12));
    assertThat(slide.name()).isEqualTo("rotations of lift's drum");
    assertThat(slide.symbol()).isEqualTo("drum rot");
  }

  /** A slide nothing turns through a drum has no Phoenix unit: a linear servo's, say. */
  @Test
  void aSlideNoShaftTurnsHasNoPhoenixUnit() {
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
        .isThrownBy(() -> PhoenixUnits.of(LIFT, cylinder))
        .withMessage("cylinder doesn't slide lift through a drum, a sprocket or a pinion");
  }

  private static Link link(String label) {
    return Link.builder().label(label).mass(MassProperties.NONE).shapes(List.of()).build();
  }
}
